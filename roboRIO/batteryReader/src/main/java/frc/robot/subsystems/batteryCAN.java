package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.util.Timer;
import java.util.TimerTask;

public class batteryCAN {
    private final CAN can;
    private final PowerDistribution pdh;

    // API IDs for RFID meta
    private final int apiIdMeta1 = 0x131;
    private final int apiIdMeta2 = 0x132;
    private final int apiIdMeta3 = 0x133;

    // API IDs for battery status
    private final int apiIdStatus1 = 0x135; // time + voltage
    private final int apiIdStatus2 = 0x136; // energy

    private Timer pollTimer;
    private Timer sendTimer;

    public String serialNumber = "";
    public int firstUseYear = 0;
    public int firstUseMonth = 0;
    public int firstUseDay = 0;
    public int cycleCount = 0;
    public int note = 0;
    public String noteText = "";
    public boolean valid = false;

    public batteryCAN() {
        can = new CAN(33); // Device number of the ESP32
        pdh = new PowerDistribution(); // Automatically detect PDP/PDH
        startPolling();
        startSending();
    }

    private void startPolling() {
        pollTimer = new Timer("BatteryCANReader", true);
        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
                    byte[] snPart1 = new byte[8];
                    byte[] snPart2 = new byte[8];

                    CANData rx = new CANData();

                    if (can.readPacketLatest(apiIdMeta1, rx) && rx.length >= 1) {
                        System.arraycopy(rx.data, 0, snPart1, 0, Math.min(rx.length, 8));
                    } else return;

                    if (can.readPacketLatest(apiIdMeta2, rx)) {
                        System.arraycopy(rx.data, 0, snPart2, 0, Math.min(rx.length, 8));
                        if (rx.length >= 8) {
                            int yearRaw = ((rx.data[5] & 0xFF) << 8) | (rx.data[6] & 0xFF);
                            firstUseYear = yearRaw;
                            firstUseMonth = rx.data[7] & 0xFF;
                            firstUseDay = rx.data[4] & 0xFF;
                        }
                    } else return;

                    if (can.readPacketLatest(apiIdMeta3, rx) && rx.length >= 3) {
                        cycleCount = ((rx.data[0] & 0xFF) << 8) | (rx.data[1] & 0xFF);
                        note = rx.data[2] & 0xFF;
                        noteText = interpretNote(note);
                    } else return;

                    byte[] full = new byte[16];
                    System.arraycopy(snPart1, 0, full, 0, 8);
                    System.arraycopy(snPart2, 0, full, 8, 8);
                    int len = 0;
while (len < full.length && full[len] >= 32 && full[len] <= 126) len++;
serialNumber = new String(full, 0, len, StandardCharsets.US_ASCII);



                    valid = true;

                } catch (Exception ignored) {}
            }
        }, 0, 100);
    }

    private void startSending() {
        sendTimer = new Timer("BatteryCANSend", true);
        sendTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
                    byte[] timeVoltage = buildTimeVoltagePayload();
                    byte[] energyBytes = buildEnergyPayload();
                    can.writePacket(timeVoltage, apiIdStatus1);
                    can.writePacket(energyBytes, apiIdStatus2);
                } catch (Exception ignored) {}
            }
        }, 0, 100); // Send every 100ms
    }

    private byte[] buildTimeVoltagePayload() {
        LocalDateTime now = LocalDateTime.now();
        int voltageRaw = (int) (RobotController.getBatteryVoltage() * 10); // 0.1V

        byte[] payload = new byte[8];
        payload[0] = (byte) (now.getYear() - 2000);
        payload[1] = (byte) now.getMonthValue();
        payload[2] = (byte) now.getDayOfMonth();
        payload[3] = (byte) now.getHour();
        payload[4] = (byte) now.getMinute();
        payload[5] = (byte) now.getSecond();
        payload[6] = (byte) voltageRaw;
        payload[7] = 0;

        return payload;
    }

    private byte[] buildEnergyPayload() {
        int energyRaw = (int) (pdh.getTotalEnergy() * 10); // 0.1J units
        return new byte[] {
            (byte) ((energyRaw >> 8) & 0xFF),
            (byte) (energyRaw & 0xFF)
        };
    }

    private String interpretNote(int noteVal) {
        return switch (noteVal) {
            case 0 -> "Normal";
            case 1 -> "Practice Only";
            case 2 -> "Scrap";
            case 3 -> "Other";
            default -> "Unknown";
        };
    }

    public void stop() {
        if (pollTimer != null) pollTimer.cancel();
        if (sendTimer != null) sendTimer.cancel();
    }
}
