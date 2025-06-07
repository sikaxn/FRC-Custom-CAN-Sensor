package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.util.Timer;
import java.util.TimerTask;

public class batteryCAN {
    private final CAN can;
    private final PowerDistribution pdh;

    private final int apiIdMeta1 = 0x131;
    private final int apiIdMeta2 = 0x132;
    private final int apiIdMeta3 = 0x133;

    private final int apiIdStatus1 = 0x135;
    private final int apiIdStatus2 = 0x136;

    private Timer pollTimer;
    private Timer sendTimer;

    public String serialNumber = "";
    public int firstUseYear = 0;
    public int firstUseMonth = 0;
    public int firstUseDay = 0;
    public int cycleCount = 0;
    public int note = 0;
    public String noteText = "Unknown";
    public boolean valid = false;

    public batteryCAN() {
        can = new CAN(33); // ESP32 device number
        pdh = new PowerDistribution();
        startPolling();
        startSending();
    }

    private void startPolling() {
        pollTimer = new Timer("BatteryCANReader", true);
        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
                    CANData rx1 = new CANData();
                    CANData rx2 = new CANData();
                    CANData rx3 = new CANData();

                    byte[] snPart1 = new byte[8];
                    byte[] snPart2 = new byte[8];

                    boolean got1 = can.readPacketLatest(apiIdMeta1, rx1) && rx1.length >= 1;
                    boolean got2 = can.readPacketLatest(apiIdMeta2, rx2);
                    boolean got3 = can.readPacketLatest(apiIdMeta3, rx3) && rx3.length >= 3;

                    if (!got1 || !got2 || !got3) return;

                    // Serial number
                    System.arraycopy(rx1.data, 0, snPart1, 0, Math.min(rx1.length, 8));
                    System.arraycopy(rx2.data, 0, snPart2, 0, Math.min(rx2.length, 8));

                    byte[] full = new byte[16];
                    System.arraycopy(snPart1, 0, full, 0, 8);
                    System.arraycopy(snPart2, 0, full, 8, 8);

                    int len = 0;
                    while (len < full.length && full[len] >= 32 && full[len] <= 126) len++;
                    serialNumber = new String(full, 0, len, StandardCharsets.US_ASCII);

                    // First use date (if valid)
                    if (rx2.length >= 8) {
                        int yearRaw = ((rx2.data[5] & 0xFF) << 8) | (rx2.data[6] & 0xFF);
                        if (yearRaw >= 2000 && yearRaw <= 2100) {
                            firstUseYear = yearRaw;
                            firstUseMonth = rx2.data[7] & 0xFF;
                            firstUseDay = rx2.data[4] & 0xFF;
                        }
                    }

                    // Cycle count and note
                    cycleCount = ((rx3.data[0] & 0xFF) << 8) | (rx3.data[1] & 0xFF);
                    note = rx3.data[2] & 0xFF;
                    noteText = interpretNote(note);

                    valid = true;

                } catch (Exception ignored) {
                    // do not throw in timer
                }
            }
        }, 0, 100);
    }

private int tickCount = 0;

private void startSending() {
    sendTimer = new Timer("BatteryCANSend", true);
    sendTimer.scheduleAtFixedRate(new TimerTask() {
        @Override
        public void run() {
            try {
                byte[] timeVoltage = buildTimeVoltagePayload();
                byte[] energy = buildEnergyPayload();

                can.writePacket(timeVoltage, apiIdStatus1);
                can.writePacket(energy, apiIdStatus2);

                // Debug: Show values on SmartDashboard
                SmartDashboard.putString("BatteryCAN TimeVoltage Payload", formatBytes(timeVoltage));
                SmartDashboard.putString("BatteryCAN Energy Payload", formatBytes(energy));

            } catch (Exception ignored) {}
        }
    }, 0, 100);
}


    private byte[] buildTimeVoltagePayload() {
        LocalDateTime now = LocalDateTime.now(ZoneOffset.UTC);
        int voltageRaw = (int) (RobotController.getBatteryVoltage() * 10); // 0.1V precision

        byte[] payload = new byte[8];
        payload[0] = (byte) (now.getYear() - 2000);
        payload[1] = (byte) now.getMonthValue();
        payload[2] = (byte) now.getDayOfMonth();
        payload[3] = (byte) now.getHour();
        payload[4] = (byte) now.getMinute();
        payload[5] = (byte) now.getSecond();
        payload[6] = (byte) voltageRaw;
        payload[7] = 0;
        //stem.out.println(voltageRaw);
        return payload;
    }

    private byte[] buildEnergyPayload() {
        int energyRaw = (int) (pdh.getTotalEnergy() * 10); // 0.1J precision
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

    private String formatBytes(byte[] data) {
        StringBuilder sb = new StringBuilder();
        for (byte b : data) {
            sb.append((b & 0xFF)).append(" ");
        }
        return sb.toString().trim();
    }
    
}
