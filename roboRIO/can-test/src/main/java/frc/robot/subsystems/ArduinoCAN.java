package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

import java.util.Timer;
import java.util.TimerTask;

public class ArduinoCAN {
    private final CAN can;

    private final int statusApiId = 0x180; // Arduino -> RIO Button and analog
    private final int colorApiId = 0x184;  // Arduino -> RIO Color sensor data
    private final int controlApiId = 0x190; // RIO -> Arduino

    public boolean[] buttons = new boolean[3];
    public int analogValue = 0;

    public int red = 0, green = 0, blue = 0, proximity = 0, ir = 0;
    public boolean colorSensorConnected = false;

    private Timer pollTimer;

    public ArduinoCAN() {
        can = new CAN(0);
        startPolling();
    }

    private void startPolling() {
        pollTimer = new Timer("ArduinoCANListener", true);
        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
                    // === Buttons and analog ===
                    CANData rx = new CANData();
                    if (can.readPacketLatest(statusApiId, rx)) {
                        byte[] data = rx.data;
                        if (rx.length >= 3) {
                            int btnMask = data[0] & 0xFF;
                            buttons[0] = (btnMask & 0x01) != 0;
                            buttons[1] = (btnMask & 0x02) != 0;
                            buttons[2] = (btnMask & 0x04) != 0;
                            analogValue = ((data[1] & 0xFF) << 8) | (data[2] & 0xFF);
                        }
                    }

                    // === Color sensor frame ===
                    CANData colorRx = new CANData();
                    if (can.readPacketLatest(colorApiId, colorRx)) {
                        byte[] data = colorRx.data;
                        if (colorRx.length >= 8) {
                            if ((data[0] & 0xFF) == 0xFF) {
                                colorSensorConnected = false;
                            } else {
                                red   = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
                                green = ((data[2] & 0xFF) << 8) | (data[3] & 0xFF);
                                blue  = ((data[4] & 0xFF) << 8) | (data[5] & 0xFF);
                                proximity = data[6] & 0xFF;
                                ir = data[7] & 0xFF;
                                colorSensorConnected = true;
                            }
                        }
                    }

                } catch (Exception e) {
                    System.err.println("CAN polling error: " + e.getMessage());
                }
            }
        }, 0, 20);
    }

    public void sendControl(boolean led0, boolean led1, int servoAngle) {
        byte[] tx = new byte[2];
        tx[0] = (byte)((led0 ? 1 : 0) | (led1 ? 2 : 0));
        tx[1] = (byte)servoAngle;
        try {
            can.writePacket(tx, controlApiId);
        } catch (Exception e) {
            System.err.println("CAN write error: " + e.getMessage());
        }
    }

    public void stop() {
        if (pollTimer != null) {
            pollTimer.cancel();
        }
    }
}
