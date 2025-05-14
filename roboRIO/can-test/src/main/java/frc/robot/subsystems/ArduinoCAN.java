package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

import java.util.Timer;
import java.util.TimerTask;

public class ArduinoCAN {
    private final CAN can;
    private final int statusApiId = 0x180; // Arduino -> RIO
    private final int controlApiId = 0x190; // RIO -> Arduino

    public boolean[] buttons = new boolean[3];
    public int analogValue = 0;

    private Timer pollTimer;

    public ArduinoCAN() {
        can = new CAN(0); // Device ID unused for direct API ID usage
        startPolling();
    }

    private void startPolling() {
        pollTimer = new Timer("ArduinoCANListener", true);
        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
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
                } catch (Exception e) {
                    System.err.println("CAN polling error: " + e.getMessage());
                }
            }
        }, 0, 20); // poll every 20 ms
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
        if (pollTimer != null) pollTimer.cancel();
    }
}
