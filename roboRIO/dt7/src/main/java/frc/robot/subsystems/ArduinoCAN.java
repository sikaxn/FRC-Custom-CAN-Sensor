package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

public class ArduinoCAN {
    private final CAN can;

    private final int statusApiId = 0x180;
    private final int colorApiId = 0x184;
    private final int controlApiId = 0x190;

    private final int dt7Api1Id = 0x185;
    private final int dt7Api2Id = 0x186;
    private final int dt7Api3Id = 0x187;

    public boolean[] buttons = new boolean[3];
    public int analogValue = 0;

    public int red = 0, green = 0, blue = 0, proximity = 0, ir = 0;
    public boolean colorSensorConnected = false;

    // DBUS decoded values
    public int ch0 = 0, ch1 = 0, ch2 = 0, ch3 = 0;
    public int s1 = 0, s2 = 0;

    public int mouseX = 0, mouseY = 0, mouseZ = 0;
    public boolean mouseLeft = false, mouseRight = false;

    public int keyboard = 0;
    public boolean[] keyFlags = new boolean[16];

    public boolean connected = false;

    private final byte[] dbusBuf = new byte[18];

    private Timer pollTimer;

    public ArduinoCAN() {
        can = new CAN(33); // CAN device ID (match what Arduino is sending to)
        startPolling();
    }

    private void startPolling() {
        pollTimer = new Timer("ArduinoCANListener", true);
        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
                    connected = false;

                    // Frame 1
                    CANData f1 = new CANData();
                    if (!can.readPacketLatest(dt7Api1Id, f1) || f1.length != 8) return;
                    System.arraycopy(f1.data, 0, dbusBuf, 0, 8);

                    // Frame 2
                    CANData f2 = new CANData();
                    if (!can.readPacketLatest(dt7Api2Id, f2) || f2.length != 8) return;
                    System.arraycopy(f2.data, 0, dbusBuf, 8, 8);

                    // Frame 3
                    CANData f3 = new CANData();
                    if (!can.readPacketLatest(dt7Api3Id, f3) || f3.length < 2) return;
                    dbusBuf[16] = f3.data[0];
                    dbusBuf[17] = f3.data[1];

                    // Check disconnect indicator
                    if ((dbusBuf[0] & 0xFF) == 0xFF) {
                        connected = false;
                        return;
                    }

                    connected = true;

                    // === Decode sticks ===
                    ch0 = ((dbusBuf[0] & 0xFF) | ((dbusBuf[1] & 0x07) << 8)) - 1024;
                    ch1 = (((dbusBuf[1] & 0xF8) >> 3) | ((dbusBuf[2] & 0x3F) << 5)) - 1024;
                    ch2 = (((dbusBuf[2] & 0xC0) >> 6) | ((dbusBuf[3] & 0xFF) << 2) | ((dbusBuf[4] & 0x01) << 10)) - 1024;
                    ch3 = (((dbusBuf[4] & 0xFE) >> 1) | ((dbusBuf[5] & 0x0F) << 7)) - 1024;

                    s1 = ((dbusBuf[5] >> 4) & 0x03) + 1;
                    s2 = ((dbusBuf[5] >> 6) & 0x03) + 1;

                    // === Mouse ===
                    mouseX = (short)((dbusBuf[6] & 0xFF) | (dbusBuf[7] << 8));
                    mouseY = (short)((dbusBuf[8] & 0xFF) | (dbusBuf[9] << 8));
                    mouseZ = (short)((dbusBuf[10] & 0xFF) | (dbusBuf[11] << 8));
                    mouseLeft = dbusBuf[12] != 0;
                    mouseRight = dbusBuf[13] != 0;

                    // === Keyboard ===
                    keyboard = ((dbusBuf[14] & 0xFF) | (dbusBuf[15] << 8));
                    for (int i = 0; i < 16; i++) {
                        keyFlags[i] = ((keyboard >> i) & 0x01) != 0;
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
