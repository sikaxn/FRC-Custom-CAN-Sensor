package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.HashMap;
import java.util.Map;

public class DJIMotorGateway {
    private final int deviceID;
    private final CAN can;
    private final Map<Integer, DJIMotor> motors = new HashMap<>();
    private final int[] motorCurrents = new int[11]; // Index 0 = Motor 1, Index 10 = Motor 11

    public DJIMotorGateway(int deviceID) {
        this.deviceID = deviceID;
        this.can = new CAN(deviceID); // CAN ID of ESP32 gateway

        // Immediately start sending data to all ESCs
        new Thread(() -> {
            while (true) {
                boolean disabled = RobotBase.isSimulation() ? false : !RobotController.isSysActive();
                if (disabled) {
                    for (int i = 0; i < motorCurrents.length; i++) {
                        motorCurrents[i] = 0;
                    }
                }
                flush();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {}
            }
        }, "DJIMotor Flusher").start();
    }

    public DJIMotor motor(int motorID) {
        if (motorID < 1 || motorID > 11)
            throw new IllegalArgumentException("Motor ID must be 1–11");
        return motors.computeIfAbsent(motorID, id -> new DJIMotor(can, deviceID, id));
    }

    public void flush() {
        sendFrame(0x111, 0); // motors 1–4 → API ID 0x111
        sendFrame(0x112, 4); // motors 5–8 → API ID 0x112
        sendFrame(0x113, 8); // motors 9–11 → API ID 0x113
    }

    private void sendFrame(int apiID, int startIdx) {
        byte[] data = new byte[8];
        for (int i = 0; i < 4; i++) {
            int motorIdx = startIdx + i;
            int current = (motorIdx < 11) ? motorCurrents[motorIdx] : 0;
            data[i * 2] = (byte) ((current >> 8) & 0xFF);
            data[i * 2 + 1] = (byte) (current & 0xFF);
        }

        int fullID = apiID;

        try {
            can.writePacket(data, fullID);
        } catch (Exception e) {
            System.err.println("CAN write error for API ID 0x" + Integer.toHexString(apiID) + ": " + e.getMessage());
        }
    }

    public class DJIMotor implements MotorController {
        private final int motorID;
        private boolean inverted = false;

        private final int feedbackApiID;

        public DJIMotor(CAN can, int deviceID, int motorID) {
            this.motorID = motorID;
            this.feedbackApiID = 0x120 + motorID; // 0x121–0x12B
        }

        @Override
        public void set(double speed) {
            if (inverted) speed = -speed;
            int current = (int)(Math.max(-1.0, Math.min(1.0, speed)) * 16384);
            motorCurrents[motorID - 1] = current;
        }

        @Override
        public double get() {
            int fullID = (deviceID << 24) | (0x08 << 16) | (feedbackApiID << 6);
            CANData rx = new CANData();
            try {
                if (can.readPacketLatest(fullID, rx) && rx.length >= 4) {
                    int speed = ((rx.data[2] & 0xFF) << 8) | (rx.data[3] & 0xFF);
                    double scaled = speed / 16384.0;
                    return inverted ? -scaled : scaled;
                }
            } catch (Exception e) {
                //System.err.println("CAN read error for motor " + motorID + ": " + e.getMessage());
            }
            return 0.0;
        }

        @Override
        public void setInverted(boolean isInverted) {
            this.inverted = isInverted;
        }

        @Override
        public boolean getInverted() {
            return inverted;
        }

        @Override
        public void disable() {
            set(0);
        }

        @Override
        public void stopMotor() {
            set(0);
        }

        public int getRotorAngle() {
            return getStatus()[0];
        }

        public int getRotorSpeed() {
            return getStatus()[1];
        }

        public int getRotorTorque() {
            return getStatus()[2];
        }

        public int getTemperature() {
            return getStatus()[3];
        }

        private int[] getStatus() {
            int fullID = (deviceID << 24) | (0x08 << 16) | (feedbackApiID << 6);
            CANData rx = new CANData();

            try {
                if (can.readPacketLatest(fullID, rx) && rx.length >= 7) {
                    byte[] data = rx.data;
                    int angle = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
                    int speed = ((data[2] & 0xFF) << 8) | (data[3] & 0xFF);
                    int torque = (short)(((data[4] & 0xFF) << 8) | (data[5] & 0xFF));
                    int temp = data[6] & 0xFF;
                    return new int[]{angle, speed, torque, temp};
                }
            } catch (Exception e) {
                System.err.println("CAN read error for motor " + motorID + ": " + e.getMessage());
            }
            return new int[]{0, 0, 0, 0};
        }
    }
}
