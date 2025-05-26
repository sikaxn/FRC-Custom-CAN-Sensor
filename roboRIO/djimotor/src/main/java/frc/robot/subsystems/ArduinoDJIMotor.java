package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;


public class ArduinoDJIMotor {
    private final CAN can;

    // API IDs for motor commands (map to 0x200, 0x1FF, 0x2FF)
    private static final int apiId111 = 0x111;
    private static final int apiId112 = 0x112;
    private static final int apiId113 = 0x113;

    public ArduinoDJIMotor() {
        can = new CAN(11);  // CAN device ID for ESP32 MCP2515
    }

    /**
     * Send current values for motor ID 1 and 4 using frame to 0x200
     * M3508 uses 0x200 ID for current control of up to 4 motors (IDs 1-4)
     */
    public void sendMotorCurrents(int motor1Current, int motor4Current) {
        // Clamp values to ESC-supported range: -16384 to +16384
        motor1Current = Math.max(-16384, Math.min(16384, motor1Current));
        motor4Current = Math.max(-16384, Math.min(16384, motor4Current));

        byte[] frame = new byte[8];

        // Motor ID 1 → bytes 0,1
        frame[0] = (byte) ((motor1Current >> 8) & 0xFF);
        frame[1] = (byte) (motor1Current & 0xFF);

        // Motor ID 4 → bytes 6,7
        frame[6] = (byte) ((motor4Current >> 8) & 0xFF);
        frame[7] = (byte) (motor4Current & 0xFF);

        try {
            can.writePacket(frame, apiId111); // Corresponds to 0x200
        } catch (Exception e) {
            System.err.println("CAN write error (motor current): " + e.getMessage());
        }
    }

    public void stop() {
        // If polling or other scheduled actions are needed, stop them here
    }
}
