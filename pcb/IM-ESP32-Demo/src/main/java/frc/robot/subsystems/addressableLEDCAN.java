package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;

public class addressableLEDCAN {
    private static final int GENERAL_API_ID = 0x350;
    private static final int CUSTOM_PIXEL_BASE_API_ID = 0x351;  // 0x351–0x358

    private final CAN can;
    private final int deviceNumber;

    public addressableLEDCAN(int deviceNumber) {
        this.deviceNumber = deviceNumber;
        this.can = new CAN(deviceNumber);
    }

    public void sendGeneralCommand(int mode, int r, int g, int b,
                                   int brightness, int onOff, int param0, int param1) {
        byte[] data = new byte[8];
        data[0] = (byte) mode;
        data[1] = (byte) r;
        data[2] = (byte) g;
        data[3] = (byte) b;
        data[4] = (byte) brightness;
        data[5] = (byte) onOff;
        data[6] = (byte) param0;
        data[7] = (byte) param1;

        try {
            can.writePacket(data, GENERAL_API_ID);
            //System.out.printf("[addressableLEDCAN] → 0x%03X GEN %s\n", GENERAL_API_ID, java.util.Arrays.toString(data));
        } catch (Exception e) {
            System.err.println("CAN send failed: " + e.getMessage());
        }
    }

    public void sendPixelWrite(int pixelIndex, int r, int g, int b, int w, int brightness, int slot) {
        if (slot < 0 || slot > 7) {
            System.err.println("Invalid slot index for pixel write.");
            return;
        }

        int apiId = CUSTOM_PIXEL_BASE_API_ID + slot;
        byte[] data = new byte[8];
        data[0] = (byte) ((pixelIndex >> 8) & 0xFF);
        data[1] = (byte) (pixelIndex & 0xFF);
        data[2] = (byte) r;
        data[3] = (byte) g;
        data[4] = (byte) b;
        data[5] = (byte) w;
        data[6] = (byte) brightness;
        data[7] = 0;

        try {
            can.writePacket(data, apiId);
        } catch (Exception e) {
            System.err.printf("Pixel write failed: apiId=0x%03X pixel=%d slot=%d\n", apiId, pixelIndex, slot);
        }
    }
}
