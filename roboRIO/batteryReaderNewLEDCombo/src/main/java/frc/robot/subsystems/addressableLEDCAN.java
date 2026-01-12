package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;

/**
 * AddressableLEDCAN driver for ESP32-based LED controller.
 *
 * RIO -> ESP32:
 *   0x350: General command (mode, RGB, brightness, on/off, param0, param1)
 *   0x351-0x358: Custom pixel writes (slot indexed)
 */
public class addressableLEDCAN {
  private static final int DEFAULT_DEVICE_NUMBER = 35;

  // --- CAN API IDs ---
  private static final int API_GENERAL_CMD = 0x350;
  private static final int API_CUSTOM_PIXEL_BASE = 0x351; // 0x351..0x358
  private static final int API_TOTAL_PIXEL_COUNT = 0x360;

  private final CAN can;

  // --- CAN error tracking ---
  private boolean canWriteError = false;
  private int totalPixelCount = 10;

  // --------------------------------------------------------------------------
  // Constructor
  // --------------------------------------------------------------------------
  public addressableLEDCAN() {
    this(DEFAULT_DEVICE_NUMBER);
  }

  public addressableLEDCAN(int deviceNumber) {
    this.can = new CAN(deviceNumber);
  }

  // --------------------------------------------------------------------------
  // RIO -> ESP32 command writers
  // --------------------------------------------------------------------------
  public void sendGeneralCommand(int mode, int r, int g, int b,
                                 int brightness, int onOff, int param0, int param1) {
    sendTotalPixel();
    byte[] data = new byte[8];
    data[0] = (byte) mode;
    data[1] = (byte) r;
    data[2] = (byte) g;
    data[3] = (byte) b;
    data[4] = (byte) brightness;
    data[5] = (byte) onOff;
    data[6] = (byte) param0;
    data[7] = (byte) param1;

    writePacket(data, API_GENERAL_CMD, "general");
  }

  public void sendPixelWrite(int pixelIndex, int r, int g, int b, int w, int brightness, int slot) {
    if (slot < 0 || slot > 7) {
      System.err.println("[AddressableLEDCAN] Invalid slot index for pixel write.");
      return;
    }

    int apiId = API_CUSTOM_PIXEL_BASE + slot;
    byte[] data = new byte[8];
    data[0] = (byte) ((pixelIndex >> 8) & 0xFF);
    data[1] = (byte) (pixelIndex & 0xFF);
    data[2] = (byte) r;
    data[3] = (byte) g;
    data[4] = (byte) b;
    data[5] = (byte) w;
    data[6] = (byte) brightness;
    data[7] = 0;

    writePacket(data, apiId, "pixel");
  }

  public void setTotalPixel(int count) {
    totalPixelCount = Math.max(0, Math.min(0xFFFF, count));
  }

  private void sendTotalPixel() {
    byte[] data = new byte[8];
    data[0] = (byte) ((totalPixelCount >> 8) & 0xFF);
    data[1] = (byte) (totalPixelCount & 0xFF);
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    writePacket(data, API_TOTAL_PIXEL_COUNT, "totalPixel");
  }

  // --------------------------------------------------------------------------
  // Internal helpers
  // --------------------------------------------------------------------------
  private void writePacket(byte[] payload, int apiId, String label) {
    try {
      can.writePacket(payload, apiId);

      if (canWriteError) {
        System.out.println("[AddressableLEDCAN] CAN bus recovered.");
        canWriteError = false;
      }
    } catch (edu.wpi.first.hal.util.UncleanStatusException e) {
      if (!canWriteError && e.getMessage() != null
          && e.getMessage().contains("CAN Output Buffer Full")) {
        System.out.println("[AddressableLEDCAN] CAN buffer full - is ESP32 disconnected?");
        canWriteError = true;
      }
    } catch (Exception e) {
      if (!canWriteError) {
        System.out.println("[AddressableLEDCAN] Unexpected CAN write exception: " + e.getMessage());
        canWriteError = true;
      }
    }
  }
}
