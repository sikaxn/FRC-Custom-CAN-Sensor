package frc.robot.subsystems;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * AddressableLEDCAN driver for ESP32-based LED controller.
 *
 * RIO -> ESP32:
 *   0x350: General command (mode, RGB, brightness, on/off, param0, param1)
 *   0x351-0x358: Custom pixel writes (slot indexed)
 *   0x360: Total pixel count + secondary color
 *
 * ESP32 -> RIO:
 *   0x359: Feedback frame (num LEDs, current mode)
 */
public class addressableLEDCAN {
  private static final int DEFAULT_DEVICE_NUMBER = 35;

  // --- CAN API IDs ---
  private static final int API_GENERAL_CMD = 0x350;
  private static final int API_CUSTOM_PIXEL_BASE = 0x351; // 0x351..0x358
  private static final int API_FEEDBACK = 0x359;
  private static final int API_GENERAL_2 = 0x360;

  private final CAN can;
  private final CANData rxFrame = new CANData();
  private final Notifier notifier;

  // --- CAN error tracking ---
  private boolean canWriteError = false;
  private static final double ESP_TIMEOUT_S = 1.000;
  private static final double RESEND_INTERVAL_S = 0.050;

  private int desiredMode = -1;
  private int desiredR = -1;
  private int desiredG = -1;
  private int desiredB = -1;
  private int desiredBrightness = -1;
  private int desiredOnOff = -1;
  private int desiredParam0 = -1;
  private int desiredParam1 = -1;
  private int desiredTotalPixels = 10;
  private int desiredR2 = 0;
  private int desiredG2 = 0;
  private int desiredB2 = 0;
  private int desiredBrightness2 = 0;
  private int desiredOnOff2 = 0;
  private boolean hasDesiredGeneral = false;

  private int lastSentMode = -1;
  private int lastSentR = -1;
  private int lastSentG = -1;
  private int lastSentB = -1;
  private int lastSentBrightness = -1;
  private int lastSentOnOff = -1;
  private int lastSentParam0 = -1;
  private int lastSentParam1 = -1;
  private int lastSentTotalPixels = -1;
  private int lastSentR2 = -1;
  private int lastSentG2 = -1;
  private int lastSentB2 = -1;
  private int lastSentBrightness2 = -1;
  private int lastSentOnOff2 = -1;
  private double lastGeneralSendTime = 0.0;

  private final int[] lastPixelIndex = new int[8];
  private final int[] lastPixelR = new int[8];
  private final int[] lastPixelG = new int[8];
  private final int[] lastPixelB = new int[8];
  private final int[] lastPixelW = new int[8];
  private final int[] lastPixelBrightness = new int[8];

  // --- ESP feedback tracking ---
  private int feedbackLedCount = -1;
  private int feedbackMode = -1;
  private double lastUpdate = 0.0;
  private boolean espOnline = false;
  private boolean lastESPOnline = false;

  // --------------------------------------------------------------------------
  // Constructor
  // --------------------------------------------------------------------------
  public addressableLEDCAN() {
    this(DEFAULT_DEVICE_NUMBER);
  }

  public addressableLEDCAN(int deviceNumber) {
    this.can = new CAN(deviceNumber);
    for (int i = 0; i < 8; i++) {
      lastPixelIndex[i] = -1;
      lastPixelR[i] = -1;
      lastPixelG[i] = -1;
      lastPixelB[i] = -1;
      lastPixelW[i] = -1;
      lastPixelBrightness[i] = -1;
    }

    notifier = new Notifier(this::update);
    notifier.startPeriodic(RESEND_INTERVAL_S);
  }

  // --------------------------------------------------------------------------
  // RIO -> ESP32 command writers
  // --------------------------------------------------------------------------
  public synchronized void sendGeneralCommand(int mode, int r, int g, int b,
                                              int brightness, int onOff, int param0, int param1) {
    desiredMode = mode;
    desiredR = r;
    desiredG = g;
    desiredB = b;
    desiredBrightness = brightness;
    desiredOnOff = onOff;
    desiredParam0 = param0;
    desiredParam1 = param1;
    hasDesiredGeneral = true;

    boolean changed =
        mode != lastSentMode ||
        r != lastSentR ||
        g != lastSentG ||
        b != lastSentB ||
        brightness != lastSentBrightness ||
        onOff != lastSentOnOff ||
        param0 != lastSentParam0 ||
        param1 != lastSentParam1;

    if (changed) {
      sendTotalPixelIfNeeded();
      sendGeneralCommandNow();
    }
  }

  public synchronized void sendPixelWrite(int pixelIndex, int r, int g, int b, int w, int brightness, int slot) {
    if (slot < 0 || slot > 7) {
      System.err.println("[AddressableLEDCAN] Invalid slot index for pixel write.");
      return;
    }

    if (pixelIndex == lastPixelIndex[slot]
        && r == lastPixelR[slot]
        && g == lastPixelG[slot]
        && b == lastPixelB[slot]
        && w == lastPixelW[slot]
        && brightness == lastPixelBrightness[slot]) {
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

    lastPixelIndex[slot] = pixelIndex;
    lastPixelR[slot] = r;
    lastPixelG[slot] = g;
    lastPixelB[slot] = b;
    lastPixelW[slot] = w;
    lastPixelBrightness[slot] = brightness;
  }

  public synchronized void setTotalPixel(int count) {
    desiredTotalPixels = Math.max(0, Math.min(0xFFFF, count));
    sendTotalPixelIfNeeded();
  }

  public synchronized void setSecondaryColor(int enable, int r, int g, int b, int brightness) {
    desiredOnOff2 = enable;
    desiredR2 = r;
    desiredG2 = g;
    desiredB2 = b;
    desiredBrightness2 = brightness;
    sendTotalPixelIfNeeded();
  }

  private void sendTotalPixelIfNeeded() {
    if (desiredTotalPixels == lastSentTotalPixels
        && desiredR2 == lastSentR2
        && desiredG2 == lastSentG2
        && desiredB2 == lastSentB2
        && desiredBrightness2 == lastSentBrightness2
        && desiredOnOff2 == lastSentOnOff2) {
      return;
    }

    byte[] data = new byte[8];
    data[0] = (byte) ((desiredTotalPixels >> 8) & 0xFF);
    data[1] = (byte) (desiredTotalPixels & 0xFF);
    data[2] = (byte) desiredR2;
    data[3] = (byte) desiredG2;
    data[4] = (byte) desiredB2;
    data[5] = (byte) desiredBrightness2;
    data[6] = (byte) desiredOnOff2;
    data[7] = 0;
    writePacket(data, API_GENERAL_2, "general2");
    lastSentTotalPixels = desiredTotalPixels;
    lastSentR2 = desiredR2;
    lastSentG2 = desiredG2;
    lastSentB2 = desiredB2;
    lastSentBrightness2 = desiredBrightness2;
    lastSentOnOff2 = desiredOnOff2;
  }

  private void sendGeneralCommandNow() {
    if (!hasDesiredGeneral) {
      return;
    }

    byte[] data = new byte[8];
    data[0] = (byte) desiredMode;
    data[1] = (byte) desiredR;
    data[2] = (byte) desiredG;
    data[3] = (byte) desiredB;
    data[4] = (byte) desiredBrightness;
    data[5] = (byte) desiredOnOff;
    data[6] = (byte) desiredParam0;
    data[7] = (byte) desiredParam1;

    writePacket(data, API_GENERAL_CMD, "general");

    lastSentMode = desiredMode;
    lastSentR = desiredR;
    lastSentG = desiredG;
    lastSentB = desiredB;
    lastSentBrightness = desiredBrightness;
    lastSentOnOff = desiredOnOff;
    lastSentParam0 = desiredParam0;
    lastSentParam1 = desiredParam1;
    lastGeneralSendTime = Timer.getFPGATimestamp();
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

  // --------------------------------------------------------------------------
  // Background update: feedback RX and online tracking
  // --------------------------------------------------------------------------
  private synchronized void update() {
    double now = Timer.getFPGATimestamp();
    boolean gotFrame = false;

    if (can.readPacketNew(API_FEEDBACK, rxFrame)) {
      parseFeedback(rxFrame.data);
      gotFrame = true;
    }

    if (gotFrame) {
      lastUpdate = now;
    }

    espOnline = (now - lastUpdate) <= ESP_TIMEOUT_S;
    if (espOnline != lastESPOnline) {
      if (espOnline) {
        System.out.println("[AddressableLEDCAN] ESP32 reconnected.");
      } else {
        System.out.println("[AddressableLEDCAN] ESP32 offline.");
      }
      lastESPOnline = espOnline;
    }

    if (gotFrame && hasDesiredGeneral && feedbackMode >= 0 && feedbackMode != desiredMode) {
      if ((now - lastGeneralSendTime) >= RESEND_INTERVAL_S) {
        sendTotalPixelIfNeeded();
        sendGeneralCommandNow();
      }
    }
  }

  private void parseFeedback(byte[] data) {
    if (data.length < 3) {
      return;
    }
    feedbackLedCount = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
    feedbackMode = data[2] & 0xFF;
  }

  // --------------------------------------------------------------------------
  // Public getters
  // --------------------------------------------------------------------------
  public boolean getIsESPOnline() {
    return espOnline;
  }

  public int getFeedbackMode() {
    return feedbackMode;
  }

  public int getFeedbackLedCount() {
    return feedbackLedCount;
  }
}
