package frc.robot.drivers;

import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.hal.can.CANReceiveMessage;
import org.wpilib.system.Timer;

import java.util.TimerTask;

/**
 * Addressable LED CAN driver for the battery + LED combo firmware.
 *
 * RIO -> ESP32:
 *   0x350: General command (mode, RGB, brightness, on/off, param0, param1)
 *   0x351-0x358: Custom pixel writes (slot indexed)
 *   0x360: Total pixel count + secondary color
 *
 * ESP32 -> RIO:
 *   0x359: Feedback frame (num LEDs, current mode)
 */
public class addressableLEDCAN implements AutoCloseable {
  private static final int API_GENERAL_CMD = 0x350;
  private static final int API_CUSTOM_PIXEL_BASE = 0x351;
  private static final int API_FEEDBACK = 0x359;
  private static final int API_GENERAL_2 = 0x360;

  private static final double ESP_TIMEOUT_S = 1.0;
  private static final double UPDATE_PERIOD_S = 0.050;
  private static final long UPDATE_PERIOD_MS = 50L;

  private final CAN can;
  private final int deviceNumber;
  private final int busId;
  private final java.util.Timer updateTimer;

  private boolean canWriteError = false;

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
  private double lastGeneralSendTimeSeconds = 0.0;

  private final int[] lastPixelIndex = new int[8];
  private final int[] lastPixelR = new int[8];
  private final int[] lastPixelG = new int[8];
  private final int[] lastPixelB = new int[8];
  private final int[] lastPixelW = new int[8];
  private final int[] lastPixelBrightness = new int[8];

  private int feedbackLedCount = -1;
  private int feedbackMode = -1;
  private double lastUpdateSeconds = 0.0;
  private boolean espOnline = false;

  public addressableLEDCAN(int deviceNumber, int busId) {
    if (deviceNumber < 0 || deviceNumber > 63) {
      throw new IllegalArgumentException("deviceNumber must be 0..63");
    }

    this.deviceNumber = deviceNumber;
    this.busId = busId;
    this.can = new CAN(busId, deviceNumber);

    for (int i = 0; i < 8; i++) {
      lastPixelIndex[i] = -1;
      lastPixelR[i] = -1;
      lastPixelG[i] = -1;
      lastPixelB[i] = -1;
      lastPixelW[i] = -1;
      lastPixelBrightness[i] = -1;
    }

    updateTimer = new java.util.Timer("AddressableLEDCANUpdate", true);
    startUpdateTask();
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public int getBusId() {
    return busId;
  }

  public synchronized void sendGeneralCommand(
      int mode, int r, int g, int b, int brightness, int onOff, int param0, int param1) {
    desiredMode = clampToByte(mode);
    desiredR = clampToByte(r);
    desiredG = clampToByte(g);
    desiredB = clampToByte(b);
    desiredBrightness = clampToByte(brightness);
    desiredOnOff = onOff != 0 ? 1 : 0;
    desiredParam0 = clampToByte(param0);
    desiredParam1 = clampToByte(param1);
    hasDesiredGeneral = true;

    boolean changed =
        desiredMode != lastSentMode
            || desiredR != lastSentR
            || desiredG != lastSentG
            || desiredB != lastSentB
            || desiredBrightness != lastSentBrightness
            || desiredOnOff != lastSentOnOff
            || desiredParam0 != lastSentParam0
            || desiredParam1 != lastSentParam1;

    if (changed) {
      sendTotalPixelIfNeeded();
      sendGeneralCommandNow();
    }
  }

  public synchronized void sendPixelWrite(
      int pixelIndex, int r, int g, int b, int w, int brightness, int slot) {
    if (slot < 0 || slot > 7) {
      System.err.println("[addressableLEDCAN] Invalid slot index for pixel write.");
      return;
    }

    int clampedIndex = clampToUnsignedShort(pixelIndex);
    int clampedR = clampToByte(r);
    int clampedG = clampToByte(g);
    int clampedB = clampToByte(b);
    int clampedW = clampToByte(w);
    int clampedBrightness = clampToByte(brightness);

    if (clampedIndex == lastPixelIndex[slot]
        && clampedR == lastPixelR[slot]
        && clampedG == lastPixelG[slot]
        && clampedB == lastPixelB[slot]
        && clampedW == lastPixelW[slot]
        && clampedBrightness == lastPixelBrightness[slot]) {
      return;
    }

    int apiId = API_CUSTOM_PIXEL_BASE + slot;
    byte[] data = new byte[8];
    data[0] = (byte) ((clampedIndex >> 8) & 0xFF);
    data[1] = (byte) (clampedIndex & 0xFF);
    data[2] = (byte) clampedR;
    data[3] = (byte) clampedG;
    data[4] = (byte) clampedB;
    data[5] = (byte) clampedW;
    data[6] = (byte) clampedBrightness;
    data[7] = 0;

    if (writePacket(apiId, data, "pixel")) {
      lastPixelIndex[slot] = clampedIndex;
      lastPixelR[slot] = clampedR;
      lastPixelG[slot] = clampedG;
      lastPixelB[slot] = clampedB;
      lastPixelW[slot] = clampedW;
      lastPixelBrightness[slot] = clampedBrightness;
    }
  }

  public synchronized void setTotalPixel(int count) {
    desiredTotalPixels = clampToUnsignedShort(count);
    sendTotalPixelIfNeeded();
  }

  public synchronized void setSecondaryColor(int enable, int r, int g, int b, int brightness) {
    desiredOnOff2 = enable != 0 ? 1 : 0;
    desiredR2 = clampToByte(r);
    desiredG2 = clampToByte(g);
    desiredB2 = clampToByte(b);
    desiredBrightness2 = clampToByte(brightness);
    sendTotalPixelIfNeeded();
  }

  public synchronized boolean getIsESPOnline() {
    return espOnline;
  }

  public synchronized int getFeedbackMode() {
    return feedbackMode;
  }

  public synchronized int getFeedbackLedCount() {
    return feedbackLedCount;
  }

  @Override
  public void close() {
    updateTimer.cancel();
    can.close();
  }

  private void startUpdateTask() {
    updateTimer.scheduleAtFixedRate(
        new TimerTask() {
          @Override
          public void run() {
            update();
          }
        },
        0,
        UPDATE_PERIOD_MS);
  }

  private synchronized void sendTotalPixelIfNeeded() {
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

    if (writePacket(API_GENERAL_2, data, "general2")) {
      lastSentTotalPixels = desiredTotalPixels;
      lastSentR2 = desiredR2;
      lastSentG2 = desiredG2;
      lastSentB2 = desiredB2;
      lastSentBrightness2 = desiredBrightness2;
      lastSentOnOff2 = desiredOnOff2;
    }
  }

  private synchronized void sendGeneralCommandNow() {
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

    if (writePacket(API_GENERAL_CMD, data, "general")) {
      lastSentMode = desiredMode;
      lastSentR = desiredR;
      lastSentG = desiredG;
      lastSentB = desiredB;
      lastSentBrightness = desiredBrightness;
      lastSentOnOff = desiredOnOff;
      lastSentParam0 = desiredParam0;
      lastSentParam1 = desiredParam1;
      lastGeneralSendTimeSeconds = Timer.getTimestamp();
    }
  }

  private synchronized void update() {
    double nowSeconds = Timer.getTimestamp();
    boolean gotFrame = false;

    CANReceiveMessage frame;
    while ((frame = readFeedbackFrame()) != null) {
      parseFeedback(frame);
      gotFrame = true;
    }

    if (gotFrame) {
      lastUpdateSeconds = nowSeconds;
    }

    boolean newOnline = (nowSeconds - lastUpdateSeconds) <= ESP_TIMEOUT_S;
    if (newOnline != espOnline) {
      espOnline = newOnline;
      if (espOnline) {
        System.out.println("[addressableLEDCAN] ESP32 reconnected.");
      } else {
        System.out.println("[addressableLEDCAN] ESP32 offline.");
      }
    }

    if (gotFrame
        && hasDesiredGeneral
        && feedbackMode >= 0
        && feedbackMode != desiredMode
        && (nowSeconds - lastGeneralSendTimeSeconds) >= UPDATE_PERIOD_S) {
      sendTotalPixelIfNeeded();
      sendGeneralCommandNow();
    }
  }

  private CANReceiveMessage readFeedbackFrame() {
    CANReceiveMessage frame = new CANReceiveMessage();
    if (can.readPacketNew(API_FEEDBACK, frame)) {
      return frame;
    }
    return null;
  }

  private synchronized void parseFeedback(CANReceiveMessage frame) {
    if (frame.length < 3) {
      return;
    }

    feedbackLedCount = ((frame.data[0] & 0xFF) << 8) | (frame.data[1] & 0xFF);
    feedbackMode = frame.data[2] & 0xFF;
  }

  private boolean writePacket(int apiId, byte[] payload, String label) {
    try {
      can.writePacket(apiId, payload, payload.length, 0);
      if (canWriteError) {
        System.out.println("[addressableLEDCAN] CAN bus recovered.");
        canWriteError = false;
      }
      return true;
    } catch (Exception e) {
      String message = e.getMessage();
      if (!canWriteError && message != null && message.contains("CAN Output Buffer Full")) {
        System.out.println("[addressableLEDCAN] CAN buffer full while sending " + label + ".");
      } else if (!canWriteError) {
        System.out.println(
            "[addressableLEDCAN] Unexpected CAN write exception while sending "
                + label
                + ": "
                + e.getMessage());
      }
      canWriteError = true;
      return false;
    }
  }

  private static int clampToByte(int value) {
    return Math.max(0, Math.min(255, value));
  }

  private static int clampToUnsignedShort(int value) {
    return Math.max(0, Math.min(0xFFFF, value));
  }
}
