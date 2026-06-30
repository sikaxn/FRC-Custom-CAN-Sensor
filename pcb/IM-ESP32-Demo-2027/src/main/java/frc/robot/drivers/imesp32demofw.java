package frc.robot.drivers;

import java.util.Timer;
import java.util.TimerTask;

import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.hal.can.CANReceiveMessage;

/**
 * ESP32 FRC-CAN helper
 *
 * FRC fields are implied by WPILib's CAN class (team manufacturer/type). We
 * keep the same simple ctor style as your addressableLEDCAN: new CAN(busId, deviceNumber).
 *
 * Protocol:
 *  RIO -> ESP32
 *    0x185 : [R,G,B,relay,0,0,0,0]
 *    0x186 : [software_ver, uptime_lo, uptime_hi, 0,0,0,0,0]
 *  ESP32 -> RIO
 *    0x195 : [ain_lo,ain_hi,btnA,btnB,0,0,0,0]
 *    0x196 : [reset_device,0,0,0,0,0,0,0]
 */
public class imesp32demofw implements AutoCloseable {
  private static final int OUTPUT_TASK_PERIOD_MS = 20;
  private static final long ONLINE_TX_PERIOD_MS = 20L;
  private static final long OFFLINE_TX_PERIOD_MS = 250L;
  private static final long MANUAL_KEEPALIVE_PERIOD_MS = 250L;
  private static final double DEFAULT_RAINBOW_PERIOD_S = 3.0;
  private static final double ESP_TIMEOUT_S = 1.0;

  // API IDs
  public static final int API_TX_CONTROL = 0x185; // R,G,B,relay
  public static final int API_TX_STATUS  = 0x186; // sw ver + uptime
  public static final int API_RX_INPUTS  = 0x195; // analog + buttons
  public static final int API_RX_RESET   = 0x196; // reset flag (from ESP)

  private final CAN can;
  private final int deviceNumber;
  private final int busId;
  private final Timer outputTimer;
  private final Object outputLock = new Object();
  private int manualR = 0;
  private int manualG = 0;
  private int manualB = 0;
  private boolean manualRelay = false;
  private boolean rainbowEnabled = false;
  private double rainbowPeriodSeconds = DEFAULT_RAINBOW_PERIOD_S;
  private long rainbowStartTimeNanos = System.nanoTime();
  private int currentOutputR = 0;
  private int currentOutputG = 0;
  private int currentOutputB = 0;
  private int lastSentR = -1;
  private int lastSentG = -1;
  private int lastSentB = -1;
  private boolean lastSentRelay = false;
  private long lastTxAttemptTimeMs = 0L;
  private long lastTxSuccessTimeMs = 0L;
  private boolean canWriteError = false;
  private int analogRaw = -1;
  private boolean buttonAReleased = true;
  private boolean buttonBReleased = true;
  private double inputsLastUpdateS = 0.0;
  private double lastAnyRxUpdateS = 0.0;
  private boolean hasSeenAnyRxFrame = false;
  private boolean espOnline = false;
  private int lastResetFlag = 0;

  public imesp32demofw(int deviceNumber, int busId) {
    if (deviceNumber < 0 || deviceNumber > 63) {
      throw new IllegalArgumentException("deviceNumber must be 0..63");
    }
    this.deviceNumber = deviceNumber;
    this.busId = busId;
    this.can = new CAN(busId, deviceNumber);
    this.outputTimer = new Timer("IMESP32DemoFWTx", true);
    startOutputTask();
  }

  public int getDeviceNumber() { return deviceNumber; }
  public int getBusId() { return busId; }

  // ----------------- TX -----------------

  public void setManualOutputs(int r, int g, int b, boolean relay) {
    synchronized (outputLock) {
      manualR = clampToByte(r);
      manualG = clampToByte(g);
      manualB = clampToByte(b);
      manualRelay = relay;
    }
  }

  public void setRainbowEnabled(boolean enabled) {
    synchronized (outputLock) {
      if (enabled && !rainbowEnabled) {
        rainbowStartTimeNanos = System.nanoTime();
      }
      rainbowEnabled = enabled;
    }
  }

  public void setRainbowPeriodSeconds(double periodSeconds) {
    synchronized (outputLock) {
      rainbowPeriodSeconds = Math.max(0.1, periodSeconds);
    }
  }

  public int getCurrentOutputR() {
    synchronized (outputLock) {
      return currentOutputR;
    }
  }

  public int getCurrentOutputG() {
    synchronized (outputLock) {
      return currentOutputG;
    }
  }

  public int getCurrentOutputB() {
    synchronized (outputLock) {
      return currentOutputB;
    }
  }

  /** Send 0x185: RGB (0..255) + relay (0/1). */
  public boolean sendRgbRelay(int r, int g, int b, boolean relay) {
    byte[] data = new byte[8];
    data[0] = (byte) (r & 0xFF);
    data[1] = (byte) (g & 0xFF);
    data[2] = (byte) (b & 0xFF);
    data[3] = (byte) (relay ? 1 : 0);
    try {
      can.writePacket(API_TX_CONTROL, data, data.length, 0);
      if (canWriteError) {
        System.out.println("[imesp32demofw] CAN bus recovered.");
        canWriteError = false;
      }
      return true;
    } catch (Exception e) {
      String message = e.getMessage();
      if (!canWriteError && message != null && message.contains("Socket Buffer full")) {
        System.out.println("[imesp32demofw] CAN socket buffer full while sending RGB.");
      } else if (!canWriteError) {
        System.out.println("[imesp32demofw] sendRgbRelay failed: " + e.getMessage());
      }
      canWriteError = true;
      return false;
    }
  }

  /** Send 0x186: software version (0..255) + uptime seconds (16-bit LE, saturating). */
  public void sendStatus(int softwareVer, int uptimeSeconds) {
    int up = Math.max(0, Math.min(0xFFFF, uptimeSeconds));
    byte[] data = new byte[8];
    data[0] = (byte) (softwareVer & 0xFF);
    data[1] = (byte) (up & 0xFF);         // lo
    data[2] = (byte) ((up >>> 8) & 0xFF); // hi
    try {
      can.writePacket(API_TX_STATUS, data, data.length, 0);
    } catch (Exception e) {
      System.err.println("[imesp32demofw] sendStatus failed: " + e.getMessage());
    }
  }

  /** Optional: ask the ESP32 to reboot (it reboots if it receives 0x196 with data[0]==1). */
  public void requestReset() {
    byte[] data = new byte[8];
    data[0] = 1;
    try {
      can.writePacket(API_RX_RESET, data, data.length, 0);
    } catch (Exception e) {
      System.err.println("[imesp32demofw] requestReset failed: " + e.getMessage());
    }
  }

  // ----------------- RX -----------------

  /** Poll and cache the latest input/reset frames. */
  public void poll(double timestampSeconds) {
    boolean gotAnyFrame = false;

    CANReceiveMessage inputsFrame;
    while ((inputsFrame = readInputsNewFrame()) != null) {
      if (inputsFrame.length >= 4) {
        analogRaw = parseAnalogFrom195(inputsFrame);
        buttonAReleased = parseBtnAFrom195(inputsFrame);
        buttonBReleased = parseBtnBFrom195(inputsFrame);
        inputsLastUpdateS = timestampSeconds;
        hasSeenAnyRxFrame = true;
        gotAnyFrame = true;
      }
    }

    CANReceiveMessage resetFrame;
    while ((resetFrame = readResetNewFrame()) != null) {
      lastResetFlag = parseResetFlagFrom196(resetFrame);
      hasSeenAnyRxFrame = true;
      gotAnyFrame = true;
    }

    if (gotAnyFrame) {
      lastAnyRxUpdateS = timestampSeconds;
    }

    boolean newOnline = hasSeenAnyRxFrame && (timestampSeconds - lastAnyRxUpdateS) <= ESP_TIMEOUT_S;
    if (newOnline != espOnline) {
      espOnline = newOnline;
      if (espOnline) {
        System.out.println("[imesp32demofw] ESP32 reconnected.");
      } else {
        System.out.println("[imesp32demofw] ESP32 offline.");
      }
    }
  }

  public int getAnalogRaw() { return analogRaw; }

  public boolean isButtonAReleased() { return buttonAReleased; }

  public boolean isButtonBReleased() { return buttonBReleased; }

  public double getInputsAgeMs(double timestampSeconds) {
    return (timestampSeconds - inputsLastUpdateS) * 1000.0;
  }

  public boolean isInputsStale(double timestampSeconds, double staleThresholdMs) {
    return getInputsAgeMs(timestampSeconds) > staleThresholdMs;
  }

  public boolean getIsESPOnline() { return espOnline; }

  public int getLastResetFlag() { return lastResetFlag; }

  /** Read NEW 0x195 (analog & buttons), once per new frame. Returns null if none. */
  private CANReceiveMessage readInputsNewFrame() {
    CANReceiveMessage d = new CANReceiveMessage();
    if (can.readPacketNew(API_RX_INPUTS, d)) {
      return d;
    }
    return null;
  }

  /** Read NEW 0x196 (reset flag), once per new frame. Returns null if none. */
  private CANReceiveMessage readResetNewFrame() {
    CANReceiveMessage d = new CANReceiveMessage();
    if (can.readPacketNew(API_RX_RESET, d)) {
      return d;
    }
    return null;
  }

  // ----------------- Parsers -----------------

  /** 0..4095; -1 if invalid. */
  private static int parseAnalogFrom195(CANReceiveMessage d) {
    if (d == null || d.length < 2) return -1;
    int lo = d.data[0] & 0xFF;
    int hi = d.data[1] & 0xFF;
    return (hi << 8) | lo;
  }

  /** true = released (INPUT_PULLUP), false = pressed. */
  private static boolean parseBtnAFrom195(CANReceiveMessage d) {
    if (d == null || d.length < 3) return false;
    return (d.data[2] & 0xFF) != 0;
  }

  /** true = released (INPUT_PULLUP), false = pressed. */
  private static boolean parseBtnBFrom195(CANReceiveMessage d) {
    if (d == null || d.length < 4) return false;
    return (d.data[3] & 0xFF) != 0;
  }

  /** 0 or 1 from 0x196. */
  private static int parseResetFlagFrom196(CANReceiveMessage d) {
    if (d == null || d.length < 1) return 0;
    return d.data[0] & 0xFF;
  }

  @Override
  public void close() {
    outputTimer.cancel();
    can.close();
  }

  private void startOutputTask() {
    outputTimer.scheduleAtFixedRate(
        new TimerTask() {
          @Override
          public void run() {
            int r;
            int g;
            int b;
            boolean relay;
            boolean rainbowActive;

            synchronized (outputLock) {
              relay = manualRelay;
              rainbowActive = rainbowEnabled;

              if (rainbowActive) {
                int[] rgb =
                    getRainbowRgb(
                        (System.nanoTime() - rainbowStartTimeNanos) * 1.0e-9, rainbowPeriodSeconds);
                r = rgb[0];
                g = rgb[1];
                b = rgb[2];
              } else {
                r = manualR;
                g = manualG;
                b = manualB;
              }

              currentOutputR = r;
              currentOutputG = g;
              currentOutputB = b;
            }

            long nowMs = System.currentTimeMillis();
            boolean changed =
                r != lastSentR || g != lastSentG || b != lastSentB || relay != lastSentRelay;
            long minIntervalMs = espOnline ? ONLINE_TX_PERIOD_MS : OFFLINE_TX_PERIOD_MS;
            boolean periodicRetryDue = (nowMs - lastTxAttemptTimeMs) >= minIntervalMs;
            boolean manualKeepaliveDue =
                !rainbowActive && (nowMs - lastTxSuccessTimeMs) >= MANUAL_KEEPALIVE_PERIOD_MS;
            boolean shouldSend =
                (changed && periodicRetryDue)
                    || (rainbowActive && periodicRetryDue)
                    || (manualKeepaliveDue && periodicRetryDue);

            if (!shouldSend) {
              return;
            }

            lastTxAttemptTimeMs = nowMs;
            if (sendRgbRelay(r, g, b, relay)) {
              lastSentR = r;
              lastSentG = g;
              lastSentB = b;
              lastSentRelay = relay;
              lastTxSuccessTimeMs = nowMs;
            }
          }
        },
        0,
        OUTPUT_TASK_PERIOD_MS);
  }

  private static int clampToByte(int value) {
    return Math.max(0, Math.min(255, value));
  }

  private static int[] getRainbowRgb(double elapsedSeconds, double periodSeconds) {
    double wrapped = (elapsedSeconds / periodSeconds) % 1.0;
    double hue = wrapped * 6.0;
    double x = 1.0 - Math.abs((hue % 2.0) - 1.0);

    double r;
    double g;
    double b;

    if (hue < 1.0) {
      r = 1.0;
      g = x;
      b = 0.0;
    } else if (hue < 2.0) {
      r = x;
      g = 1.0;
      b = 0.0;
    } else if (hue < 3.0) {
      r = 0.0;
      g = 1.0;
      b = x;
    } else if (hue < 4.0) {
      r = 0.0;
      g = x;
      b = 1.0;
    } else if (hue < 5.0) {
      r = x;
      g = 0.0;
      b = 1.0;
    } else {
      r = 1.0;
      g = 0.0;
      b = x;
    }

    return new int[] {(int) (r * 255.0), (int) (g * 255.0), (int) (b * 255.0)};
  }
}
