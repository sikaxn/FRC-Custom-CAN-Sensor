package frc.robot.subsystems;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * amColourCAN — ESP32 Colour Sensor CAN Driver (LF-style).
 *
 * ESP32 → RIO:
 *   0x194 – RGB + Proximity
 *   0x195 – Clear + sensorGood
 *
 * RIO → ESP32:
 *   0x197 – Reboot request
 */
public class amColourCAN {

  // === CAN API IDs ===
  private static final int API_COLOUR_FRAME1 = 0x194; // RGB + prox
  private static final int API_COLOUR_FRAME2 = 0x195; // clear + good
  private static final int API_REQUEST_REBOOT = 0x197;

  private static final double SEND_INTERVAL_S = 0.020; // 20ms (50Hz)
  private static final double ESP_TIMEOUT_S   = 0.300; // offline if >300ms

  // === CAN Interface (LF-style: ONE CAN object) ===
  private final CAN can;
  private final CANData rxFrame = new CANData();
  private final Notifier notifier;

  // --- ESP → RIO data ---
  private volatile int clear = 0;
  private volatile int red = 0;
  private volatile int green = 0;
  private volatile int blue = 0;
  private volatile int proximity = 0;
  private volatile boolean sensorGood = false;

  private volatile boolean valid = false;
  private volatile double lastUpdate = 0;

  // --- ESP online state ---
  private volatile boolean espOnline = false;
  private volatile boolean lastESPOnline = false;

  // --- CAN write error flag ---
  private boolean canWriteError = false;

  // --- reboot debounce ---
  private boolean espRebootRequested = false;
  private double lastRebootRequestTime = 0;

  // --------------------------------------------------------------------------
  // Constructor
  // --------------------------------------------------------------------------
  public amColourCAN(int deviceNumber) {
    this.can = new CAN(deviceNumber);
    notifier = new Notifier(this::update);
    notifier.startPeriodic(SEND_INTERVAL_S);
  }

  // --------------------------------------------------------------------------
  // Main update loop (RX + TX)
  // --------------------------------------------------------------------------
  private void update() {
    double now = Timer.getFPGATimestamp();
    boolean gotAnyFrame = false;

    // ---- RX: 0x194 (RGB + prox) ----
    if (can.readPacketNew(API_COLOUR_FRAME1, rxFrame)) {
      byte[] d = rxFrame.data;

      if (rxFrame.length >= 8) {
        red       = ((d[0] & 0xFF) << 8) | (d[1] & 0xFF);
        green     = ((d[2] & 0xFF) << 8) | (d[3] & 0xFF);
        blue      = ((d[4] & 0xFF) << 8) | (d[5] & 0xFF);
        proximity = ((d[6] & 0xFF) << 8) | (d[7] & 0xFF);
      }

      gotAnyFrame = true;
      valid = true;
    }

    // ---- RX: 0x195 (clear + good) ----
    if (can.readPacketNew(API_COLOUR_FRAME2, rxFrame)) {
      byte[] d = rxFrame.data;

      if (rxFrame.length >= 3) {
        clear = ((d[0] & 0xFF) << 8) | (d[1] & 0xFF);
        sensorGood = (d[2] != 0);
      }

      gotAnyFrame = true;
      valid = true;
    }

    // Track last update time
    if (gotAnyFrame) {
      lastUpdate = now;
    }

    // ---- Online/offline tracking ----
    espOnline = (now - lastUpdate) <= ESP_TIMEOUT_S;

    if (espOnline != lastESPOnline) {
      if (espOnline)
        System.out.println("[amColourCAN] ESP32 Colour Sensor reconnected.");
      else
        System.out.println("[amColourCAN] ESP32 Colour Sensor offline.");
      lastESPOnline = espOnline;
    }

    // ---- TX: reboot (one-shot) ----
    if (espRebootRequested) {
      sendReboot();
    }
  }

  // --------------------------------------------------------------------------
  // RIO → ESP: Reboot frame (0x197)
  // --------------------------------------------------------------------------
  private void sendReboot() {
    byte[] payload = { 1 };

    try {
      can.writePacket(payload, API_REQUEST_REBOOT);

      if (canWriteError) {
        System.out.println("[amColourCAN] CAN TX recovered.");
        canWriteError = false;
      }

    } catch (edu.wpi.first.hal.util.UncleanStatusException e) {
      if (!canWriteError && e.getMessage() != null &&
          e.getMessage().contains("CAN Output Buffer Full")) {
        System.out.println("[amColourCAN] CAN buffer full — likely no ESP32.");
        canWriteError = true;
      }

    } catch (Exception e) {
      if (!canWriteError) {
        System.out.println("[amColourCAN] Unexpected CAN write exception: " + e.getMessage());
        canWriteError = true;
      }
    }

    espRebootRequested = false; // only send once
  }

  // --------------------------------------------------------------------------
  // Public Getters
  // --------------------------------------------------------------------------
  public int getClear() { return clear; }
  public int getRed() { return red; }
  public int getGreen() { return green; }
  public int getBlue() { return blue; }
  public int getProximity() { return proximity; }
  public boolean getStatus() { return sensorGood; }

  public boolean isValid() { return valid; }
  public boolean getESPState() { return espOnline; }

  // --------------------------------------------------------------------------
  // Request a reboot (debounced 1s)
  // --------------------------------------------------------------------------
  public void requestReboot() {
    double now = Timer.getFPGATimestamp();
    if (!espRebootRequested && (now - lastRebootRequestTime) > 1.0) {
      espRebootRequested = true;
      lastRebootRequestTime = now;
      System.out.println("[amColourCAN] ESP32 Colour Sensor reboot requested.");
    }
  }
}
