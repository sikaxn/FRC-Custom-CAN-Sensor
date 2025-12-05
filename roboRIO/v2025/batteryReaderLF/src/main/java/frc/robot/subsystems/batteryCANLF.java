package frc.robot.subsystems;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * BatteryCANLF — LF RFID Reader (ESP32) CAN driver.
 *
 * ESP32 → RIO:
 *   0x111 – Serial Number (first 8 bytes)
 *   0x112 – Serial Number (last 6 bytes) + [6]=tagPresent [7]=newTagCount
 *
 * RIO → ESP32:
 *   0x110 – Reboot request
 */
public class batteryCANLF {
  // === CAN API IDs (per FRC Spec) ===
  private static final int API_REQUEST_REBOOT  = 0x110;
  private static final int API_SN_FIRST8       = 0x111;
  private static final int API_SN_LAST6_STATUS = 0x112;

  private static final double SEND_INTERVAL_S = 0.050;  // 50ms loop
  private static final double ESP_TIMEOUT_S   = 1.000;  // offline if >1s

  // === CAN Interfaces ===
  private final CAN can;
  private final CANData rxFrame = new CANData();
  private final Notifier notifier;

  // --- ESP → RIO data ---
  private String serial = "";
  private boolean tagPresent = false;
  private int newTagCount = 0;
  private boolean valid = false;
  private double lastUpdate = 0;

  // --- ESP Online tracking ---
  private boolean espOnline = false;
  private boolean lastESPOnline = false;

  // --- CAN write error flag ---
  private boolean canWriteError = false;

  // --- Reboot debounce ---
  private boolean espRebootRequested = false;
  private double lastRebootRequestTime = 0;

  // --------------------------------------------------------------------------
  // Constructor: start periodic CAN handler (50 ms)
  // --------------------------------------------------------------------------
  public batteryCANLF(int deviceNumber) {
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

    // --- RX ---
    if (can.readPacketNew(API_SN_FIRST8, rxFrame)) {
      parseSerialPart(rxFrame.data, 0);
      gotAnyFrame = true;
    }

    if (can.readPacketNew(API_SN_LAST6_STATUS, rxFrame)) {
      parseSerialPart(rxFrame.data, 8);
      if (rxFrame.length >= 8) {
        tagPresent = (rxFrame.data[6] & 0x01) == 1;
        newTagCount = Byte.toUnsignedInt(rxFrame.data[7]);
      }
      gotAnyFrame = true;
    }

    // --- Online status tracking ---
    if (gotAnyFrame) {
      lastUpdate = now;
    }

    espOnline = (now - lastUpdate) <= ESP_TIMEOUT_S;
    if (espOnline != lastESPOnline) {
      if (espOnline)
        System.out.println("[BatteryCANLF] ESP32 LF reconnected.");
      else
        System.out.println("[BatteryCANLF] ESP32 LF offline.");
      lastESPOnline = espOnline;
    }

    // --- TX: send reboot if requested ---
    if (espRebootRequested) {
      sendReboot();
    }
  }

  // --------------------------------------------------------------------------
  // ESP → RIO parsers
  // --------------------------------------------------------------------------
  private void parseSerialPart(byte[] data, int offset) {
    for (int i = 0; i < data.length && (i + offset) < 14; i++) {
      byte b = data[i];
      if (b != 0) {
        // Grow string dynamically as needed
        if (serial.length() < i + offset + 1) {
          serial = String.format("%-" + (i + offset + 1) + "s", serial);
        }
        char[] chars = serial.toCharArray();
        chars[i + offset] = (char) b;
        serial = new String(chars).trim();
      }
    }
    valid = true;
  }

  // --------------------------------------------------------------------------
  // RIO → ESP Control: Reboot frame (0x110)
  // --------------------------------------------------------------------------
  private void sendReboot() {
    byte[] payload = { 1 };
    try {
      can.writePacket(payload, API_REQUEST_REBOOT);

      // Recovery print (only once)
      if (canWriteError) {
        System.out.println("[BatteryCANLF] CAN TX recovered.");
        canWriteError = false;
      }
    } catch (edu.wpi.first.hal.util.UncleanStatusException e) {
      if (!canWriteError && e.getMessage() != null &&
          e.getMessage().contains("CAN Output Buffer Full")) {
        System.out.println("[BatteryCANLF] CAN buffer full — likely no ESP32.");
        canWriteError = true;
      }
    } catch (Exception e) {
      if (!canWriteError) {
        System.out.println("[BatteryCANLF] Unexpected CAN write exception: " + e.getMessage());
        canWriteError = true;
      }
    }

    espRebootRequested = false; // one-shot
  }

  // --------------------------------------------------------------------------
  // Public methods (for Robot.java)
  // --------------------------------------------------------------------------
  public String getSerial() {
    // return last known serial — even if offline
    return serial != null ? serial.trim() : "";
  }
  

  public boolean getTagPresent() {
    return tagPresent;
  }

  public int getNewTagCount() {
    return newTagCount;
  }

  public boolean getESPState() {
    return espOnline;
  }

  public boolean isValid() {
    return valid;
  }

  /** Request ESP reboot, debounced to once per second. */
  public void requestReboot() {
    double now = Timer.getFPGATimestamp();
    if (!espRebootRequested && (now - lastRebootRequestTime) > 1.0) {
      espRebootRequested = true;
      lastRebootRequestTime = now;
      System.out.println("[BatteryCANLF] ESP32 LF reboot requested.");
    }
  }
}
