package frc.robot.subsystems;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * BatteryCAN driver for ESP32-based battery tracker.
 *
 * ESP32 → RIO:
 *   0x131 – Battery Serial (8 bytes)
 *   0x132 – Metadata (yy mm dd HH mm cycle note)
 *   0x133 – System State (espState, pdType, readerLock, authFail, writeCount)
 *
 * RIO → ESP32 (0x135):
 *   [0] = rioVoltage × 10
 *   [1] = overrideState
 *   [2] = useRIOEnergy (flag if energyKJ > 0)
 *   [3] = energyKJ MSB
 *   [4] = energyKJ LSB
 *   [5] = espReboot request
 *   [6-7] = unused
 */
public class batteryCAN {
  private static final int DEFAULT_DEVICE_NUMBER = 33;

  // --- CAN API IDs ---
  private static final int API_ESP_SN    = 0x131;
  private static final int API_ESP_META  = 0x132;
  private static final int API_ESP_STATE = 0x133;
  private static final int API_RIO_CTRL  = 0x135;

  private static final double SEND_INTERVAL_S = 0.050;  // 50 ms
  private static final double ESP_TIMEOUT_S   = 1.000;  // 1000 ms

  private final CAN can;
  private final CANData rxFrame = new CANData();
  private final Notifier notifier;

  // --- ESP → RIO data ---
  private String serial = "";
  private int year, month, day, hour, minute;
  private int cycleCount, note, espState, pdType;
  private boolean readerDetected;
  private int authFailCount, writeCount;
  private boolean valid;
  private double lastUpdate;

  // --- RIO → ESP control data ---
  private int energyKJ = 0;
  private int overrideState = 0;
  private boolean espRebootRequested = false;

  // --- ESP Online Tracking ---
  private boolean espOnline = false;
  private boolean lastESPOnline = false;

  // --- Internal scheduler ---
  private double lastSendTime = 0.0;

  // --------------------------------------------------------------------------
  // Constructor: starts background Notifier
  // --------------------------------------------------------------------------
  public batteryCAN() {
    this(DEFAULT_DEVICE_NUMBER);
  }

  public batteryCAN(int deviceNumber) {
    this.can = new CAN(deviceNumber);
    this.lastSendTime = Timer.getFPGATimestamp();

    // Start periodic CAN handler (50 ms)
    notifier = new Notifier(this::update);
    notifier.startPeriodic(SEND_INTERVAL_S);
  }

  // --------------------------------------------------------------------------
  // Main update loop (RX + TX)
  // --------------------------------------------------------------------------
  private void update() {
    double now = Timer.getFPGATimestamp();
    boolean gotAnyFrame = false;

    // --- RX Frames ---
    if (can.readPacketNew(API_ESP_SN, rxFrame))  { parseSerial(rxFrame.data); gotAnyFrame = true; }
    if (can.readPacketNew(API_ESP_META, rxFrame)) { parseMeta(rxFrame.data); gotAnyFrame = true; }
    if (can.readPacketNew(API_ESP_STATE, rxFrame)) { parseState(rxFrame.data); gotAnyFrame = true; }

    // --- Update online/offline status ---
    if (gotAnyFrame) {
        lastUpdate = now;
    }

    espOnline = (now - lastUpdate) <= ESP_TIMEOUT_S;
    if (espOnline != lastESPOnline) {
        if (espOnline)
            System.out.println("[BatteryCAN] ESP32 reconnected.");
        else
            System.out.println("[BatteryCAN] ESP32 offline.");
        lastESPOnline = espOnline;
    }

    // --- TX ---
    if ((now - lastSendTime) >= SEND_INTERVAL_S) {
        lastSendTime = now;
        sendControl();
    }
}


  // --------------------------------------------------------------------------
  // ESP → RIO parsers
  // --------------------------------------------------------------------------
  private void parseSerial(byte[] d) {
    serial = new String(d).trim();
    valid = true;
  }

  private void parseMeta(byte[] d) {
    if (d.length < 6) return;
    year   = 2000 + (d[0] & 0xFF);
    month  = d[1] & 0xFF;
    day    = d[2] & 0xFF;
    hour   = d[3] & 0xFF;
    minute = d[4] & 0xFF;
    cycleCount = d[5] & 0xFF;
    note   = (d.length > 6) ? (d[6] & 0xFF) : 0;
    valid = true;
  }

  private void parseState(byte[] d) {
    if (d.length < 3) return;
    espState = d[0] & 0xFF;
    pdType   = d[1] & 0xFF;
    readerDetected = (d[2] & 0xFF) != 0;
    if (d.length >= 5)
      authFailCount = ((d[3] & 0xFF) << 8) | (d[4] & 0xFF);
    if (d.length >= 7)
      writeCount = ((d[5] & 0xFF) << 8) | (d[6] & 0xFF);
    valid = true;
  }

  // --------------------------------------------------------------------------
  // RIO → ESP control frame builder
  // --------------------------------------------------------------------------

private boolean canWriteError = false;  // add this field near the top with other state vars

private void sendControl() {
    byte[] payload = new byte[8];
    int v10 = (int)Math.round(RobotController.getBatteryVoltage() * 10.0);
    payload[0] = (byte)(v10 & 0xFF);
    payload[1] = (byte)overrideState;
    payload[2] = (byte)((energyKJ > 0) ? 1 : 0); // useRIOEnergy flag
    payload[3] = (byte)((energyKJ >> 8) & 0xFF); // MSB first
    payload[4] = (byte)(energyKJ & 0xFF);         // LSB
    payload[5] = (byte)(espRebootRequested ? 1 : 0);
    payload[6] = 0;
    payload[7] = 0;

    try {
        can.writePacket(payload, API_RIO_CTRL);

        // Recovery message (print once)
        if (canWriteError) {
            System.out.println("[BatteryCAN] CAN bus recovered.");
            canWriteError = false;
        }

    } catch (edu.wpi.first.hal.util.UncleanStatusException e) {
        // Suppress spam: print once when first seen
        if (!canWriteError && e.getMessage() != null &&
            e.getMessage().contains("CAN Output Buffer Full")) {
            System.out.println("[BatteryCAN] CAN buffer full — no ESP32 disconnected?");
            canWriteError = true;
        }
        return; // skip reboot flag reset
    } catch (Exception e) {
        if (!canWriteError) {
            System.out.println("[BatteryCAN] Unexpected CAN write exception: " + e.getMessage());
            canWriteError = true;
        }
        return;
    }

    espRebootRequested = false; // one-shot
}

  // --------------------------------------------------------------------------
  // Public setters
  // --------------------------------------------------------------------------
  public void setEnergyKJ(int value) {
    energyKJ = Math.max(0, Math.min(65535, value));
  }

  public void setOverrideState(int state) {
    int newState = state & 0xFF;

    // Only print when changing from 0 → non-zero
    if (overrideState == 0 && newState != 0) {
        System.out.println("[BatteryCAN] Dangerous debug override enabled! "
            + "Use ESP reboot if scanning for a new tag or new PD instead.");
    }

    overrideState = newState;
}


private double lastRebootRequestTime = 0;  // add this field near the top with other variables

public void requestReboot() {
    double now = Timer.getFPGATimestamp();

    // Only act if flipping from false → true, and debounce at 1s
    if (!espRebootRequested && (now - lastRebootRequestTime) > 1.0) {
        espRebootRequested = true;
        lastRebootRequestTime = now;
        System.out.println("[BatteryCAN] ESP32 reboot requested.");
    }
}


  // --------------------------------------------------------------------------
  // Public getters
  // --------------------------------------------------------------------------
  public String  getSerial()        { return serial; }
  public int     getCycleCount()    { return cycleCount; }
  public int     getNote()          { return note; }
  public int     getESPState()      { return espState; }
  public int     getPDType()        { return pdType; }
  public boolean isReaderDetected() { return readerDetected; }
  public int     getAuthFailCount() { return authFailCount; }
  public int     getWriteCount()    { return writeCount; }
  public boolean isValid()          { return valid; }
  public double  getLastUpdate()    { return lastUpdate; }

  /** Returns formatted first-use timestamp as "YYYY-MM-DD HH:MM". */
  public String getFirstUseDateTime() {
    return String.format("%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);
  }

  /** Returns true if ESP has sent any CAN frame within the last 1000ms. */
  public boolean getIsESPOnline() {
    return espOnline;
  }
}
