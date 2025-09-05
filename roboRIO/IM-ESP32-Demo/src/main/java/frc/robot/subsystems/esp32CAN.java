package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

/**
 * ESP32 FRC-CAN helper
 *
 * FRC fields are implied by WPILib's CAN class (team manufacturer/type). We
 * keep the same simple ctor style as your addressableLEDCAN: new CAN(deviceNumber).
 *
 * Protocol:
 *  RIO -> ESP32
 *    0x185 : [R,G,B,relay,0,0,0,0]
 *    0x186 : [software_ver, uptime_lo, uptime_hi, 0,0,0,0,0]
 *  ESP32 -> RIO
 *    0x195 : [ain_lo,ain_hi,btnA,btnB,0,0,0,0]
 *    0x196 : [reset_device,0,0,0,0,0,0,0]
 */
public class esp32CAN implements AutoCloseable {
  // API IDs
  public static final int API_TX_CONTROL = 0x185; // R,G,B,relay
  public static final int API_TX_STATUS  = 0x186; // sw ver + uptime
  public static final int API_RX_INPUTS  = 0x195; // analog + buttons
  public static final int API_RX_RESET   = 0x196; // reset flag (from ESP)

  private final CAN can;
  private final int deviceNumber;

  public esp32CAN(int deviceNumber) {
    if (deviceNumber < 0 || deviceNumber > 63) {
      throw new IllegalArgumentException("deviceNumber must be 0..63");
    }
    this.deviceNumber = deviceNumber;
    this.can = new CAN(deviceNumber); // matches your reference style
  }

  public int getDeviceNumber() { return deviceNumber; }

  // ----------------- TX -----------------

  /** Send 0x185: RGB (0..255) + relay (0/1). */
  public void sendRgbRelay(int r, int g, int b, boolean relay) {
    byte[] data = new byte[8];
    data[0] = (byte) (r & 0xFF);
    data[1] = (byte) (g & 0xFF);
    data[2] = (byte) (b & 0xFF);
    data[3] = (byte) (relay ? 1 : 0);
    try {
      can.writePacket(data, API_TX_CONTROL);
    } catch (Exception e) {
      System.err.println("[esp32CAN] sendRgbRelay failed: " + e.getMessage());
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
      can.writePacket(data, API_TX_STATUS);
    } catch (Exception e) {
      System.err.println("[esp32CAN] sendStatus failed: " + e.getMessage());
    }
  }

  /** Optional: ask the ESP32 to reboot (it reboots if it receives 0x196 with data[0]==1). */
  public void requestReset() {
    byte[] data = new byte[8];
    data[0] = 1;
    try {
      can.writePacket(data, API_RX_RESET);
    } catch (Exception e) {
      System.err.println("[esp32CAN] requestReset failed: " + e.getMessage());
    }
  }

  // ----------------- RX -----------------

  /** Read the latest 0x195 (analog & buttons). Returns null if none. */
  public CANData readInputsLatest() {
    CANData d = new CANData();
    if (can.readPacketLatest(API_RX_INPUTS, d)) {
      return d;
    }
    return null;
  }

  /** Read NEW 0x196 (reset flag), once per new frame. Returns null if none. */
  public CANData readResetNew() {
    CANData d = new CANData();
    if (can.readPacketNew(API_RX_RESET, d)) {
      return d;
    }
    return null;
  }

  // ----------------- Parsers -----------------

  /** 0..4095; -1 if invalid. */
  public static int parseAnalogFrom195(CANData d) {
    if (d == null || d.length < 2) return -1;
    int lo = d.data[0] & 0xFF;
    int hi = d.data[1] & 0xFF;
    return (hi << 8) | lo;
  }

  /** true = released (INPUT_PULLUP), false = pressed. */
  public static boolean parseBtnAFrom195(CANData d) {
    if (d == null || d.length < 3) return false;
    return (d.data[2] & 0xFF) != 0;
  }

  /** true = released (INPUT_PULLUP), false = pressed. */
  public static boolean parseBtnBFrom195(CANData d) {
    if (d == null || d.length < 4) return false;
    return (d.data[3] & 0xFF) != 0;
  }

  /** 0 or 1 from 0x196. */
  public static int parseResetFlagFrom196(CANData d) {
    if (d == null || d.length < 1) return 0;
    return d.data[0] & 0xFF;
  }

  @Override
  public void close() {
    // WPILib CAN has no explicit close; kept for symmetry
  }
}
