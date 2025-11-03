package frc.robot.subsystems;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;

/**
 * BatteryCAN driver for ESP32-based battery tracker.
 *
 * Handles both ESP32→RIO and RIO→ESP32 CAN messages.
 * RIO→ESP32 (0x135) is automatically transmitted every 100ms.
 */
public class batteryCAN {

  // ===================== Constants =====================
  private static final int DEVICE_NUMBER = 33;

  private static final int API_ESP_SN     = 0x131;
  private static final int API_ESP_META   = 0x132;
  private static final int API_ESP_STATE  = 0x133;
  private static final int API_RIO_CTRL   = 0x135;

  private static final double SEND_INTERVAL = 0.1; // seconds (100 ms)

  // ===================== CAN Interface =====================
  private final CAN can;
  public boolean valid = false;

  // ===================== Data from ESP32 =====================
  private String serialNumber = "";
  private int year, month, day, hour, minute;
  private int cycleCount;
  private int note;

  private int espState;
  private int pdType;
  private boolean readerDetected;
  private int authFailCount;
  private int writeCount;

  private double lastUpdate = 0;

  // ===================== Control Parameters =====================
  private double rioVoltage = 12.8;
  private double lastSendTime = 0;

  private int overrideState = 0;
  private boolean useRIOEnergy = false;
  private int energyKJ = 0;
  private boolean espRebootRequested = false;

  // ===================== Constructor =====================
  public batteryCAN() {
    can = new CAN(DEVICE_NUMBER);
    lastSendTime = Timer.getFPGATimestamp();
  }

  // ===================== Periodic Update =====================
  /** Poll CAN bus for new ESP32 packets and auto-send control frame. */
  public void update() {
    CANData data = new CANData();

    // --- 0x131: Battery Serial ---
    if (can.readPacketNew((API_ESP_SN << 6) | (DEVICE_NUMBER & 0x3F), data)) {
      serialNumber = new String(data.data).trim();
      valid = true;
      lastUpdate = Timer.getFPGATimestamp();
    }

    // --- 0x132: Metadata ---
    if (can.readPacketNew((API_ESP_META << 6) | (DEVICE_NUMBER & 0x3F), data)) {
      if (data.length >= 7) {
        year   = 2000 + (data.data[0] & 0xFF);
        month  = data.data[1] & 0xFF;
        day    = data.data[2] & 0xFF;
        hour   = data.data[3] & 0xFF;
        minute = data.data[4] & 0xFF;
        cycleCount = data.data[5] & 0xFF;
        note = data.data[6] & 0xFF;
      }
      valid = true;
      lastUpdate = Timer.getFPGATimestamp();
    }

    // --- 0x133: ESP32 State ---
    if (can.readPacketNew((API_ESP_STATE << 6) | (DEVICE_NUMBER & 0x3F), data)) {
      if (data.length >= 7) {
        espState = data.data[0] & 0xFF;
        pdType = data.data[1] & 0xFF;
        readerDetected = (data.data[2] & 0xFF) != 0;
        authFailCount = ((data.data[3] & 0xFF) << 8) | (data.data[4] & 0xFF);
        writeCount    = ((data.data[5] & 0xFF) << 8) | (data.data[6] & 0xFF);
      }
      valid = true;
      lastUpdate = Timer.getFPGATimestamp();
    }

    // --- Auto-send control frame every 100ms ---
    double now = Timer.getFPGATimestamp();
    if (now - lastSendTime > SEND_INTERVAL) {
      sendControlFrame(rioVoltage);
      lastSendTime = now;
    }
  }

  // ===================== Control Setters =====================

  /** Set measured RIO voltage. */
  public void setVoltage(double voltage) {
    rioVoltage = voltage;
  }

  /** Request ESP32 to use RIO-provided energy (kJ). */
  public void setEnergyKJ(int value) {
    if (value < 0) value = 0;
    energyKJ = value;
    useRIOEnergy = true;
  }

  /** Clear RIO energy override (ESP32 resumes local calc). */
  public void clearEnergyControl() {
    energyKJ = 0;
    useRIOEnergy = false;
  }

  /** Override ESP32 state machine (0 = off). */
  public void overrideState(int state) {
    overrideState = state & 0xFF;
  }

  /** Request ESP32 reboot once. */
  public void requestESPReboot() {
    espRebootRequested = true;
  }

  // ===================== Control Frame Transmit =====================
  private void sendControlFrame(double voltage) {
    byte[] payload = new byte[8];
    int v10 = (int) Math.round(voltage * 10);

    payload[0] = (byte) (v10 & 0xFF);
    payload[1] = (byte) (overrideState & 0xFF);
    payload[2] = (byte) (useRIOEnergy ? 1 : 0);
    payload[3] = (byte) ((energyKJ >> 8) & 0xFF);
    payload[4] = (byte) (energyKJ & 0xFF);
    payload[5] = (byte) (espRebootRequested ? 1 : 0);
    payload[6] = 0;
    payload[7] = 0;

    can.writePacket(payload, (API_RIO_CTRL << 6) | (DEVICE_NUMBER & 0x3F));

    espRebootRequested = false;  // one-shot flag
  }

  // ===================== Getters =====================
  public String getSerial() { return serialNumber; }
  public int getYear() { return year; }
  public int getMonth() { return month; }
  public int getDay() { return day; }
  public int getHour() { return hour; }
  public int getMinute() { return minute; }
  public int getCycleCount() { return cycleCount; }
  public int getNote() { return note; }

  public int getESPState() { return espState; }
  public int getPDType() { return pdType; }
  public boolean isReaderDetected() { return readerDetected; }
  public int getAuthFailCount() { return authFailCount; }
  public int getWriteCount() { return writeCount; }

  public boolean isValid() { return valid; }
  public double getLastUpdate() { return lastUpdate; }

  // Diagnostics for SmartDashboard
  public boolean isUseRIOEnergy() { return useRIOEnergy; }
  public int getOverrideState() { return overrideState; }
  public int getEnergyKJ() { return energyKJ; }
}
