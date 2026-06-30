package frc.robot.drivers;

import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.hal.can.CANReceiveMessage;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;

import java.nio.charset.StandardCharsets;
import java.util.TimerTask;

/**
 * Battery CAN driver for the battery + LED combo firmware.
 *
 * ESP32 -> RIO:
 *   0x131: Battery Serial (8 bytes)
 *   0x132: Metadata (yy mm dd HH mm cycle note)
 *   0x133: System State (espState, pdType, readerLock, authFail, writeCount)
 *
 * RIO -> ESP32:
 *   0x135: [voltage*10, overrideState, useRIOEnergy, energyMSB, energyLSB, reboot, 0, 0]
 */
public class batteryCAN implements AutoCloseable {
  private static final int API_ESP_SN = 0x131;
  private static final int API_ESP_META = 0x132;
  private static final int API_ESP_STATE = 0x133;
  private static final int API_RIO_CTRL = 0x135;

  private static final double SEND_INTERVAL_S = 0.050;
  private static final long SEND_INTERVAL_MS = 50L;
  private static final double ESP_TIMEOUT_S = 1.0;

  private final CAN can;
  private final int deviceNumber;
  private final int busId;
  private final java.util.Timer updateTimer;

  private String serial = "";
  private int year = 0;
  private int month = 0;
  private int day = 0;
  private int hour = 0;
  private int minute = 0;
  private int cycleCount = 0;
  private int note = 0;
  private int espState = 0;
  private int pdType = 0;
  private boolean readerDetected = false;
  private int authFailCount = 0;
  private int writeCount = 0;
  private boolean valid = false;
  private double lastUpdateSeconds = 0.0;

  private int energyKJ = 0;
  private boolean useRIOEnergy = false;
  private int overrideState = 0;
  private boolean espRebootRequested = false;
  private double lastRebootRequestTimeSeconds = 0.0;

  private boolean espOnline = false;
  private boolean canWriteError = false;

  public batteryCAN(int deviceNumber, int busId) {
    if (deviceNumber < 0 || deviceNumber > 63) {
      throw new IllegalArgumentException("deviceNumber must be 0..63");
    }

    this.deviceNumber = deviceNumber;
    this.busId = busId;
    this.can = new CAN(busId, deviceNumber);
    this.updateTimer = new java.util.Timer("BatteryCANUpdate", true);
    startUpdateTask();
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public int getBusId() {
    return busId;
  }

  public synchronized void setEnergyKJ(int value) {
    energyKJ = clampToUnsignedShort(value);
  }

  public synchronized void setEnergyKJAndSend(int value) {
    energyKJ = clampToUnsignedShort(value);
    useRIOEnergy = true;
    sendControl();
  }

  public synchronized void setUseRIOEnergy(boolean value) {
    useRIOEnergy = value;
  }

  public synchronized void setOverrideState(int state) {
    int newState = clampToByte(state);
    if (overrideState == 0 && newState != 0) {
      System.out.println(
          "[batteryCAN] Dangerous debug override enabled. Prefer an ESP reboot when possible.");
    }
    overrideState = newState;
  }

  public synchronized void requestReboot() {
    double nowSeconds = Timer.getTimestamp();
    if (!espRebootRequested && (nowSeconds - lastRebootRequestTimeSeconds) > 1.0) {
      espRebootRequested = true;
      lastRebootRequestTimeSeconds = nowSeconds;
      System.out.println("[batteryCAN] ESP32 reboot requested.");
    }
  }

  public synchronized String getSerial() {
    return serial;
  }

  public synchronized int getCycleCount() {
    return cycleCount;
  }

  public synchronized int getNote() {
    if (!espOnline || serial.isEmpty()) {
      return -1;
    }
    return note;
  }

  public synchronized int getESPState() {
    return espState;
  }

  public synchronized int getPDType() {
    return pdType;
  }

  public synchronized boolean isReaderDetected() {
    return readerDetected;
  }

  public synchronized int getWriteFailCount() {
    return authFailCount;
  }

  public synchronized int getWriteCount() {
    return writeCount;
  }

  public synchronized boolean isValid() {
    return valid;
  }

  public synchronized double getLastUpdate() {
    return lastUpdateSeconds;
  }

  public synchronized String getFirstUseDateTime() {
    return String.format("%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);
  }

  public synchronized boolean getIsESPOnline() {
    return espOnline;
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
        SEND_INTERVAL_MS);
  }

  private synchronized void update() {
    double nowSeconds = Timer.getTimestamp();
    boolean gotAnyFrame = false;

    CANReceiveMessage frame;

    while ((frame = readNewFrame(API_ESP_SN)) != null) {
      parseSerial(frame);
      gotAnyFrame = true;
    }

    while ((frame = readNewFrame(API_ESP_META)) != null) {
      parseMeta(frame);
      gotAnyFrame = true;
    }

    while ((frame = readNewFrame(API_ESP_STATE)) != null) {
      parseState(frame);
      gotAnyFrame = true;
    }

    if (gotAnyFrame) {
      lastUpdateSeconds = nowSeconds;
    }

    boolean newOnline = (nowSeconds - lastUpdateSeconds) <= ESP_TIMEOUT_S;
    if (newOnline != espOnline) {
      espOnline = newOnline;
      if (espOnline) {
        System.out.println("[batteryCAN] ESP32 reconnected.");
      } else {
        System.out.println("[batteryCAN] ESP32 offline.");
      }
    }

    sendControl();
  }

  private CANReceiveMessage readNewFrame(int apiId) {
    CANReceiveMessage frame = new CANReceiveMessage();
    if (can.readPacketNew(apiId, frame)) {
      return frame;
    }
    return null;
  }

  private void parseSerial(CANReceiveMessage frame) {
    int length = 0;
    while (length < frame.length && frame.data[length] != 0) {
      length++;
    }

    synchronized (this) {
      serial = new String(frame.data, 0, length, StandardCharsets.US_ASCII).trim();
      valid = true;
    }
  }

  private void parseMeta(CANReceiveMessage frame) {
    if (frame.length < 6) {
      return;
    }

    synchronized (this) {
      year = 2000 + (frame.data[0] & 0xFF);
      month = frame.data[1] & 0xFF;
      day = frame.data[2] & 0xFF;
      hour = frame.data[3] & 0xFF;
      minute = frame.data[4] & 0xFF;
      cycleCount = frame.data[5] & 0xFF;
      note = frame.length > 6 ? frame.data[6] & 0xFF : 0;
      valid = true;
    }
  }

  private void parseState(CANReceiveMessage frame) {
    if (frame.length < 3) {
      return;
    }

    synchronized (this) {
      espState = frame.data[0] & 0xFF;
      pdType = frame.data[1] & 0xFF;
      readerDetected = (frame.data[2] & 0xFF) != 0;
      if (frame.length >= 5) {
        authFailCount = ((frame.data[3] & 0xFF) << 8) | (frame.data[4] & 0xFF);
      }
      if (frame.length >= 7) {
        writeCount = ((frame.data[5] & 0xFF) << 8) | (frame.data[6] & 0xFF);
      }
      valid = true;
    }
  }

  private synchronized void sendControl() {
    byte[] payload = new byte[8];
    int voltageTimesTen = clampToByte((int) Math.round(RobotController.getBatteryVoltage() * 10.0));

    payload[0] = (byte) voltageTimesTen;
    payload[1] = (byte) overrideState;
    payload[2] = (byte) (useRIOEnergy ? 1 : 0);
    payload[3] = (byte) ((energyKJ >> 8) & 0xFF);
    payload[4] = (byte) (energyKJ & 0xFF);
    payload[5] = (byte) (espRebootRequested ? 1 : 0);
    payload[6] = 0;
    payload[7] = 0;

    try {
      can.writePacket(API_RIO_CTRL, payload, payload.length, 0);
      if (canWriteError) {
        System.out.println("[batteryCAN] CAN bus recovered.");
        canWriteError = false;
      }
    } catch (Exception e) {
      String message = e.getMessage();
      if (!canWriteError && message != null && message.contains("CAN Output Buffer Full")) {
        System.out.println("[batteryCAN] CAN buffer full while sending control.");
      } else if (!canWriteError) {
        System.out.println(
            "[batteryCAN] Unexpected CAN write exception while sending control: "
                + e.getMessage());
      }
      canWriteError = true;
      return;
    }

    espRebootRequested = false;
  }

  private static int clampToByte(int value) {
    return Math.max(0, Math.min(255, value));
  }

  private static int clampToUnsignedShort(int value) {
    return Math.max(0, Math.min(0xFFFF, value));
  }
}
