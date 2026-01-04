package frc.robot.subsystems;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * REV Color Sensor V3 (APDS-9151) ESP32 CAN driver.
 *
 * API IDs:
 * 0x184 ESP -> RIO: RGB + Prox
 * 0x185 ESP -> RIO: IR + config enums
 * 0x186 ESP -> RIO: Gain + online flag
 * 0x187 RIO -> ESP: Reboot + config enums
 */
public class REVColourCAN {

  // === CAN API IDs ===
  private static final int API_COLOR_DATA1 = 0x184;
  private static final int API_COLOR_DATA2 = 0x185;
  private static final int API_COLOR_STATUS = 0x186;
  private static final int API_COLOR_CONFIG = 0x187;

  private static final double SEND_INTERVAL_S = 0.020; // 20ms (50Hz)
  private static final double ESP_TIMEOUT_S = 0.300; // offline if >300ms

  // === CAN Interface ===
  private final CAN can;
  private final CANData rxFrame = new CANData();
  private final Notifier notifier;

  // --- ESP -> RIO data ---
  private volatile int red = 0;
  private volatile int green = 0;
  private volatile int blue = 0;
  private volatile int ir = 0;
  private volatile int proximity = 0;
  private volatile boolean sensorOnlineFlag = false;
  private volatile boolean statusSeen = false;

  private volatile boolean valid = false;
  private volatile double lastUpdate = 0;

  // --- ESP online state ---
  private volatile boolean espOnline = false;
  private volatile boolean lastESPOnline = false;

  // --- CAN write error flag ---
  private boolean canWriteError = false;

  // --- config / reboot ---
  private final Object configLock = new Object();
  private int ledPulseFreq = LEDPulseFreq.LED_FREQ_100k.value;
  private int ledCurrent = LEDCurrent.LED_CURR_50mA.value;
  private int proxResolution = ProxResolution.PROX_RES_11b.value;
  private int proxRate = ProxRate.PROX_RATE_100ms.value;
  private int colorResolution = ColorResolution.COLOR_RES_20b.value;
  private int colorRate = ColorRate.COLOR_RATE_25ms.value;
  private int gainFactor = GainFactor.GAIN_3X.value;
  private boolean configDirty = false;
  private boolean espRebootRequested = false;
  private double lastRebootRequestTime = 0;

  // --------------------------------------------------------------------------
  // Constructor
  // --------------------------------------------------------------------------
  public REVColourCAN(int deviceNumber) {
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

    // ---- RX: 0x184 (RGB + prox) ----
    if (can.readPacketNew(API_COLOR_DATA1, rxFrame)) {
      byte[] d = rxFrame.data;
      if (rxFrame.length >= 8) {
        red = ((d[0] & 0xFF) << 8) | (d[1] & 0xFF);
        green = ((d[2] & 0xFF) << 8) | (d[3] & 0xFF);
        blue = ((d[4] & 0xFF) << 8) | (d[5] & 0xFF);
        proximity = ((d[6] & 0xFF) << 8) | (d[7] & 0xFF);
      }
      gotAnyFrame = true;
      valid = true;
    }

    // ---- RX: 0x185 (IR + enums) ----
    if (can.readPacketNew(API_COLOR_DATA2, rxFrame)) {
      byte[] d = rxFrame.data;
      if (rxFrame.length >= 8) {
        ir = ((d[0] & 0xFF) << 8) | (d[1] & 0xFF);
        synchronized (configLock) {
          ledPulseFreq = d[2] & 0xFF;
          ledCurrent = d[3] & 0xFF;
          proxResolution = d[4] & 0xFF;
          proxRate = d[5] & 0xFF;
          colorResolution = d[6] & 0xFF;
          colorRate = d[7] & 0xFF;
        }
      }
      gotAnyFrame = true;
      valid = true;
    }

    // ---- RX: 0x186 (Gain + online) ----
    if (can.readPacketNew(API_COLOR_STATUS, rxFrame)) {
      byte[] d = rxFrame.data;
      if (rxFrame.length >= 2) {
        gainFactor = d[0] & 0xFF;
        sensorOnlineFlag = d[1] != 0;
        statusSeen = true;
      }
      gotAnyFrame = true;
      valid = true;
    }

    if (gotAnyFrame) {
      lastUpdate = now;
    }

    boolean timedIn = (now - lastUpdate) <= ESP_TIMEOUT_S;
    espOnline = statusSeen ? (timedIn && sensorOnlineFlag) : timedIn;

    if (espOnline != lastESPOnline) {
      if (espOnline) {
        System.out.println("[REVColourCAN] ESP32 Color Sensor reconnected.");
      } else {
        System.out.println("[REVColourCAN] ESP32 Color Sensor offline.");
      }
      lastESPOnline = espOnline;
    }

    if (!espOnline) {
      red = 0;
      green = 0;
      blue = 0;
      ir = 0;
      proximity = 0;
    }

    if (configDirty || espRebootRequested) {
      boolean reboot;
      synchronized (configLock) {
        reboot = espRebootRequested;
      }
      if (sendConfig(reboot)) {
        synchronized (configLock) {
          configDirty = false;
          espRebootRequested = false;
        }
      }
    }
  }

  // --------------------------------------------------------------------------
  // RIO -> ESP: Config/Reboot frame (0x187)
  // --------------------------------------------------------------------------
  private boolean sendConfig(boolean reboot) {
    byte[] payload = new byte[8];
    payload[0] = (byte) (reboot ? 1 : 0);
    synchronized (configLock) {
      payload[1] = (byte) ledPulseFreq;
      payload[2] = (byte) ledCurrent;
      payload[3] = (byte) proxResolution;
      payload[4] = (byte) proxRate;
      payload[5] = (byte) colorResolution;
      payload[6] = (byte) colorRate;
      payload[7] = (byte) gainFactor;
    }

    try {
      can.writePacket(payload, API_COLOR_CONFIG);
      if (canWriteError) {
        System.out.println("[REVColourCAN] CAN TX recovered.");
        canWriteError = false;
      }
      return true;
    } catch (edu.wpi.first.hal.util.UncleanStatusException e) {
      if (!canWriteError && e.getMessage() != null &&
          e.getMessage().contains("CAN Output Buffer Full")) {
        System.out.println("[REVColourCAN] CAN buffer full - likely no ESP32.");
        canWriteError = true;
      }
    } catch (Exception e) {
      if (!canWriteError) {
        System.out.println("[REVColourCAN] Unexpected CAN write exception: " + e.getMessage());
        canWriteError = true;
      }
    }

    return false;
  }

  // --------------------------------------------------------------------------
  // Public Getters
  // --------------------------------------------------------------------------
  public int getRed() { return red; }
  public int getGreen() { return green; }
  public int getBlue() { return blue; }
  public int getIR() { return ir; }
  public int getProximity() { return proximity; }
  public int getGainFactorRaw() { return gainFactor; }

  public int getLEDPulseFreqRaw() { return ledPulseFreq; }
  public int getLEDCurrentRaw() { return ledCurrent; }
  public int getProxResolutionRaw() { return proxResolution; }
  public int getProxRateRaw() { return proxRate; }
  public int getColorResolutionRaw() { return colorResolution; }
  public int getColorRateRaw() { return colorRate; }

  public boolean isValid() { return valid; }
  public boolean getESPState() { return espOnline; }

  // --------------------------------------------------------------------------
  // Config setters
  // --------------------------------------------------------------------------
  public void setConfig(LEDPulseFreq ledPulseFreq,
                        LEDCurrent ledCurrent,
                        ProxResolution proxResolution,
                        ProxRate proxRate,
                        ColorResolution colorResolution,
                        ColorRate colorRate,
                        GainFactor gainFactor) {
    synchronized (configLock) {
      this.ledPulseFreq = ledPulseFreq.value;
      this.ledCurrent = ledCurrent.value;
      this.proxResolution = proxResolution.value;
      this.proxRate = proxRate.value;
      this.colorResolution = colorResolution.value;
      this.colorRate = colorRate.value;
      this.gainFactor = gainFactor.value;
      configDirty = true;
    }
  }

  public void setLEDPulseFreq(LEDPulseFreq value) {
    synchronized (configLock) {
      ledPulseFreq = value.value;
      configDirty = true;
    }
  }

  public void setLEDCurrent(LEDCurrent value) {
    synchronized (configLock) {
      ledCurrent = value.value;
      configDirty = true;
    }
  }

  public void setProxResolution(ProxResolution value) {
    synchronized (configLock) {
      proxResolution = value.value;
      configDirty = true;
    }
  }

  public void setProxRate(ProxRate value) {
    synchronized (configLock) {
      proxRate = value.value;
      configDirty = true;
    }
  }

  public void setColorResolution(ColorResolution value) {
    synchronized (configLock) {
      colorResolution = value.value;
      configDirty = true;
    }
  }

  public void setColorRate(ColorRate value) {
    synchronized (configLock) {
      colorRate = value.value;
      configDirty = true;
    }
  }

  public void setGainFactor(GainFactor value) {
    synchronized (configLock) {
      gainFactor = value.value;
      configDirty = true;
    }
  }

  // --------------------------------------------------------------------------
  // Request a reboot (debounced 1s)
  // --------------------------------------------------------------------------
  public void requestReboot() {
    double now = Timer.getFPGATimestamp();
    synchronized (configLock) {
      if (!espRebootRequested && (now - lastRebootRequestTime) > 1.0) {
        espRebootRequested = true;
        lastRebootRequestTime = now;
        System.out.println("[REVColourCAN] ESP32 Color Sensor reboot requested.");
      }
    }
  }

  // --------------------------------------------------------------------------
  // Enums (matching ESP32 firmware)
  // --------------------------------------------------------------------------
  public enum LEDPulseFreq {
    LED_FREQ_60k(0x18),
    LED_FREQ_70k(0x40),
    LED_FREQ_80k(0x28),
    LED_FREQ_90k(0x30),
    LED_FREQ_100k(0x38);

    public final int value;
    LEDPulseFreq(int value) { this.value = value; }
  }

  public enum LEDCurrent {
    LED_CURR_2mA(0),
    LED_CURR_5mA(1),
    LED_CURR_10mA(2),
    LED_CURR_25mA(3),
    LED_CURR_50mA(4),
    LED_CURR_75mA(5),
    LED_CURR_100mA(6),
    LED_CURR_125mA(7);

    public final int value;
    LEDCurrent(int value) { this.value = value; }
  }

  public enum ProxResolution {
    PROX_RES_8b(0x00),
    PROX_RES_9b(0x08),
    PROX_RES_10b(0x10),
    PROX_RES_11b(0x18);

    public final int value;
    ProxResolution(int value) { this.value = value; }
  }

  public enum ProxRate {
    PROX_RATE_6ms(1),
    PROX_RATE_12ms(2),
    PROX_RATE_25ms(3),
    PROX_RATE_50ms(4),
    PROX_RATE_100ms(5),
    PROX_RATE_200ms(6),
    PROX_RATE_400ms(7);

    public final int value;
    ProxRate(int value) { this.value = value; }
  }

  public enum ColorResolution {
    COLOR_RES_20b(0x00),
    COLOR_RES_19b(0x10),
    COLOR_RES_18b(0x20),
    COLOR_RES_17b(0x30),
    COLOR_RES_16b(0x40),
    COLOR_RES_13b(0x50);

    public final int value;
    ColorResolution(int value) { this.value = value; }
  }

  public enum ColorRate {
    COLOR_RATE_25ms(0),
    COLOR_RATE_50ms(1),
    COLOR_RATE_100ms(2),
    COLOR_RATE_200ms(3),
    COLOR_RATE_500ms(4),
    COLOR_RATE_1000ms(5),
    COLOR_RATE_2000ms(7);

    public final int value;
    ColorRate(int value) { this.value = value; }
  }

  public enum GainFactor {
    GAIN_1X(0),
    GAIN_3X(1),
    GAIN_6X(2),
    GAIN_9X(3),
    GAIN_18X(4);

    public final int value;
    GainFactor(int value) { this.value = value; }
  }
}
