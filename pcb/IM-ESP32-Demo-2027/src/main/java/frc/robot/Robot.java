package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.concurrent.ThreadLocalRandom;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

import frc.robot.drivers.addressableLEDCAN;
// import frc.robot.drivers.batteryCAN;
import frc.robot.drivers.imesp32demofw;

public class Robot extends TimedRobot {
  private static final int SYSTEMCORE_CAN_BUS = 1;
  private static final double ESP_INPUTS_STALE_MS = 500.0;

  private static final int LEDS_DN = 33;
  private addressableLEDCAN leds;
  private int lastMode = -1;
  private int lastR = -1;
  private int lastG = -1;
  private int lastB = -1;
  private int lastBrightness = -1;
  private int lastOnOff = -1;
  private int lastParam0 = -1;
  private int lastParam1 = -1;
  private boolean lastWritePixel = false;

  // ESP32 demo firmware controller
  private static final int ESP32_DN = 9;
  private imesp32demofw esp32;
  private static final int SPARK_MAX_DN = 50;
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final double DRIVE_AXIS_DEADBAND = 0.05;
  private SparkMax sparkMax;
  private Gamepad driverController;

  // Battery CAN is disabled until the hardware is connected.
  /*
  private static final int BATTERY_DN = 33;
  private batteryCAN batteryCan;
  */

  // ADC calibration (V ~= k0 + k1*adc + k2*adc^2)
  private static final double ADC_K0 = 0.915434204;
  private static final double ADC_K1 = 0.008560656043;
  private static final double ADC_K2 = -0.000000358496267;
  private static final double ESP_RAINBOW_PERIOD_S = 3.0;

  private boolean prevBtnAReleased = true;
  private boolean prevBtnBReleased = true;
  private int radioLEDState = 0;
  private boolean prevUserButton = false;

  private NetworkTableEntry userButtonEntry;

  private NetworkTableEntry ledModeEntry;
  private NetworkTableEntry ledREntry;
  private NetworkTableEntry ledGEntry;
  private NetworkTableEntry ledBEntry;
  private NetworkTableEntry ledBrightnessEntry;
  private NetworkTableEntry ledOnOffEntry;
  private NetworkTableEntry ledParam0Entry;
  private NetworkTableEntry ledParam1Entry;
  private NetworkTableEntry ledTotalPixelsEntry;
  private NetworkTableEntry ledSecondaryREntry;
  private NetworkTableEntry ledSecondaryGEntry;
  private NetworkTableEntry ledSecondaryBEntry;
  private NetworkTableEntry ledSecondaryBrightnessEntry;
  private NetworkTableEntry ledSecondaryEnableEntry;
  private NetworkTableEntry ledPixelIndexEntry;
  private NetworkTableEntry ledPixelREntry;
  private NetworkTableEntry ledPixelGEntry;
  private NetworkTableEntry ledPixelBEntry;
  private NetworkTableEntry ledPixelBrightnessEntry;
  private NetworkTableEntry ledWritePixelEntry;
  private NetworkTableEntry ledFeedbackModeEntry;
  private NetworkTableEntry ledFeedbackLedCountEntry;
  private NetworkTableEntry ledOnlineEntry;

  private NetworkTableEntry espREntry;
  private NetworkTableEntry espGEntry;
  private NetworkTableEntry espBEntry;
  private NetworkTableEntry espRainbowEntry;
  private NetworkTableEntry espRelayEntry;
  private NetworkTableEntry espRequestResetEntry;
  private NetworkTableEntry espAnalogEntry;
  private NetworkTableEntry espVoltageEntry;
  private NetworkTableEntry espButtonAEntry;
  private NetworkTableEntry espButtonBEntry;
  private NetworkTableEntry espResetFlagSeenEntry;
  private NetworkTableEntry espInputsAgeMsEntry;
  private NetworkTableEntry espInputsStaleEntry;
  private NetworkTableEntry espOnlineEntry;
  private NetworkTableEntry espRadioLEDStateEntry;
  private NetworkTableEntry espCanBusIdEntry;

  /*
  private NetworkTableEntry batteryValidEntry;
  private NetworkTableEntry batterySnEntry;
  private NetworkTableEntry batteryFirstUseEntry;
  private NetworkTableEntry batteryNoteEntry;
  private NetworkTableEntry batteryCycleCountEntry;
  private NetworkTableEntry batteryTimeVoltagePayloadEntry;
  private NetworkTableEntry batteryEnergyPayloadEntry;
  */

  public Robot() {
    leds = new addressableLEDCAN(LEDS_DN, SYSTEMCORE_CAN_BUS);
    esp32 = new imesp32demofw(ESP32_DN, SYSTEMCORE_CAN_BUS);
    sparkMax = new SparkMax(SYSTEMCORE_CAN_BUS, SPARK_MAX_DN, MotorType.kBrushed);
    driverController = new Gamepad(DRIVER_CONTROLLER_PORT);
    esp32.setRainbowPeriodSeconds(ESP_RAINBOW_PERIOD_S);
    /*
    batteryCan = new batteryCAN(BATTERY_DN, SYSTEMCORE_CAN_BUS);
    */

    NetworkTable root = NetworkTableInstance.getDefault().getTable("IMDemo");
    NetworkTable controlsTable = root.getSubTable("Controls");
    NetworkTable ledTable = root.getSubTable("LED");
    NetworkTable espTable = root.getSubTable("ESP");
    /*
    NetworkTable batteryTable = root.getSubTable("Battery");
    NetworkTable debugTable = root.getSubTable("Debug");
    */

    userButtonEntry = controlsTable.getEntry("UserButton");
    userButtonEntry.setDefaultBoolean(false);

    ledModeEntry = ledTable.getEntry("Mode");
    ledREntry = ledTable.getEntry("R");
    ledGEntry = ledTable.getEntry("G");
    ledBEntry = ledTable.getEntry("B");
    ledBrightnessEntry = ledTable.getEntry("Brightness");
    ledOnOffEntry = ledTable.getEntry("OnOff");
    ledParam0Entry = ledTable.getEntry("Param0");
    ledParam1Entry = ledTable.getEntry("Param1");
    ledTotalPixelsEntry = ledTable.getEntry("TotalPixels");
    ledSecondaryREntry = ledTable.getEntry("SecondaryR");
    ledSecondaryGEntry = ledTable.getEntry("SecondaryG");
    ledSecondaryBEntry = ledTable.getEntry("SecondaryB");
    ledSecondaryBrightnessEntry = ledTable.getEntry("SecondaryBrightness");
    ledSecondaryEnableEntry = ledTable.getEntry("SecondaryEnable");
    ledPixelIndexEntry = ledTable.getEntry("PixelIndex");
    ledPixelREntry = ledTable.getEntry("PixelR");
    ledPixelGEntry = ledTable.getEntry("PixelG");
    ledPixelBEntry = ledTable.getEntry("PixelB");
    ledPixelBrightnessEntry = ledTable.getEntry("PixelBrightness");
    ledWritePixelEntry = ledTable.getEntry("WritePixel");
    ledFeedbackModeEntry = ledTable.getEntry("FeedbackMode");
    ledFeedbackLedCountEntry = ledTable.getEntry("FeedbackLedCount");
    ledOnlineEntry = ledTable.getEntry("Online");

    ledModeEntry.setDefaultDouble(1);
    ledREntry.setDefaultDouble(255);
    ledGEntry.setDefaultDouble(255);
    ledBEntry.setDefaultDouble(255);
    ledBrightnessEntry.setDefaultDouble(128);
    ledOnOffEntry.setDefaultDouble(1);
    ledParam0Entry.setDefaultDouble(20);
    ledParam1Entry.setDefaultDouble(20);
    ledTotalPixelsEntry.setDefaultDouble(150);
    ledSecondaryREntry.setDefaultDouble(0);
    ledSecondaryGEntry.setDefaultDouble(0);
    ledSecondaryBEntry.setDefaultDouble(0);
    ledSecondaryBrightnessEntry.setDefaultDouble(0);
    ledSecondaryEnableEntry.setDefaultBoolean(false);
    ledPixelIndexEntry.setDefaultDouble(0);
    ledPixelREntry.setDefaultDouble(255);
    ledPixelGEntry.setDefaultDouble(0);
    ledPixelBEntry.setDefaultDouble(0);
    ledPixelBrightnessEntry.setDefaultDouble(128);
    ledWritePixelEntry.setDefaultBoolean(false);
    ledFeedbackModeEntry.setDouble(-1);
    ledFeedbackLedCountEntry.setDouble(-1);
    ledOnlineEntry.setBoolean(false);

    espREntry = espTable.getEntry("R");
    espGEntry = espTable.getEntry("G");
    espBEntry = espTable.getEntry("B");
    espRainbowEntry = espTable.getEntry("Rainbow");
    espRelayEntry = espTable.getEntry("Relay");
    espRequestResetEntry = espTable.getEntry("RequestReset");
    espAnalogEntry = espTable.getEntry("Analog");
    espVoltageEntry = espTable.getEntry("Voltage");
    espButtonAEntry = espTable.getEntry("ButtonA");
    espButtonBEntry = espTable.getEntry("ButtonB");
    espResetFlagSeenEntry = espTable.getEntry("ResetFlagSeen");
    espInputsAgeMsEntry = espTable.getEntry("InputsAgeMs");
    espInputsStaleEntry = espTable.getEntry("InputsStale");
    espOnlineEntry = espTable.getEntry("Online");
    espRadioLEDStateEntry = espTable.getEntry("RadioLEDState");
    espCanBusIdEntry = espTable.getEntry("CANBusId");

    espREntry.setDefaultDouble(0);
    espGEntry.setDefaultDouble(0);
    espBEntry.setDefaultDouble(0);
    espRainbowEntry.setDefaultBoolean(false);
    espRelayEntry.setDefaultBoolean(false);
    espRequestResetEntry.setDefaultBoolean(false);
    espAnalogEntry.setDouble(-1);
    espVoltageEntry.setDouble(-1);
    espButtonAEntry.setString("Unknown");
    espButtonBEntry.setString("Unknown");
    espResetFlagSeenEntry.setDouble(0);
    espInputsAgeMsEntry.setDouble(-1);
    espInputsStaleEntry.setBoolean(true);
    espOnlineEntry.setBoolean(false);
    espRadioLEDStateEntry.setDouble(radioLEDState);
    espCanBusIdEntry.setInteger(SYSTEMCORE_CAN_BUS);

    /*
    batteryValidEntry = batteryTable.getEntry("Valid");
    batterySnEntry = batteryTable.getEntry("SN");
    batteryFirstUseEntry = batteryTable.getEntry("FirstUseUtc");
    batteryNoteEntry = batteryTable.getEntry("Note");
    batteryCycleCountEntry = batteryTable.getEntry("CycleCount");
    batteryTimeVoltagePayloadEntry = debugTable.getEntry("BatteryTimeVoltagePayload");
    batteryEnergyPayloadEntry = debugTable.getEntry("BatteryEnergyPayload");

    batteryValidEntry.setBoolean(false);
    batterySnEntry.setString("INVALID");
    batteryFirstUseEntry.setString("0000-00-00");
    batteryNoteEntry.setString("INVALID");
    batteryCycleCountEntry.setDouble(0);
    batteryTimeVoltagePayloadEntry.setString("");
    batteryEnergyPayloadEntry.setString("");
    */
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    boolean userButton = userButtonEntry.getBoolean(false);
    if (userButton && !prevUserButton) {
      espREntry.setDouble(ThreadLocalRandom.current().nextInt(0, 256));
      espGEntry.setDouble(ThreadLocalRandom.current().nextInt(0, 256));
      espBEntry.setDouble(ThreadLocalRandom.current().nextInt(0, 256));
      espRelayEntry.setBoolean(!espRelayEntry.getBoolean(false));
      userButtonEntry.setBoolean(false);
    }
    prevUserButton = userButton;

    int mode = getInt(ledModeEntry, 1);
    int r = getInt(ledREntry, 255);
    int g = getInt(ledGEntry, 255);
    int b = getInt(ledBEntry, 255);
    int brightness = getInt(ledBrightnessEntry, 128);
    int onOff = getInt(ledOnOffEntry, 1);
    int param0 = getInt(ledParam0Entry, 20);
    int param1 = getInt(ledParam1Entry, 20);
    int totalPixels = getInt(ledTotalPixelsEntry, 150);
    int secondaryR = getInt(ledSecondaryREntry, 0);
    int secondaryG = getInt(ledSecondaryGEntry, 0);
    int secondaryB = getInt(ledSecondaryBEntry, 0);
    int secondaryBrightness = getInt(ledSecondaryBrightnessEntry, 0);
    int secondaryEnable = ledSecondaryEnableEntry.getBoolean(false) ? 1 : 0;

    leds.setTotalPixel(totalPixels);
    leds.setSecondaryColor(
        secondaryEnable, secondaryR, secondaryG, secondaryB, secondaryBrightness);

    boolean changed =
        mode != lastMode
            || r != lastR
            || g != lastG
            || b != lastB
            || brightness != lastBrightness
            || onOff != lastOnOff
            || param0 != lastParam0
            || param1 != lastParam1;

    if (changed) {
      leds.sendGeneralCommand(mode, r, g, b, brightness, onOff, param0, param1);
      lastMode = mode;
      lastR = r;
      lastG = g;
      lastB = b;
      lastBrightness = brightness;
      lastOnOff = onOff;
      lastParam0 = param0;
      lastParam1 = param1;
    }

    boolean writePixel = ledWritePixelEntry.getBoolean(false);
    if (writePixel && !lastWritePixel) {
      int index = getInt(ledPixelIndexEntry, 0);
      int pr = getInt(ledPixelREntry, 255);
      int pg = getInt(ledPixelGEntry, 0);
      int pb = getInt(ledPixelBEntry, 0);
      int pbrig = getInt(ledPixelBrightnessEntry, 128);

      leds.sendPixelWrite(index, pr, pg, pb, 0, pbrig, 0);
      ledWritePixelEntry.setBoolean(false);
    }
    lastWritePixel = writePixel;

    ledFeedbackModeEntry.setDouble(leds.getFeedbackMode());
    ledFeedbackLedCountEntry.setDouble(leds.getFeedbackLedCount());
    ledOnlineEntry.setBoolean(leds.getIsESPOnline());

    boolean rainbowEnabled = espRainbowEntry.getBoolean(false);
    int er = getInt(espREntry, 0);
    int eg = getInt(espGEntry, 0);
    int eb = getInt(espBEntry, 0);
    boolean erelay = espRelayEntry.getBoolean(false);

    esp32.setManualOutputs(er, eg, eb, erelay);
    esp32.setRainbowEnabled(rainbowEnabled);

    if (rainbowEnabled) {
      espREntry.setDouble(esp32.getCurrentOutputR());
      espGEntry.setDouble(esp32.getCurrentOutputG());
      espBEntry.setDouble(esp32.getCurrentOutputB());
    }

    if (espRequestResetEntry.getBoolean(false)) {
      esp32.requestReset();
      espRequestResetEntry.setBoolean(false);
    }

    double nowS = Timer.getTimestamp();
    esp32.poll(nowS);

    boolean btnAReleased = esp32.isButtonAReleased();
    boolean btnBReleased = esp32.isButtonBReleased();

    boolean aPressedEdge = !btnAReleased && prevBtnAReleased;
    boolean bPressedEdge = !btnBReleased && prevBtnBReleased;

    if (aPressedEdge) {
      radioLEDState = 1;
      espRadioLEDStateEntry.setDouble(radioLEDState);
    }
    if (bPressedEdge) {
      radioLEDState = 0;
      espRadioLEDStateEntry.setDouble(radioLEDState);
    }

    prevBtnAReleased = btnAReleased;
    prevBtnBReleased = btnBReleased;

    int analogRaw = esp32.getAnalogRaw();
    if (analogRaw >= 0) {
      double vRead = ADC_K0 + ADC_K1 * analogRaw + ADC_K2 * (analogRaw * (double) analogRaw);
      espVoltageEntry.setDouble(vRead);
      espAnalogEntry.setDouble(analogRaw);
    } else {
      espVoltageEntry.setDouble(-1);
      espAnalogEntry.setDouble(-1);
    }

    espButtonAEntry.setString(btnAReleased ? "Released" : "Pressed");
    espButtonBEntry.setString(btnBReleased ? "Released" : "Pressed");

    double ageMs = esp32.getInputsAgeMs(nowS);
    espInputsAgeMsEntry.setDouble(ageMs);
    espInputsStaleEntry.setBoolean(esp32.isInputsStale(nowS, ESP_INPUTS_STALE_MS));
    espOnlineEntry.setBoolean(esp32.getIsESPOnline());
    espResetFlagSeenEntry.setDouble(esp32.getLastResetFlag());

    /*
    if (batteryCan != null && batteryCan.valid) {
      batteryValidEntry.setBoolean(true);
      batterySnEntry.setString(batteryCan.serialNumber);
      batteryFirstUseEntry.setString(
          String.format(
              "%04d-%02d-%02d",
              batteryCan.firstUseYear, batteryCan.firstUseMonth, batteryCan.firstUseDay));
      batteryNoteEntry.setString(batteryCan.noteText);
      batteryCycleCountEntry.setDouble(batteryCan.cycleCount);
    } else {
      batteryValidEntry.setBoolean(false);
      batterySnEntry.setString("INVALID");
      batteryFirstUseEntry.setString("0000-00-00");
      batteryNoteEntry.setString("INVALID");
      batteryCycleCountEntry.setDouble(0);
    }

    batteryTimeVoltagePayloadEntry.setString(batteryCan.lastTimeVoltagePayload);
    batteryEnergyPayloadEntry.setString(batteryCan.lastEnergyPayload);
    */
  }

  @Override
  public void autonomousInit() {
    sparkMax.stopMotor();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    sparkMax.stopMotor();
  }

  @Override
  public void teleopPeriodic() {
    // Gamepad left Y is positive when pulled back, so negate it for forward-positive output.
    double motorOutput = -applyDeadband(driverController.getLeftY(), DRIVE_AXIS_DEADBAND);
    sparkMax.setThrottle(motorOutput);
  }

  @Override
  public void disabledInit() {
    sparkMax.stopMotor();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void utilityInit() {}

  @Override
  public void utilityPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private static int getInt(NetworkTableEntry entry, int defaultValue) {
    return (int) entry.getDouble(defaultValue);
  }

  private static double applyDeadband(double value, double deadband) {
    return Math.abs(value) > deadband ? value : 0.0;
  }
}
