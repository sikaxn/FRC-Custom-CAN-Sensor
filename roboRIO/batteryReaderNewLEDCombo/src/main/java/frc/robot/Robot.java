package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.addressableLEDCAN;
import frc.robot.subsystems.batteryCAN;

public class Robot extends TimedRobot {
  private batteryCAN battery;
  private addressableLEDCAN leds;

  private int lastMode = -1;
  private int lastR = -1, lastG = -1, lastB = -1;
  private int lastBrightness = -1, lastOnOff = -1;
  private int lastParam0 = -1, lastParam1 = -1;
  private int lastTotalPixels = -1;
  private boolean lastWritePixel = false;

  @Override
  public void robotInit() {
    battery = new batteryCAN(33); // your ESP32 device number
    leds = new addressableLEDCAN(33); // Match LED ESP32 device number
    //If using Battery + LED combo firmware, device number for both init should be the same.
    
    SmartDashboard.putNumber("Override State", 0);
    SmartDashboard.putNumber("Energy kJ", 0);
    SmartDashboard.putBoolean("ESP Reboot", false);
    SmartDashboard.putBoolean("Using RIO energy", false);

    // General LED control
    SmartDashboard.setDefaultNumber("LED Mode", 1);
    SmartDashboard.setDefaultNumber("LED R", 255);
    SmartDashboard.setDefaultNumber("LED G", 255);
    SmartDashboard.setDefaultNumber("LED B", 255);
    SmartDashboard.setDefaultNumber("LED Brightness", 128);
    SmartDashboard.setDefaultNumber("LED OnOff", 1);
    SmartDashboard.setDefaultNumber("LED Param0", 20);
    SmartDashboard.setDefaultNumber("LED Param1", 20);
    SmartDashboard.setDefaultNumber("LED Total Pixels", 10);

    // Custom pixel write
    SmartDashboard.setDefaultNumber("LED Pixel Index", 0);
    SmartDashboard.setDefaultNumber("LED Pixel R", 255);
    SmartDashboard.setDefaultNumber("LED Pixel G", 0);
    SmartDashboard.setDefaultNumber("LED Pixel B", 0);
    SmartDashboard.setDefaultNumber("LED Pixel Brightness", 128);
    SmartDashboard.setDefaultBoolean("LED Write Pixel", false);
  }

  @Override
  public void robotPeriodic() {


    // === Display telemetry from ESP32 ===
    SmartDashboard.putString("Battery Serial", battery.getSerial());
    SmartDashboard.putNumber("Cycle Count", battery.getCycleCount());
    String noteLabel = switch (battery.getNote()) {
        case 0 -> "Normal";
        case 1 -> "Practice Only";
        case 2 -> "Scrap";
        case 3 -> "Other";
        default -> "Unknown";
    };
    SmartDashboard.putString("NotCause", noteLabel);
    SmartDashboard.putNumber("Note", battery.getNote());
    SmartDashboard.putNumber("ESP State", battery.getESPState());
    SmartDashboard.putNumber("PD Type", battery.getPDType());
    SmartDashboard.putBoolean("Reader Detected", battery.isReaderDetected());
    SmartDashboard.putNumber("Write Fail Count", battery.getWriteFailCount());
    SmartDashboard.putNumber("Write Count", battery.getWriteCount());
    SmartDashboard.putString("Battery Date", battery.getFirstUseDateTime());
    SmartDashboard.putString("ESP32 Status", battery.getIsESPOnline() ? "ONLINE" : "OFFLINE");

    // === Dashboard Controls ===
    int overrideState = (int) SmartDashboard.getNumber("Override State", 0); //this is a dangerous action and should only be used for debuging.
    int energyKJ = (int) SmartDashboard.getNumber("Energy kJ", 0);
    boolean reboot = SmartDashboard.getBoolean("ESP Reboot", false);
    boolean isUsingRIOEnergy = SmartDashboard.getBoolean("Using RIO energy", false);

    battery.setOverrideState(overrideState);
    battery.setEnergyKJ(energyKJ);
    battery.setUseRIOEnergy(isUsingRIOEnergy);
    // Button-style reboot trigger
    if (reboot) {
      battery.requestReboot();
      SmartDashboard.putBoolean("ESP Reboot", false);
    }

    // === LED Control ===
    int mode       = (int) SmartDashboard.getNumber("LED Mode", 1);
    int r          = (int) SmartDashboard.getNumber("LED R", 255);
    int g          = (int) SmartDashboard.getNumber("LED G", 255);
    int b          = (int) SmartDashboard.getNumber("LED B", 255);
    int brightness = (int) SmartDashboard.getNumber("LED Brightness", 128);
    int onOff      = (int) SmartDashboard.getNumber("LED OnOff", 1);
    int param0     = (int) SmartDashboard.getNumber("LED Param0", 20);
    int param1     = (int) SmartDashboard.getNumber("LED Param1", 20);
    int totalPixels = (int) SmartDashboard.getNumber("LED Total Pixels", 10);

    boolean changed =
        mode       != lastMode       ||
        r          != lastR          ||
        g          != lastG          ||
        b          != lastB          ||
        brightness != lastBrightness ||
        onOff      != lastOnOff      ||
        param0     != lastParam0     ||
        param1     != lastParam1;

    if (changed) {
      if (totalPixels != lastTotalPixels) {
        leds.setTotalPixel(totalPixels);
        lastTotalPixels = totalPixels;
      }
      leds.sendGeneralCommand(mode, r, g, b, brightness, onOff, param0, param1);
      lastMode       = mode;
      lastR          = r;
      lastG          = g;
      lastB          = b;
      lastBrightness = brightness;
      lastOnOff      = onOff;
      lastParam0     = param0;
      lastParam1     = param1;
    }

    // Pixel write logic
    boolean writePixel = SmartDashboard.getBoolean("LED Write Pixel", false);
    if (writePixel && !lastWritePixel) {
      int index = (int) SmartDashboard.getNumber("LED Pixel Index", 0);
      int pr    = (int) SmartDashboard.getNumber("LED Pixel R", 255);
      int pg    = (int) SmartDashboard.getNumber("LED Pixel G", 0);
      int pb    = (int) SmartDashboard.getNumber("LED Pixel B", 0);
      int pbrig = (int) SmartDashboard.getNumber("LED Pixel Brightness", 128);

      leds.sendPixelWrite(index, pr, pg, pb, 0, pbrig, 0); // w=0, slot=0
      SmartDashboard.putBoolean("LED Write Pixel", false); // auto-reset trigger
    }
    lastWritePixel = writePixel;
  }
}
