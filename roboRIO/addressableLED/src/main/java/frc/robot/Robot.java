package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.addressableLEDCAN;

public class Robot extends TimedRobot {
  private addressableLEDCAN leds;
  private int lastMode = -1;
  private int lastR = -1, lastG = -1, lastB = -1;
  private int lastBrightness = -1, lastOnOff = -1;
  private int lastParam0 = -1, lastParam1 = -1;

  private boolean lastWritePixel = false;

  @Override
  public void robotInit() {
    leds = new addressableLEDCAN(8);  // Match ESP32 device number

    // General LED control
    SmartDashboard.setDefaultNumber("LED Mode", 1);
    SmartDashboard.setDefaultNumber("LED R", 255);
    SmartDashboard.setDefaultNumber("LED G", 255);
    SmartDashboard.setDefaultNumber("LED B", 255);
    SmartDashboard.setDefaultNumber("LED Brightness", 128);
    SmartDashboard.setDefaultNumber("LED OnOff", 1);
    SmartDashboard.setDefaultNumber("LED Param0", 20);
    SmartDashboard.setDefaultNumber("LED Param1", 20);

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
    // General mode control
    int mode       = (int) SmartDashboard.getNumber("LED Mode", 1);
    int r          = (int) SmartDashboard.getNumber("LED R", 255);
    int g          = (int) SmartDashboard.getNumber("LED G", 255);
    int b          = (int) SmartDashboard.getNumber("LED B", 255);
    int brightness = (int) SmartDashboard.getNumber("LED Brightness", 128);
    int onOff      = (int) SmartDashboard.getNumber("LED OnOff", 1);
    int param0     = (int) SmartDashboard.getNumber("LED Param0", 20);
    int param1     = (int) SmartDashboard.getNumber("LED Param1", 20);

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

      leds.sendPixelWrite(index, pr, pg, pb, 0, pbrig, 0); // w=0, slot=0 (use more slots later)

      SmartDashboard.putBoolean("LED Write Pixel", false); // auto-reset trigger
    }
    lastWritePixel = writePixel;
  }

  @Override public void autonomousInit() {}
  @Override public void autonomousPeriodic() {}
  @Override public void teleopInit() {}
  @Override public void teleopPeriodic() {}
  @Override public void disabledInit() {}
  @Override public void disabledPeriodic() {}
  @Override public void testInit() {}
  @Override public void testPeriodic() {}
  @Override public void simulationInit() {}
  @Override public void simulationPeriodic() {}
}
