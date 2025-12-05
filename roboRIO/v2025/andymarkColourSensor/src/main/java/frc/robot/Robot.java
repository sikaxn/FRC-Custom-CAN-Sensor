package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.amColourCAN;

public class Robot extends TimedRobot {

  private amColourCAN colourCAN;
  private boolean lastRebootButton = false;

  @Override
  public void robotInit() {
    colourCAN = new amColourCAN(33);

    // Prepare dashboard fields
    SmartDashboard.putBoolean("Colour_Reboot", false);
    SmartDashboard.putBoolean("ESP Status", colourCAN.getESPState());


  }

  @Override
  public void robotPeriodic() {

    // --------- COLOUR SENSOR DISPLAY ----------
    SmartDashboard.putNumber("Colour_Clear", colourCAN.getClear());
    SmartDashboard.putNumber("Colour_Red", colourCAN.getRed());
    SmartDashboard.putNumber("Colour_Green", colourCAN.getGreen());
    SmartDashboard.putNumber("Colour_Blue", colourCAN.getBlue());
    SmartDashboard.putNumber("Colour_Prox", colourCAN.getProximity());
    SmartDashboard.putBoolean("Colour_Good", colourCAN.getStatus());
    SmartDashboard.putBoolean("ESP Status", colourCAN.getESPState());
    // --------- REBOOT BUTTON ----------
    boolean rebootButton = SmartDashboard.getBoolean("Colour_Reboot", false);

    // Rising edge detect
    if (rebootButton && !lastRebootButton) {
      if (colourCAN != null) {
        colourCAN.requestReboot();
      }
    }
    lastRebootButton = rebootButton;

    // Always reset dashboard button (one-shot)
    SmartDashboard.putBoolean("Colour_Reboot", false);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
