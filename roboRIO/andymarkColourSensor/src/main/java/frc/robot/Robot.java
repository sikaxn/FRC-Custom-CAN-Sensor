package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.REVColourCAN;
import frc.robot.subsystems.amColourCAN;

public class Robot extends TimedRobot {

  private REVColourCAN revColourCAN;
  private amColourCAN amColourCAN;
  private boolean lastRevRebootButton = false;
  private boolean lastAmRebootButton = false;

  @Override
  public void robotInit() {
    revColourCAN = new REVColourCAN(33);
    amColourCAN = new amColourCAN(33);

    // Prepare dashboard fields
    SmartDashboard.putBoolean("REV_Reboot", false);
    SmartDashboard.putBoolean("AM_Reboot", false);
    SmartDashboard.putBoolean("REV_ESP_Status", revColourCAN.getESPState());
    SmartDashboard.putBoolean("AM_ESP_Status", amColourCAN.getESPState());

  }

  @Override
  public void robotPeriodic() {

    // --------- REV COLOUR SENSOR DISPLAY ----------
    SmartDashboard.putNumber("REV_IR", revColourCAN.getIR());
    SmartDashboard.putNumber("REV_Red", revColourCAN.getRed());
    SmartDashboard.putNumber("REV_Green", revColourCAN.getGreen());
    SmartDashboard.putNumber("REV_Blue", revColourCAN.getBlue());
    SmartDashboard.putNumber("REV_Prox", revColourCAN.getProximity());
    SmartDashboard.putBoolean("REV_Good", revColourCAN.getESPState());
    SmartDashboard.putBoolean("REV_ESP_Status", revColourCAN.getESPState());
    // --------- AM COLOUR SENSOR DISPLAY ----------
    SmartDashboard.putNumber("AM_Clear", amColourCAN.getClear());
    SmartDashboard.putNumber("AM_Red", amColourCAN.getRed());
    SmartDashboard.putNumber("AM_Green", amColourCAN.getGreen());
    SmartDashboard.putNumber("AM_Blue", amColourCAN.getBlue());
    SmartDashboard.putNumber("AM_Prox", amColourCAN.getProximity());
    SmartDashboard.putBoolean("AM_Good", amColourCAN.getStatus());
    SmartDashboard.putBoolean("AM_ESP_Status", amColourCAN.getESPState());

    // --------- REBOOT BUTTONS ----------
    boolean revRebootButton = SmartDashboard.getBoolean("REV_Reboot", false);
    boolean amRebootButton = SmartDashboard.getBoolean("AM_Reboot", false);

    // Rising edge detect
    if (revRebootButton && !lastRevRebootButton) {
      revColourCAN.requestReboot();
    }
    if (amRebootButton && !lastAmRebootButton) {
      amColourCAN.requestReboot();
    }
    lastRevRebootButton = revRebootButton;
    lastAmRebootButton = amRebootButton;

    // Always reset dashboard button (one-shot)
    SmartDashboard.putBoolean("REV_Reboot", false);
    SmartDashboard.putBoolean("AM_Reboot", false);
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
