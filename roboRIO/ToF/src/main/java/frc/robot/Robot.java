package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ToFCAN;

public class Robot extends TimedRobot {
  private ToFCAN tof;

  public Robot() {}

  @Override
  public void robotInit() {
    tof = new ToFCAN(); // Initialize ToF CAN subsystem
  }

  @Override
  public void robotPeriodic() {
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("ToF Distance " + i, tof.getDistance(i));
      SmartDashboard.putNumber("ToF Mode " + i, tof.getMode(i));
      SmartDashboard.putNumber("ToF ROI " + i, tof.getROI(i));
      SmartDashboard.putNumber("ToF TimingBudget " + i, tof.getTimingBudget(i));
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
