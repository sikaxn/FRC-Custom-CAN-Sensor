package first.robot;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

public class Robot extends TimedRobot {
  private final RobotContainer robotContainer = new RobotContainer();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    robotContainer.onAutonomousInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    robotContainer.onTeleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    robotContainer.onDisabledInit();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void utilityInit() {
    robotContainer.onUtilityInit();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
