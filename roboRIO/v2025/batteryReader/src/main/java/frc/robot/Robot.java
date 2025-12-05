package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.batteryCAN;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private batteryCAN batteryCan;

  @Override
  public void robotInit() {
    batteryCan = new batteryCAN();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (batteryCan != null && batteryCan.valid) {
      SmartDashboard.putString("Battery SN", batteryCan.serialNumber);
      SmartDashboard.putString("Battery First Use (UTC)", 
          String.format("%04d-%02d-%02d",
              batteryCan.firstUseYear,
              batteryCan.firstUseMonth,
              batteryCan.firstUseDay
          )
      );
      SmartDashboard.putString("Battery Note", batteryCan.noteText);
      SmartDashboard.putNumber("Battery Cycle Count", batteryCan.cycleCount);
  } else {
      SmartDashboard.putString("Battery SN", "INVALID");
      SmartDashboard.putString("Battery First Use (UTC)", "0000-00-00");
      SmartDashboard.putString("Battery Note", "INVALID");
      SmartDashboard.putNumber("Battery Cycle Count", 0);
      }
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
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
