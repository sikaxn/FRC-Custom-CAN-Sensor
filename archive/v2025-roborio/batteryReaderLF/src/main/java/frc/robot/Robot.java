package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.batteryCANLF;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private batteryCANLF batteryLF;

  // Track last reboot button state to detect rising edge
  private boolean lastRebootButton = false;

  @Override
  public void robotInit() {
    // Initialize LF reader (Device Number = 55 per your ESP32 firmware)
    batteryLF = new batteryCANLF(55);

    // Add SmartDashboard control
    SmartDashboard.putBoolean("ESP Reboot", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // --- SmartDashboard Reboot Button Handling ---
    boolean rebootButton = SmartDashboard.getBoolean("ESP Reboot", false);
    if (rebootButton && !lastRebootButton) {
      // Trigger on rising edge
      batteryLF.requestReboot();
      System.out.println("[Robot] ESP Reboot command sent.");
      SmartDashboard.putBoolean("ESP Reboot", false); // auto-reset button
    }
    lastRebootButton = rebootButton;

    // --- Display Tag Info ---
    if (batteryLF != null) {
      SmartDashboard.putString("Tag Serial", batteryLF.getSerial());
      SmartDashboard.putBoolean("Tag Present", batteryLF.getTagPresent());
      SmartDashboard.putNumber("New Tag Count", batteryLF.getNewTagCount());
      SmartDashboard.putBoolean("ESP Online", batteryLF.getESPState());
    } else {
      SmartDashboard.putString("Tag Serial", "INVALID");
      SmartDashboard.putBoolean("Tag Present", false);
      SmartDashboard.putNumber("New Tag Count", 0);
      SmartDashboard.putBoolean("ESP Online", false);
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
