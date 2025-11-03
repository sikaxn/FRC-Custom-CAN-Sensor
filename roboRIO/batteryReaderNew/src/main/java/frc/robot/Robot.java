package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.batteryCAN;

public class Robot extends TimedRobot {

  private batteryCAN batteryCan;
  private boolean lastRebootButton = false;

  @Override
  public void robotInit() {
    batteryCan = new batteryCAN();

    // Initialize SmartDashboard fields
    SmartDashboard.putNumber("Set Energy (kJ)", 0);
    SmartDashboard.putNumber("State Override", 0);
    SmartDashboard.putBoolean("Request ESP Reboot", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update CAN communication (auto handles TX & RX)
    batteryCan.update();

    // --------------------- Dashboard Input ---------------------
    double energyInput = SmartDashboard.getNumber("Set Energy (kJ)", 0);
    int overrideState = (int) SmartDashboard.getNumber("State Override", 0);
    boolean requestReboot = SmartDashboard.getBoolean("Request ESP Reboot", false);

    // Energy override
    if (energyInput > 0) {
      batteryCan.setEnergyKJ((int) energyInput);
    } else {
      batteryCan.clearEnergyControl();
    }

    // State override
    batteryCan.overrideState(overrideState);

    // One-shot ESP reboot
    if (requestReboot && !lastRebootButton) {
      batteryCan.requestESPReboot();
      SmartDashboard.putBoolean("Request ESP Reboot", false); // reset toggle
    }
    lastRebootButton = requestReboot;

    // --------------------- Dashboard Output ---------------------
    SmartDashboard.putString("Battery SN", batteryCan.getSerial());
    SmartDashboard.putString("First Use (UTC)",
        String.format("%04d-%02d-%02d %02d:%02d",
            batteryCan.getYear(), batteryCan.getMonth(),
            batteryCan.getDay(), batteryCan.getHour(),
            batteryCan.getMinute()));
    SmartDashboard.putNumber("Cycle Count", batteryCan.getCycleCount());
    SmartDashboard.putNumber("Battery Note", batteryCan.getNote());

    SmartDashboard.putBoolean("Reader Detected", batteryCan.isReaderDetected());
    SmartDashboard.putNumber("ESP State", batteryCan.getESPState());
    SmartDashboard.putNumber("PD Type", batteryCan.getPDType());
    SmartDashboard.putNumber("Auth Fail Count", batteryCan.getAuthFailCount());
    SmartDashboard.putNumber("Write Count", batteryCan.getWriteCount());

    SmartDashboard.putBoolean("Use RIO Energy", batteryCan.isUseRIOEnergy());
    SmartDashboard.putNumber("Energy KJ (TX)", batteryCan.getEnergyKJ());
    SmartDashboard.putNumber("Override State (TX)", batteryCan.getOverrideState());
  }
}
