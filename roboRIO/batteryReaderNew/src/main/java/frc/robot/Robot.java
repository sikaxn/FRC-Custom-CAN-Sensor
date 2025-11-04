package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.batteryCAN;

public class Robot extends TimedRobot {
  private batteryCAN battery;

  @Override
  public void robotInit() {
    battery = new batteryCAN(33); // your ESP32 device number
    SmartDashboard.putNumber("Override State", 0);
    SmartDashboard.putNumber("Energy kJ", 0);
    SmartDashboard.putBoolean("ESP Reboot", false);
  }

  @Override
  public void robotPeriodic() {
    // Handle CAN RX/TX
    battery.update();

    // === Display telemetry from ESP32 ===
    SmartDashboard.putString("Battery Serial", battery.getSerial());
    SmartDashboard.putNumber("Cycle Count", battery.getCycleCount());
    SmartDashboard.putNumber("Note", battery.getNote());
    SmartDashboard.putNumber("ESP State", battery.getESPState());
    SmartDashboard.putNumber("PD Type", battery.getPDType());
    SmartDashboard.putBoolean("Reader Detected", battery.isReaderDetected());
    SmartDashboard.putNumber("Auth Fail Count", battery.getAuthFailCount());
    SmartDashboard.putNumber("Write Count", battery.getWriteCount());
    SmartDashboard.putString("Battery Date", battery.getFirstUseDateTime());

    // === Dashboard Controls ===
    int overrideState = (int) SmartDashboard.getNumber("Override State", 0);
    int energyKJ = (int) SmartDashboard.getNumber("Energy kJ", 0);
    boolean reboot = SmartDashboard.getBoolean("ESP Reboot", false);

    battery.setOverrideState(overrideState);
    battery.setEnergyKJ(energyKJ);

    // Button-style reboot trigger
    if (reboot) {
      battery.requestReboot();
      SmartDashboard.putBoolean("ESP Reboot", false);
    }
  }
}
