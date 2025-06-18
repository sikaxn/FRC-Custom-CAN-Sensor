package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ToFCAN;

public class Robot extends TimedRobot {
  private ToFCAN tof;
  private final int[] lastSetMode = new int[4];
  private final int[] lastSetCenter = new int[4];
  private final int[] lastSetX = new int[4];
  private final int[] lastSetY = new int[4];

  @Override
  public void robotInit() {
    tof = new ToFCAN();

    for (int i = 0; i < 4; i++) {
      SmartDashboard.setDefaultNumber("ToF Set Mode " + i, 2);        // Default: Long
      SmartDashboard.setDefaultNumber("ToF Set ROI Center " + i, 199);
      SmartDashboard.setDefaultNumber("ToF Set ROI Size X " + i, 8);
      SmartDashboard.setDefaultNumber("ToF Set ROI Size Y " + i, 8);

      lastSetMode[i] = -1;
      lastSetCenter[i] = -1;
      lastSetX[i] = -1;
      lastSetY[i] = -1;
    }
  }

  @Override
  public void robotPeriodic() {
    for (int i = 0; i < 4; i++) {
      // Read data from sensor
      SmartDashboard.putNumber("ToF Distance " + i, tof.getDistance(i));
      SmartDashboard.putNumber("ToF Mode " + i, tof.getMode(i));
      SmartDashboard.putNumber("ToF ROI Center " + i, tof.getROICenter(i));
      SmartDashboard.putNumber("ToF ROI Size X " + i, tof.getROISizeX(i));
      SmartDashboard.putNumber("ToF ROI Size Y " + i, tof.getROISizeY(i));
      SmartDashboard.putNumber("ToF TimingBudget " + i, tof.getTimingBudget(i));

      // Get desired values from SmartDashboard
      int mode = (int) SmartDashboard.getNumber("ToF Set Mode " + i, 2);
      int center = (int) SmartDashboard.getNumber("ToF Set ROI Center " + i, 199);
      int x = (int) SmartDashboard.getNumber("ToF Set ROI Size X " + i, 8);
      int y = (int) SmartDashboard.getNumber("ToF Set ROI Size Y " + i, 8);

      // Apply if changed
      if (mode != lastSetMode[i] || center != lastSetCenter[i] || x != lastSetX[i] || y != lastSetY[i]) {
        tof.setDesiredConfig(i, mode, center, x, y);
        lastSetMode[i] = mode;
        lastSetCenter[i] = center;
        lastSetX[i] = x;
        lastSetY[i] = y;
      }
    }
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
