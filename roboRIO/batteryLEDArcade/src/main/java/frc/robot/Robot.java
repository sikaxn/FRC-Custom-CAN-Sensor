package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.addressableLEDCAN;
import frc.robot.subsystems.batteryCAN;

public class Robot extends TimedRobot {
  private static final int LED_DEFAULT_BRIGHTNESS = 128;
  private static final int LED_DEFAULT_ON_OFF = 1;
  private static final int LED_DEFAULT_TOTAL_PIXELS = 150;
  private static final int LED_DEFAULT_PARAM1 = 20;
  private batteryCAN battery;
  private addressableLEDCAN leds;
  private SparkMax leftLeader;
  private SparkMax leftFollower;
  private SparkMax rightLeader;
  private SparkMax rightFollower;
  private XboxController controller;

  @Override
  public void robotInit() {
    battery = new batteryCAN(33); // your ESP32 device number
    leds = new addressableLEDCAN(33); // Match LED ESP32 device number
    //If using Battery + LED combo firmware, device number for both init should be the same.
    
    SmartDashboard.putNumber("Override State", 0);
    SmartDashboard.putNumber("Energy kJ", 0);
    SmartDashboard.putBoolean("ESP Reboot", false);
    SmartDashboard.putBoolean("Using RIO energy", false);

    leftLeader = new SparkMax(11, MotorType.kBrushed); // Left side leader
    leftFollower = new SparkMax(21, MotorType.kBrushed); // Left side follower
    rightLeader = new SparkMax(12, MotorType.kBrushed); // Right side leader
    rightFollower = new SparkMax(22, MotorType.kBrushed); // Right side follower

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    leftLeaderConfig
        .apply(globalConfig);

    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    leftFollowerConfig
        .apply(globalConfig)
        .follow(leftLeader);

    rightFollowerConfig
        .apply(globalConfig)
        .follow(rightLeader);

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {


    // === Display telemetry from ESP32 ===
    SmartDashboard.putString("Battery Serial", battery.getSerial());
    SmartDashboard.putNumber("Cycle Count", battery.getCycleCount());
    String noteLabel = switch (battery.getNote()) {
        case 0 -> "Normal";
        case 1 -> "Practice Only";
        case 2 -> "Scrap";
        case 3 -> "Other";
        default -> "Unknown";
    };
    SmartDashboard.putString("NotCause", noteLabel);
    SmartDashboard.putNumber("Note", battery.getNote());
    SmartDashboard.putNumber("ESP State", battery.getESPState());
    SmartDashboard.putNumber("PD Type", battery.getPDType());
    SmartDashboard.putBoolean("Reader Detected", battery.isReaderDetected());
    SmartDashboard.putNumber("Write Fail Count", battery.getWriteFailCount());
    SmartDashboard.putNumber("Write Count", battery.getWriteCount());
    SmartDashboard.putString("Battery Date", battery.getFirstUseDateTime());
    SmartDashboard.putString("ESP32 Status", battery.getIsESPOnline() ? "ONLINE" : "OFFLINE");

    // === Dashboard Controls ===
    int overrideState = (int) SmartDashboard.getNumber("Override State", 0); //this is a dangerous action and should only be used for debuging.
    int energyKJ = (int) SmartDashboard.getNumber("Energy kJ", 0);
    boolean reboot = SmartDashboard.getBoolean("ESP Reboot", false);
    boolean isUsingRIOEnergy = SmartDashboard.getBoolean("Using RIO energy", false);

    battery.setOverrideState(overrideState);
    battery.setEnergyKJ(energyKJ);
    battery.setUseRIOEnergy(isUsingRIOEnergy);
    // Button-style reboot trigger
    if (reboot) {
      battery.requestReboot();
      SmartDashboard.putBoolean("ESP Reboot", false);
    }

    // === LED Control ===
    int mode;
    int r;
    int g;
    int b;
    int brightness = LED_DEFAULT_BRIGHTNESS;
    int onOff      = LED_DEFAULT_ON_OFF;
    int param0;
    int param1;
    int totalPixels = LED_DEFAULT_TOTAL_PIXELS;

    double leftOutput = leftLeader.getAppliedOutput();
    double rightOutput = rightLeader.getAppliedOutput();
    double avgOutput = (leftOutput + rightOutput) / 2.0;
    double speedAbs = Math.abs(avgOutput);
    double turnDiff = Math.abs(leftOutput - rightOutput);

    if (!DriverStation.isEnabled()) {
      mode = 5;
      param0 = 5;
      param1 = 5;
      r = 255;
      g = 255;
      b = 255;
    } else if (turnDiff > 0.1) {
      mode = 4;
      double turnNorm = MathUtil.clamp((turnDiff - 0.1) / 1.9, 0.0, 1.0);
      param0 = (int) Math.round(100.0 - (100.0 * turnNorm));
      param0 = (int) MathUtil.clamp(param0, 0, 100);
      param1 = LED_DEFAULT_PARAM1;
      r = speedAbs > 0.2 && avgOutput < 0 ? 255 : 0;
      g = speedAbs > 0.2 && avgOutput > 0 ? 255 : 0;
      b = 0;
    } else if (speedAbs < 0.5) {
      mode = 13;
      param0 = (int) Math.round(30.0 - (20.0 * (speedAbs / 0.5)));
      param1 = 15;
      r = speedAbs > 0.2 && avgOutput < 0 ? 255 : 0;
      g = speedAbs > 0.2 && avgOutput > 0 ? 255 : 0;
      b = 255;
    } else {
      mode = 16;
      param0 = (int) Math.round(30.0 + (40.0 * ((speedAbs - 0.5) / 0.5)));
      param0 = (int) MathUtil.clamp(param0, 30, 70);
      param1 = 2;
      r = avgOutput < 0 ? 255 : 0;
      g = avgOutput > 0 ? 255 : 0;
      b = 0;
    }

    leds.setTotalPixel(totalPixels);
    leds.sendGeneralCommand(mode, r, g, b, brightness, onOff, param0, param1);

    SmartDashboard.putNumber("LED Mode", mode);
    SmartDashboard.putNumber("LED R", r);
    SmartDashboard.putNumber("LED G", g);
    SmartDashboard.putNumber("LED B", b);
    SmartDashboard.putNumber("LED Brightness", brightness);
    SmartDashboard.putNumber("LED OnOff", onOff);
    SmartDashboard.putNumber("LED Param0", param0);
    SmartDashboard.putNumber("LED Param1", param1);
    SmartDashboard.putNumber("LED Total Pixels", totalPixels);

    SmartDashboard.putNumber("Left Out", leftLeader.getAppliedOutput());
    SmartDashboard.putNumber("Right Out", rightLeader.getAppliedOutput());
  }

  @Override
  public void teleopPeriodic() {
    double speed = MathUtil.applyDeadband(-controller.getLeftY(), 0.04);
    double rotation = MathUtil.applyDeadband(-controller.getRightX(), 0.04);
    leftLeader.set(speed + rotation);
    rightLeader.set(speed - rotation);
  }
}
