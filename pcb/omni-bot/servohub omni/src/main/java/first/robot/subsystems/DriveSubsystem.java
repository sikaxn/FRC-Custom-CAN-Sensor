package first.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.util.Signal;
import first.robot.drivers.ScWitMotionDriver;
import java.util.Objects;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

public class DriveSubsystem extends SubsystemBase {
  private static final int CAN_BUS_ID = 1;
  private static final int SERVO_HUB_DEVICE_ID = 3;
  private static final boolean DEFAULT_FIELD_ORIENTED = true;
  private static final double FIELD_YAW_SIGN = -1.0;
  private static final double YAW_RESET_FLASH_PHASE_SECONDS = 0.10;
  private static final int YAW_RESET_FLASH_PHASE_COUNT = 4;
  private static final double DRIVE_DEADBAND = 0.08;
  private static final double MIN_SPEED_SCALE = 0.5;
  private static final int PWM_MIN_US = 500;
  private static final int PWM_NEUTRAL_US = 1500;
  private static final int PWM_MAX_US = 2500;
  private static final int PWM_TRAVEL_US = (PWM_MAX_US - PWM_MIN_US) / 2;
  private static final int PWM_PERIOD_US = 20000;

  private static final boolean FRONT_LEFT_INVERTED = false;
  private static final boolean FRONT_RIGHT_INVERTED = true;
  private static final boolean REAR_RIGHT_INVERTED = true;
  private static final boolean REAR_LEFT_INVERTED = false;

  private final WitMotionSubsystem witMotionSubsystem;
  private final ServoHub servoHub = new ServoHub(CAN_BUS_ID, SERVO_HUB_DEVICE_ID);
  private final ServoChannel frontLeftChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId0);
  private final ServoChannel frontRightChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId1);
  private final ServoChannel rearRightChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId2);
  private final ServoChannel rearLeftChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId3);

  private final NetworkTableEntry servoHubVoltageEntry;
  private final NetworkTableEntry servoHubVoltageValidEntry;
  private final NetworkTableEntry driveModeEntry;
  private final NetworkTableEntry fieldOrientedEntry;
  private final NetworkTableEntry driveHeadingEntry;
  private final NetworkTableEntry driveDesiredDegreesEntry;
  private final NetworkTableEntry driveDesiredMagnitudeEntry;
  private final NetworkTableEntry driveHeadingErrorEntry;
  private final NetworkTableEntry driveRawForwardEntry;
  private final NetworkTableEntry driveRawStrafeEntry;
  private final NetworkTableEntry driveCommandedForwardEntry;
  private final NetworkTableEntry driveCommandedStrafeEntry;
  private final NetworkTableEntry driveRotateEntry;
  private final NetworkTableEntry driveSpeedScaleEntry;
  private final NetworkTableEntry yawZeroEntry;
  private final NetworkTableEntry yawResetPendingEntry;

  private double driveSpeedFraction = 0.0;
  private boolean fieldOrientedEnabled = DEFAULT_FIELD_ORIENTED;
  private double yawZeroDegrees = 0.0;
  private boolean yawResetPending = true;
  private double yawResetFlashStartSeconds = Double.NEGATIVE_INFINITY;
  private double driveDesiredDegrees = Double.NaN;
  private double driveDesiredMagnitude = 0.0;
  private double driveHeadingErrorDegrees = Double.NaN;
  private double driveRawForward = 0.0;
  private double driveRawStrafe = 0.0;
  private double driveCommandedForward = 0.0;
  private double driveCommandedStrafe = 0.0;
  private double driveRotate = 0.0;
  private double driveSpeedScale = 0.0;

  public DriveSubsystem(WitMotionSubsystem witMotionSubsystem) {
    this.witMotionSubsystem = Objects.requireNonNull(witMotionSubsystem, "witMotionSubsystem");

    configureServoHub();
    stopDrive();
    setDriveEnabled(false);

    NetworkTable rootTable = NetworkTableInstance.getDefault().getTable("IMDemo");
    NetworkTable servoHubTable = rootTable.getSubTable("ServoHub");
    NetworkTable driveDebugTable = rootTable.getSubTable("DriveDebug");

    servoHubVoltageEntry = servoHubTable.getEntry("DeviceVoltage");
    servoHubVoltageValidEntry = servoHubTable.getEntry("DeviceVoltageValid");
    driveModeEntry = driveDebugTable.getEntry("DriveMode");
    fieldOrientedEntry = driveDebugTable.getEntry("FieldOrientedEnabled");
    driveHeadingEntry = driveDebugTable.getEntry("DriveHeadingDegrees");
    driveDesiredDegreesEntry = driveDebugTable.getEntry("DriveDesiredDegrees");
    driveDesiredMagnitudeEntry = driveDebugTable.getEntry("DriveDesiredMagnitude");
    driveHeadingErrorEntry = driveDebugTable.getEntry("DriveHeadingErrorDegrees");
    driveRawForwardEntry = driveDebugTable.getEntry("DriveRawForward");
    driveRawStrafeEntry = driveDebugTable.getEntry("DriveRawStrafe");
    driveCommandedForwardEntry = driveDebugTable.getEntry("DriveCommandedForward");
    driveCommandedStrafeEntry = driveDebugTable.getEntry("DriveCommandedStrafe");
    driveRotateEntry = driveDebugTable.getEntry("DriveRotate");
    driveSpeedScaleEntry = driveDebugTable.getEntry("DriveSpeedScale");
    yawZeroEntry = driveDebugTable.getEntry("DriveYawZeroDegrees");
    yawResetPendingEntry = driveDebugTable.getEntry("DriveYawResetPending");

    servoHubVoltageEntry.setDouble(-1.0);
    servoHubVoltageValidEntry.setBoolean(false);
    driveModeEntry.setString(getDriveModeLabel());
    fieldOrientedEntry.setBoolean(fieldOrientedEnabled);
    driveHeadingEntry.setDouble(0.0);
    driveDesiredDegreesEntry.setDouble(Double.NaN);
    driveDesiredMagnitudeEntry.setDouble(0.0);
    driveHeadingErrorEntry.setDouble(Double.NaN);
    driveRawForwardEntry.setDouble(0.0);
    driveRawStrafeEntry.setDouble(0.0);
    driveCommandedForwardEntry.setDouble(0.0);
    driveCommandedStrafeEntry.setDouble(0.0);
    driveRotateEntry.setDouble(0.0);
    driveSpeedScaleEntry.setDouble(0.0);
    yawZeroEntry.setDouble(yawZeroDegrees);
    yawResetPendingEntry.setBoolean(yawResetPending);
  }

  public void onAutonomousInit() {
    stopDrive();
    setDriveEnabled(false);
    resetDriveDebugState();
  }

  public void onTeleopInit() {
    setDriveEnabled(true);
    stopDrive();
    resetDriveDebugState();
    requestYawReset("Teleop enabled");
  }

  public void onDisabledInit() {
    stopDrive();
    setDriveEnabled(false);
    resetDriveDebugState();
  }

  public void onUtilityInit() {
    stopDrive();
    setDriveEnabled(false);
    resetDriveDebugState();
  }

  public void driveFromController(
      double leftY, double leftX, double rightX, double rightTriggerAxis) {
    if (!RobotState.isTeleopEnabled()) {
      stopDrive();
      return;
    }

    ScWitMotionDriver.Sample imuSample = witMotionSubsystem.getLatestSample();
    updateYawReferenceIfPending(imuSample);

    double forwardInput = -applyDeadband(leftY, DRIVE_DEADBAND);
    double strafeInput = applyDeadband(leftX, DRIVE_DEADBAND);
    double rotate = applyDeadband(rightX, DRIVE_DEADBAND);
    double speedScale =
        MIN_SPEED_SCALE + (1.0 - MIN_SPEED_SCALE) * clamp(rightTriggerAxis, 0.0, 1.0);

    driveRawForward = forwardInput;
    driveRawStrafe = strafeInput;
    driveRotate = rotate;
    driveSpeedScale = speedScale;
    driveDesiredMagnitude = Math.min(1.0, Math.hypot(strafeInput, forwardInput));
    if (driveDesiredMagnitude > 1e-6) {
      driveDesiredDegrees = Math.toDegrees(Math.atan2(strafeInput, forwardInput));
    } else {
      driveDesiredDegrees = Double.NaN;
    }

    double forward = forwardInput;
    double strafe = strafeInput;
    if (shouldUseFieldOriented(imuSample)) {
      double headingRadians = Math.toRadians(getDriveHeadingDegrees(imuSample) * FIELD_YAW_SIGN);
      double cos = Math.cos(headingRadians);
      double sin = Math.sin(headingRadians);
      forward = forwardInput * cos + strafeInput * sin;
      strafe = -forwardInput * sin + strafeInput * cos;
    }

    driveCommandedForward = forward;
    driveCommandedStrafe = strafe;
    if (shouldUseFieldOriented(imuSample) && !Double.isNaN(driveDesiredDegrees)) {
      driveHeadingErrorDegrees =
          normalizeDegrees(driveDesiredDegrees - getDriveHeadingDegrees(imuSample));
    } else {
      driveHeadingErrorDegrees = Double.NaN;
    }

    double frontLeft = forward + strafe + rotate;
    double frontRight = forward - strafe - rotate;
    double rearRight = forward + strafe - rotate;
    double rearLeft = forward - strafe + rotate;

    double maxMagnitude =
        Math.max(
            1.0,
            Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(rearRight), Math.abs(rearLeft))));

    frontLeft /= maxMagnitude;
    frontRight /= maxMagnitude;
    rearRight /= maxMagnitude;
    rearLeft /= maxMagnitude;

    frontLeft *= speedScale;
    frontRight *= speedScale;
    rearRight *= speedScale;
    rearLeft *= speedScale;

    setDriveOutputs(frontLeft, frontRight, rearRight, rearLeft);
  }

  public void requestYawReset(String reason) {
    yawResetPending = true;
    System.out.println("[Robot] Yaw reset requested: " + reason + ".");
    updateYawReferenceIfPending(witMotionSubsystem.getLatestSample());
  }

  public void toggleDriveMode() {
    fieldOrientedEnabled = !fieldOrientedEnabled;
    requestYawReset("Drive mode switched to " + getDriveModeLabel());
    System.out.println("[Robot] Drive mode is now " + getDriveModeLabel() + ".");
  }

  public boolean isFieldOrientedEnabled() {
    return fieldOrientedEnabled;
  }

  public double getDriveSpeedFraction() {
    return driveSpeedFraction;
  }

  public boolean isYawResetFlashOn() {
    double elapsedSeconds = Timer.getTimestamp() - yawResetFlashStartSeconds;
    if (elapsedSeconds < 0.0) {
      return false;
    }

    double totalFlashSeconds = YAW_RESET_FLASH_PHASE_SECONDS * YAW_RESET_FLASH_PHASE_COUNT;
    if (elapsedSeconds >= totalFlashSeconds) {
      return false;
    }

    int phase = (int) (elapsedSeconds / YAW_RESET_FLASH_PHASE_SECONDS);
    return phase % 2 == 0;
  }

  @Override
  public void periodic() {
    updateServoHubTelemetry();
    updateDriveDebugTelemetry(witMotionSubsystem.getLatestSample());
  }

  private void configureServoHub() {
    ServoHubConfig config = new ServoHubConfig();
    config.channel0
        .pulseRange(PWM_MIN_US, PWM_NEUTRAL_US, PWM_MAX_US)
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower);
    config.channel1
        .pulseRange(PWM_MIN_US, PWM_NEUTRAL_US, PWM_MAX_US)
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower);
    config.channel2
        .pulseRange(PWM_MIN_US, PWM_NEUTRAL_US, PWM_MAX_US)
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower);
    config.channel3
        .pulseRange(PWM_MIN_US, PWM_NEUTRAL_US, PWM_MAX_US)
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower);

    servoHub.configure(config, ResetMode.kResetSafeParameters);
    servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, PWM_PERIOD_US);
    servoHub.setBankPulsePeriod(ServoHub.Bank.kBank3_5, PWM_PERIOD_US);

    frontLeftChannel.setPowered(true);
    frontRightChannel.setPowered(true);
    rearRightChannel.setPowered(true);
    rearLeftChannel.setPowered(true);
  }

  private void setDriveEnabled(boolean enabled) {
    frontLeftChannel.setEnabled(enabled);
    frontRightChannel.setEnabled(enabled);
    rearRightChannel.setEnabled(enabled);
    rearLeftChannel.setEnabled(enabled);
  }

  private void stopDrive() {
    setDriveOutputs(0.0, 0.0, 0.0, 0.0);
  }

  private void setDriveOutputs(
      double frontLeft, double frontRight, double rearRight, double rearLeft) {
    driveSpeedFraction =
        Math.max(
            Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
            Math.max(Math.abs(rearRight), Math.abs(rearLeft)));

    setWheelOutput(frontLeftChannel, frontLeft, FRONT_LEFT_INVERTED);
    setWheelOutput(frontRightChannel, frontRight, FRONT_RIGHT_INVERTED);
    setWheelOutput(rearRightChannel, rearRight, REAR_RIGHT_INVERTED);
    setWheelOutput(rearLeftChannel, rearLeft, REAR_LEFT_INVERTED);
  }

  private void setWheelOutput(ServoChannel channel, double output, boolean inverted) {
    double adjustedOutput = clamp(inverted ? -output : output, -1.0, 1.0);
    int pulseWidthUs = PWM_NEUTRAL_US + (int) Math.round(adjustedOutput * PWM_TRAVEL_US);
    channel.setPulseWidth(clamp(pulseWidthUs, PWM_MIN_US, PWM_MAX_US));
  }

  private void updateServoHubTelemetry() {
    Signal<Double> deviceVoltageSignal = servoHub.getDeviceVoltage();
    servoHubVoltageEntry.setDouble(deviceVoltageSignal.get(-1.0));
    servoHubVoltageValidEntry.setBoolean(deviceVoltageSignal.isValid());
  }

  private void updateDriveDebugTelemetry(ScWitMotionDriver.Sample sample) {
    driveModeEntry.setString(getDriveModeLabel());
    fieldOrientedEntry.setBoolean(fieldOrientedEnabled);
    driveHeadingEntry.setDouble(getDriveHeadingDegrees(sample));
    driveDesiredDegreesEntry.setDouble(driveDesiredDegrees);
    driveDesiredMagnitudeEntry.setDouble(driveDesiredMagnitude);
    driveHeadingErrorEntry.setDouble(driveHeadingErrorDegrees);
    driveRawForwardEntry.setDouble(driveRawForward);
    driveRawStrafeEntry.setDouble(driveRawStrafe);
    driveCommandedForwardEntry.setDouble(driveCommandedForward);
    driveCommandedStrafeEntry.setDouble(driveCommandedStrafe);
    driveRotateEntry.setDouble(driveRotate);
    driveSpeedScaleEntry.setDouble(driveSpeedScale);
    yawZeroEntry.setDouble(yawZeroDegrees);
    yawResetPendingEntry.setBoolean(yawResetPending);
  }

  private boolean shouldUseFieldOriented(ScWitMotionDriver.Sample sample) {
    return fieldOrientedEnabled && sample.isFresh();
  }

  private double getDriveHeadingDegrees(ScWitMotionDriver.Sample sample) {
    if (!sample.isFresh()) {
      return 0.0;
    }
    return normalizeDegrees(sample.getYawDegrees() - yawZeroDegrees);
  }

  private void updateYawReferenceIfPending(ScWitMotionDriver.Sample sample) {
    if (!yawResetPending || !sample.isFresh()) {
      return;
    }

    yawZeroDegrees = sample.getYawDegrees();
    yawResetPending = false;
    yawResetFlashStartSeconds = Timer.getTimestamp();
    System.out.println("[Robot] Yaw reference set to " + yawZeroDegrees + " degrees.");
  }

  private String getDriveModeLabel() {
    return fieldOrientedEnabled ? "FieldOriented" : "JoystickDirect";
  }

  private void resetDriveDebugState() {
    driveDesiredDegrees = Double.NaN;
    driveDesiredMagnitude = 0.0;
    driveHeadingErrorDegrees = Double.NaN;
    driveRawForward = 0.0;
    driveRawStrafe = 0.0;
    driveCommandedForward = 0.0;
    driveCommandedStrafe = 0.0;
    driveRotate = 0.0;
    driveSpeedScale = 0.0;
  }

  private static double applyDeadband(double value, double deadband) {
    return Math.abs(value) > deadband ? value : 0.0;
  }

  private static double normalizeDegrees(double degrees) {
    double normalized = degrees % 360.0;
    if (normalized > 180.0) {
      normalized -= 360.0;
    }
    if (normalized <= -180.0) {
      normalized += 360.0;
    }
    return normalized;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }
}
