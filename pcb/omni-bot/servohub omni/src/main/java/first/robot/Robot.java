package first.robot;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.util.Signal;
import first.robot.drivers.addressableLEDCAN;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.vision.camera.UsbCamera;
import org.wpilib.vision.stream.CameraServer;

public class Robot extends TimedRobot {
  private static final int CAN_BUS_ID = 1;
  private static final int LED_DEVICE_ID = 33;
  private static final int SERVO_HUB_DEVICE_ID = 3;
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int LED_MODE_SOLID = 1;
  private static final int LED_TOTAL_PIXELS = 30;
  private static final int LED_BRIGHTNESS = 128;
  private static final int LED_ON = 1;
  private static final int CAMERA_WIDTH = 1920 ;
  private static final int CAMERA_HEIGHT = 1080;
  private static final int CAMERA_FPS = 30;

  private static final double DRIVE_DEADBAND = 0.08;
  private static final double MIN_SPEED_SCALE = 0.5;
  private static final int PWM_MIN_US = 500;
  private static final int PWM_NEUTRAL_US = 1500;
  private static final int PWM_MAX_US = 2500;
  private static final int PWM_TRAVEL_US = (PWM_MAX_US - PWM_MIN_US) / 2;
  private static final int PWM_PERIOD_US = 20000;

  // Flip individual wheels here if a motor/controller is mounted opposite the others.
  private static final boolean FRONT_LEFT_INVERTED = false;
  private static final boolean FRONT_RIGHT_INVERTED = true;
  private static final boolean REAR_RIGHT_INVERTED = true;
  private static final boolean REAR_LEFT_INVERTED = false;

  private final Gamepad driverController = new Gamepad(DRIVER_CONTROLLER_PORT);
  private final ServoHub servoHub = new ServoHub(CAN_BUS_ID, SERVO_HUB_DEVICE_ID);
  private final ServoChannel frontLeftChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId0);
  private final ServoChannel frontRightChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId1);
  private final ServoChannel rearRightChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId2);
  private final ServoChannel rearLeftChannel =
      servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId3);

  private final addressableLEDCAN leds = new addressableLEDCAN(LED_DEVICE_ID, CAN_BUS_ID);
  private UsbCamera driverCamera;

  private final NetworkTableEntry servoHubVoltageEntry;
  private final NetworkTableEntry servoHubVoltageValidEntry;

  private double driveSpeedFraction = 0.0;

  public Robot() {
    configureServoHub();
    stopDrive();
    setDriveEnabled(false);
    leds.setTotalPixel(LED_TOTAL_PIXELS);
    leds.setSecondaryColor(0, 0, 0, 0, 0);
    startUsbCamera();

    NetworkTable rootTable = NetworkTableInstance.getDefault().getTable("IMDemo");
    NetworkTable servoHubTable = rootTable.getSubTable("ServoHub");

    servoHubVoltageEntry = servoHubTable.getEntry("DeviceVoltage");
    servoHubVoltageValidEntry = servoHubTable.getEntry("DeviceVoltageValid");

    servoHubVoltageEntry.setDouble(-1.0);
    servoHubVoltageValidEntry.setBoolean(false);
  }

  @Override
  public void robotPeriodic() {
    updateServoHubTelemetry();
    updateLedStatus();
  }

  @Override
  public void autonomousInit() {
    stopDrive();
    setDriveEnabled(false);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    setDriveEnabled(true);
    stopDrive();
  }

  @Override
  public void teleopPeriodic() {
    double forward = -applyDeadband(driverController.getLeftY(), DRIVE_DEADBAND);
    double strafe = applyDeadband(driverController.getLeftX(), DRIVE_DEADBAND);
    double rotate = applyDeadband(driverController.getRightX(), DRIVE_DEADBAND);
    double speedScale =
        MIN_SPEED_SCALE
            + (1.0 - MIN_SPEED_SCALE) * clamp(driverController.getRightTriggerAxis(), 0.0, 1.0);

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

  @Override
  public void disabledInit() {
    stopDrive();
    setDriveEnabled(false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void utilityInit() {
    stopDrive();
    setDriveEnabled(false);
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

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

  private void startUsbCamera() {
    try {
      driverCamera = CameraServer.startAutomaticCapture(0);
      driverCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
      driverCamera.setFPS(CAMERA_FPS);
      System.out.println(
          "[Robot] CameraServer started USB camera 0 at "
              + CAMERA_WIDTH
              + "x"
              + CAMERA_HEIGHT
              + " @ "
              + CAMERA_FPS
              + " FPS.");
    } catch (Exception e) {
      System.out.println("[Robot] Failed to start USB camera: " + e.getMessage());
    }
  }

  private void updateLedStatus() {
    if (isDisabled()) {
      leds.sendGeneralCommand(LED_MODE_SOLID, 255, 255, 0, LED_BRIGHTNESS, LED_ON, 0, 0);
      return;
    }

    int whiteAmount = 255 - (int) Math.round(clamp(driveSpeedFraction, 0.0, 1.0) * 255.0);
    leds.sendGeneralCommand(
        LED_MODE_SOLID, 255, whiteAmount, whiteAmount, LED_BRIGHTNESS, LED_ON, 0, 0);
  }

  private void updateServoHubTelemetry() {
    Signal<Double> deviceVoltageSignal = servoHub.getDeviceVoltage();
    servoHubVoltageEntry.setDouble(deviceVoltageSignal.get(-1.0));
    servoHubVoltageValidEntry.setBoolean(deviceVoltageSignal.isValid());
  }

  private static double applyDeadband(double value, double deadband) {
    return Math.abs(value) > deadband ? value : 0.0;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }
}
