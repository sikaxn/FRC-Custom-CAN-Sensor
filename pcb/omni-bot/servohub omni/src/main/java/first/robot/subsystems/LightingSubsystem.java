package first.robot.subsystems;

import first.robot.drivers.ScWitMotionDriver;
import first.robot.drivers.addressableLEDCAN;
import java.util.Objects;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;

public class LightingSubsystem extends SubsystemBase {
  private static final int CAN_BUS_ID = 1;
  private static final int LED_DEVICE_ID = 33;
  private static final int LED_MIRROR_DEVICE_ID = 53;
  private static final int LED_MODE_SOLID = 1;
  private static final int LED_TOTAL_PIXELS = 30;
  private static final int LED_BRIGHTNESS = 128;
  private static final int LED_ON = 1;

  private final DriveSubsystem driveSubsystem;
  private final WitMotionSubsystem witMotionSubsystem;
  private final addressableLEDCAN leds = new addressableLEDCAN(LED_DEVICE_ID, CAN_BUS_ID);
  private final addressableLEDCAN mirrorLeds =
      new addressableLEDCAN(LED_MIRROR_DEVICE_ID, CAN_BUS_ID);

  public LightingSubsystem(DriveSubsystem driveSubsystem, WitMotionSubsystem witMotionSubsystem) {
    this.driveSubsystem = Objects.requireNonNull(driveSubsystem, "driveSubsystem");
    this.witMotionSubsystem = Objects.requireNonNull(witMotionSubsystem, "witMotionSubsystem");

    configureLedDevice(leds);
    configureLedDevice(mirrorLeds);
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      sendLedCommandToAll(LED_MODE_SOLID, 255, 255, 0, LED_BRIGHTNESS, LED_ON, 0, 0);
      return;
    }

    if (driveSubsystem.isYawResetFlashOn()) {
      sendLedCommandToAll(LED_MODE_SOLID, 255, 0, 0, LED_BRIGHTNESS, LED_ON, 0, 0);
      return;
    }

    ScWitMotionDriver.Sample imuSample = witMotionSubsystem.getLatestSample();
    if (driveSubsystem.isFieldOrientedEnabled()) {
      if (imuSample.isFresh()) {
        sendSpeedTintedStatusColor(0, 255, 0);
      } else {
        sendSpeedTintedStatusColor(255, 0, 0);
      }
      return;
    }

    sendSpeedTintedStatusColor(0, 96, 255);
  }

  private void configureLedDevice(addressableLEDCAN ledDevice) {
    ledDevice.setTotalPixel(LED_TOTAL_PIXELS);
    ledDevice.setSecondaryColor(0, 0, 0, 0, 0);
  }

  private void sendSpeedTintedStatusColor(int baseR, int baseG, int baseB) {
    double speed = clamp(driveSubsystem.getDriveSpeedFraction(), 0.0, 1.0);
    sendLedCommandToAll(
        LED_MODE_SOLID,
        blendWithWhite(baseR, speed),
        blendWithWhite(baseG, speed),
        blendWithWhite(baseB, speed),
        LED_BRIGHTNESS,
        LED_ON,
        0,
        0);
  }

  private void sendLedCommandToAll(
      int mode, int r, int g, int b, int brightness, int onOff, int param0, int param1) {
    leds.sendGeneralCommand(mode, r, g, b, brightness, onOff, param0, param1);
    mirrorLeds.sendGeneralCommand(mode, r, g, b, brightness, onOff, param0, param1);
  }

  private static int blendWithWhite(int baseChannel, double speed) {
    return clamp((int) Math.round(baseChannel + speed * (255.0 - baseChannel)), 0, 255);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }
}
