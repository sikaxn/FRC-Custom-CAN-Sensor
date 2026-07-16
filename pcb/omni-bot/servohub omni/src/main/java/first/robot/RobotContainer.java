package first.robot;

import first.robot.subsystems.DriveSubsystem;
import first.robot.subsystems.LightingSubsystem;
import first.robot.subsystems.WitMotionSubsystem;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.vision.camera.UsbCamera;
import org.wpilib.vision.stream.CameraServer;

public class RobotContainer {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int CAMERA_WIDTH = 1920;
  private static final int CAMERA_HEIGHT = 1080;
  private static final int CAMERA_FPS = 30;

  private final CommandGamepad driverController = new CommandGamepad(DRIVER_CONTROLLER_PORT);
  private final WitMotionSubsystem witMotionSubsystem = new WitMotionSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(witMotionSubsystem);
  private final LightingSubsystem lightingSubsystem =
      new LightingSubsystem(driveSubsystem, witMotionSubsystem);

  private UsbCamera driverCamera;

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
    startUsbCamera();
  }

  public void onAutonomousInit() {
    driveSubsystem.onAutonomousInit();
  }

  public void onTeleopInit() {
    driveSubsystem.onTeleopInit();
  }

  public void onDisabledInit() {
    driveSubsystem.onDisabledInit();
  }

  public void onUtilityInit() {
    driveSubsystem.onUtilityInit();
  }

  private void configureBindings() {
    driverController
        .leftBumper()
        .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.requestYawReset("Left bumper")));
    driverController.rightBumper().onTrue(driveSubsystem.runOnce(driveSubsystem::toggleDriveMode));
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.run(
            () ->
                driveSubsystem.driveFromController(
                    driverController.getLeftY(),
                    driverController.getLeftX(),
                    driverController.getRightX(),
                    driverController.getRightTriggerAxis())));
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
}
