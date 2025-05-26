package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DJIMotorGateway;
import frc.robot.subsystems.DJIMotorGateway.DJIMotor;

public class Robot extends TimedRobot {
    private DJIMotorGateway gateway;
    private DJIMotor leftMotor;
    private DJIMotor rightMotor;
    private DifferentialDrive drive;
    private XboxController controller;

    @Override
    public void robotInit() {
        gateway = new DJIMotorGateway(11); // CAN device ID for ESP32 gateway
        leftMotor = gateway.motor(1);  // Motor ID 1
        rightMotor = gateway.motor(4); // Motor ID 4
        rightMotor.setInverted(true); // Optional: invert right side if needed
        drive = new DifferentialDrive(leftMotor, rightMotor);
        controller = new XboxController(0);
    }

    private double applyDeadband(double value, double threshold) {
        return (Math.abs(value) < threshold) ? 0.0 : value;
    }

    @Override
    public void robotPeriodic() {
        double fwd = applyDeadband(-controller.getLeftY(), 0.15);
        double turn = applyDeadband(controller.getRightX(), 0.15);
        drive.arcadeDrive(fwd, turn);

        // Publish telemetry to SmartDashboard
        SmartDashboard.putNumber("Left Motor Angle", leftMotor.getRotorAngle());
        SmartDashboard.putNumber("Left Motor Speed", leftMotor.getRotorSpeed());
        SmartDashboard.putNumber("Left Motor Torque", leftMotor.getRotorTorque());
        SmartDashboard.putNumber("Left Motor Temp", leftMotor.getTemperature());

        SmartDashboard.putNumber("Right Motor Angle", rightMotor.getRotorAngle());
        SmartDashboard.putNumber("Right Motor Speed", rightMotor.getRotorSpeed());
        SmartDashboard.putNumber("Right Motor Torque", rightMotor.getRotorTorque());
        SmartDashboard.putNumber("Right Motor Temp", rightMotor.getTemperature());
    }
}
