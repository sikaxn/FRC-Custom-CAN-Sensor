package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ArduinoDJIMotor;

public class Robot extends TimedRobot {
    private ArduinoDJIMotor djiMotor;
    private XboxController controller;

    @Override
    public void robotInit() {
        djiMotor = new ArduinoDJIMotor();
        controller = new XboxController(0);
    }

    private double applyDeadband(double value, double threshold) {
        return (Math.abs(value) < threshold) ? 0.0 : value;
    }
    
    public void robotPeriodic() {
        double fwd = applyDeadband(-controller.getLeftY(), 0.15);
        double turn = applyDeadband(controller.getRightX(), 0.15);
    
        double left = fwd + turn;
        double right = fwd - turn;
    
        int leftCurrent = (int)(Math.max(-1.0, Math.min(1.0, -1 * left)) * 16384);
        int rightCurrent = (int)(Math.max(-1.0, Math.min(1.0, right)) * 16384);
    
        djiMotor.sendMotorCurrents(leftCurrent, rightCurrent);
    }
    
}
