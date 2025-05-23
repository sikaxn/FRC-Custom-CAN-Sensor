package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArduinoCAN;

public class Robot extends TimedRobot {
    private ArduinoCAN arduino;

    @Override
    public void robotInit() {
        arduino = new ArduinoCAN();

        SmartDashboard.putBoolean("LED 0 State", false);
        SmartDashboard.putBoolean("LED 1 State", false);
        SmartDashboard.putNumber("Servo Angle", 90);
    }

    @Override
    public void robotPeriodic() {
        boolean led0 = SmartDashboard.getBoolean("LED 0 State", false);
        boolean led1 = SmartDashboard.getBoolean("LED 1 State", false);
        int servo = (int) SmartDashboard.getNumber("Servo Angle", 90);

        arduino.sendControl(led0, led1, servo);

        SmartDashboard.putBoolean("Button 0", arduino.buttons[0]);
        SmartDashboard.putBoolean("Button 1", arduino.buttons[1]);
        SmartDashboard.putBoolean("Button 2", arduino.buttons[2]);
        SmartDashboard.putNumber("Analog Value", arduino.analogValue);

        // === Color Sensor Values ===
        SmartDashboard.putBoolean("Color Sensor Connected", arduino.colorSensorConnected);
        //if (arduino.colorSensorConnected) {
            SmartDashboard.putNumber("Color Red", arduino.red);
            SmartDashboard.putNumber("Color Green", arduino.green);
            SmartDashboard.putNumber("Color Blue", arduino.blue);
            SmartDashboard.putNumber("Color Proximity", arduino.proximity);
            SmartDashboard.putNumber("Color IR", arduino.ir);
        //}
    }

    @Override
    public void disabledExit() {
        // arduino.stop(); // optional
    }
}
