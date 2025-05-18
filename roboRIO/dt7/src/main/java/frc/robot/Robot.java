package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArduinoCAN;

public class Robot extends TimedRobot {
    private ArduinoCAN arduino;

    // Standard DJI keyboard layout for DBUS
    private static final String[] KEYBOARD_KEYS = {
        "W", "S", "A", "D", "Shift", "Ctrl",
        "Q", "E", "R", "F", "Z", "X", "C", "V", "B", "G"
    };

    @Override
    public void robotInit() {
        arduino = new ArduinoCAN();

        // Initialize dashboard entries for control
        SmartDashboard.putBoolean("LED 0 State", false);
        SmartDashboard.putBoolean("LED 1 State", false);
        SmartDashboard.putNumber("Servo Angle", 90);
    }

    @Override
    public void robotPeriodic() {
        // Read dashboard values
        boolean led0 = SmartDashboard.getBoolean("LED 0 State", false);
        boolean led1 = SmartDashboard.getBoolean("LED 1 State", false);
        int servo = (int) SmartDashboard.getNumber("Servo Angle", 90);
        arduino.sendControl(led0, led1, servo);

        // Status feedback
        SmartDashboard.putBoolean("DT7 Connected", arduino.connected);

        // Button and analog status
        SmartDashboard.putBoolean("Button 0", arduino.buttons[0]);
        SmartDashboard.putBoolean("Button 1", arduino.buttons[1]);
        SmartDashboard.putBoolean("Button 2", arduino.buttons[2]);
        SmartDashboard.putNumber("Analog Value", arduino.analogValue);

        // Color sensor
        SmartDashboard.putBoolean("Color Sensor Connected", arduino.colorSensorConnected);
        SmartDashboard.putNumber("Color Red", arduino.red);
        SmartDashboard.putNumber("Color Green", arduino.green);
        SmartDashboard.putNumber("Color Blue", arduino.blue);
        SmartDashboard.putNumber("Color Proximity", arduino.proximity);
        SmartDashboard.putNumber("Color IR", arduino.ir);

        // DT7 stick and switches
        SmartDashboard.putNumber("DT7 ch0", arduino.ch0);
        SmartDashboard.putNumber("DT7 ch1", arduino.ch1);
        SmartDashboard.putNumber("DT7 ch2", arduino.ch2);
        SmartDashboard.putNumber("DT7 ch3", arduino.ch3);
        SmartDashboard.putNumber("DT7 s1", arduino.s1);
        SmartDashboard.putNumber("DT7 s2", arduino.s2);

        // DT7 mouse
        SmartDashboard.putNumber("DT7 mouseX", arduino.mouseX);
        SmartDashboard.putNumber("DT7 mouseY", arduino.mouseY);
        SmartDashboard.putNumber("DT7 mouseZ", arduino.mouseZ);
        SmartDashboard.putBoolean("DT7 Mouse Left", arduino.mouseLeft);
        SmartDashboard.putBoolean("DT7 Mouse Right", arduino.mouseRight);

        // DT7 keyboard
        for (int i = 0; i < arduino.keyFlags.length && i < KEYBOARD_KEYS.length; i++) {
            SmartDashboard.putBoolean("DT7 Key " + KEYBOARD_KEYS[i], arduino.keyFlags[i]);
        }
    }
}
