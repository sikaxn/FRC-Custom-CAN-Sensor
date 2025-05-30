package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.ArduinoLCD;

public class Robot extends TimedRobot {

    private ArduinoLCD lcd;
    private double lastVoltage = -1;

    @Override
    public void robotInit() {
        lcd = new ArduinoLCD(33); // LCD device number
    }

    @Override
    public void robotPeriodic() {
        lcd.processQueue();

        double voltage = RobotController.getBatteryVoltage();
        if (Math.abs(voltage - lastVoltage) > 0.01) { // update if significant change
            String text = String.format("Battery: %.2f V", voltage);
            lcd.send(String.format("%-20s", text), 0, 0);
            lastVoltage = voltage;
        }
    }
} 
