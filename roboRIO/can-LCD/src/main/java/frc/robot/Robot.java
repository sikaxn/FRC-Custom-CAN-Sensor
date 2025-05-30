package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArduinoLCD;

public class Robot extends TimedRobot {

    private ArduinoLCD lcd;
    private String lastText = "";

    @Override
    public void robotInit() {
        lcd = new ArduinoLCD(33); // LCD device number
        SmartDashboard.putString("LCD Text", "Fennekin is so cute!");
        SmartDashboard.putBoolean("LCD Clear", false);
    }
    

    
    @Override
    public void robotPeriodic() {
        lcd.processQueue();
    
        // Handle clear request
        boolean clearRequested = SmartDashboard.getBoolean("LCD Clear", false);
        if (clearRequested) {
            lcd.clearScreen();
            SmartDashboard.putBoolean("LCD Clear", false); // reset button
        }
    
        // Handle text update
        String currentText = SmartDashboard.getString("LCD Text", "");
        if (!currentText.equals(lastText)) {
            lcd.clearAndSend(currentText, 0, 0);
            lastText = currentText;
        }
    }
    
    
}