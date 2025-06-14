package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

import java.util.Timer;
import java.util.TimerTask;

public class ToFCAN {
    private static final int BASE_API_ID = 0x0301;
    private static final int SENSOR_COUNT = 4;

    private final CAN can;
    private final int[] distances = new int[SENSOR_COUNT];
    private final int[] modes = new int[SENSOR_COUNT];
    private final int[] rois = new int[SENSOR_COUNT];
    private final int[] timingBudgets = new int[SENSOR_COUNT];

    private final Timer pollTimer = new Timer("ToF-CAN-Poller", true);

    public ToFCAN() {
        this.can = new CAN(50);  // Use global access for API ID targeting

        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                for (int i = 0; i < SENSOR_COUNT; i++) {
                    int apiId = BASE_API_ID + i;
                    try {
                        CANData frame = new CANData();
                        if (can.readPacketLatest(apiId, frame)) {
                            byte[] data = frame.data;
                            if (frame.length >= 8) {
                                distances[i] = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
                                modes[i] = data[2] & 0xFF;
                                timingBudgets[i] = ((data[3] & 0xFF) << 24) | ((data[4] & 0xFF) << 16) |
                                                   ((data[5] & 0xFF) << 8) | (data[6] & 0xFF);
                                rois[i] = data[7] & 0xFF;
                            }
                        }
                    } catch (Exception e) {
                        System.err.println("ToFCAN read error on sensor " + i + ": " + e.getMessage());
                    }
                }
            }
        }, 0, 20); // poll every 20 ms
    }

    public int getDistance(int index) {
        return validIndex(index) ? distances[index] : -1;
    }

    public int getMode(int index) {
        return validIndex(index) ? modes[index] : -1;
    }

    public int getROI(int index) {
        return validIndex(index) ? rois[index] : -1;
    }

    public int getTimingBudget(int index) {
        return validIndex(index) ? timingBudgets[index] : -1;
    }

    private boolean validIndex(int i) {
        return i >= 0 && i < SENSOR_COUNT;
    }

    public void stop() {
        pollTimer.cancel();
    }
}
