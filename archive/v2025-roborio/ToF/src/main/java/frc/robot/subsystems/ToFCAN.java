package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

public class ToFCAN {
    private static final int BASE_API_ID = 0x0301;
    private static final int CONFIG_API_ID = 0x0305;
    private static final int SENSOR_COUNT = 4;

    private final CAN can;
    private final int[] distances = new int[SENSOR_COUNT];
    private final int[] modes = new int[SENSOR_COUNT];
    private final int[] roiCenters = new int[SENSOR_COUNT];
    private final int[] roiXs = new int[SENSOR_COUNT];
    private final int[] roiYs = new int[SENSOR_COUNT];
    private final int[] timingBudgets = new int[SENSOR_COUNT];

    private final int[] desiredRoiCenters = {199, 199, 199, 199};
    private final int[] desiredRoiXs = {8, 8, 8, 8};
    private final int[] desiredRoiYs = {8, 8, 8, 8};
    private final int[] desiredModes = {2, 2, 2, 2}; // Long

    private final long[] lastConfigSent = new long[SENSOR_COUNT];

    private final Timer pollTimer = new Timer("ToF-CAN-Poller", true);

    public ToFCAN() {
        this.can = new CAN(50);  // Global CAN

        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                long now = System.currentTimeMillis();

                for (int i = 0; i < SENSOR_COUNT; i++) {
                    try {
                        int apiId = BASE_API_ID + i;
                        CANData frame = new CANData();

                        if (can.readPacketLatest(apiId, frame)) {
                            byte[] data = frame.data;
                            if (frame.length >= 8) {
                                distances[i]    = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
                                modes[i]        = data[2] & 0xFF;
                                roiCenters[i]   = data[3] & 0xFF;
                                roiXs[i]        = data[4] & 0xFF;
                                roiYs[i]        = data[5] & 0xFF;
                                timingBudgets[i] = ((data[6] & 0xFF) << 8) | (data[7] & 0xFF);
                            }
                        }

                        // Compare with desired and send if mismatch
                        if ((modes[i] != desiredModes[i] ||
                             roiCenters[i] != desiredRoiCenters[i] ||
                             roiXs[i] != desiredRoiXs[i] ||
                             roiYs[i] != desiredRoiYs[i]) &&
                             (now - lastConfigSent[i] > 200)) {

                            byte[] config = new byte[] {
                                (byte) desiredModes[i],
                                (byte) desiredRoiCenters[i],
                                (byte) desiredRoiXs[i],
                                (byte) desiredRoiYs[i]
                            };

                            can.writePacket(config, CONFIG_API_ID + i); 
                            lastConfigSent[i] = now;

                            System.out.printf("[ToFCAN] Sent config to Sensor %d | Mode: %d, ROI: %d %dx%d\n",
                                    i, desiredModes[i], desiredRoiCenters[i], desiredRoiXs[i], desiredRoiYs[i]);
                        }

                    } catch (Exception e) {
                        System.err.println("ToFCAN read/send error on sensor " + i + ": " + e.getMessage());
                    }
                }
            }
        }, 0, 20); // Poll every 20ms
    }

    public int getDistance(int index) {
        return validIndex(index) ? distances[index] : -1;
    }

    public int getMode(int index) {
        return validIndex(index) ? modes[index] : -1;
    }

    public int getROICenter(int index) {
        return validIndex(index) ? roiCenters[index] : -1;
    }

    public int getROISizeX(int index) {
        return validIndex(index) ? roiXs[index] : -1;
    }

    public int getROISizeY(int index) {
        return validIndex(index) ? roiYs[index] : -1;
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

    // Optional: Setters if you want dynamic config from UI or NetworkTable
    public void setDesiredConfig(int index, int mode, int center, int x, int y) {
        if (!validIndex(index)) return;
        desiredModes[index] = mode;
        desiredRoiCenters[index] = center;
        desiredRoiXs[index] = x;
        desiredRoiYs[index] = y;
    }
}
