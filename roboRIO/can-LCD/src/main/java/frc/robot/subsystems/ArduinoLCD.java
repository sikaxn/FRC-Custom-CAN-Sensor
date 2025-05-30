package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import java.nio.charset.StandardCharsets;
import java.util.LinkedList;
import java.util.Queue;

public class ArduinoLCD {
    private final CAN can;
    private final int lcdApiId = 0x187;
    private final int deviceNumber;
    private final Queue<byte[]> messageQueue = new LinkedList<>();

    private String pendingText = null;
    private boolean pendingClear = false;
    private int delayCounter = 0;

    public ArduinoLCD(int deviceNumber) {
        this.deviceNumber = deviceNumber;
        this.can = new CAN(deviceNumber);
    }

    public void clearScreen() {
        byte[] data = new byte[8];
        data[0] = 0x01; // CMD_CLEAR
        enqueue(data);
    }

    public void setCursor(int row, int col) {
        byte[] data = new byte[8];
        data[0] = 0x02; // CMD_CURSOR
        data[1] = 0;
        data[2] = (byte) row;
        data[3] = (byte) col;
        enqueue(data);
    }

    public void send(String text, int row, int col) {
        // Remove manual newlines and pad the text to 80 characters (20 x 4)
        String cleaned = text.replace("\n", " ");
        for (int i = 0; i < 4; i++) {
            int start = i * 20;
            String line = start < cleaned.length() ? cleaned.substring(start, Math.min(start + 20, cleaned.length())) : "";
            queueLine(line, i, 0);
        }
    }
    

    public void queueLine(String line, int row, int col) {
        byte[] bytes = line.getBytes(StandardCharsets.US_ASCII);
        int index = 0;
        boolean first = true;

        while (index < bytes.length) {
            byte[] data = new byte[8];
            data[0] = (byte) (first ? 0x03 : 0x04);
            data[1] = (byte) ((index + 3 >= bytes.length) ? 1 : 0);
            data[2] = (byte) row;
            data[3] = (byte) (first ? col : 0);
            for (int i = 0; i < 3; i++) {
                if (index + i < bytes.length) {
                    data[4 + i] = bytes[index + i];
                } else {
                    data[4 + i] = 0x20;
                }
            }
            data[7] = (byte) (System.currentTimeMillis() & 0xFF);
            enqueue(data);
            index += 3;
            first = false;
        }
    }

    public void clearAndSend(String text, int row, int col) {
        pendingClear = true;
        pendingText = text;
        delayCounter = 5;
    }

    public void processQueue() {
        if (!messageQueue.isEmpty()) {
            byte[] frame = messageQueue.poll();
            sendPacket(frame);
            return;
        }

        if (pendingClear) {
            if (delayCounter == 5) {
                clearScreen();
            } else if (delayCounter == 0) {
                send(pendingText, 0, 0);
                pendingClear = false;
                pendingText = null;
            }
            delayCounter--;
        }
    }

    private void enqueue(byte[] data) {
        messageQueue.offer(data);
    }

    private void sendPacket(byte[] data) {
        try {
            can.writePacket(data, lcdApiId);
        } catch (Exception e) {
            System.err.println("LCD CAN send error: " + e.getMessage());
        }
    }
}
