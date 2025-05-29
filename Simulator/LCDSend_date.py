import can
import psutil
import time
import datetime
import random

# === LCD CAN Constants ===
CMD_CLEAR = 0x01
CMD_WRITE = 0x03
CMD_WRITE_NEXT = 0x04
CMD_ACK = 0xAA

DEVICE_ID = 0x0A
MANUFACTURER_ID = 0x08
API_ID = 0x187
ACK_API_ID = 0x188
DEVICE_NUMBER = 33

CAN_ID = ((DEVICE_ID << 24) | (MANUFACTURER_ID << 16) | (API_ID << 6) | DEVICE_NUMBER)
ACK_ID = ((DEVICE_ID << 24) | (MANUFACTURER_ID << 16) | (ACK_API_ID << 6) | DEVICE_NUMBER)

bus = can.interface.Bus(channel=0, bustype='canalystii', bitrate=1000000)

def wait_for_ack(tid, timeout=0.2):
    start = time.time()
    while time.time() - start < timeout:
        msg = bus.recv(timeout)
        if msg and msg.arbitration_id == ACK_ID and msg.data[0] == CMD_ACK and msg.data[1] == tid:
            return True
    return False

def send_lcd_line(row, col, text):
    tid = random.randint(1, 254)
    data = text.encode("ascii", errors="ignore")
    chunks = [data[i:i+3] for i in range(0, len(data), 3)]

    for attempt in range(3):
        for i, chunk in enumerate(chunks):
            isFinal = 1 if i == len(chunks) - 1 else 0
            payload = list(chunk)
            if len(payload) < 3 and isFinal:
                payload += [32] * (3 - len(payload))
            else:
                while len(payload) < 3:
                    payload.append(0)
            cmd = CMD_WRITE if i == 0 else CMD_WRITE_NEXT
            frame_col = col if i == 0 else 0
            frame = [cmd, isFinal, row, frame_col] + payload + [tid]
            msg = can.Message(arbitration_id=CAN_ID, data=frame, is_extended_id=True)
            bus.send(msg)
            time.sleep(0.01)

        if wait_for_ack(tid):
            return True
        time.sleep(0.05)
    return False

def send_clear():
    frame = [CMD_CLEAR, 0, 0, 0, 0, 0, 0, random.randint(1, 254)]
    msg = can.Message(arbitration_id=CAN_ID, data=frame, is_extended_id=True)
    bus.send(msg)
    time.sleep(0.1)

# === Main Loop ===
send_clear()
while True:
    now = datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")
    cpu = psutil.cpu_percent()
    ram = psutil.virtual_memory().percent
    usage_line = f"CPU:{cpu:>3.0f}% RAM:{ram:>3.0f}%"

    send_lcd_line(0, 0, now[:20])
    send_lcd_line(1, 0, usage_line[:20])
    time.sleep(1)
