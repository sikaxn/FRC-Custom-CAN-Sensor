import can
import time

CAN_CHANNEL = 0
CAN_DEVICE = 0
CAN_BITRATE = 1000000

DEVICE_ID = 0x0A
MFR_ID = 0x08
API_ID = 0x190  # LED control command

# Create FRC-style CAN ID
def make_can_id(device_number):
    return (DEVICE_ID << 24) | (MFR_ID << 16) | (API_ID << 6) | (device_number & 0x3F)

# Construct payload for mode 1, white, half-bright
def create_led_payload():
    mode = 1               # solid color
    r, g, b = 255, 255, 255  # white
    brightness = 128       # half brightness
    return [mode, r, g, b, brightness, 0, 0, 0]

def main():
    print("Sending LED color commands to device numbers 1–60 with 100ms delay")

    try:
        bus = can.Bus(interface='canalystii', channel=CAN_CHANNEL, device=CAN_DEVICE, bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"CAN init failed: {e}")
        return

    payload = create_led_payload()

    try:
        for device_num in range(1, 61):
            can_id = make_can_id(device_num)
            msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=True)
            try:
                bus.send(msg)
                print(f"[✓] Sent to device #{device_num}: CAN ID = 0x{can_id:X}")
            except can.CanError:
                print(f"[x] Failed to send to device #{device_num}")
            time.sleep(0.1)  # 100 ms delay
    finally:
        bus.shutdown()

if __name__ == "__main__":
    while(True):
        main()
