import can
import time
import colorsys

# —— CAN Constants ——
DEVICE_ID           = 0x0A
MANUFACTURER_ID     = 0x08
DEVICE_NUMBER       = 33
GENERAL_API         = 0x350
CUSTOM_PATTERN_API  = 0x351
ESP_FEEDBACK_API    = 0x359

bus = can.Bus(interface='canalystii', channel=0, device=0, bitrate=1_000_000)

# —— Helpers ——
def make_can_msg_id(device_id, manufacturer_id, api_id, device_number):
    return (
        (device_id       & 0xFF) << 24 |
        (manufacturer_id & 0xFF) << 16 |
        (api_id          & 0x3FF) << 6  |
        (device_number   & 0x3F)
    )

def send_mode_255():
    arb_id = make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, GENERAL_API, DEVICE_NUMBER)
    data = [255, 0, 0, 0, 128, 1, 0, 0]  # Mode 255, black, brightness=128, on
    msg = can.Message(arbitration_id=arb_id, is_extended_id=True, data=data)
    bus.send(msg)
    print("→ Switched to mode 255")

def send_pixel(index, r, g, b, brightness=128):
    api_index = (index % 8)
    arb_id = make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, CUSTOM_PATTERN_API + api_index, DEVICE_NUMBER)
    data = [
        (index >> 8) & 0xFF,
        index & 0xFF,
        r, g, b,
        0x00,         # W not used
        brightness,
        0
    ]
    msg = can.Message(arbitration_id=arb_id, is_extended_id=True, data=data)
    bus.send(msg)

# —— Main ——
def read_led_info():
    while True:
        msg = bus.recv(timeout=2)
        if msg and msg.arbitration_id == make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, ESP_FEEDBACK_API, DEVICE_NUMBER):
            num_leds = (msg.data[0] << 8) | msg.data[1]
            mode = msg.data[2]
            print(f"← Feedback: {num_leds} LEDs | Mode: {mode}")
            return num_leds, mode
        print("Waiting for 0x359 feedback...")

if __name__ == "__main__":
    num_leds, current_mode = read_led_info()
    if current_mode != 255:
        send_mode_255()
        time.sleep(0.2)

    hue = 0.0
    while True:
        for i in range(num_leds):
            h = (hue + i / num_leds) % 1.0
            r, g, b = [int(x * 255) for x in colorsys.hsv_to_rgb(h, 1, 1)]
            send_pixel(i, r, g, b, brightness=128)
        hue = (hue + 0.01) % 1.0
        time.sleep(0.03)  # 30ms between frames
