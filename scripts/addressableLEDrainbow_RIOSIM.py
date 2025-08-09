import can
import time

# —— Constants ——
DEVICE_ID           = 0x0A
MANUFACTURER_ID     = 0x08
DEVICE_NUMBER       = 8
GENERAL_API         = 0x350
CUSTOM_PATTERN_API  = 0x351
ESP_FEEDBACK_API    = 0x359

bus = can.Bus(interface='canalystii', channel=0, device=0, bitrate=1_000_000)

def make_can_msg_id(device_id, manufacturer_id, api_id, device_number):
    return (
        (device_id       & 0xFF) << 24 |
        (manufacturer_id & 0xFF) << 16 |
        (api_id          & 0x3FF) << 6 |
        (device_number   & 0x3F)
    )

def send_mode_255():
    arb_id = make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, GENERAL_API, DEVICE_NUMBER)
    data = [255, 0, 0, 0, 128, 1, 0, 0]  # mode 255, brightness 128, ON
    msg = can.Message(arbitration_id=arb_id, is_extended_id=True, data=data)
    bus.send(msg)
    print("→ Set mode 255")

def send_pixel_batch(batch, base_api=0x351):
    for i, (index, r, g, b) in enumerate(batch):
        api_id = base_api + i
        arb_id = make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, api_id, DEVICE_NUMBER)
        data = [
            (index >> 8) & 0xFF,
            index & 0xFF,
            r, g, b,
            0,      # W not used
            128,    # Brightness
            0
        ]
        msg = can.Message(arbitration_id=arb_id, is_extended_id=True, data=data)
        bus.send(msg)

def read_led_info():
    print("Waiting for 0x359 feedback...")
    while True:
        msg = bus.recv(timeout=2)
        if msg and msg.arbitration_id == make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, ESP_FEEDBACK_API, DEVICE_NUMBER):
            num_leds = (msg.data[0] << 8) | msg.data[1]
            mode = msg.data[2]
            print(f"← Feedback: {num_leds} LEDs | Mode: {mode}")
            return num_leds, mode

def color_wheel(i):
    i = i % 256
    if i < 85:
        return (255 - i * 3, i * 3, 0)
    elif i < 170:
        i -= 85
        return (0, 255 - i * 3, i * 3)
    else:
        i -= 170
        return (i * 3, 0, 255 - i * 3)

# —— Main Rainbow Loop ——
if __name__ == "__main__":
    num_leds, mode = read_led_info()
    if mode != 255:
        send_mode_255()
        time.sleep(0.2)

    rainbow_index = 0
    while True:
        for segment_start in range(0, num_leds, 8):
            batch = []
            for i in range(8):
                led_index = segment_start + i
                if led_index >= num_leds:
                    break
                r, g, b = color_wheel((rainbow_index + led_index * 4) % 256)
                batch.append((led_index, r, g, b))
            send_pixel_batch(batch)
        rainbow_index = (rainbow_index + 1) % 256
        time.sleep(0.02)
