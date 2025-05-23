import can
import tkinter as tk
from tkinter import ttk, StringVar, BooleanVar
from can.notifier import Notifier, Listener

# CAN Settings
CAN_CHANNEL = 0
CAN_BITRATE = 1000000

# Default CAN ID settings
DEFAULT_DEVICE_TYPE = "0x0A"
DEFAULT_MANUFACTURER_ID = "0x08"
DEFAULT_STATUS_API_ID = "0x180"
DEFAULT_CONTROL_API_ID = "0x190"
DEFAULT_DEVICE_NUMBER = 0

def decode_id(msg_id):
    device_type     = (msg_id >> 24) & 0xFF
    manufacturer_id = (msg_id >> 16) & 0xFF
    api_id          = (msg_id >> 6) & 0x3FF
    device_number   = msg_id & 0x3F
    return device_type, manufacturer_id, api_id, device_number

def encode_id(device_type, manufacturer_id, api_id, device_number):
    return ((device_type & 0xFF) << 24) | ((manufacturer_id & 0xFF) << 16) | ((api_id & 0x3FF) << 6) | (device_number & 0x3F)

class ArduinoListener(Listener):
    def __init__(self, app):
        self.app = app

    def on_message_received(self, msg):
        if msg.arbitration_id == self.app.control_id_val and msg.is_extended_id and len(msg.data) >= 2:
            led_byte = msg.data[0]
            angle = msg.data[1]
            self.app.update_arduino_display(led_byte, angle)

class ArduinoGUI:
    def __init__(self, root):
        self.root = root
        root.title("Arduino CAN Monitor & Simulator")

        self.device_type = tk.StringVar(value=DEFAULT_DEVICE_TYPE)
        self.manufacturer_id = tk.StringVar(value=DEFAULT_MANUFACTURER_ID)
        self.device_number = tk.IntVar(value=DEFAULT_DEVICE_NUMBER)
        self.api_id_status = tk.StringVar(value=DEFAULT_STATUS_API_ID)
        self.api_id_control = tk.StringVar(value=DEFAULT_CONTROL_API_ID)
        self.status_id = StringVar()
        self.control_id = StringVar()

        self.led0 = StringVar(value="--")
        self.led1 = StringVar(value="--")
        self.angle = tk.IntVar(value=-1)

        self.button0 = BooleanVar(value=False)
        self.button1 = BooleanVar(value=False)
        self.button2 = BooleanVar(value=False)
        self.analog = tk.IntVar(value=512)

        self.build_ui()

        self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='canalystii', bitrate=CAN_BITRATE)
        self.notifier = Notifier(self.bus, [ArduinoListener(self)])

        self.update_ids()
        self.send_loop()

    def build_ui(self):
        frame_cfg = ttk.LabelFrame(self.root, text="CAN ID Configuration")
        frame_cfg.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(frame_cfg, text="Device Type (hex):").grid(row=0, column=0)
        ttk.Entry(frame_cfg, textvariable=self.device_type, width=6).grid(row=0, column=1)

        ttk.Label(frame_cfg, text="Manufacturer ID (hex):").grid(row=1, column=0)
        ttk.Entry(frame_cfg, textvariable=self.manufacturer_id, width=6).grid(row=1, column=1)

        ttk.Label(frame_cfg, text="Device Number (0-63):").grid(row=2, column=0)
        ttk.Entry(frame_cfg, textvariable=self.device_number, width=6).grid(row=2, column=1)

        ttk.Label(frame_cfg, text="Status API ID (hex):").grid(row=3, column=0)
        ttk.Entry(frame_cfg, textvariable=self.api_id_status, width=6).grid(row=3, column=1)

        ttk.Label(frame_cfg, text="Control API ID (hex):").grid(row=4, column=0)
        ttk.Entry(frame_cfg, textvariable=self.api_id_control, width=6).grid(row=4, column=1)

        ttk.Button(frame_cfg, text="Update IDs", command=self.update_ids).grid(row=5, column=0, columnspan=2)
        ttk.Label(frame_cfg, textvariable=self.status_id, foreground="blue").grid(row=6, column=0, columnspan=2, sticky="w")
        ttk.Label(frame_cfg, textvariable=self.control_id, foreground="green").grid(row=7, column=0, columnspan=2, sticky="w")

        frame_rx = ttk.LabelFrame(self.root, text="RX: roboRIO Control Frame")
        frame_rx.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(frame_rx, text="LED 0:").grid(row=0, column=0)
        ttk.Label(frame_rx, textvariable=self.led0).grid(row=0, column=1)
        ttk.Label(frame_rx, text="LED 1:").grid(row=1, column=0)
        ttk.Label(frame_rx, textvariable=self.led1).grid(row=1, column=1)
        ttk.Label(frame_rx, text="Servo Angle:").grid(row=2, column=0)
        ttk.Label(frame_rx, textvariable=self.angle).grid(row=2, column=1)

        frame_tx = ttk.LabelFrame(self.root, text="TX: Arduino Status Frame")
        frame_tx.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        ttk.Checkbutton(frame_tx, text="Button 0", variable=self.button0).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(frame_tx, text="Button 1", variable=self.button1).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(frame_tx, text="Button 2", variable=self.button2).grid(row=2, column=0, sticky="w")
        ttk.Label(frame_tx, text="Analog:").grid(row=3, column=0, sticky="w")
        tk.Scale(frame_tx, from_=0, to=1023, orient="horizontal", variable=self.analog, length=200).grid(row=3, column=1)

    def update_ids(self):
        try:
            dt = int(self.device_type.get(), 0)
            man = int(self.manufacturer_id.get(), 0)
            dn = self.device_number.get()
            api_status = int(self.api_id_status.get(), 0)
            api_control = int(self.api_id_control.get(), 0)
        except ValueError as e:
            self.status_id.set("Invalid Input")
            self.control_id.set(str(e))
            return

        self.status_id_val = encode_id(dt, man, api_status, dn)
        self.control_id_val = encode_id(dt, man, api_control, dn)

        self.status_id.set(f"STATUS ID:  0x{self.status_id_val:08X}")
        self.control_id.set(f"CONTROL ID: 0x{self.control_id_val:08X}")

    def update_arduino_display(self, led_byte, angle):
        self.led0.set("ON" if led_byte & 0x01 else "OFF")
        self.led1.set("ON" if led_byte & 0x02 else "OFF")
        self.angle.set(angle)

    def send_loop(self):
        btn_mask = (
            (1 if self.button0.get() else 0) |
            (2 if self.button1.get() else 0) |
            (4 if self.button2.get() else 0)
        )
        analog_val = self.analog.get()
        data = [
            btn_mask,
            (analog_val >> 8) & 0xFF,
            analog_val & 0xFF
        ] + [0] * 5

        msg = can.Message(arbitration_id=self.status_id_val, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"[CAN] Send error: {e}")

        self.root.after(100, self.send_loop)

    def close(self):
        self.notifier.stop()
        self.bus.shutdown()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
