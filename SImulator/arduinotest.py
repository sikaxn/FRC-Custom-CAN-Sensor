import can
import tkinter as tk
from tkinter import ttk, StringVar, IntVar, BooleanVar
from can.notifier import Notifier, Listener

# CAN Settings
CAN_CHANNEL = 0
CAN_BITRATE = 1000000

# CAN ID Constants
CONTROL_ID = 0x0A086400  # roboRIO -> Arduino
STATUS_ID = 0x0A086000   # Arduino -> roboRIO

class ArduinoListener(Listener):
    def __init__(self, app):
        self.app = app

    def on_message_received(self, msg):
        if msg.arbitration_id == CONTROL_ID and msg.is_extended_id and len(msg.data) >= 2:
            led_byte = msg.data[0]
            angle = msg.data[1]
            self.app.update_arduino_display(led_byte, angle)


class ArduinoGUI:
    def __init__(self, root):
        self.root = root
        root.title("Arduino CAN Monitor & Simulator")

        self.led0 = StringVar(value="--")
        self.led1 = StringVar(value="--")
        self.angle = IntVar(value=-1)

        self.button0 = BooleanVar(value=False)
        self.button1 = BooleanVar(value=False)
        self.button2 = BooleanVar(value=False)
        self.analog = IntVar(value=512)

        self.build_ui()

        self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='canalystii', bitrate=CAN_BITRATE)
        self.notifier = Notifier(self.bus, [ArduinoListener(self)])

        self.send_loop()

    def build_ui(self):
        frame_rx = ttk.LabelFrame(self.root, text="RX: roboRIO Control Frame (0x190)")
        frame_rx.grid(row=0, column=0, padx=10, pady=5)

        ttk.Label(frame_rx, text="LED 0:").grid(row=0, column=0)
        ttk.Label(frame_rx, textvariable=self.led0).grid(row=0, column=1)
        ttk.Label(frame_rx, text="LED 1:").grid(row=1, column=0)
        ttk.Label(frame_rx, textvariable=self.led1).grid(row=1, column=1)
        ttk.Label(frame_rx, text="Servo Angle:").grid(row=2, column=0)
        ttk.Label(frame_rx, textvariable=self.angle).grid(row=2, column=1)

        frame_tx = ttk.LabelFrame(self.root, text="TX: Arduino Status Frame (0x180)")
        frame_tx.grid(row=1, column=0, padx=10, pady=5)

        ttk.Checkbutton(frame_tx, text="Button 0", variable=self.button0).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(frame_tx, text="Button 1", variable=self.button1).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(frame_tx, text="Button 2", variable=self.button2).grid(row=2, column=0, sticky="w")

        ttk.Label(frame_tx, text="Analog:").grid(row=3, column=0, sticky="w")
        tk.Scale(frame_tx, from_=0, to=1023, orient="horizontal", variable=self.analog, length=200).grid(row=3, column=1)

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
        ] + [0] * 5  # pad to 8 bytes

        msg = can.Message(arbitration_id=STATUS_ID, data=data, is_extended_id=True)
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
