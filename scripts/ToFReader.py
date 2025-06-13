import can
import tkinter as tk
from tkinter import ttk

# CAN Config
CAN_CHANNEL = 0
CAN_BITRATE = 1000000
BUS_TYPE = "canalystii"

# CAN ID constants
DEVICE_ID = 0x0A
MANUFACTURER_ID = 0x08
SENSOR_BASE_API_ID = 0x0301
SENSOR_CONFIG_API_ID = 0x0305

# Ranging modes
RANGING_MODES = {"Short": 0, "Medium": 1, "Long": 2}
REVERSE_MODES = {v: k for k, v in RANGING_MODES.items()}


def make_can_id(api_id, device_number):
    return ((DEVICE_ID & 0xFF) << 24) | ((MANUFACTURER_ID & 0xFF) << 16) | ((api_id & 0x3FF) << 6) | (device_number & 0x3F)


class SensorListener(can.Listener):
    def __init__(self, app):
        self.app = app

    def on_message_received(self, msg):
        api_id = (msg.arbitration_id >> 6) & 0x3FF
        device_number = msg.arbitration_id & 0x3F

        if device_number != self.app.device_number.get():
            return

        if SENSOR_BASE_API_ID <= api_id <= SENSOR_BASE_API_ID + 3 and msg.dlc == 8:
            idx = api_id - SENSOR_BASE_API_ID
            dist = (msg.data[0] << 8) | msg.data[1]
            mode = msg.data[2]
            timing_budget = (msg.data[3] << 24) | (msg.data[4] << 16) | (msg.data[5] << 8) | msg.data[6]
            roi = msg.data[7]
            self.app.update_full_status(idx, dist, mode, roi, timing_budget)

        elif SENSOR_CONFIG_API_ID <= api_id <= SENSOR_CONFIG_API_ID + 3 and msg.dlc >= 2:
            idx = api_id - SENSOR_CONFIG_API_ID
            mode = msg.data[0]
            roi = msg.data[1]
            self.app.update_config(idx, mode, roi)


class ToFSensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ToF CAN Sensor Debugger")

        self.device_number = tk.IntVar(value=50)
        self.dist_vars = [tk.StringVar(value="--") for _ in range(4)]
        self.mode_vars = [tk.StringVar(value="--") for _ in range(4)]
        self.roi_vars = [tk.StringVar(value="--") for _ in range(4)]

        self.mode_select_vars = [tk.StringVar(value="Long") for _ in range(4)]
        self.roi_entry_vars = [tk.StringVar(value="199") for _ in range(4)]

        self.build_ui()

        self.bus = can.interface.Bus(bustype=BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        self.notifier = can.Notifier(self.bus, [SensorListener(self)])

    def update_full_status(self, index, dist, mode, roi, tb):
        self.dist_vars[index].set(f"{dist} mm")
        self.mode_vars[index].set(REVERSE_MODES.get(mode, f"?({mode})"))
        self.roi_vars[index].set(str(roi))

    def build_ui(self):
        tk.Label(self.root, text="Device #:").grid(row=0, column=0, sticky="e")
        tk.Spinbox(self.root, from_=0, to=63, textvariable=self.device_number, width=5).grid(row=0, column=1, sticky="w")

        headers = ["Sensor", "Distance", "Mode", "ROI", "New Mode", "New ROI", "Send"]
        for c, txt in enumerate(headers):
            tk.Label(self.root, text=txt).grid(row=1, column=c)

        for i in range(4):
            tk.Label(self.root, text=f"Sensor {i}").grid(row=2 + i, column=0)
            tk.Label(self.root, textvariable=self.dist_vars[i]).grid(row=2 + i, column=1)
            tk.Label(self.root, textvariable=self.mode_vars[i]).grid(row=2 + i, column=2)
            tk.Label(self.root, textvariable=self.roi_vars[i]).grid(row=2 + i, column=3)

            ttk.Combobox(self.root, textvariable=self.mode_select_vars[i],
                         values=list(RANGING_MODES.keys()), width=8).grid(row=2 + i, column=4)
            tk.Entry(self.root, textvariable=self.roi_entry_vars[i], width=6).grid(row=2 + i, column=5)
            tk.Button(self.root, text="Apply", command=lambda i=i: self.send_config(i)).grid(row=2 + i, column=6)

    def update_config(self, index, mode, roi):
        self.mode_vars[index].set(REVERSE_MODES.get(mode, f"?({mode})"))
        self.roi_vars[index].set(str(roi))

    def send_config(self, index):
        try:
            mode_str = self.mode_select_vars[index].get()
            mode = RANGING_MODES.get(mode_str)
            roi = int(self.roi_entry_vars[index].get()) & 0xFF
            if mode is None or not (0 <= roi <= 255):
                raise ValueError("Invalid mode or ROI")

            api_id = SENSOR_CONFIG_API_ID + index
            msg = can.Message(arbitration_id=make_can_id(api_id, self.device_number.get()),
                              is_extended_id=True, data=[mode, roi], dlc=2)
            self.bus.send(msg)
            print(f"[CAN] Sent config to Sensor {index} | Mode: {mode_str}, ROI: {roi}")
        except Exception as e:
            print(f"[ERROR] Failed to send config for Sensor {index}: {e}")

    def close(self):
        self.notifier.stop()
        self.bus.shutdown()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ToFSensorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
