import tkinter as tk
from tkinter import ttk
import can
import threading
import time

# ---------------------------
# FRC CAN helpers
# ---------------------------
DEVICE_TYPE_ID = 0x0A
MANUFACTURER_ID = 0x08

def make_can_id(api_id, device_number):
    return ((DEVICE_TYPE_ID & 0xFF) << 24) | \
           ((MANUFACTURER_ID & 0xFF) << 16) | \
           ((api_id & 0x3FF) << 6) | \
           (device_number & 0x3F)

API_194 = 0x194
API_195 = 0x195
API_197 = 0x197   # RIO->ESP reboot

# ---------------------------
# GUI Application
# ---------------------------
class ColorSensorGUI:
    def __init__(self, master):
        self.master = master
        master.title("ESP32 Color Sensor – CAN Monitor")
        master.geometry("420x350")

        # ---------------------
        # CAN Setup
        # ---------------------
        try:
            # Modify based on your adapter initialization style (from test.py)
            self.bus = can.Bus(bustype="canalystii", channel=0, bitrate=1000000)
        except Exception as e:
            print("CAN init failed:", e)
            self.bus = None

        self.device_number = tk.IntVar(value=33)

        # Data variables
        self.red = tk.StringVar(value="0")
        self.green = tk.StringVar(value="0")
        self.blue = tk.StringVar(value="0")
        self.prox = tk.StringVar(value="0")
        self.clear = tk.StringVar(value="0")
        self.good = tk.StringVar(value="No")

        # ---------------------
        # Layout
        # ---------------------
        self.build_layout()

        # Start CAN listener thread
        if self.bus:
            self.running = True
            self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
            self.rx_thread.start()

    # ---------------------------
    # GUI Layout
    # ---------------------------
    def build_layout(self):
        frm = ttk.Frame(self.master)
        frm.pack(padx=20, pady=20, fill="both", expand=True)

        # Device Number
        ttk.Label(frm, text="Device Number (0–63):").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(frm, from_=0, to=63, textvariable=self.device_number, width=5).grid(row=0, column=1)

        ttk.Separator(frm, orient="horizontal").grid(row=1, column=0, columnspan=4, pady=10, sticky="ew")

        # Sensor Display
        ttk.Label(frm, text="Red:").grid(row=2, column=0, sticky="e")
        ttk.Label(frm, textvariable=self.red).grid(row=2, column=1, sticky="w")

        ttk.Label(frm, text="Green:").grid(row=3, column=0, sticky="e")
        ttk.Label(frm, textvariable=self.green).grid(row=3, column=1, sticky="w")

        ttk.Label(frm, text="Blue:").grid(row=4, column=0, sticky="e")
        ttk.Label(frm, textvariable=self.blue).grid(row=4, column=1, sticky="w")

        ttk.Label(frm, text="Proximity:").grid(row=5, column=0, sticky="e")
        ttk.Label(frm, textvariable=self.prox).grid(row=5, column=1, sticky="w")

        ttk.Label(frm, text="Clear:").grid(row=6, column=0, sticky="e")
        ttk.Label(frm, textvariable=self.clear).grid(row=6, column=1, sticky="w")

        ttk.Label(frm, text="Sensor Good:").grid(row=7, column=0, sticky="e")
        ttk.Label(frm, textvariable=self.good).grid(row=7, column=1, sticky="w")

        ttk.Separator(frm, orient="horizontal").grid(row=8, column=0, columnspan=4, pady=10, sticky="ew")

        # Reboot button
        ttk.Button(frm, text="Send Reboot (0x197)", command=self.send_reboot).grid(row=9, column=0, columnspan=2, pady=5)

        # Exit button
        ttk.Button(frm, text="Exit", command=self.close).grid(row=10, column=0, columnspan=2, pady=5)

    # ---------------------------
    # CAN RX Thread
    # ---------------------------
    def rx_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(0)

                if not msg:
                    continue

                api_id = (msg.arbitration_id >> 6) & 0x3FF
                dev_num = msg.arbitration_id & 0x3F

                if dev_num != self.device_number.get():
                    continue  # Ignore other device numbers

                # 0x194 → RGB + Proximity
                if api_id == API_194 and msg.dlc >= 8:
                    r = (msg.data[0] << 8) | msg.data[1]
                    g = (msg.data[2] << 8) | msg.data[3]
                    b = (msg.data[4] << 8) | msg.data[5]
                    p = (msg.data[6] << 8) | msg.data[7]

                    self.red.set(str(r))
                    self.green.set(str(g))
                    self.blue.set(str(b))
                    self.prox.set(str(p))

                # 0x195 → Clear + SensorGood
                elif api_id == API_195 and msg.dlc >= 3:
                    c = (msg.data[0] << 8) | msg.data[1]
                    sg = msg.data[2]

                    self.clear.set(str(c))
                    self.good.set("Yes" if sg else "No")

            except Exception:
                pass

    # ---------------------------
    # Send reboot message
    # ---------------------------
    def send_reboot(self):
        if not self.bus:
            return

        msg = can.Message(
            arbitration_id=make_can_id(API_197, self.device_number.get()),
            is_extended_id=True,
            data=[1]   # non-zero → reboot
        )
        try:
            self.bus.send(msg)
        except Exception:
            pass

    # ---------------------------
    # Cleanup
    # ---------------------------
    def close(self):
        self.running = False
        time.sleep(0.1)
        self.master.destroy()


# ---------------------------
# Main Entry
# ---------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = ColorSensorGUI(root)
    root.mainloop()
