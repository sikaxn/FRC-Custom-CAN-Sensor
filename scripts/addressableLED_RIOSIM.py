import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
import can

# —— FRC CAN Constants ——
DEVICE_ID           = 0x0A  # DO NOT CHANGE
MANUFACTURER_ID     = 0x08  # DO NOT CHANGE
GENERAL_API         = 0x350
CUSTOM_PATTERN_API  = 0x351  # base for 0x351–0x358

# —— Build a 29-bit FRC CAN ID ——
def make_can_msg_id(device_id, manufacturer_id, api_id, device_number):
    return (
        (device_id        & 0xFF) << 24 |
        (manufacturer_id  & 0xFF) << 16 |
        (api_id           & 0x3FF) << 6  |
        (device_number    & 0x3F)
    )

# —— Initialize CAN bus (Canalystii adapter) ——
bus = can.Bus(interface='canalystii',
              channel=0,
              device=0,
              bitrate=1_000_000)

# —— GUI App ——
class CANApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FRC NeoPixel CAN Sender")
        self.device_number = tk.IntVar(value=33)
        self.build_device_number()
        self.build_general_frame()
        self.build_custom_frame()
        self.build_log()

    def build_device_number(self):
        frm = ttk.Frame(self)
        frm.pack(fill="x", padx=8, pady=(4, 0))
        ttk.Label(frm, text="Device # (0–63):").pack(side="left")
        ttk.Spinbox(frm, from_=0, to=63, width=5, textvariable=self.device_number).pack(side="left")

    def build_general_frame(self):
        frm = ttk.LabelFrame(self, text="General Frame (0x350)")
        frm.pack(fill="x", padx=8, pady=4)

        # Mode
        ttk.Label(frm, text="Mode (0–255):").grid(row=0, column=0, sticky="e")
        self.mode_var = tk.IntVar(value=1)
        ttk.Spinbox(frm, from_=0, to=255, width=5, textvariable=self.mode_var).grid(row=0, column=1)

        # R,G,B sliders + entry
        self.rgb_vars = {}
        for i, color in enumerate(("R", "G", "B")):
            ttk.Label(frm, text=f"{color}:").grid(row=1, column=2*i, sticky="e")
            var = tk.IntVar(value=255 if color == "R" else 0)
            self.rgb_vars[color] = var
            scale = tk.Scale(frm, from_=0, to=255, orient="horizontal", variable=var, length=100)
            scale.grid(row=1, column=2*i+1)

        # Brightness
        ttk.Label(frm, text="Brightness:").grid(row=2, column=0, sticky="e")
        self.brig_var = tk.IntVar(value=128)
        ttk.Scale(frm, from_=0, to=255, orient="horizontal", variable=self.brig_var, length=100).grid(row=2, column=1)

        # On/Off
        ttk.Label(frm, text="On/Off:").grid(row=2, column=2, sticky="e")
        self.onoff_var = tk.IntVar(value=1)
        ttk.Checkbutton(frm, variable=self.onoff_var).grid(row=2, column=3, sticky="w")

        # Param 0 / 1
        ttk.Label(frm, text="Param 0:").grid(row=3, column=0, sticky="e")
        self.param0_var = tk.IntVar(value=0)
        ttk.Spinbox(frm, from_=0, to=255, width=5, textvariable=self.param0_var).grid(row=3, column=1)

        ttk.Label(frm, text="Param 1:").grid(row=3, column=2, sticky="e")
        self.param1_var = tk.IntVar(value=0)
        ttk.Spinbox(frm, from_=0, to=255, width=5, textvariable=self.param1_var).grid(row=3, column=3)

        btn = ttk.Button(frm, text="Send General", command=self.send_general)
        btn.grid(row=4, column=0, columnspan=4, pady=4)

    def build_custom_frame(self):
        frm = ttk.LabelFrame(self, text="Custom Pixel (0x351–0x358)")
        frm.pack(fill="x", padx=8, pady=4)

        # API index 0–7
        ttk.Label(frm, text="Pattern idx (0–7):").grid(row=0, column=0, sticky="e")
        self.idx_var = tk.IntVar(value=0)
        ttk.Spinbox(frm, from_=0, to=7, width=5, textvariable=self.idx_var).grid(row=0, column=1)

        # Pixel#
        ttk.Label(frm, text="Pixel # (0–65535):").grid(row=0, column=2, sticky="e")
        self.pix_var = tk.IntVar(value=0)
        ttk.Spinbox(frm, from_=0, to=65535, width=7, textvariable=self.pix_var).grid(row=0, column=3)

        # R,G,B,W
        self.cvars = {}
        for i, color in enumerate(("R","G","B","W")):
            ttk.Label(frm, text=f"{color}:").grid(row=1, column=2*i, sticky="e")
            var = tk.IntVar(value=0)
            self.cvars[color] = var
            ttk.Spinbox(frm, from_=0, to=255, width=5, textvariable=var).grid(row=1, column=2*i+1)

        # Brightness
        ttk.Label(frm, text="Brightness:").grid(row=2, column=0, sticky="e")
        self.cbrig_var = tk.IntVar(value=128)
        ttk.Spinbox(frm, from_=0, to=255, width=5, textvariable=self.cbrig_var).grid(row=2, column=1)

        btn = ttk.Button(frm, text="Send Custom", command=self.send_custom)
        btn.grid(row=3, column=0, columnspan=4, pady=4)

    def build_log(self):
        ttk.Label(self, text="Log:").pack(anchor="w", padx=8)
        self.log = ScrolledText(self, height=10, state="disabled")
        self.log.pack(fill="both", expand=True, padx=8, pady=4)

    def log_msg(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.yview("end")
        self.log.configure(state="disabled")

    def send_general(self):
        dev_num = self.device_number.get() & 0x3F
        api_id = GENERAL_API
        arb_id = make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, api_id, dev_num)
        data = bytearray(8)
        data[0] = self.mode_var.get() & 0xFF
        data[1] = self.rgb_vars["R"].get() & 0xFF
        data[2] = self.rgb_vars["G"].get() & 0xFF
        data[3] = self.rgb_vars["B"].get() & 0xFF
        data[4] = self.brig_var.get() & 0xFF
        data[5] = self.onoff_var.get() & 0xFF
        data[6] = self.param0_var.get() & 0xFF
        data[7] = self.param1_var.get() & 0xFF
        msg = can.Message(arbitration_id=arb_id,
                          is_extended_id=True,
                          data=data)
        try:
            bus.send(msg)
            self.log_msg(f"→ 0x{arb_id:08X} GEN {list(data)}")
        except can.CanError as e:
            messagebox.showerror("CAN Error", str(e))

    def send_custom(self):
        dev_num = self.device_number.get() & 0x3F
        idx = self.idx_var.get()
        api_id = CUSTOM_PATTERN_API + idx
        arb_id = make_can_msg_id(DEVICE_ID, MANUFACTURER_ID, api_id, dev_num)
        pix = self.pix_var.get() & 0xFFFF
        data = bytearray(8)
        data[0] = (pix >> 8) & 0xFF
        data[1] = pix & 0xFF
        data[2] = self.cvars["R"].get() & 0xFF
        data[3] = self.cvars["G"].get() & 0xFF
        data[4] = self.cvars["B"].get() & 0xFF
        data[5] = self.cvars["W"].get() & 0xFF
        data[6] = self.cbrig_var.get() & 0xFF
        data[7] = 0
        msg = can.Message(arbitration_id=arb_id,
                          is_extended_id=True,
                          data=data)
        try:
            bus.send(msg)
            self.log_msg(f"→ 0x{arb_id:08X} CUST idx={idx} pix={pix} {list(data)}")
        except can.CanError as e:
            messagebox.showerror("CAN Error", str(e))


if __name__ == "__main__":
    app = CANApp()
    app.mainloop()
