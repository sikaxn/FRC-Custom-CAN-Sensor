import tkinter as tk
from tkinter import ttk
import threading
import time
import can

# =========================
#   CAN / FRC DEFINITIONS
# =========================
# Fixed per your spec
DEVICE_ID       = 0x0A  # DO NOT change
MANUFACTURER_ID = 0x08  # DO NOT change

API_RX_CONTROL  = 0x185  # PC -> ESP32: [R,G,B,relay,0,0,0,0]
API_RX_STATUS   = 0x186  # PC -> ESP32: [software_ver, uptime_lo, uptime_hi, 0,0,0,0,0]
API_TX_INPUTS   = 0x195  # ESP32 -> PC: [ain_lo, ain_hi, btnA, btnB, 0,0,0,0]
API_TX_RESET    = 0x196  # ESP32 -> PC: [reset_device, 0..] (we don't use for control here)

SOFTWARE_VER = 1  # 0..255

# Canalyst-II bus (exact line as requested)
bus = can.Bus(interface='canalystii', channel=0, device=0, bitrate=1000000)

def make_can_id(api: int, device_number: int) -> int:
    """FRC extended ID: (DEVICE_ID<<24) | (MANUFACTURER_ID<<16) | (API<<6) | DN"""
    return ((DEVICE_ID & 0xFF) << 24) | ((MANUFACTURER_ID & 0xFF) << 16) | ((api & 0x3FF) << 6) | (device_number & 0x3F)

def send_frame(api: int, dn: int, data: bytes):
    msg = can.Message(arbitration_id=make_can_id(api, dn), is_extended_id=True, data=data)
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"CAN send failed: {e}")

# =========================
#       TK APP
# =========================
class CANTester(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IM ESP32-FRC-devkit Tester")
        self.geometry("720x420")
        self.minsize(680, 360)

        # State
        self.device_number = tk.IntVar(value=9)  # default DN
        self.r_val = tk.IntVar(value=0)
        self.g_val = tk.IntVar(value=0)
        self.b_val = tk.IntVar(value=0)
        self.relay_on = tk.BooleanVar(value=False)

        self.ain_bits = tk.IntVar(value=0)
        self.btnA_val = tk.IntVar(value=1)  # 1=released
        self.btnB_val = tk.IntVar(value=1)  # 1=released

        self._uptime = 0  # seconds, saturate at 0xFFFF
        self._run_threads = True

        self._build_ui()

        # Threads / timers
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        # periodic senders
        self.after(100, self._send_185_periodic)  # RGB/relay @10 Hz
        self.after(1000, self._send_186_periodic) # ver/uptime @1 Hz

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 8}

        top = ttk.Frame(self)
        top.pack(fill="x", **pad)

        ttk.Label(top, text="Device Number (0–63):").pack(side="left")
        dn_entry = ttk.Entry(top, width=5, textvariable=self.device_number)
        dn_entry.pack(side="left", padx=6)
        ttk.Button(top, text="Apply", command=self._clamp_dn).pack(side="left")

        # Sliders
        sliders = ttk.LabelFrame(self, text="LED Control (0–255)")
        sliders.pack(fill="x", **pad)

        self._add_slider(sliders, "Red", self.r_val)
        self._add_slider(sliders, "Green", self.g_val)
        self._add_slider(sliders, "Blue", self.b_val)

        # Relay
        relay_frame = ttk.Frame(self)
        relay_frame.pack(fill="x", **pad)
        ttk.Button(relay_frame, text="Toggle Relay", command=self._toggle_relay, width=18).pack(side="left")
        self.relay_label = ttk.Label(relay_frame, text="Relay: OFF", foreground="red")
        self.relay_label.pack(side="left", padx=12)

        # Readback
        rb = ttk.LabelFrame(self, text="Readback (from 0x195)")
        rb.pack(fill="x", **pad)

        self.ain_label = ttk.Label(rb, text="Analog (12-bit): 0")
        self.ain_label.pack(anchor="w", padx=8, pady=4)

        btn_row = ttk.Frame(rb)
        btn_row.pack(fill="x", padx=8, pady=4)
        self.btnA_label = ttk.Label(btn_row, text="Button A: Released")
        self.btnB_label = ttk.Label(btn_row, text="Button B: Released")
        self.btnA_label.pack(side="left", padx=6)
        self.btnB_label.pack(side="left", padx=6)

        # Status
        stat = ttk.LabelFrame(self, text="Status (we send 0x186 once/sec)")
        stat.pack(fill="x", **pad)
        self.uptime_label = ttk.Label(stat, text="Uptime (sec): 0 / 65535")
        self.uptime_label.pack(anchor="w", padx=8, pady=4)
        self.ver_label = ttk.Label(stat, text=f"Software Ver: {SOFTWARE_VER}")
        self.ver_label.pack(anchor="w", padx=8, pady=2)

    def _add_slider(self, parent, name, var):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=8, pady=6)
        ttk.Label(row, text=name, width=8).pack(side="left")
        scale = ttk.Scale(row, from_=0, to=255, variable=var, orient="horizontal")
        scale.pack(side="left", fill="x", expand=True, padx=6)
        val_lbl = ttk.Label(row, textvariable=var, width=4)
        val_lbl.pack(side="right")

    def _clamp_dn(self):
        try:
            dn = int(self.device_number.get())
        except Exception:
            dn = 0
        dn = max(0, min(63, dn))
        self.device_number.set(dn)

    def _toggle_relay(self):
        self.relay_on.set(not self.relay_on.get())
        self.relay_label.configure(text=f"Relay: {'ON' if self.relay_on.get() else 'OFF'}",
                                   foreground=("green" if self.relay_on.get() else "red"))

    # ------------------------
    # Periodic Senders
    # ------------------------
    def _send_185_periodic(self):
        """Send RGB + relay every 100 ms."""
        dn = max(0, min(63, int(self.device_number.get())))
        r = max(0, min(255, int(self.r_val.get())))
        g = max(0, min(255, int(self.g_val.get())))
        b = max(0, min(255, int(self.b_val.get())))
        relay = 1 if self.relay_on.get() else 0
        data = bytes([r, g, b, relay, 0, 0, 0, 0])
        send_frame(API_RX_CONTROL, dn, data)

        # Schedule next
        if self._run_threads:
            self.after(100, self._send_185_periodic)

    def _send_186_periodic(self):
        """Send software_ver + uptime (saturating 0xFFFF) every 1s."""
        # Increment uptime with saturation; if max, hold
        if self._uptime < 0xFFFF:
            self._uptime += 1
        up_lo = self._uptime & 0xFF
        up_hi = (self._uptime >> 8) & 0xFF
        dn = max(0, min(63, int(self.device_number.get())))
        data = bytes([SOFTWARE_VER, up_lo, up_hi, 0, 0, 0, 0, 0])
        send_frame(API_RX_STATUS, dn, data)

        # UI
        self.uptime_label.config(text=f"Uptime (sec): {self._uptime} / 65535")

        # Schedule next
        if self._run_threads:
            self.after(1000, self._send_186_periodic)

    # ------------------------
    # Receiver
    # ------------------------
    def _rx_loop(self):
        """Background receiver that updates readback labels on 0x195 frames."""
        while self._run_threads:
            try:
                msg = bus.recv(0.1)
            except can.CanError:
                msg = None

            if not msg:
                continue

            if not msg.is_extended_id:
                continue

            # Decode FRC fields
            canid = msg.arbitration_id
            dt  = (canid >> 24) & 0xFF
            man = (canid >> 16) & 0xFF
            api = (canid >> 6)  & 0x3FF
            dn  = canid & 0x3F

            # Filter: from our device (type/manufacturer) and our chosen DN, API 0x195
            if dt != DEVICE_ID or man != MANUFACTURER_ID:
                continue
            if api != API_TX_INPUTS:
                continue
            if dn != (int(self.device_number.get()) & 0x3F):
                continue

            data = msg.data
            if len(data) >= 4:
                ain = data[0] | (data[1] << 8)
                btnA = data[2] & 0x01
                btnB = data[3] & 0x01

                # Update UI on main thread
                self.after(0, self._update_readback, ain, btnA, btnB)

    def _update_readback(self, ain, btnA, btnB):
        self.ain_bits.set(ain)
        self.btnA_val.set(btnA)
        self.btnB_val.set(btnB)
        self.ain_label.config(text=f"Analog (12-bit): {ain}")
        self.btnA_label.config(text=f"Button A: {'Pressed' if btnA == 0 else 'Released'}")
        self.btnB_label.config(text=f"Button B: {'Pressed' if btnB == 0 else 'Released'}")

    def _on_close(self):
        self._run_threads = False
        # small delay to let thread exit recv loop
        self.after(120, self._finalize)

    def _finalize(self):
        try:
            bus.shutdown()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = CANTester()
    app.mainloop()
