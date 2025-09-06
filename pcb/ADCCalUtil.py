#!/usr/bin/env python3
# adc_calibrator_v3.py
#
# KIPRIM DC310S + CANalyst-II ADC calibration (Tk UI)
# - COM port: Connect / Disconnect
# - CAN panel shows live ADC
# - Indeterminate progress ("loading") during discovery & PSU set operations
# - Determinate countdown while stabilizing at each setpoint
#
# Safety:
# - Current limit forced to 1.0 A
# - Abort button immediately cuts PSU
# - Auto-abort if measured power > 5.0 W
# - Error if ADC < 50 (jumper not connected)
#
# Changes in v3:
# - Use PSU measured voltage (median) for each sample (instead of setpoint)
# - Fixed-point formatting everywhere (no scientific notation)

import os
import time
import json
import queue
import serial
import threading
import statistics
import serial.tools.list_ports
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import can

# -----------------------------
# Utility: fixed-point formatter
# -----------------------------
def fmt(x, digits):
    """Return fixed-point string with up to `digits` decimals (no exponent)."""
    s = f"{x:.{digits}f}"
    return s.rstrip('0').rstrip('.') if '.' in s else s

# -----------------------------
# FRC-style CAN ID definitions
# -----------------------------
DEVICE_ID        = 0x0A
MANUFACTURER_ID  = 0x08
API_ID_ADC_FRAME = 0x195  # ESP32 -> roboRIO: [ain_lo, ain_hi, btnA, btnB, 0,0,0,0]

def make_can_id(device_id, manufacturer_id, api_id, device_number):
    # Full 29-bit FRC-style CAN ID:
    # CAN_ID = (deviceID<<24) | (manufacturerID<<16) | (apiID<<6) | (deviceNumber & 0x3F)
    return ((device_id & 0xFF) << 24) | ((manufacturer_id & 0xFF) << 16) | ((api_id & 0x7FF) << 6) | (device_number & 0x3F)

def parse_can_id(can_id):
    device_number   = can_id & 0x3F
    api_id          = (can_id >> 6) & 0x7FF
    manufacturer_id = (can_id >> 16) & 0xFF
    device_id       = (can_id >> 24) & 0xFF
    return device_id, manufacturer_id, api_id, device_number

# -----------------------------
# DC310S minimal driver
# -----------------------------
class DC310S:
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.port = port
        self.ser  = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def _send(self, cmd):
        try:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + '\n').encode('utf-8'))
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            return line
        except Exception:
            return None

    def set_output(self, on: bool):
        return self._send(f"output {1 if on else 0}")

    def set_voltage(self, volts: float):
        return self._send(f"voltage {volts:.3f}")

    def set_current(self, amps: float):
        return self._send(f"current {amps:.3f}")

    def meas_voltage(self):
        resp = self._send("measure:voltage?")
        try: return float(resp)
        except: return None

    def meas_current(self):
        resp = self._send("measure:current?")
        try: return float(resp)
        except: return None

    def close(self):
        try: self.ser.close()
        except: pass

# -----------------------------
# CAN receive worker
# -----------------------------
class CANReader(threading.Thread):
    def __init__(self, msg_queue, stop_event, channel=0, device=0, bitrate=1_000_000):
        super().__init__(daemon=True)
        self.msg_queue = msg_queue
        self.stop_event = stop_event
        # Preferred adapter: CANalyst-II
        self.bus = can.Bus(interface='canalystii', channel=channel, device=device, bitrate=bitrate)

    def run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(0.1)
                if msg is not None:
                    self.msg_queue.put(msg)
            except Exception:
                time.sleep(0.1)
        try: self.bus.shutdown()
        except: pass

# -----------------------------
# Calibration engine
# -----------------------------
class Calibrator(threading.Thread):
    def __init__(self, ui, dc, msg_queue, stop_event, setpoints, wait_seconds=30):
        super().__init__(daemon=True)
        self.ui = ui
        self.dc = dc
        self.msg_queue = msg_queue
        self.stop_event = stop_event
        self.setpoints = setpoints
        self.wait_seconds = wait_seconds

        self.device_number = None
        self.target_can_id = None

        self.samples = []            # [(adc, vtrue_measured), ...]
        self.cur_adc_window = []     # rolling ADC samples per step
        self.cur_v_window = []       # rolling PSU V readings per step
        self.last_adc_for_ui = None  # for live ADC label

    def discover_device_number(self, timeout=20.0):
        self.ui.set_busy(True, "Discovering board on CAN…")
        t0 = time.time()
        found = False
        while time.time() - t0 < timeout and not self.stop_event.is_set():
            try:
                msg = self.msg_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            dev_id, man_id, api_id, dev_num = parse_can_id(msg.arbitration_id)
            if dev_id == DEVICE_ID and man_id == MANUFACTURER_ID and api_id == API_ID_ADC_FRAME:
                self.device_number = dev_num
                self.target_can_id = make_can_id(DEVICE_ID, MANUFACTURER_ID, API_ID_ADC_FRAME, self.device_number)
                self.ui.log(f"✔ Found board: DEVICE_NUMBER={self.device_number}")
                self.ui.set_can_status(True, f"Board #{self.device_number}")
                found = True
                break
        self.ui.set_busy(False)
        if not found:
            self.ui.log("✖ Device discovery timed out.")
        return found

    def collect_adc_from_msg(self, msg):
        if len(msg.data) < 2: return None
        ain = msg.data[0] | (msg.data[1] << 8)
        ain &= 0x3FFF
        self.last_adc_for_ui = ain
        self.ui.update_adc_label(ain)
        return ain

    def drain_can_for_adc(self):
        drained = 0
        while True:
            try:
                msg = self.msg_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            if self.target_can_id is not None and msg.arbitration_id == self.target_can_id:
                adc = self.collect_adc_from_msg(msg)
                if adc is not None:
                    self.cur_adc_window.append(adc)
        if drained:
            self.ui.bump_can_rx()

    def set_psu_safe(self, volts):
        # Show "loading" while we push settings
        self.ui.set_busy(True, f"Setting PSU {volts:.2f} V / 1.00 A…")
        self.dc.set_current(1.0)       # force 1 A limit
        self.dc.set_voltage(volts)
        self.dc.set_output(True)
        self.ui.set_psu_status(True, f"{volts:.2f} V @ 1.00 A")
        self.ui.set_busy(False)

    def step_wait_and_sample(self, vset):
        self.set_psu_safe(vset)
        self.cur_adc_window.clear()
        self.cur_v_window.clear()
        self.ui.switch_to_countdown_mode(self.wait_seconds)

        for remaining in range(self.wait_seconds, -1, -1):
            if self.stop_event.is_set():
                return None
            # Live PSU safety check
            v_meas = self.dc.meas_voltage()
            i_meas = self.dc.meas_current()
            p = (v_meas or 0) * (i_meas or 0)
            self.ui.update_live_psu(v_meas, i_meas, p)
            if p > 5.0:
                self.ui.log("⚠ Power > 5 W, aborting!")
                self.dc.set_output(False)
                return None

            # Record PSU voltage sample if valid
            if v_meas is not None:
                self.cur_v_window.append(v_meas)

            # Pull CAN for ADCs
            self.drain_can_for_adc()
            self.ui.update_countdown(remaining)
            time.sleep(1)

        # Validate windows
        if not self.cur_adc_window:
            self.ui.log("No ADC samples received in this window.")
            return None
        if not self.cur_v_window:
            self.ui.log("No PSU voltage readings captured.")
            return None

        # Median over the last 10 (or all if <10)
        median_adc = int(statistics.median(self.cur_adc_window[-10:]))
        vtrue = float(statistics.median(self.cur_v_window[-10:]))

        if median_adc < 50:
            self.ui.log(f"✖ ADC too low ({median_adc}); jumper not connected?")
            return None

        self.ui.log(f"Recorded ADC={median_adc} at PSU={fmt(vtrue, 6)} V")
        return (median_adc, vtrue)

    def run(self):
        try:
            # Ensure PSU OFF initially
            try:
                self.dc.set_output(False)
            except: pass
            self.ui.set_psu_status(False, "Output OFF")

            # Apply minimal power to boot device for discovery
            min_v = 5.5 if self.setpoints[0] < 5.5 else self.setpoints[0]
            self.set_psu_safe(min_v)
            self.ui.log("Power applied for discovery…")
            time.sleep(3.0)

            if not self.discover_device_number():
                self.dc.set_output(False)
                self.ui.set_psu_status(False, "Discovery failed")
                self.ui.finish(False)
                return

            # Calibration loop
            self.samples.clear()
            for idx, vset in enumerate(self.setpoints, 1):
                if self.stop_event.is_set():
                    break
                self.ui.set_step(idx, len(self.setpoints), vset)
                pair = self.step_wait_and_sample(vset)
                if pair is None:
                    self.dc.set_output(False)
                    self.ui.set_psu_status(False, "Aborted/Failed")
                    self.ui.finish(False)
                    return
                adc, vtrue = pair
                self.samples.append((adc, vtrue))

            # Done — PSU off
            self.dc.set_output(False)
            self.ui.set_psu_status(False, "Output OFF")

            # Fit quadratic V = k0 + k1*adc + k2*adc^2 using measured PSU voltages
            adcs  = np.array([a for a, _ in self.samples], dtype=float)
            volts = np.array([v for _, v in self.samples], dtype=float)
            coeffs = np.polyfit(adcs, volts, 2)  # [k2,k1,k0]
            k2, k1, k0 = coeffs.tolist()

            pred  = np.polyval(coeffs, adcs)
            ss_res = np.sum((volts - pred)**2)
            ss_tot = np.sum((volts - np.mean(volts))**2)
            r2 = 1.0 - (ss_res / ss_tot if ss_tot > 0 else 0.0)

            self.ui.log("")
            self.ui.log("=== Calibration Result (V = k0 + k1*adc + k2*adc^2) ===")
            self.ui.log(f"k0 = {fmt(k0, 9)}")
            self.ui.log(f"k1 = {fmt(k1, 12)}")
            self.ui.log(f"k2 = {fmt(k2, 15)}")
            self.ui.log(f"R^2 = {fmt(r2, 6)}")

            # Save (fixed-point strings to avoid exponent notation)
            out = {
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "device_id": DEVICE_ID,
                "manufacturer_id": MANUFACTURER_ID,
                "api_id": API_ID_ADC_FRAME,
                "device_number": self.device_number,
                "setpoints_v": self.setpoints,
                "samples": [{"adc": int(a), "v": fmt(v, 9)} for a, v in self.samples],
                "fit": {"k0": fmt(k0, 9), "k1": fmt(k1, 12), "k2": fmt(k2, 15), "r2": fmt(r2, 6)}
            }
            jname = f"calibration_board_{self.device_number}.json"
            cname = f"calibration_board_{self.device_number}.csv"
            with open(jname, "w") as f:
                json.dump(out, f, indent=2)
            with open(cname, "w") as f:
                f.write("adc,voltage\n")
                for a, v in self.samples:
                    f.write(f"{a},{fmt(v, 9)}\n")
                f.write("\n# k0,k1,k2,R2\n")
                f.write(f"{fmt(k0,9)},{fmt(k1,12)},{fmt(k2,15)},{fmt(r2,6)}\n")

            self.ui.log(f"Saved {jname} and {cname}")
            # UI labels are already fixed-width decimals, so plain floats are OK here
            self.ui.show_constants(k0, k1, k2, r2)
            self.ui.finish(True)

        except Exception as e:
            try: self.dc.set_output(False)
            except: pass
            self.ui.log(f"Exception: {e}")
            self.ui.finish(False)

# -----------------------------
# Tkinter UI
# -----------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dev Board ADC Calibration")
        self.geometry("1000x560")
        self.resizable(True, True)

        # State
        self.running = False
        self.stop_event = threading.Event()
        self.msg_queue = queue.Queue()
        self.dc = None
        self.can_thread = None

        self._build_ui()

    def _build_ui(self):
        # Left = status; Right = progress/controls
        self.left = tk.Frame(self, padx=8, pady=8)
        self.left.pack(side="left", fill="y")

        self.right = tk.Frame(self, padx=8, pady=8)
        self.right.pack(side="right", fill="both", expand=True)

        # ---- Left panel: PSU & CAN status ----
        lf = ttk.LabelFrame(self.left, text="Power Supply")
        lf.pack(fill="x")

        self.port_var = tk.StringVar(value=self._default_port())
        ttk.Label(lf, text="COM Port:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.port_cb = ttk.Combobox(lf, values=self._list_ports(), textvariable=self.port_var, width=12, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=4, pady=2)

        self.conn_btn = ttk.Button(lf, text="Connect", command=self.on_connect_psu)
        self.conn_btn.grid(row=0, column=2, padx=4, pady=2)
        self.disc_btn = ttk.Button(lf, text="Disconnect", command=self.on_disconnect_psu, state="disabled")
        self.disc_btn.grid(row=0, column=3, padx=4, pady=2)

        self.psu_status = ttk.Label(lf, text="Disconnected", foreground="red")
        self.psu_status.grid(row=1, column=0, columnspan=4, sticky="w", padx=4, pady=2)

        self.lv = ttk.Label(lf, text="V: --.- V")
        self.li = ttk.Label(lf, text="I: -.--- A")
        self.lp = ttk.Label(lf, text="P: --.- W")
        self.lv.grid(row=2, column=0, padx=4, pady=2, sticky="w")
        self.li.grid(row=2, column=1, padx=4, pady=2, sticky="w")
        self.lp.grid(row=2, column=2, padx=4, pady=2, sticky="w")

        lf2 = ttk.LabelFrame(self.left, text="CAN Status")
        lf2.pack(fill="x", pady=8)
        self.can_status = ttk.Label(lf2, text="Not connected", foreground="red")
        self.can_status.grid(row=0, column=0, columnspan=3, sticky="w", padx=4, pady=2)
        self.can_rx_label = ttk.Label(lf2, text="RX: 0")
        self.can_rx_label.grid(row=1, column=0, padx=4, pady=2, sticky="w")
        self.rx_count = 0
        self.adc_label = ttk.Label(lf2, text="ADC: ---")
        self.adc_label.grid(row=1, column=1, padx=4, pady=2, sticky="w")

        # ---- Right panel: progress / controls ----
        rf = ttk.LabelFrame(self.right, text="Calibration")
        rf.pack(fill="both", expand=True)

        ttk.Label(rf, text="Voltage setpoints (V, comma-separated):").grid(row=0, column=0, sticky="w")
        self.setpoints_var = tk.StringVar(value="5.5, 10, 14, 20, 28")
        ttk.Entry(rf, textvariable=self.setpoints_var, width=40).grid(row=0, column=1, sticky="w")

        ttk.Label(rf, text="Per-step wait (s):").grid(row=1, column=0, sticky="w")
        self.wait_var = tk.IntVar(value=30)
        ttk.Entry(rf, textvariable=self.wait_var, width=8).grid(row=1, column=1, sticky="w")

        self.step_label = ttk.Label(rf, text="Step: 0/0")
        self.step_label.grid(row=2, column=0, sticky="w", pady=4)

        # Progress bar supports both modes
        self.countdown_label = ttk.Label(rf, text="Waiting: -- s")
        self.countdown_label.grid(row=2, column=1, sticky="w", pady=4)

        self.progress = ttk.Progressbar(rf, mode="determinate", maximum=100)
        self.progress.grid(row=3, column=0, columnspan=2, sticky="we", pady=6)

        # Start/Abort
        self.start_btn = ttk.Button(rf, text="Start", command=self.on_start)
        self.start_btn.grid(row=4, column=0, sticky="w", pady=6)
        self.abort_btn = ttk.Button(rf, text="Abort", command=self.on_abort, state="disabled")
        self.abort_btn.grid(row=4, column=1, sticky="w", pady=6)

        # Results
        sep = ttk.Separator(rf, orient="horizontal")
        sep.grid(row=5, column=0, columnspan=2, sticky="we", pady=8)

        self.k0_var = tk.StringVar(value="k0: —")
        self.k1_var = tk.StringVar(value="k1: —")
        self.k2_var = tk.StringVar(value="k2: —")
        self.r2_var = tk.StringVar(value="R²: —")
        ttk.Label(rf, textvariable=self.k0_var).grid(row=6, column=0, sticky="w")
        ttk.Label(rf, textvariable=self.k1_var).grid(row=6, column=1, sticky="w")
        ttk.Label(rf, textvariable=self.k2_var).grid(row=7, column=0, sticky="w")
        ttk.Label(rf, textvariable=self.r2_var).grid(row=7, column=1, sticky="w")

        # Log
        self.logbox = tk.Text(rf, height=10)
        self.logbox.grid(row=8, column=0, columnspan=2, sticky="nsew", pady=6)
        rf.rowconfigure(8, weight=1)
        rf.columnconfigure(1, weight=1)

        ttk.Label(self.right, text="Tip: Calibrate multiple boards without restarting — just click Start again.").pack(anchor="w", pady=4)

    # ---------- UI helpers ----------
    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _default_port(self):
        ports = self._list_ports()
        # pick highest COM number on Windows; else pick first
        def key(x):
            if x.upper().startswith("COM"):
                n = x[3:]
                return int(n) if n.isdigit() else -1
            return 0
        return sorted(ports, key=key)[-1] if ports else ""

    def set_psu_status(self, on, text=""):
        self.psu_status.config(text=("ON: " if on else "OFF: ") + text, foreground=("green" if on else "red"))

    def set_can_status(self, ok, text=""):
        self.can_status.config(text=("OK: " if ok else "ERR: ") + text, foreground=("green" if ok else "red"))

    def bump_can_rx(self):
        self.rx_count += 1
        self.can_rx_label.config(text=f"RX: {self.rx_count}")

    def update_live_psu(self, v, i, p):
        self.lv.config(text=f"V: {v:.2f} V" if v is not None else "V: ---")
        self.li.config(text=f"I: {i:.3f} A" if i is not None else "I: ---")
        self.lp.config(text=f"P: {p:.2f} W")

    def update_countdown(self, remaining):
        self.countdown_label.config(text=f"Waiting: {remaining:02d} s")
        wait = max(1, int(self.wait_var.get()))
        pct = int((1.0 - remaining / wait) * 100) if remaining <= wait else 0
        if self.progress["mode"] != "determinate":
            self.progress.config(mode="determinate", maximum=100)
        self.progress["value"] = pct

    def switch_to_countdown_mode(self, wait_seconds):
        self.progress.stop()
        self.progress.config(mode="determinate", maximum=100, value=0)
        self.countdown_label.config(text=f"Waiting: {wait_seconds:02d} s")

    def set_step(self, idx, total, vset):
        self.step_label.config(text=f"Step: {idx}/{total}  @ {vset:.2f} V")
        if self.progress["mode"] != "determinate":
            self.progress.stop()
            self.progress.config(mode="determinate", maximum=100, value=0)

    def set_busy(self, busy: bool, msg: str = ""):
        if busy:
            self.countdown_label.config(text=msg if msg else "Working…")
            if self.progress["mode"] != "indeterminate":
                self.progress.config(mode="indeterminate")
            self.progress.start(12)  # nice smooth spin
            self.start_btn.config(state="disabled")
            self.abort_btn.config(state="normal" if self.running else "disabled")
            self.conn_btn.config(state="disabled")
            self.disc_btn.config(state="disabled" if self.dc is None else "normal")
        else:
            self.progress.stop()
            # Do not switch mode here; caller (countdown or idle) will set it
            self.start_btn.config(state="disabled" if self.running else "normal")
            self.conn_btn.config(state="disabled" if self.dc is None else "normal")
            self.disc_btn.config(state="normal" if self.dc is not None else "disabled")

    def update_adc_label(self, adc_value):
        self.adc_label.config(text=f"ADC: {adc_value}")

    def show_constants(self, k0, k1, k2, r2):
        # UI uses fixed decimals—no scientific format
        self.k0_var.set(f"k0: {k0:.9f}")
        self.k1_var.set(f"k1: {k1:.12f}")
        self.k2_var.set(f"k2: {k2:.15f}")
        self.r2_var.set(f"R²: {r2:.6f}")

    def log(self, s):
        self.logbox.insert("end", s + "\n")
        self.logbox.see("end")
        self.update_idletasks()

    # ---------- PSU connect/disconnect ----------
    def on_connect_psu(self):
        if self.dc is not None:
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a COM port first.")
            return
        try:
            self.dc = DC310S(port)
            self.psu_status.config(text=f"Connected ({port})", foreground="green")
            self.conn_btn.config(state="disabled")
            self.disc_btn.config(state="normal")
        except Exception as e:
            self.psu_status.config(text=f"Connect failed: {e}", foreground="red")
            self.dc = None

    def on_disconnect_psu(self):
        if self.dc is None:
            return
        try:
            try: self.dc.set_output(False)
            except: pass
            self.dc.close()
        finally:
            self.dc = None
            self.set_psu_status(False, "Disconnected")
            self.conn_btn.config(state="normal")
            self.disc_btn.config(state="disabled")

    # ---------- Start / Abort ----------
    def on_start(self):
        if self.running:
            return

        # Require PSU connected
        if self.dc is None:
            messagebox.showerror("Error", "Connect the DC310S first.")
            return

        # Parse setpoints
        try:
            setpoints = [float(x) for x in self.setpoints_var.get().replace(" ", "").split(",") if x]
        except:
            messagebox.showerror("Error", "Invalid setpoints format.")
            return
        if len(setpoints) < 3:
            messagebox.showerror("Error", "Provide at least 3 setpoints.")
            return
        if min(setpoints) < 5.5 or max(setpoints) > 28.0:
            messagebox.showerror("Error", "Setpoints must be between 5.5 V and 28 V.")
            return

        wait_s = int(self.wait_var.get())
        if wait_s < 5:
            messagebox.showerror("Error", "Per-step wait must be at least 5 seconds.")
            return

        # Reset counters/UI
        self.rx_count = 0
        self.can_rx_label.config(text="RX: 0")
        self.adc_label.config(text="ADC: ---")
        self.progress.config(mode="indeterminate", value=0)
        self.k0_var.set("k0: —")
        self.k1_var.set("k1: —")
        self.k2_var.set("k2: —")
        self.r2_var.set("R²: —")
        self.logbox.delete("1.0", "end")
        self.set_can_status(False, "Discovering…")

        # Start CAN reader
        self.stop_event.clear()
        try:
            self.can_thread = CANReader(self.msg_queue, self.stop_event, channel=0, device=0, bitrate=1_000_000)
            self.can_thread.start()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open CAN: {e}")
            return

        # Spin up calibration
        self.running = True
        self.start_btn.config(text="Start", state="disabled")
        self.abort_btn.config(text="Abort", state="normal")
        self.conn_btn.config(state="disabled")
        self.disc_btn.config(state="disabled" if self.dc is None else "normal")

        self.cal_thread = Calibrator(self, self.dc, self.msg_queue, self.stop_event, setpoints, wait_s)
        self.cal_thread.start()

    def on_abort(self):
        if not self.running:
            return
        self.log("Abort requested… cutting PSU.")
        self.stop_event.set()
        try:
            if self.dc: self.dc.set_output(False)
        except: pass
        self.finish(False)

    def finish(self, ok: bool):
        # Marshal to main thread
        def _do_finish():
            if self.dc:
                try: self.dc.set_output(False)
                except: pass
                self.set_psu_status(False, "Output OFF")
            if self.can_thread:
                self.stop_event.set()
                self.can_thread = None
            self.running = False
            self.start_btn.config(text="Start", state="normal")
            self.abort_btn.config(state="disabled")
            self.conn_btn.config(state="disabled" if self.dc is None else "normal")
            self.disc_btn.config(state="normal" if self.dc is not None else "disabled")
            self.progress.stop()
            self.progress.config(mode="determinate", value=0)
            self.countdown_label.config(text="Waiting: -- s")
            if ok:
                self.log("✔ Calibration complete.")
            else:
                self.log("✖ Calibration stopped.")
        self.after(0, _do_finish)

# -----------------------------
# main
# -----------------------------
if __name__ == "__main__":
    app = App()
    app.mainloop()
