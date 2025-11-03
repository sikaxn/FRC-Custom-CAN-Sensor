import can
import threading
import struct
import time
import tkinter as tk
from tkinter import ttk

# ==============================================================
# FRC CAN ID Computation
# ==============================================================
DEVICE_TYPE = 0x0A        # Team Custom Device
MANUFACTURER = 0x08       # Iron Maple
DEVICE_NUMBER = 0x00      # Fixed

def frc_can_id(api_id):
    """Compute 29-bit FRC CAN ID."""
    return (DEVICE_TYPE << 24) | (MANUFACTURER << 16) | ((api_id & 0x3FF) << 6) | (DEVICE_NUMBER & 0x3F)

# ==============================================================
# CAN CONFIGURATION (CANalyst-II)
# ==============================================================
bus = can.interface.Bus(
    channel=0,
    interface='canalystii',
    bitrate=1000000  # 1 Mbps typical FRC CAN rate
)

# ==============================================================
# GLOBALS
# ==============================================================
rio_voltage = 13.2
state_override = 0
use_rio_energy = False
energy_kj = 0
esp_reboot_flag = False

esp_serial = ""
esp_meta = {}
esp_status = {}

# Precomputed IDs
ID_131 = frc_can_id(0x131)
ID_132 = frc_can_id(0x132)
ID_133 = frc_can_id(0x133)
ID_135 = frc_can_id(0x135)

# ==============================================================
# Build & Send Control Frame (RIO→ESP32)
# ==============================================================
def send_control_frame():
    global rio_voltage, state_override, use_rio_energy, energy_kj, esp_reboot_flag

    v10 = int(round(rio_voltage * 10)) & 0xFF
    payload = struct.pack(
        ">BBBBHBB",
        v10,
        state_override & 0xFF,
        1 if use_rio_energy else 0,
        (energy_kj >> 8) & 0xFF,
        energy_kj & 0xFF,
        1 if esp_reboot_flag else 0,
        0
    )

    msg = can.Message(
        arbitration_id=ID_135,
        is_extended_id=True,
        data=payload
    )
    try:
        bus.send(msg)
    except can.CanError:
        print("[CAN] Send failed (0x135).")

    esp_reboot_flag = False

# ==============================================================
# CAN RX Loop
# ==============================================================
def can_rx_loop():
    global esp_serial, esp_meta, esp_status

    while True:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue

        if msg.arbitration_id == ID_131:
            esp_serial = msg.data.decode(errors="ignore").strip("\x00")
            ui_update()

        elif msg.arbitration_id == ID_132 and len(msg.data) >= 7:
            yy, mm, dd, hh, mn, cycle, note = msg.data[:7]
            esp_meta.update({
                "year": 2000 + yy,
                "month": mm,
                "day": dd,
                "hour": hh,
                "minute": mn,
                "cycle": cycle,
                "note": note
            })
            ui_update()

        elif msg.arbitration_id == ID_133 and len(msg.data) >= 7:
            state, pdtype, reader, authH, authL, wrH, wrL = msg.data[:7]
            esp_status.update({
                "state": state,
                "pdType": pdtype,
                "reader": reader,
                "authFail": (authH << 8) | authL,
                "writeCount": (wrH << 8) | wrL
            })
            ui_update()

# ==============================================================
# UI Update
# ==============================================================
def ui_update():
    lblSerial.config(text=f"Serial: {esp_serial or '---'}")
    if esp_meta:
        lblMeta.config(
            text=f"Metadata: {esp_meta['year']:04d}-{esp_meta['month']:02d}-{esp_meta['day']:02d} "
                 f"{esp_meta['hour']:02d}:{esp_meta['minute']:02d} | "
                 f"Cycle={esp_meta['cycle']} Note={esp_meta['note']}"
        )
    if esp_status:
        lblStatus.config(
            text=f"State={esp_status['state']} PD={esp_status['pdType']} "
                 f"Reader={esp_status['reader']} "
                 f"AuthFail={esp_status['authFail']} Write={esp_status['writeCount']}"
        )

# ==============================================================
# Periodic Sender Thread (10 Hz)
# ==============================================================
def periodic_sender():
    while True:
        send_control_frame()
        time.sleep(0.1)

# ==============================================================
# UI Callbacks
# ==============================================================
def on_voltage_change(val):
    global rio_voltage
    rio_voltage = float(val)

def on_energy_change():
    global energy_kj, use_rio_energy
    try:
        energy_kj = int(entryEnergy.get())
        use_rio_energy = energy_kj > 0
    except ValueError:
        energy_kj = 0
        use_rio_energy = False

def on_state_override():
    global state_override
    try:
        state_override = int(entryState.get())
    except ValueError:
        state_override = 0

def on_reboot_request():
    global esp_reboot_flag
    esp_reboot_flag = True
    print("[UI] ESP32 reboot requested.")

# ==============================================================
# Tkinter UI
# ==============================================================
root = tk.Tk()
root.title("FRC RoboRIO CAN Simulator (FRC ID Compliant)")
root.geometry("650x400")

frm = ttk.Frame(root, padding=10)
frm.pack(fill="both", expand=True)

ttk.Label(frm, text="RIO Voltage (V):").grid(row=0, column=0, sticky="w")
voltageScale = ttk.Scale(frm, from_=10.0, to=15.0, orient="horizontal", command=on_voltage_change)
voltageScale.set(rio_voltage)
voltageScale.grid(row=0, column=1, sticky="ew")

ttk.Label(frm, text="Energy (kJ):").grid(row=1, column=0, sticky="w")
entryEnergy = ttk.Entry(frm)
entryEnergy.insert(0, "0")
entryEnergy.grid(row=1, column=1, sticky="ew")
ttk.Button(frm, text="Apply Energy", command=on_energy_change).grid(row=1, column=2)

ttk.Label(frm, text="State Override:").grid(row=2, column=0, sticky="w")
entryState = ttk.Entry(frm)
entryState.insert(0, "0")
entryState.grid(row=2, column=1, sticky="ew")
ttk.Button(frm, text="Apply State", command=on_state_override).grid(row=2, column=2)

ttk.Button(frm, text="Request ESP Reboot", command=on_reboot_request).grid(row=3, column=0, columnspan=3, pady=10)

lblSerial = ttk.Label(frm, text="Serial: ---")
lblSerial.grid(row=5, column=0, columnspan=3, sticky="w", pady=2)

lblMeta = ttk.Label(frm, text="Metadata: ---")
lblMeta.grid(row=6, column=0, columnspan=3, sticky="w", pady=2)

lblStatus = ttk.Label(frm, text="Status: ---")
lblStatus.grid(row=7, column=0, columnspan=3, sticky="w", pady=2)

frm.columnconfigure(1, weight=1)

# ==============================================================
# Thread Startup
# ==============================================================
threading.Thread(target=can_rx_loop, daemon=True).start()
threading.Thread(target=periodic_sender, daemon=True).start()

root.mainloop()
