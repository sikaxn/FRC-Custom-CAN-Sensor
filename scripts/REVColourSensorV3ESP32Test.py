import tkinter as tk
from tkinter import ttk
import can
import threading
import time

# ===============================================================
# CAN SETTINGS
# ===============================================================
CAN_CHANNEL = 0
CAN_DEVICE  = 0
CAN_BITRATE = 1000000

# ===============================================================
# GLOBAL DEVICE NUMBER (editable via GUI)
# ===============================================================
DEVICE_TYPE_ID        = 0x0A
MANUFACTURER_ID  = 0x08
DEVICE_NUMBER    = 33     # default

# ===============================================================
# API IDs
# ===============================================================
API_COLOR_DATA1  = 0x184
API_COLOR_DATA2  = 0x185
API_COLOR_STATUS = 0x186
API_COLOR_CONFIG = 0x187


# ===============================================================
# ENUM DEFINITIONS
# ===============================================================
LED_FREQ = {
    "60 kHz": 0x18,
    "70 kHz": 0x40,
    "80 kHz": 0x28,
    "90 kHz": 0x30,
    "100 kHz": 0x38
}

LED_CURRENT = {
    "2 mA": 0,
    "5 mA": 1,
    "10 mA": 2,
    "25 mA": 3,
    "50 mA": 4,
    "75 mA": 5,
    "100 mA": 6,
    "125 mA": 7
}

PROX_RES = {
    "8 bit": 0x00,
    "9 bit": 0x08,
    "10 bit": 0x10,
    "11 bit": 0x18
}

PROX_RATE = {
    "6 ms": 1,
    "12 ms": 2,
    "25 ms": 3,
    "50 ms": 4,
    "100 ms": 5,
    "200 ms": 6,
    "400 ms": 7
}

COLOR_RES = {
    "20 bit": 0x00,
    "19 bit": 0x10,
    "18 bit": 0x20,
    "17 bit": 0x30,
    "16 bit": 0x40,
    "13 bit": 0x50
}

COLOR_RATE = {
    "25 ms": 0,
    "50 ms": 1,
    "100 ms": 2,
    "200 ms": 3,
    "500 ms": 4,
    "1000 ms": 5,
    "2000 ms": 7
}

GAIN = {
    "1x": 0,
    "3x": 1,
    "6x": 2,
    "9x": 3,
    "18x": 4
}

# ===============================================================
# REVERSE LOOKUP TABLES (value → text)
# ===============================================================
LED_FREQ_REV     = {v: k for k, v in LED_FREQ.items()}
LED_CURRENT_REV  = {v: k for k, v in LED_CURRENT.items()}
PROX_RES_REV     = {v: k for k, v in PROX_RES.items()}
PROX_RATE_REV    = {v: k for k, v in PROX_RATE.items()}
COLOR_RES_REV    = {v: k for k, v in COLOR_RES.items()}
COLOR_RATE_REV   = {v: k for k, v in COLOR_RATE.items()}
GAIN_REV         = {v: k for k, v in GAIN.items()}


# ===============================================================
# CAN ID Helpers
# ===============================================================
def make_can_id(api):
    return ((DEVICE_TYPE_ID & 0xFF) << 24) | \
           ((MANUFACTURER_ID & 0xFF) << 16) | \
           ((api & 0x3FF) << 6) | \
           (DEVICE_NUMBER & 0x3F)

def extract_api(arbid):
    return (arbid >> 6) & 0x3FF

def extract_devnum(arbid):
    return arbid & 0x3F


# ===============================================================
# CAN BUS
# ===============================================================
bus = can.Bus(
    interface='canalystii',
    channel=CAN_CHANNEL,
    device=CAN_DEVICE,
    bitrate=CAN_BITRATE
)


# ===============================================================
# GUI SETUP
# ===============================================================
root = tk.Tk()
root.title("REV Color Sensor V3 – CAN Tool")
root.geometry("650x580")


# ===============================================================
# Device Number Entry
# ===============================================================
tk.Label(root, text="Device Number (0–63):", font=("Arial", 10))\
    .grid(row=0, column=0, sticky="w", padx=5, pady=5)

devnum_var = tk.StringVar(value=str(DEVICE_NUMBER))
devnum_entry = tk.Entry(root, textvariable=devnum_var, width=8)
devnum_entry.grid(row=0, column=1)

def apply_device_number():
    global DEVICE_NUMBER
    try:
        dn = int(devnum_var.get())
        if 0 <= dn <= 63:
            DEVICE_NUMBER = dn
            print("Device number updated:", dn)
    except:
        pass

tk.Button(root, text="Set Device #", command=apply_device_number)\
    .grid(row=0, column=2, padx=5)


# ===============================================================
# SENSOR VALUE DISPLAY
# ===============================================================
lbl_red    = tk.Label(root, text="Red: 0", font=("Arial", 11))
lbl_green  = tk.Label(root, text="Green: 0", font=("Arial", 11))
lbl_blue   = tk.Label(root, text="Blue: 0", font=("Arial", 11))
lbl_ir     = tk.Label(root, text="IR: 0", font=("Arial", 11))
lbl_prox   = tk.Label(root, text="Proximity: 0", font=("Arial", 11))
lbl_online = tk.Label(root, text="Online: ?", font=("Arial", 11))

lbl_red.grid(row=1, column=0, sticky="w", padx=5)
lbl_green.grid(row=2, column=0, sticky="w", padx=5)
lbl_blue.grid(row=3, column=0, sticky="w", padx=5)
lbl_ir.grid(row=4, column=0, sticky="w", padx=5)
lbl_prox.grid(row=5, column=0, sticky="w", padx=5)
lbl_online.grid(row=6, column=0, sticky="w", padx=5)





# ===============================================================
# CURRENT SENSOR CONFIG (READ ONLY FROM CAN)
# ===============================================================
tk.Label(root, text="--- Current Sensor Config (from ESP32) ---",
         font=("Arial", 11, "bold")).grid(row=9, column=0, sticky="w", padx=5, pady=10)

cfg_ledfreq  = tk.Label(root, text="LED Freq: ?", font=("Arial", 10)); cfg_ledfreq.grid(row=10, column=0, sticky="w", padx=10)
cfg_ledcurr  = tk.Label(root, text="LED Current: ?", font=("Arial", 10)); cfg_ledcurr.grid(row=11, column=0, sticky="w", padx=10)
cfg_proxres  = tk.Label(root, text="Prox Res: ?", font=("Arial", 10)); cfg_proxres.grid(row=12, column=0, sticky="w", padx=10)
cfg_proxrate = tk.Label(root, text="Prox Rate: ?", font=("Arial", 10)); cfg_proxrate.grid(row=13, column=0, sticky="w", padx=10)
cfg_colres   = tk.Label(root, text="Color Res: ?", font=("Arial", 10)); cfg_colres.grid(row=14, column=0, sticky="w", padx=10)
cfg_colrate  = tk.Label(root, text="Color Rate: ?", font=("Arial", 10)); cfg_colrate.grid(row=15, column=0, sticky="w", padx=10)
cfg_gain     = tk.Label(root, text="Gain: ?", font=("Arial", 10)); cfg_gain.grid(row=16, column=0, sticky="w", padx=10)


# ===============================================================
# CONFIG SEND UI (User Controls)
# ===============================================================
def make_dropdown(label, mapping, row):
    tk.Label(root, text=label, font=("Arial", 10))\
        .grid(row=row, column=1, sticky="e", padx=5)
    var = tk.StringVar()
    cb = ttk.Combobox(root, textvariable=var,
                      values=list(mapping.keys()), state="readonly", width=15)
    cb.grid(row=row, column=2, padx=3, pady=2)
    cb.current(0)
    return cb, var


freq_cb,  freq_var  = make_dropdown("LED Freq", LED_FREQ, 10)
curr_cb,  curr_var  = make_dropdown("LED Current", LED_CURRENT, 11)
pres_cb,  pres_var  = make_dropdown("Prox Res", PROX_RES, 12)
prate_cb, prate_var = make_dropdown("Prox Rate", PROX_RATE, 13)
cres_cb,  cres_var  = make_dropdown("Color Res", COLOR_RES, 14)
crate_cb, crate_var = make_dropdown("Color Rate", COLOR_RATE, 15)
gain_cb,  gain_var  = make_dropdown("Gain", GAIN, 16)


# ===============================================================
# SEND CONFIG (0x187)
# ===============================================================
def send_config(reboot=False):
    apply_device_number()

    data = [
        1 if reboot else 0,
        LED_FREQ[freq_var.get()],
        LED_CURRENT[curr_var.get()],
        PROX_RES[pres_var.get()],
        PROX_RATE[prate_var.get()],
        COLOR_RES[cres_var.get()],
        COLOR_RATE[crate_var.get()],
        GAIN[gain_var.get()]
    ]

    msg = can.Message(
        arbitration_id=make_can_id(API_COLOR_CONFIG),
        data=bytes(data),
        is_extended_id=True
    )

    try:
        bus.send(msg)
    except Exception as e:
        print("CAN send error:", e)


tk.Button(root, text="Send Config", width=15,
          command=lambda: send_config(False)).grid(row=17, column=1, pady=10)

tk.Button(root, text="Reboot ESP32", width=15,
          command=lambda: send_config(True)).grid(row=17, column=2, pady=10)


# ===============================================================
# CAN RECEIVE THREAD
# ===============================================================
def can_rx_worker():
    while True:
        msg = bus.recv()

        if msg is None:
            continue

        # Filter by device number
        if extract_devnum(msg.arbitration_id) != DEVICE_NUMBER:
            continue

        api = extract_api(msg.arbitration_id)

        # ----------- SENSOR DATA (0x184) -------------------------
        if api == API_COLOR_DATA1 and len(msg.data) == 8:
            red   = (msg.data[0] << 8) | msg.data[1]
            green = (msg.data[2] << 8) | msg.data[3]
            blue  = (msg.data[4] << 8) | msg.data[5]
            prox  = (msg.data[6] << 8) | msg.data[7]

            lbl_red.config(text=f"Red: {red}")
            lbl_green.config(text=f"Green: {green}")
            lbl_blue.config(text=f"Blue: {blue}")
            lbl_prox.config(text=f"Proximity: {prox}")

        

        # ----------- CONFIG DATA (0x185) -------------------------
        elif api == API_COLOR_DATA2 and len(msg.data) == 8:
            ir = (msg.data[0] << 8) | msg.data[1]
            lbl_ir.config(text=f"IR: {ir}")

            cfg_ledfreq.config(text=f"LED Freq: {LED_FREQ_REV.get(msg.data[2], msg.data[2])}")
            cfg_ledcurr.config(text=f"LED Current: {LED_CURRENT_REV.get(msg.data[3], msg.data[3])}")
            cfg_proxres.config(text=f"Prox Res: {PROX_RES_REV.get(msg.data[4], msg.data[4])}")
            cfg_proxrate.config(text=f"Prox Rate: {PROX_RATE_REV.get(msg.data[5], msg.data[5])}")
            cfg_colres.config(text=f"Color Res: {COLOR_RES_REV.get(msg.data[6], msg.data[6])}")
            cfg_colrate.config(text=f"Color Rate: {COLOR_RATE_REV.get(msg.data[7], msg.data[7])}")

        # ----------- STATUS (0x186) ------------------------------
        elif api == API_COLOR_STATUS and len(msg.data) == 8:
            online = msg.data[1]
            gain   = msg.data[0]

            lbl_online.config(text=f"Online: {online}")
            cfg_gain.config(text=f"Gain: {GAIN_REV.get(gain, gain)}")


threading.Thread(target=can_rx_worker, daemon=True).start()


# ===============================================================
# RUN GUI
# ===============================================================
root.mainloop()
