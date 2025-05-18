#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox

def decode_id(msg_id):
    # 29-bit FRC CAN ID breakdown:
    # [Device Type:8][Manufacturer:8][API/Message:10][Device Number:6]
    device_type     = (msg_id >> 24) & 0xFF
    manufacturer_id = (msg_id >> 16) & 0xFF
    api_id          = (msg_id >>  6) & 0x3FF
    device_number   = msg_id & 0x3F
    return device_type, manufacturer_id, api_id, device_number

def encode_id(device_type, manufacturer_id, api_id, device_number):
    # Rebuild 29-bit CAN ID
    return ((device_type & 0xFF) << 24) | ((manufacturer_id & 0xFF) << 16) | ((api_id & 0x3FF) << 6) | (device_number & 0x3F)

def on_decode():
    raw = decode_var.get().strip()
    try:
        msg = int(raw, 0)
    except ValueError:
        messagebox.showerror("Invalid Input", f"Cannot parse '{raw}' as hex or int")
        return
    dt, man, api, dn = decode_id(msg)
    decode_result_var.set(f"0x{msg:08X}")
    decode_dt_var.set(f"0x{dt:02X} ({dt})")
    decode_man_var.set(f"0x{man:02X} ({man})")
    decode_api_var.set(f"0x{api:03X} ({api})")
    decode_dn_var.set(f"{dn}")

def on_encode():
    try:
        dt = int(encode_dt_var.get().strip(), 0)
        man = int(encode_man_var.get().strip(), 0)
        api = int(encode_api_var.get().strip(), 0)
        dn = int(encode_dn_var.get().strip(), 0)
    except ValueError as e:
        messagebox.showerror("Invalid Input", str(e))
        return
    msg = encode_id(dt, man, api, dn)
    encode_result_var.set(f"0x{msg:08X}")

app = tk.Tk()
app.title("FRC CAN ID Bidirectional Converter")

main = ttk.Frame(app, padding=12)
main.grid(column=0, row=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Decode section
dec_frame = ttk.LabelFrame(main, text="Decode CAN ID", padding=10)
dec_frame.grid(column=0, row=0, padx=5, pady=5, sticky=(tk.W, tk.E))

ttk.Label(dec_frame, text="Full CAN ID:").grid(column=0, row=0, sticky=tk.W)
decode_var = tk.StringVar(value="0x0A086003")
ttk.Entry(dec_frame, textvariable=decode_var, width=20).grid(column=1, row=0, sticky=(tk.W, tk.E))
ttk.Button(dec_frame, text="Decode", command=on_decode).grid(column=2, row=0, padx=5)

decode_result_var = tk.StringVar()
ttk.Label(dec_frame, text="Raw ID:").grid(column=0, row=1, sticky=tk.W)
ttk.Label(dec_frame, textvariable=decode_result_var).grid(column=1, row=1, columnspan=2, sticky=tk.W)

decode_dt_var = tk.StringVar()
ttk.Label(dec_frame, text="Device Type:").grid(column=0, row=2, sticky=tk.W)
ttk.Label(dec_frame, textvariable=decode_dt_var).grid(column=1, row=2, columnspan=2, sticky=tk.W)

decode_man_var = tk.StringVar()
ttk.Label(dec_frame, text="Manufacturer ID:").grid(column=0, row=3, sticky=tk.W)
ttk.Label(dec_frame, textvariable=decode_man_var).grid(column=1, row=3, columnspan=2, sticky=tk.W)

decode_api_var = tk.StringVar()
ttk.Label(dec_frame, text="API ID:").grid(column=0, row=4, sticky=tk.W)
ttk.Label(dec_frame, textvariable=decode_api_var).grid(column=1, row=4, columnspan=2, sticky=tk.W)

decode_dn_var = tk.StringVar()
ttk.Label(dec_frame, text="Device Number:").grid(column=0, row=5, sticky=tk.W)
ttk.Label(dec_frame, textvariable=decode_dn_var).grid(column=1, row=5, columnspan=2, sticky=tk.W)

# Encode section
enc_frame = ttk.LabelFrame(main, text="Encode to CAN ID", padding=10)
enc_frame.grid(column=0, row=1, padx=5, pady=5, sticky=(tk.W, tk.E))

ttk.Label(enc_frame, text="Device Type:").grid(column=0, row=0, sticky=tk.W)
encode_dt_var = tk.StringVar(value="0x0A")
ttk.Entry(enc_frame, textvariable=encode_dt_var, width=10).grid(column=1, row=0, sticky=tk.W)

ttk.Label(enc_frame, text="Manufacturer ID:").grid(column=0, row=1, sticky=tk.W)
encode_man_var = tk.StringVar(value="0x08")
ttk.Entry(enc_frame, textvariable=encode_man_var, width=10).grid(column=1, row=1, sticky=tk.W)

ttk.Label(enc_frame, text="API ID:").grid(column=0, row=2, sticky=tk.W)
encode_api_var = tk.StringVar(value="0x180")
ttk.Entry(enc_frame, textvariable=encode_api_var, width=10).grid(column=1, row=2, sticky=tk.W)

ttk.Label(enc_frame, text="Device Number:").grid(column=0, row=3, sticky=tk.W)
encode_dn_var = tk.StringVar(value="0")
ttk.Entry(enc_frame, textvariable=encode_dn_var, width=10).grid(column=1, row=3, sticky=tk.W)

ttk.Button(enc_frame, text="Encode", command=on_encode).grid(column=2, row=1, rowspan=3, padx=5)

encode_result_var = tk.StringVar()
ttk.Label(enc_frame, text="Full CAN ID:").grid(column=0, row=4, sticky=tk.W)
ttk.Label(enc_frame, textvariable=encode_result_var).grid(column=1, row=4, columnspan=2, sticky=tk.W)

# Layout adjustments
app.columnconfigure(0, weight=1)
main.columnconfigure(0, weight=1)
dec_frame.columnconfigure(1, weight=1)
enc_frame.columnconfigure(1, weight=1)

app.mainloop()