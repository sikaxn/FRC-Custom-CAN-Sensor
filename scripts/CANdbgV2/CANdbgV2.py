import json
import os
import threading
import time
import tkinter as tk
from tkinter import ttk

import can
from can.notifier import Notifier, Listener


# --- Global Data ---
can_messages = {}
last_updated = {}
tree_items = {}
paused = False
show_hex = True
refresh_scheduled = False
decode_windows = {}

# Heartbeat
HEARTBEAT_ID = 0x01011840
last_heartbeat_time = 0
decoded_heartbeat = None
heartbeat_status_text = "No roboRIO heartbeat detected."

# JSON Frame Definitions
FRAME_DEFS = {}

# Maps
DEVICE_TYPE_MAP = {
    0: "Broadcast", 1: "Robot Controller", 2: "Motor Controller", 3: "Relay Controller",
    4: "Gyro Sensor", 5: "Accelerometer", 6: "Ultrasonic Sensor", 7: "Gear Tooth Sensor",
    8: "Power Distribution Module", 9: "Pneumatics Controller", 10: "Miscellaneous",
    11: "IO Breakout", 12: "Servo Controller", 31: "Firmware Update"
}
MANUFACTURER_MAP = {
    0: "Broadcast", 1: "NI", 2: "Luminary Micro", 3: "DEKA", 4: "CTR Electronics",
    5: "REV Robotics", 6: "Grapple", 7: "MindSensors", 8: "Team Use", 9: "Kauai Labs",
    10: "Copperforge", 11: "Playing With Fusion", 12: "Studica", 13: "The Thrifty Bot",
    14: "Redux Robotics", 15: "AndyMark", 16: "Vivid Hosting"
}

BROADCAST_API_MAP = {
    0: "Disable",
    1: "System Halt",
    2: "System Reset",
    3: "Device Assign",
    4: "Device Query",
    5: "Heartbeat",
    6: "Sync",
    7: "Update",
    8: "Firmware Version",
    9: "Enumerate",
    10: "System Resume"
}


# --- Decode ---
def get_bits(bitstring, start, length):
    return int(bitstring[start:start + length], 2)


def decode_frc_payload(payload):
    bits = ''.join(f'{b:08b}' for b in payload)
    return {
        "match_time":      get_bits(bits, 56, 8),
        "match_number":    get_bits(bits, 46, 10),
        "replay_number":   get_bits(bits, 40, 6),
        "red_alliance":    get_bits(bits, 39, 1),
        "enabled":         get_bits(bits, 38, 1),
        "autonomous":      get_bits(bits, 37, 1),
        "test":            get_bits(bits, 36, 1),
        "watchdog":        get_bits(bits, 35, 1),
        "tournament_type": get_bits(bits, 32, 3),
        "year":            2000 + get_bits(bits, 26, 6) - 36,
        "month":           get_bits(bits, 22, 4) + 1,
        "day":             get_bits(bits, 17, 5),
        "seconds":         get_bits(bits, 11, 6),
        "minutes":         get_bits(bits, 5, 6),
        "hours":           get_bits(bits, 0, 5),
    }


def format_time(d):
    return f"{d['year']:04}-{d['month']:02}-{d['day']:02} {d['hours']:02}:{d['minutes']:02}:{min(d['seconds'], 59):02}"


def get_name(map_dict, value):
    return f"0x{value:02X} / {value} ({map_dict.get(value, 'Unknown')})"


def parse_frc_id(can_id):
    return (can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF, can_id & 0x3F, (can_id >> 6) & 0x3FF


def split_api_id(api_id):
    return (api_id >> 4) & 0x3F, api_id & 0x0F


def format_value(value):
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, float):
        if value.is_integer():
            return str(int(value))
        return f"{value:.3f}".rstrip("0").rstrip(".")
    return str(value)

def format_range(signal):
    sig_type = str(signal.get("type", "")).lower()
    if sig_type == "boolean":
        return "true or false"

    decoded_min = signal.get("decodedMin")
    decoded_max = signal.get("decodedMax")
    if decoded_min is not None or decoded_max is not None:
        return f"{decoded_min} <-> {decoded_max}"

    encoded_min = signal.get("encodedMin")
    encoded_max = signal.get("encodedMax")
    if encoded_min is None or encoded_max is None:
        return ""

    scale = signal.get("decodeScaleFactor", 1)
    offset = signal.get("offset", 0)
    try:
        decoded_min = (encoded_min * scale) + offset
        decoded_max = (encoded_max * scale) + offset
        return f"{decoded_min} <-> {decoded_max}"
    except Exception:
        return f"{encoded_min} <-> {encoded_max}"


def decode_signal_value(data, signal):
    bit_pos = int(signal.get("bitPosition", 0))
    length_bits = int(signal.get("lengthBits", 0))
    if length_bits <= 0:
        return None

    is_big = bool(signal.get("isBigEndian", False))
    bit_len = len(data) * 8
    if bit_pos + length_bits > bit_len:
        return None

    if is_big:
        bits = ''.join(f'{b:08b}' for b in data)
        raw_bits = bits[bit_pos:bit_pos + length_bits]
        raw = int(raw_bits, 2) if raw_bits else 0
    else:
        raw_int = int.from_bytes(data, byteorder="little", signed=False)
        raw = (raw_int >> bit_pos) & ((1 << length_bits) - 1)

    sig_type = str(signal.get("type", "uint")).lower()
    if sig_type == "ascii":
        if bit_pos % 8 != 0 or length_bits % 8 != 0:
            return None
        byte_start = bit_pos // 8
        byte_len = length_bits // 8
        if byte_start + byte_len > len(data):
            return None
        raw_bytes = bytes(data[byte_start:byte_start + byte_len])
        try:
            return raw_bytes.decode("ascii", errors="ignore").rstrip("\x00")
        except Exception:
            return None
    if sig_type in ("int", "sint", "signed"):
        sign_bit = 1 << (length_bits - 1)
        if raw & sign_bit:
            raw -= 1 << length_bits

    if sig_type == "boolean":
        return bool(raw)

    scale = signal.get("decodeScaleFactor", 1)
    offset = signal.get("offset", 0)
    try:
        return (raw * scale) + offset
    except Exception:
        return raw


def decode_frame(data, frame_def):
    signals = frame_def.get("signals", {})
    decoded_entries = []
    signal_items = []
    for key, spec in signals.items():
        name = spec.get("name") or key
        signal_items.append((spec.get("bitPosition", 0), name, spec))

    for _, name, spec in sorted(signal_items, key=lambda item: item[0]):
        value = decode_signal_value(data, spec)
        if value is None:
            continue
        decoded_entries.append({
            "name": name,
            "value": format_value(value),
            "type": str(spec.get("type", "uint")),
            "range": format_range(spec)
        })

    if not decoded_entries:
        return None

    return decoded_entries


def load_frame_definitions(folder):
    device_frames = {}
    if not os.path.isdir(folder):
        return device_frames

    for filename in os.listdir(folder):
        path = os.path.join(folder, filename)
        if not os.path.isfile(path):
            continue
        if filename.lower().endswith(".html"):
            continue

        try:
            with open(path, "r", encoding="utf-8") as f:
                spec = json.load(f)
        except Exception as exc:
            print(f"JSON load failed for {filename}: {exc}")
            continue

        device_info = spec.get("deviceInfo", {})
        device_type = device_info.get("deviceTypeNumber")
        manufacturer = device_info.get("manufacturerNumber")
        if device_type is None or manufacturer is None:
            continue

        frame_map = {}
        for group_key in ("periodicFrames", "controlFrames", "faultFrames", "statusFrames", "frames"):
            frames = spec.get(group_key, {})
            if not isinstance(frames, dict):
                continue
            for _, frame_def in frames.items():
                api_class = frame_def.get("apiClass")
                api_index = frame_def.get("apiIndex")
                if api_class is None or api_index is None:
                    continue
                api_id = ((int(api_class) & 0x3F) << 4) | (int(api_index) & 0x0F)
                frame_map[api_id] = frame_def

        device_frames[(int(device_type), int(manufacturer))] = frame_map

    return device_frames


# --- GUI Update ---
def debounced_refresh():
    global refresh_scheduled
    refresh_table_named()
    refresh_scheduled = False


def update_heartbeat_display():
    global heartbeat_status_text
    if last_heartbeat_time == 0:
        heartbeat_label.config(fg="gray", text="No roboRIO heartbeat detected.")
    else:
        time_since = time.time() - last_heartbeat_time
        heartbeat_label.config(fg="gray" if time_since > 1.0 else "black")
        if decoded_heartbeat:
            d = decoded_heartbeat
            heartbeat_label.config(text=(
                f"[{format_time(d)}] "
                f"{'RED' if d['red_alliance'] else 'BLUE'} | "
                f"{'ENABLED' if d['enabled'] else 'DISABLED'} | "
                f"{'AUTO' if d['autonomous'] else 'TELEOP'} | "
                f"Match {d['match_number']} Replay {d['replay_number']} | "
                f"Time Left: {d['match_time']}s"
            ))
    root.after(200, update_heartbeat_display)


def build_message_text(entry):
    return entry['raw_hex'] if show_hex else entry['raw_dec']


def refresh_table_named():
    grouped = {}
    for msg_id, entry in can_messages.items():
        key = (entry['device_type'], entry['manufacturer'], entry['device_number'])
        grouped.setdefault(key, []).append((msg_id, entry))

    for (device_type, manufacturer, device_number), messages in grouped.items():
        parent_key = f"{device_type}_{manufacturer}_{device_number}"
        dev_type_str = get_name(DEVICE_TYPE_MAP, device_type)
        manuf_str = get_name(MANUFACTURER_MAP, manufacturer)
        dev_num_str = f"0x{device_number:02X} / {device_number}"

        if not tree.exists(parent_key):
            tree.insert("", "end", iid=parent_key,
                        values=("", dev_type_str, manuf_str, dev_num_str, "", "", "", "", "(combined)"),
                        tags=("group",))
        else:
            tree.item(parent_key, values=("", dev_type_str, manuf_str, dev_num_str, "", "", "", "", "(combined)"))

        for msg_id, entry in sorted(messages):
            api = f"0x{entry['api_id']:03X}"
            api_class = f"0x{entry['api_class']:02X}"
            api_index = f"0x{entry['api_index']:01X}"
            data = build_message_text(entry)
            decoded_available = "Yes" if entry.get("decoded_entries") else "No"
            dev_type_str = get_name(DEVICE_TYPE_MAP, entry['device_type'])
            manuf_str = get_name(MANUFACTURER_MAP, entry['manufacturer'])
            dev_num_str = f"0x{entry['device_number']:02X} / {entry['device_number']}"
            row_data = (
                f"0x{msg_id:08X}",
                dev_type_str,
                manuf_str,
                dev_num_str,
                api,
                api_class,
                api_index,
                decoded_available,
                data
            )
            iid = f"{parent_key}_{msg_id:08X}"
            is_stale = (time.time() - last_updated.get(msg_id, 0)) > 1.0
            tag = "stale" if is_stale else ""

            if tree.exists(iid):
                tree.item(iid, values=row_data, tags=(tag,))
            else:
                tree.insert(parent_key, "end", iid=iid, values=row_data, tags=(tag,))

            tree_items[msg_id] = iid


# --- Listener Class ---
class CANMessageListener(Listener):
    def on_message_received(self, msg):
        global decoded_heartbeat, last_heartbeat_time, refresh_scheduled

        if paused:
            return

        msg_id = msg.arbitration_id
        device_type, manufacturer, device_number, api_id = parse_frc_id(msg_id)
        api_class, api_index = split_api_id(api_id)
        data_hex = ' '.join(f'{b:02X}' for b in msg.data)
        data_dec = ' '.join(str(b) for b in msg.data)

        decoded_entries = None
        if msg_id == HEARTBEAT_ID:
            try:
                decoded = decode_frc_payload(msg.data)
                decoded_heartbeat = decoded
                last_heartbeat_time = time.time()
                decoded_entries = [
                    {"name": "Timestamp", "value": format_time(decoded), "type": "datetime", "range": ""},
                    {"name": "Alliance", "value": "RED" if decoded["red_alliance"] else "BLUE", "type": "enum", "range": ""},
                    {"name": "Enabled", "value": "Yes" if decoded["enabled"] else "No", "type": "bool", "range": ""},
                    {"name": "Mode", "value": "AUTO" if decoded["autonomous"] else "TELEOP", "type": "enum", "range": ""},
                    {"name": "Match", "value": decoded["match_number"], "type": "uint", "range": ""},
                    {"name": "Replay", "value": decoded["replay_number"], "type": "uint", "range": ""},
                    {"name": "Time Left (s)", "value": decoded["match_time"], "type": "uint", "range": ""}
                ]
            except Exception:
                decoded_entries = None
        else:
            frame_map = FRAME_DEFS.get((device_type, manufacturer))
            if frame_map:
                frame_def = frame_map.get(api_id)
                if frame_def:
                    decoded_entries = decode_frame(msg.data, frame_def)
            if decoded_entries is None and device_type == 0 and manufacturer == 0 and api_class == 0:
                decoded_entries = [{
                    "name": "Broadcast",
                    "value": BROADCAST_API_MAP.get(api_index, "Unknown"),
                    "type": "enum",
                    "range": ""
                }]

        can_messages[msg_id] = {
            'device_type': device_type,
            'manufacturer': manufacturer,
            'device_number': device_number,
            'api_id': api_id,
            'api_class': api_class,
            'api_index': api_index,
            'raw_hex': data_hex,
            'raw_dec': data_dec,
            'decoded_entries': decoded_entries
        }
        last_updated[msg_id] = time.time()

        if not refresh_scheduled:
            refresh_scheduled = True
            root.after(50, debounced_refresh)


base_dir = os.path.dirname(__file__)
FRAME_DEFS = {}
FRAME_DEFS.update(load_frame_definitions(os.path.join(base_dir, "frame-json")))
FRAME_DEFS.update(load_frame_definitions(os.path.join(base_dir, "im-frame-json")))

# --- UI Setup ---
root = tk.Tk()
root.title("CAN Message Viewer V2")
root.geometry("1120x600")
root.attributes("-topmost", True)

frame = tk.Frame(root)
frame.pack(fill="both", expand=True)

vsb = ttk.Scrollbar(frame, orient="vertical")
vsb.pack(side="right", fill="y")

columns = ("msg_id", "device_id", "manuf_id", "dev_num", "api_id", "api_class", "api_index", "decode_available", "message")
tree = ttk.Treeview(frame, columns=columns, show="tree headings", yscrollcommand=vsb.set)
tree.heading("#0", text="Group")
tree.column("#0", width=30, anchor="w")
tree.tag_configure("group", background="#d1e7dd")
tree.tag_configure("stale", background="#e0e0e0")

for col in columns:
    tree.heading(col, text=col.replace("_", " ").upper())
tree.column("msg_id", width=100)
tree.column("device_id", width=160)
tree.column("manuf_id", width=160)
tree.column("dev_num", width=90)
tree.column("api_id", width=80)
tree.column("api_class", width=90)
tree.column("api_index", width=80)
tree.column("decode_available", width=120)
tree.column("message", width=520)
tree.pack(side="left", fill="both", expand=True)
vsb.config(command=tree.yview)

# Controls
control_frame = tk.Frame(root)
control_frame.pack(fill="x", pady=5)

pause_btn = tk.Button(control_frame, text="Pause", command=lambda: toggle_pause())
pause_btn.pack(side="left", padx=10)

hex_btn = tk.Button(control_frame, text="Show Decimal", command=lambda: toggle_hex())
hex_btn.pack(side="left")

decode_hint = tk.Label(control_frame, text="Double-click a row to open decode window")
decode_hint.pack(side="left", padx=12)

close_all_btn = tk.Button(control_frame, text="Close All Decode Windows", command=lambda: close_all_decode_windows())
close_all_btn.pack(side="left", padx=10)

heartbeat_label = tk.Label(root, text=heartbeat_status_text, anchor="w")
heartbeat_label.pack(fill="x", padx=10, pady=(0, 5))
update_heartbeat_display()


def toggle_pause():
    global paused
    paused = not paused
    pause_btn.config(text="Resume" if paused else "Pause",
                     background="red" if paused else "SystemButtonFace")


def toggle_hex():
    global show_hex
    show_hex = not show_hex
    hex_btn.config(text="Show Decimal" if show_hex else "Show Hex")
    refresh_table_named()


def copy_selection(event=None):
    selection = tree.selection()
    rows = ['\t'.join(tree.item(item, "values")) for item in selection]
    if rows:
        root.clipboard_clear()
        root.clipboard_append('\n'.join(rows))

def close_all_decode_windows():
    for msg_id, win in list(decode_windows.items()):
        if win.winfo_exists():
            win.destroy()
        decode_windows.pop(msg_id, None)


tree.bind("<Control-c>", copy_selection)


# --- Decode Window ---
def open_decode_window(event=None):
    selection = tree.selection()
    if not selection:
        return

    item_id = selection[0]
    if item_id.count("_") < 3:
        return

    try:
        msg_id = int(item_id.split("_")[-1], 16)
    except ValueError:
        return

    entry = can_messages.get(msg_id)
    if not entry:
        return

    existing = decode_windows.get(msg_id)
    if existing and existing.winfo_exists():
        existing.deiconify()
        existing.lift()
        existing.focus_force()
        return

    win = tk.Toplevel(root)
    win.title(f"Decoded Message 0x{msg_id:08X}")
    win.geometry("700x260")
    win.attributes("-topmost", True)
    win.transient(root)
    win.lift()
    decode_windows[msg_id] = win

    def on_close():
        if decode_windows.get(msg_id) is win:
            decode_windows.pop(msg_id, None)
        win.destroy()

    win.protocol("WM_DELETE_WINDOW", on_close)

    raw_frame = ttk.Frame(win)
    raw_frame.pack(fill="both", expand=True)

    info_label = tk.Label(raw_frame, anchor="w", justify="left", wraplength=660)
    info_label.pack(fill="x", padx=10, pady=(10, 6))

    raw_text = entry['raw_hex'] if show_hex else entry['raw_dec']
    raw_label = tk.Label(raw_frame, text=raw_text, anchor="w", justify="left", wraplength=660)
    raw_label.pack(fill="x", padx=10, pady=(0, 6))

    decode_frame = ttk.Frame(raw_frame)
    decode_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    decode_vsb = ttk.Scrollbar(decode_frame, orient="vertical")
    decode_vsb.pack(side="right", fill="y")

    decode_columns = ("name", "value", "type", "range")
    decode_tree = ttk.Treeview(
        decode_frame,
        columns=decode_columns,
        show="headings",
        yscrollcommand=decode_vsb.set,
        height=8
    )
    for col in decode_columns:
        decode_tree.heading(col, text=col.upper())
    decode_tree.column("name", width=180)
    decode_tree.column("value", width=200)
    decode_tree.column("type", width=80)
    decode_tree.column("range", width=180)
    decode_tree.pack(side="left", fill="both", expand=True)
    decode_vsb.config(command=decode_tree.yview)

    def copy_decode_selection(event=None):
        selection = decode_tree.selection()
        rows = ['\t'.join(decode_tree.item(item, "values")) for item in selection]
        if rows:
            win.clipboard_clear()
            win.clipboard_append('\n'.join(rows))

    decode_tree.bind("<Control-c>", copy_decode_selection)

    def refresh_decode_text():
        current = can_messages.get(msg_id)
        if not current or not win.winfo_exists():
            return
        dev_type_str = get_name(DEVICE_TYPE_MAP, current['device_type'])
        manuf_str = get_name(MANUFACTURER_MAP, current['manufacturer'])
        dev_num_str = f"0x{current['device_number']:02X} / {current['device_number']}"
        api_id_str = f"0x{current['api_id']:03X}"
        api_class_str = f"0x{current['api_class']:02X}"
        api_index_str = f"0x{current['api_index']:01X}"
        msg_id_str = f"0x{msg_id:08X}"
        decode_available = "Yes" if current.get("decoded_lines") else "No"
        info_label.config(text=(
            f"MSG ID: {msg_id_str} | "
            f"DEVICE: {dev_type_str} | "
            f"MANUF: {manuf_str} | "
            f"DEV NUM: {dev_num_str} | "
            f"API ID: {api_id_str} | "
            f"API CLASS: {api_class_str} | "
            f"API INDEX: {api_index_str} | "
            f"DECODE AVAILABLE: {decode_available}"
        ))
        raw_text = current['raw_hex'] if show_hex else current['raw_dec']
        raw_label.config(text=raw_text)

        decode_tree.delete(*decode_tree.get_children())
        entries = current.get("decoded_entries") or []
        if not entries:
            decode_tree.insert("", "end", values=("No decoded data available.", "", "", ""))
        else:
            for item in entries:
                decode_tree.insert(
                    "",
                    "end",
                    values=(item.get("name", ""),
                            item.get("value", ""),
                            item.get("type", ""),
                            item.get("range", ""))
                )
        win.after(200, refresh_decode_text)

    refresh_decode_text()


tree.bind("<Double-1>", open_decode_window)


# --- Start CAN Listener ---
def start_can():
    try:
        bus = can.Bus(interface='canalystii', channel=0, device=0, bitrate=1000000)  # canalystii
        # bus = can.Bus(interface='gs_usb', channel=0, bitrate=1000000)  # Canable
        # bus = can.Bus(bustype='slcan', channel='COM8', bitrate=1000000)

        Notifier(bus, [CANMessageListener()], timeout=1)
        print("CAN interface ready.")
    except Exception as e:
        print(f"CAN init error: {e}")


threading.Thread(target=start_can, daemon=True).start()
root.mainloop()
