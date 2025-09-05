import tkinter as tk
from tkinter import ttk
import threading
import time
import can
from can.notifier import Notifier, Listener

#import usb # needed for Canable


# --- Global Data ---
can_messages = {}
last_updated = {}
tree_items = {}
paused = False
show_hex = True
refresh_scheduled = False

# Heartbeat
HEARTBEAT_ID = 0x01011840
last_heartbeat_time = 0
decoded_heartbeat = None
heartbeat_status_text = "No roboRIO heartbeat detected."

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
                        values=("", dev_type_str, manuf_str, dev_num_str, "", "(combined)"),
                        tags=("group",))
        else:
            tree.item(parent_key, values=("", dev_type_str, manuf_str, dev_num_str, "", "(combined)"))

        for msg_id, entry in sorted(messages):
            api = f"0x{entry['api_id']:03X}"
            data = entry['data_hex'] if show_hex else entry['data_dec']
            dev_type_str = get_name(DEVICE_TYPE_MAP, entry['device_type'])
            manuf_str = get_name(MANUFACTURER_MAP, entry['manufacturer'])
            dev_num_str = f"0x{entry['device_number']:02X} / {entry['device_number']}"
            row_data = (f"0x{msg_id:08X}", dev_type_str, manuf_str, dev_num_str, api, data)
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
        data_hex = ' '.join(f'{b:02X}' for b in msg.data)
        data_dec = ' '.join(str(b) for b in msg.data)

        if msg_id == HEARTBEAT_ID:
            try:
                decoded = decode_frc_payload(msg.data)
                decoded_heartbeat = decoded
                last_heartbeat_time = time.time()
                data_text = (
                    f"[{format_time(decoded)}] "
                    f"{'RED' if decoded['red_alliance'] else 'BLUE'} | "
                    f"{'ENABLED' if decoded['enabled'] else 'DISABLED'} | "
                    f"{'AUTO' if decoded['autonomous'] else 'TELEOP'} | "
                    f"Match {decoded['match_number']} Replay {decoded['replay_number']} | "
                    f"Time Left: {decoded['match_time']}s"
                )
            except Exception:
                data_text = data_hex if show_hex else data_dec
        else:
            data_text = data_hex if show_hex else data_dec

        can_messages[msg_id] = {
            'device_type': device_type,
            'manufacturer': manufacturer,
            'device_number': device_number,
            'api_id': api_id,
            'data_hex': data_hex,
            'data_dec': data_dec,
            'data': data_text
        }
        last_updated[msg_id] = time.time()

        if not refresh_scheduled:
            refresh_scheduled = True
            root.after(50, debounced_refresh)

# --- UI Setup ---
root = tk.Tk()
root.title("CAN Message Viewer")
root.geometry("1000x600")

frame = tk.Frame(root)
frame.pack(fill="both", expand=True)

vsb = ttk.Scrollbar(frame, orient="vertical")
vsb.pack(side="right", fill="y")

columns = ("msg_id", "device_id", "manuf_id", "dev_num", "api_id", "message")
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
tree.column("api_id", width=90)
tree.column("message", width=500)
tree.pack(side="left", fill="both", expand=True)
vsb.config(command=tree.yview)

# Controls
control_frame = tk.Frame(root)
control_frame.pack(fill="x", pady=5)

pause_btn = tk.Button(control_frame, text="Pause", command=lambda: toggle_pause())
pause_btn.pack(side="left", padx=10)

hex_btn = tk.Button(control_frame, text="Show Decimal", command=lambda: toggle_hex())
hex_btn.pack(side="left")

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
    rows = [ '\t'.join(tree.item(item, "values")) for item in selection ]
    if rows:
        root.clipboard_clear()
        root.clipboard_append('\n'.join(rows))

tree.bind("<Control-c>", copy_selection)

# --- Start CAN Listener ---
def start_can():
    try:
        bus = can.Bus(interface='canalystii', channel=0, device=0, bitrate=1000000) #setting if using canalystii
        #bus = can.Bus(interface='gs_usb', channel=0, bitrate=1000000) # setting if using Canable
        #bus = can.Bus(bustype='slcan', channel='COM8', bitrate=1000000)

        Notifier(bus, [CANMessageListener()], timeout=1)
        print("CAN interface ready.")
    except Exception as e:
        print(f"CAN init error: {e}")

threading.Thread(target=start_can, daemon=True).start()
root.mainloop()
