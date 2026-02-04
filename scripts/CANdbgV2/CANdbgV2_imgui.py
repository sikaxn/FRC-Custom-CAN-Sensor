import json
import os
import platform
import threading
import time

import can
from imgui_bundle import imgui, immapp


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


class CanDbgImGui:
    def __init__(self):
        self.can_messages = {}
        self.last_updated = {}
        self.paused = False
        self.show_hex = True
        self.selected_msg_id = None
        self.heartbeat_last_time = 0
        self.decoded_heartbeat = None

        self.frame_defs = {}
        self.load_frame_definitions()

        self.always_on_top = True
        self.debug_trace = False
        self.interface = "canalystii"
        self.channel = "0"
        self.config_path = os.path.join(os.path.dirname(__file__), "config.json")
        self.load_config()

        self.start_can()

    # --- CAN + Decode ---
    @staticmethod
    def parse_frc_id(can_id):
        return (can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF, can_id & 0x3F, (can_id >> 6) & 0x3FF

    @staticmethod
    def decode_frc_payload(payload):
        bits = ''.join(f'{b:08b}' for b in payload)
        def get_bits(bitstring, start, length):
            return int(bitstring[start:start + length], 2)
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

    @staticmethod
    def format_time(d):
        return f"{d['year']:04}-{d['month']:02}-{d['day']:02} {d['hours']:02}:{d['minutes']:02}:{min(d['seconds'], 59):02}"

    @staticmethod
    def split_api_id(api_id):
        return (api_id >> 4) & 0x3F, api_id & 0x0F

    @staticmethod
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

    @staticmethod
    def format_value(value):
        if isinstance(value, bool):
            return "True" if value else "False"
        if isinstance(value, float):
            if value.is_integer():
                return str(int(value))
            return f"{value:.3f}".rstrip("0").rstrip(".")
        return str(value)

    @staticmethod
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

    def decode_frame(self, data, frame_def):
        signals = frame_def.get("signals", {})
        entries = []
        items = []
        for key, spec in signals.items():
            name = spec.get("name") or key
            items.append((spec.get("bitPosition", 0), name, spec))

        for _, name, spec in sorted(items, key=lambda item: item[0]):
            value = self.decode_signal_value(data, spec)
            if value is None:
                continue
            entries.append({
                "name": name,
                "value": self.format_value(value),
                "type": str(spec.get("type", "uint")),
                "range": self.format_range(spec),
                "description": spec.get("description", "")
            })
        return entries

    def load_frame_definitions(self):
        base_dir = os.path.dirname(__file__)
        for folder in ("frame-json", "im-frame-json"):
            path = os.path.join(base_dir, folder)
            if not os.path.isdir(path):
                continue
            for filename in os.listdir(path):
                if not filename.lower().endswith(".json"):
                    continue
                full = os.path.join(path, filename)
                try:
                    with open(full, "r", encoding="utf-8") as f:
                        spec = json.load(f)
                except Exception:
                    continue

                device_info = spec.get("deviceInfo", {})
                device_type = device_info.get("deviceTypeNumber")
                manufacturer = device_info.get("manufacturerNumber")
                if device_type is None or manufacturer is None:
                    continue

                frame_map = {}
                for group_key in ("periodicFrames", "nonPeriodicFrames", "controlFrames", "faultFrames", "statusFrames", "frames"):
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

                self.frame_defs[(int(device_type), int(manufacturer))] = frame_map

    def load_config(self):
        defaults = {
            "always_on_top": True,
            "debug_trace": False,
            "show_hex": True,
            "interface": "canalystii",
            "channel": "0"
        }
        if not os.path.isfile(self.config_path):
            try:
                with open(self.config_path, "w", encoding="utf-8") as f:
                    json.dump(defaults, f, indent=4)
                    f.write("\n")
            except Exception:
                return
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.always_on_top = bool(cfg.get("always_on_top", True))
            self.debug_trace = bool(cfg.get("debug_trace", False))
            self.show_hex = bool(cfg.get("show_hex", True))
            self.interface = str(cfg.get("interface", "canalystii"))
            self.channel = str(cfg.get("channel", "0"))
        except Exception:
            self.always_on_top = True
            self.debug_trace = False
            self.show_hex = True
            self.interface = "canalystii"
            self.channel = "0"

    def save_config(self):
        cfg = {
            "always_on_top": bool(self.always_on_top),
            "debug_trace": bool(self.debug_trace),
            "show_hex": bool(self.show_hex),
            "interface": self.interface,
            "channel": self.channel
        }
        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=4)
                f.write("\n")
        except Exception:
            pass

    @staticmethod
    def get_serial_ports():
        try:
            import serial.tools.list_ports as list_ports
        except Exception:
            return []
        return [port.device for port in list_ports.comports()]

    @staticmethod
    def get_vcan_interfaces():
        if platform.system() == "Windows":
            return []
        try:
            return sorted([name for name in os.listdir("/sys/class/net") if name.startswith("vcan")])
        except Exception:
            return []

    def on_message(self, msg):
        if self.paused:
            return

        msg_id = msg.arbitration_id
        device_type, manufacturer, device_number, api_id = self.parse_frc_id(msg_id)
        data_hex = ' '.join(f'{b:02X}' for b in msg.data)
        data_dec = ' '.join(str(b) for b in msg.data)

        decoded_entries = None
        frame_name = None
        frame_desc = None
        # Heartbeat
        if msg_id == 0x01011840:
            try:
                self.decoded_heartbeat = self.decode_frc_payload(msg.data)
                self.heartbeat_last_time = time.time()
            except Exception:
                pass

        frame_map = self.frame_defs.get((device_type, manufacturer))
        if frame_map:
            frame_def = frame_map.get(api_id)
            if frame_def:
                decoded_entries = self.decode_frame(msg.data, frame_def)
                frame_name = frame_def.get("name")
                frame_desc = frame_def.get("description")
            elif self.debug_trace:
                api_class, api_index = self.split_api_id(api_id)
                print(f"[CANdbgImGui] No frame for API 0x{api_id:03X} ({api_class},{api_index}) "
                      f"device ({device_type},{manufacturer}) devnum {device_number} data {data_hex}")
        elif self.debug_trace:
            api_class, api_index = self.split_api_id(api_id)
            print(f"[CANdbgImGui] No frame map for device ({device_type},{manufacturer}) "
                  f"API 0x{api_id:03X} ({api_class},{api_index}) devnum {device_number} data {data_hex}")

        if decoded_entries is None and device_type == 0 and manufacturer == 0:
            api_class, api_index = self.split_api_id(api_id)
            if api_class == 0:
                decoded_entries = [{
                    "name": "Broadcast",
                    "value": BROADCAST_API_MAP.get(api_index, "Unknown"),
                    "type": "enum",
                    "range": "",
                    "description": ""
                }]

        self.can_messages[msg_id] = {
            "device_type": device_type,
            "manufacturer": manufacturer,
            "device_number": device_number,
            "api_id": api_id,
            "data_hex": data_hex,
            "data_dec": data_dec,
            "decoded_entries": decoded_entries,
            "frame_name": frame_name,
            "frame_desc": frame_desc
        }
        self.last_updated[msg_id] = time.time()

    def start_can(self):
        def run():
            try:
                self.bus = self._connect_can_bus()
                self.notifier = can.Notifier(self.bus, [self.on_message_received])
                print("CAN interface ready.")
            except Exception as exc:
                print(f"CAN init error: {exc}")

        threading.Thread(target=run, daemon=True).start()

    def _connect_can_bus(self):
        iface = self.interface
        chan = self.channel
        if iface == "canalystii":
            return can.Bus(interface="canalystii", channel=int(chan or "0"), device=0, bitrate=1000000)
        if iface == "slcan":
            if not chan:
                raise ValueError("COM port required for slcan")
            return can.Bus(bustype="slcan", channel=chan, bitrate=1000000)
        if iface == "socketcan":
            if platform.system() == "Windows":
                raise ValueError("socketcan is not available on Windows")
            return can.Bus(channel=chan or "vcan0", interface="socketcan")
        raise ValueError(f"Unknown interface: {iface}")

    def reconnect_can(self):
        try:
            if getattr(self, "notifier", None):
                self.notifier.stop()
                self.notifier = None
            if getattr(self, "bus", None):
                self.bus.shutdown()
                self.bus = None
        except Exception:
            pass
        self.start_can()

    def shutdown(self):
        try:
            if getattr(self, "notifier", None):
                self.notifier.stop()
                self.notifier = None
            if getattr(self, "bus", None):
                self.bus.shutdown()
                self.bus = None
        except Exception:
            pass

    # python-can listener hook
    def on_message_received(self, msg):  # noqa: N802
        self.on_message(msg)

    # --- UI ---
    def gui(self):
        if hasattr(immapp, "set_window_always_on_top"):
            immapp.set_window_always_on_top(self.always_on_top)

        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("Window"):
                if imgui.menu_item("Reset Window Layout", "", False)[0]:
                    self._reset_layout = True
                imgui.end_menu()
            imgui.end_main_menu_bar()

        if getattr(self, "_reset_layout", False):
            imgui.set_next_window_pos(imgui.ImVec2(20, 40), imgui.Cond_.always)
            imgui.set_next_window_size(imgui.ImVec2(1300, 700), imgui.Cond_.always)
        else:
            imgui.set_next_window_size(imgui.ImVec2(1300, 700), imgui.Cond_.first_use_ever)

        imgui.begin("Iron Maple FRC-Custom-CAN-Sensor CAN analyzer")

        if imgui.button("Pause" if self.paused else "Pause"):
            self.paused = not self.paused
        imgui.same_line()
        if imgui.button("Clear All"):
            self.can_messages.clear()
            self.last_updated.clear()
            self.selected_msg_id = None
        imgui.same_line()
        if imgui.button("Clear Line") and self.selected_msg_id in self.can_messages:
            self.can_messages.pop(self.selected_msg_id, None)
            self.last_updated.pop(self.selected_msg_id, None)
            self.selected_msg_id = None
        imgui.same_line()
        if imgui.button("Clear Inactive"):
            now = time.time()
            stale_ids = [mid for mid, ts in self.last_updated.items() if (now - ts) > 1.0]
            for mid in stale_ids:
                self.can_messages.pop(mid, None)
                self.last_updated.pop(mid, None)
            if self.selected_msg_id not in self.can_messages:
                self.selected_msg_id = None
        imgui.same_line()
        if imgui.button("Collapse All"):
            self._collapse_all = True
        imgui.same_line()
        changed, self.show_hex = imgui.checkbox("Show Hex", self.show_hex)
        if changed:
            self.save_config()
        imgui.same_line()
        changed, self.always_on_top = imgui.checkbox("Always On Top", self.always_on_top)
        if changed:
            self.save_config()
        imgui.same_line()
        changed, self.debug_trace = imgui.checkbox("Print Debug Trace", self.debug_trace)
        if changed:
            self.save_config()

        imgui.separator()

        imgui.text("CAN Interface")
        imgui.same_line()
        iface_items = ["canalystii", "slcan", "socketcan"]
        if platform.system() == "Windows":
            iface_items = ["canalystii", "slcan", "socketcan (disabled)"]
        display_iface = self.interface
        if display_iface == "socketcan" and platform.system() == "Windows":
            display_iface = "socketcan (disabled)"
        try:
            current_iface_idx = iface_items.index(display_iface)
        except ValueError:
            current_iface_idx = 0
        changed, current_iface_idx = imgui.combo("##iface", current_iface_idx, iface_items)
        if changed:
            selected_iface = iface_items[current_iface_idx]
            self.interface = "socketcan" if selected_iface == "socketcan (disabled)" else selected_iface
            self.save_config()
        imgui.same_line()
        imgui.text("Channel")
        imgui.same_line()
        if self.interface == "canalystii":
            channel_items = ["0", "1"]
            try:
                channel_idx = channel_items.index(self.channel)
            except ValueError:
                channel_idx = 0
            changed, channel_idx = imgui.combo("##chan", channel_idx, channel_items)
            if changed:
                self.channel = channel_items[channel_idx]
                self.save_config()
        elif self.interface == "slcan":
            ports = self.get_serial_ports()
            if ports:
                try:
                    port_idx = ports.index(self.channel)
                except ValueError:
                    port_idx = 0
                changed, port_idx = imgui.combo("##chan", port_idx, ports)
                if changed:
                    self.channel = ports[port_idx]
                    self.save_config()
            else:
                changed, self.channel = imgui.input_text("##chan", self.channel, 32)
                if changed:
                    self.save_config()
        else:
            if platform.system() == "Windows":
                imgui.text_disabled("socketcan disabled on Windows")
            else:
                vcans = self.get_vcan_interfaces()
                if vcans:
                    try:
                        vcan_idx = vcans.index(self.channel)
                    except ValueError:
                        vcan_idx = 0
                    changed, vcan_idx = imgui.combo("##chan", vcan_idx, vcans)
                    if changed:
                        self.channel = vcans[vcan_idx]
                        self.save_config()
                else:
                    changed, self.channel = imgui.input_text("##chan", self.channel or "vcan0", 32)
                    if changed:
                        self.save_config()
        imgui.same_line()
        if imgui.button("Connect"):
            self.reconnect_can()

        imgui.separator()

        now = time.time()
        if not hasattr(self, "_collapse_all"):
            self._collapse_all = False

        # Device tree/table uses full height
        avail = imgui.get_content_region_avail()
        table_height = max(200.0, avail.y)
        if imgui.begin_table(
            "can_table",
            7,
            imgui.TableFlags_.scroll_y | imgui.TableFlags_.row_bg | imgui.TableFlags_.borders,
            imgui.ImVec2(0, table_height)
        ):
            imgui.table_setup_scroll_freeze(0, 1)
            imgui.table_setup_column("Msg ID", imgui.TableColumnFlags_.width_fixed, 90)
            imgui.table_setup_column("Device", imgui.TableColumnFlags_.width_fixed, 90)
            imgui.table_setup_column("Manuf", imgui.TableColumnFlags_.width_fixed, 90)
            imgui.table_setup_column("Dev#", imgui.TableColumnFlags_.width_fixed, 60)
            imgui.table_setup_column("API", imgui.TableColumnFlags_.width_fixed, 60)
            imgui.table_setup_column("Decode", imgui.TableColumnFlags_.width_fixed, 60)
            imgui.table_setup_column("Data", imgui.TableColumnFlags_.width_stretch)
            imgui.table_headers_row()

            grouped = {}
            for msg_id, entry in self.can_messages.items():
                key = (entry["device_type"], entry["manufacturer"], entry["device_number"])
                grouped.setdefault(key, []).append((msg_id, entry))

            for (device_type, manufacturer, device_number), rows in sorted(grouped.items()):
                imgui.table_next_row()
                imgui.table_set_column_index(0)
                if self._collapse_all:
                    imgui.set_next_item_open(False, imgui.Cond_.always)
                open_node = imgui.tree_node_ex(
                    f"{device_type}_{manufacturer}_{device_number}",
                    imgui.TreeNodeFlags_.span_all_columns,
                    f"Device {device_type}/{manufacturer} #{device_number}"
                )
                if open_node:
                    for msg_id, entry in sorted(rows):
                        imgui.table_next_row()
                        imgui.push_id(msg_id)
                        imgui.table_set_column_index(0)
                        flags = (
                            imgui.TreeNodeFlags_.leaf
                            | imgui.TreeNodeFlags_.no_tree_push_on_open
                            | imgui.TreeNodeFlags_.span_all_columns
                        )
                        imgui.tree_node_ex(f"0x{msg_id:08X}", flags)
                        if imgui.is_item_clicked() and not imgui.is_item_toggled_open():
                            self.selected_msg_id = msg_id
                        imgui.table_set_column_index(1)
                        imgui.text(f"{entry['device_type']}")
                        imgui.table_set_column_index(2)
                        imgui.text(f"{entry['manufacturer']}")
                        imgui.table_set_column_index(3)
                        imgui.text(f"{entry['device_number']}")
                        imgui.table_set_column_index(4)
                        imgui.text(f"0x{entry['api_id']:03X}")
                        imgui.table_set_column_index(5)
                        imgui.text("Yes" if entry.get("decoded_entries") else "No")
                        imgui.table_set_column_index(6)
                        imgui.text(entry["data_hex"] if self.show_hex else entry["data_dec"])
                        imgui.pop_id()
                    imgui.tree_pop()
            if self._collapse_all:
                self._collapse_all = False

            imgui.end_table()

        imgui.end()

        # Decode window
        if getattr(self, "_reset_layout", False):
            imgui.set_next_window_pos(imgui.ImVec2(1350, 40), imgui.Cond_.always)
            imgui.set_next_window_size(imgui.ImVec2(900, 600), imgui.Cond_.always)
            self._reset_layout = False
        else:
            imgui.set_next_window_size(imgui.ImVec2(900, 600), imgui.Cond_.first_use_ever)
        imgui.begin("Decoded Message")
        if self.selected_msg_id in self.can_messages:
            entry = self.can_messages[self.selected_msg_id]
            if imgui.begin_table("meta_table", 2, imgui.TableFlags_.row_bg | imgui.TableFlags_.borders):
                imgui.table_setup_column("Name", imgui.TableColumnFlags_.width_fixed, 140)
                imgui.table_setup_column("Value", imgui.TableColumnFlags_.width_stretch)
                imgui.table_headers_row()
                meta_rows = [
                    ("MSG ID", f"0x{self.selected_msg_id:08X}"),
                    ("DEVICE", str(entry.get("device_type"))),
                    ("MANUF", str(entry.get("manufacturer"))),
                    ("DEV NUM", str(entry.get("device_number"))),
                    ("API ID", f"0x{entry.get('api_id', 0):03X}"),
                    ("FRAME NAME", entry.get("frame_name") or ""),
                    ("FRAME DESC", entry.get("frame_desc") or ""),
                    ("RAW", entry["data_hex"] if self.show_hex else entry["data_dec"])
                ]
                for name, value in meta_rows:
                    imgui.table_next_row()
                    imgui.table_next_column()
                    imgui.text(name)
                    imgui.table_next_column()
                    imgui.text(str(value))
                imgui.end_table()
            imgui.separator()
            if imgui.begin_table("decode_table", 5, imgui.TableFlags_.row_bg | imgui.TableFlags_.borders | imgui.TableFlags_.scroll_y):
                imgui.table_setup_column("Name", imgui.TableColumnFlags_.width_fixed, 180)
                imgui.table_setup_column("Value", imgui.TableColumnFlags_.width_fixed, 160)
                imgui.table_setup_column("Type", imgui.TableColumnFlags_.width_fixed, 90)
                imgui.table_setup_column("Range", imgui.TableColumnFlags_.width_fixed, 160)
                imgui.table_setup_column("Description", imgui.TableColumnFlags_.width_stretch)
                imgui.table_headers_row()
                if entry.get("decoded_entries"):
                    for item in entry["decoded_entries"]:
                        imgui.table_next_row()
                        imgui.table_next_column()
                        imgui.text(str(item.get("name", "")))
                        imgui.table_next_column()
                        imgui.text(str(item.get("value", "")))
                        imgui.table_next_column()
                        imgui.text(str(item.get("type", "")))
                        imgui.table_next_column()
                        imgui.text(str(item.get("range", "")))
                        imgui.table_next_column()
                        imgui.text(str(item.get("description", "")))
                else:
                    imgui.table_next_row()
                    imgui.table_next_column()
                    imgui.text("No decoded data available.")
                    imgui.table_next_column()
                    imgui.text("")
                    imgui.table_next_column()
                    imgui.text("")
                    imgui.table_next_column()
                    imgui.text("")
                    imgui.table_next_column()
                    imgui.text("")
                imgui.end_table()
        else:
            imgui.text("Selected msg id: none")
        imgui.end()

        # Heartbeat window
        imgui.set_next_window_size(imgui.ImVec2(600, 160), imgui.Cond_.first_use_ever)
        imgui.begin("Heartbeat Status")
        if self.heartbeat_last_time == 0:
            imgui.text("No roboRIO heartbeat detected.")
        else:
            time_since = time.time() - self.heartbeat_last_time
            if time_since > 1.0:
                imgui.text_colored(imgui.ImVec4(1.0, 0.6, 0.2, 1.0), "Heartbeat stale")
            if self.decoded_heartbeat:
                d = self.decoded_heartbeat
                imgui.text(f"[{self.format_time(d)}]")
                imgui.text(f"{'RED' if d['red_alliance'] else 'BLUE'} | {'ENABLED' if d['enabled'] else 'DISABLED'} | {'AUTO' if d['autonomous'] else 'TELEOP'}")
                imgui.text(f"Match {d['match_number']} Replay {d['replay_number']} | Time Left: {d['match_time']}s")
            else:
                imgui.text("Heartbeat received but decode failed.")
        imgui.end()


def main():
    app = CanDbgImGui()
    try:
        immapp.run(gui_function=app.gui)
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
