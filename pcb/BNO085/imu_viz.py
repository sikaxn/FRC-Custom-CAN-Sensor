import sys
import threading
import time
from collections import deque

import pygame
import serial
import serial.tools.list_ports

# ---------------------------
# Config
# ---------------------------
WIN_W, WIN_H = 1280, 720
FPS = 60
PLOT_LEN = 1200               # number of samples kept for scroll plot
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1          # seconds
GYRO_FULL_SCALE = 500.0       # dps full-scale for y-axis +/- (auto-scaled if needed)
BG_COLOR = (18, 18, 20)
GRID_COLOR = (40, 40, 45)
TEXT_COLOR = (230, 230, 235)
ACCEL_COLOR = (140, 200, 255)
GYRO_COLORS = [(240, 120, 120), (120, 240, 120), (120, 140, 240)]  # gx, gy, gz
MAG_COLOR = (200, 200, 120)
TEMP_COLOR = (200, 160, 220)

# Layout rects
LEFT_PAD = 16
TOP_PAD = 16

# Put UI bar at the very top
UI_RECT = pygame.Rect(LEFT_PAD, TOP_PAD, WIN_W - 2 * LEFT_PAD, 64)

# Move plot and side panel below it
PLOT_RECT = pygame.Rect(LEFT_PAD, UI_RECT.bottom + 16, 900, WIN_H - UI_RECT.bottom - 32)
SIDE_RECT = pygame.Rect(PLOT_RECT.right + 12, PLOT_RECT.y, WIN_W - PLOT_RECT.right - 28, PLOT_RECT.height)

# ---------------------------
# Serial reader thread
# ---------------------------
class SerialReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.ser = None
        self.running = False
        self.lock = threading.Lock()
        self.columns = []  # header columns
        self.latest = {}   # latest parsed values
        self.buffers = {}  # history per column -> deque
        self.connected_port = None

    def connect(self, port):
        self.disconnect()
        try:
            self.ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            self.running = True
            self.connected_port = port
            return True, ""
        except Exception as e:
            self.ser = None
            self.running = False
            self.connected_port = None
            return False, str(e)

    def disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected_port = None

    def clear_buffers(self):
        with self.lock:
            for k in self.buffers:
                self.buffers[k].clear()

    def ensure_buffer(self, key):
        if key not in self.buffers:
            self.buffers[key] = deque(maxlen=PLOT_LEN)

    def run(self):
        buf = b""
        while True:
            if not self.running or self.ser is None:
                time.sleep(0.05)
                continue
            try:
                chunk = self.ser.read(1024)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip().decode(errors="ignore")
                    if not line:
                        continue
                    self._process_line(line)
            except Exception:
                # keep thread alive, but drop connection on hard errors
                self.disconnect()
                time.sleep(0.25)

    def _process_line(self, line):
        # Accept either header or data lines (tab or space separated)
        parts = [p for p in line.replace(",", " ").replace("\t", " ").split(" ") if p != ""]
        if not parts:
            return
        # Header detection: any token contains non-numeric chars
        is_header = any(not self._is_float(tok) for tok in parts)
        with self.lock:
            if is_header:
                # New header → reset column map if it looks like names
                self.columns = parts
                # Initialize buffers for known keys
                for name in self.columns:
                    self.ensure_buffer(name)
            else:
                # Data line: map by index if we have a header; else fallback to fixed order
                if self.columns and len(parts) == len(self.columns):
                    for name, val in zip(self.columns, parts):
                        if self._is_float(val):
                            f = float(val)
                            self.latest[name] = f
                            self.ensure_buffer(name)
                            self.buffers[name].append(f)
                else:
                    # Fallback assumed order: ax ay az gx gy gz temp
                    keys = ["ax", "ay", "az", "gx", "gy", "gz", "temp"]
                    for i, val in enumerate(parts[:len(keys)]):
                        if self._is_float(val):
                            f = float(val)
                            self.latest[keys[i]] = f
                            self.ensure_buffer(keys[i])
                            self.buffers[keys[i]].append(f)

    @staticmethod
    def _is_float(s):
        try:
            float(s)
            return True
        except Exception:
            return False

# ---------------------------
# Simple UI widgets
# ---------------------------
class Button:
    def __init__(self, rect, label):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.hover = False

    def draw(self, surf, font):
        base = (60, 62, 70)
        hover = (80, 82, 90)
        border = (110, 110, 120)
        pygame.draw.rect(surf, hover if self.hover else base, self.rect, border_radius=8)
        pygame.draw.rect(surf, border, self.rect, width=1, border_radius=8)
        txt = font.render(self.label, True, TEXT_COLOR)
        surf.blit(txt, txt.get_rect(center=self.rect.center))

    def handle(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hover = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos):
            return True
        return False

class Dropdown:
    def __init__(self, rect, items=None, placeholder="Select COM"):
        self.rect = pygame.Rect(rect)
        self.items = items or []
        self.placeholder = placeholder
        self.open = False
        self.selected_index = None
        self.hover = False
        self.item_height = self.rect.height

    def set_items(self, items):
        self.items = items
        if self.selected_index is not None and self.selected_index >= len(items):
            self.selected_index = None

    def selected(self):
        if self.selected_index is None: return None
        return self.items[self.selected_index]

    def draw(self, surf, font):
        base = (60, 62, 70)
        hover = (80, 82, 90)
        border = (110, 110, 120)
        pygame.draw.rect(surf, hover if self.hover else base, self.rect, border_radius=8)
        pygame.draw.rect(surf, border, self.rect, width=1, border_radius=8)
        label = self.selected() or self.placeholder
        txt = font.render(label, True, TEXT_COLOR)
        surf.blit(txt, (self.rect.x + 10, self.rect.y + (self.rect.height - txt.get_height()) // 2))
        # caret
        pygame.draw.polygon(surf, TEXT_COLOR, [
            (self.rect.right - 18, self.rect.y + self.rect.height // 2 - 3),
            (self.rect.right - 8,  self.rect.y + self.rect.height // 2 - 3),
            (self.rect.right - 13, self.rect.y + self.rect.height // 2 + 5),
        ])
        # dropdown list
        if self.open:
            list_rect = pygame.Rect(self.rect.x, self.rect.bottom + 2, self.rect.width, self.item_height * min(8, len(self.items)))
            pygame.draw.rect(surf, base, list_rect, border_radius=6)
            pygame.draw.rect(surf, border, list_rect, width=1, border_radius=6)
            for i, item in enumerate(self.items[:8]):
                r = pygame.Rect(list_rect.x, list_rect.y + i * self.item_height, list_rect.width, self.item_height)
                pygame.draw.rect(surf, hover if r.collidepoint(pygame.mouse.get_pos()) else base, r)
                t = font.render(item, True, TEXT_COLOR)
                surf.blit(t, (r.x + 8, r.y + (r.height - t.get_height()) // 2))

    def handle(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hover = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.open = not self.open
                return ("toggle", None)
            if self.open:
                # clicked list?
                y = event.pos[1] - (self.rect.bottom + 2)
                if 0 <= y < self.item_height * min(8, len(self.items)):
                    idx = y // self.item_height
                    if idx < len(self.items[:8]):
                        self.selected_index = int(idx)
                        self.open = False
                        return ("select", self.selected())
                self.open = False
        return (None, None)

# ---------------------------
# Helpers
# ---------------------------
def list_com_ports():
    ports = []
    for p in serial.tools.list_ports.comports():
        # Format: "COM3 - USB-SERIAL (CH340)"
        label = f"{p.device}"
        if p.description:
            label += f" - {p.description}"
        ports.append(label)
    # Return device token at start before ' - '
    return [label.split(' - ')[0] for label in ports] or ["COM1", "COM2", "COM3"]

def draw_grid(surf, rect, x_div=10, y_div=8):
    pygame.draw.rect(surf, GRID_COLOR, rect, 1)
    for i in range(1, x_div):
        x = rect.x + int(rect.w * i / x_div)
        pygame.draw.line(surf, GRID_COLOR, (x, rect.y), (x, rect.bottom))
    for j in range(1, y_div):
        y = rect.y + int(rect.h * j / y_div)
        pygame.draw.line(surf, GRID_COLOR, (rect.x, y), (rect.right, y))

def plot_series(surf, rect, data, color, vmin, vmax):
    # data: deque of floats
    if not data or len(data) < 2:
        return
    n = len(data)
    # x step fits to rect width (scrolling)
    step = max(1, rect.w / max(1, PLOT_LEN - 1))
    points = []
    for i, v in enumerate(list(data)[-PLOT_LEN:]):
        x = rect.x + i * step
        # clamp
        vv = max(vmin, min(v, vmax))
        # map value to y
        t = (vv - vmin) / (vmax - vmin) if vmax > vmin else 0.5
        y = rect.bottom - t * rect.h
        points.append((x, y))
    if len(points) >= 2:
        pygame.draw.lines(surf, color, False, points, 2)

def text_line(surf, font, x, y, label, value, color=TEXT_COLOR):
    txt = font.render(f"{label}: {value}", True, color)
    surf.blit(txt, (x, y))
    return y + txt.get_height() + 6

# ---------------------------
# Main
# ---------------------------
def main():
    pygame.init()
    pygame.display.set_caption("IMU Gyro Visualizer (ESP32/Arduino)")
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)
    big  = pygame.font.SysFont("consolas", 22, bold=True)

    reader = SerialReader()
    reader.start()

    # UI
    ports = list_com_ports()
    dd_ports = Dropdown((UI_RECT.x, UI_RECT.y, 280, 40), ports, "Select COM")
    btn_refresh = Button((dd_ports.rect.right + 10, UI_RECT.y, 120, 40), "Refresh")
    btn_connect = Button((btn_refresh.rect.right + 10, UI_RECT.y, 140, 40), "Connect")
    btn_disconnect = Button((btn_connect.rect.right + 10, UI_RECT.y, 140, 40), "Disconnect")
    btn_clear = Button((btn_disconnect.rect.right + 10, UI_RECT.y, 120, 40), "Clear")

    status_msg = "Idle."
    status_color = (180, 180, 180)
    auto_scale = True
    dyn_min, dyn_max = -GYRO_FULL_SCALE, GYRO_FULL_SCALE

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if btn_refresh.handle(event):
                dd_ports.set_items(list_com_ports())
            if btn_connect.handle(event):
                port = dd_ports.selected()
                if not port:
                    status_msg = "Select a COM port first."
                    status_color = (255, 180, 120)
                else:
                    ok, err = reader.connect(port)
                    if ok:
                        status_msg = f"Connected to {port}"
                        status_color = (150, 240, 150)
                    else:
                        status_msg = f"Connect failed: {err}"
                        status_color = (255, 120, 120)
            if btn_disconnect.handle(event):
                reader.disconnect()
                status_msg = "Disconnected."
                status_color = (200, 200, 200)
            if btn_clear.handle(event):
                reader.clear_buffers()
                status_msg = "Cleared buffers."
                status_color = (180, 200, 255)

            _act, _val = dd_ports.handle(event)

        screen.fill(BG_COLOR)

        # Draw plot area & grid
        draw_grid(screen, PLOT_RECT)

        # Pull latest safely
        with reader.lock:
            # Gyro series names (prefer header names if present)
            gx_name = "gx" if "gx" in reader.buffers else next((c for c in reader.columns if c.lower() in ("gx","gyrox","gyro_x")), "gx")
            gy_name = "gy" if "gy" in reader.buffers else next((c for c in reader.columns if c.lower() in ("gy","gyroy","gyro_y")), "gy")
            gz_name = "gz" if "gz" in reader.buffers else next((c for c in reader.columns if c.lower() in ("gz","gyroz","gyro_z")), "gz")

            gx = reader.buffers.get(gx_name, deque(maxlen=PLOT_LEN))
            gy = reader.buffers.get(gy_name, deque(maxlen=PLOT_LEN))
            gz = reader.buffers.get(gz_name, deque(maxlen=PLOT_LEN))

            # Optional auto-scaling based on last 5 sec of data
            if auto_scale:
                vals = []
                for dq in (gx, gy, gz):
                    vals.extend(list(dq)[-min(len(dq), 600):])
                if vals:
                    vmin = min(vals)
                    vmax = max(vals)
                    span = max(50.0, (vmax - vmin))
                    mid = 0.5 * (vmax + vmin)
                    dyn_min = mid - span * 0.6
                    dyn_max = mid + span * 0.6

            # Plot gyro traces
            plot_series(screen, PLOT_RECT, gx, GYRO_COLORS[0], dyn_min, dyn_max)
            plot_series(screen, PLOT_RECT, gy, GYRO_COLORS[1], dyn_min, dyn_max)
            plot_series(screen, PLOT_RECT, gz, GYRO_COLORS[2], dyn_min, dyn_max)

            latest = dict(reader.latest)

        # Axis labels
        title = big.render("Gyro (deg/s) — gx/gy/gz", True, TEXT_COLOR)
        screen.blit(title, (PLOT_RECT.x, PLOT_RECT.y - 28))
        y_max_txt = font.render(f"{dyn_max:+.0f} dps", True, TEXT_COLOR)
        y_min_txt = font.render(f"{dyn_min:+.0f} dps", True, TEXT_COLOR)
        screen.blit(y_max_txt, (PLOT_RECT.right - y_max_txt.get_width(), PLOT_RECT.y - 20))
        screen.blit(y_min_txt, (PLOT_RECT.right - y_min_txt.get_width(), PLOT_RECT.bottom + 4))

        # Legend
        legend_y = PLOT_RECT.y + 8
        for name, col in [("gx", GYRO_COLORS[0]), ("gy", GYRO_COLORS[1]), ("gz", GYRO_COLORS[2])]:
            pygame.draw.rect(screen, col, (PLOT_RECT.right - 220, legend_y + 3, 16, 8))
            txt = font.render(name, True, TEXT_COLOR)
            screen.blit(txt, (PLOT_RECT.right - 200, legend_y))
            legend_y += 22

        # Side panel: latest values
        pygame.draw.rect(screen, GRID_COLOR, SIDE_RECT, 1)
        sx, sy = SIDE_RECT.x + 12, SIDE_RECT.y + 12
        sy = text_line(screen, big, sx, sy, "Latest values", "", TEXT_COLOR)
        def g(k, fmt="{:.2f}"):
            return fmt.format(latest[k]) if k in latest else "—"

        sy += 6
        sy = text_line(screen, font, sx, sy, "ax (g)", g("ax"))
        sy = text_line(screen, font, sx, sy, "ay (g)", g("ay"))
        sy = text_line(screen, font, sx, sy, "az (g)", g("az"))
        sy += 10
        sy = text_line(screen, font, sx, sy, "gx (dps)", g("gx"))
        sy = text_line(screen, font, sx, sy, "gy (dps)", g("gy"))
        sy = text_line(screen, font, sx, sy, "gz (dps)", g("gz"))
        sy += 10
        sy = text_line(screen, font, sx, sy, "temp (C)", g("temp", "{:.1f}"))

        # UI row
        pygame.draw.rect(screen, (30, 30, 36), UI_RECT, border_radius=10)
        dd_ports.draw(screen, font)
        btn_refresh.draw(screen, font)
        btn_connect.draw(screen, font)
        btn_disconnect.draw(screen, font)
        btn_clear.draw(screen, font)

        status = font.render(status_msg, True, status_color)
        screen.blit(status, (UI_RECT.x, UI_RECT.bottom + 6))

        pygame.display.flip()
        clock.tick(FPS)

    reader.disconnect()
    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
