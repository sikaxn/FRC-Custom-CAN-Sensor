import time
import math
import os
import sys
import glob
import tkinter as tk
from tkinter import ttk

HERE = os.path.dirname(os.path.abspath(__file__))
found = False

env_dir = os.environ.get("LEDSIM_PYD_DIR")
if env_dir and os.path.isdir(env_dir):
    sys.path.insert(0, env_dir)
    found = True

if not found:
    for base in ("build-py312", "build"):
        for sub in ("Release", "Debug"):
            candidate = os.path.join(HERE, base, sub)
            if os.path.isdir(candidate):
                if glob.glob(os.path.join(candidate, "ledsim*.pyd")):
                    sys.path.insert(0, candidate)
                    found = True
                    break
        if found:
            break

try:
    import ledsim
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "Could not find ledsim module. Build it first:\n"
        "  LEDsimulator\\buildbinding.bat\n"
        "Then run with Python 3.12:\n"
        "  py -3.12 LEDsimulator\\led_sim.py\n"
        "Or set LEDSIM_PYD_DIR to the folder containing ledsim*.pyd."
    ) from exc

WINDOW_W = 1200
WINDOW_H = 900


def clamp_u8(v):
    return max(0, min(255, int(v)))


def init_leds(count):
    ledsim.set_num_leds(count)
    ledsim.sim_reset_time()


class LedSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LED Simulator")
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")

        self.led_count = tk.IntVar(value=60)
        self.loop_us = tk.IntVar(value=10000)
        self.realtime = tk.BooleanVar(value=True)

        self.mode = tk.IntVar(value=1)
        self.r = tk.IntVar(value=255)
        self.g = tk.IntVar(value=0)
        self.b = tk.IntVar(value=0)
        self.brig = tk.IntVar(value=128)
        self.onoff = tk.BooleanVar(value=True)
        self.param0 = tk.IntVar(value=10)
        self.param1 = tk.IntVar(value=10)

        self.r2 = tk.IntVar(value=0)
        self.g2 = tk.IntVar(value=0)
        self.b2 = tk.IntVar(value=0)
        self.brig2 = tk.IntVar(value=0)
        self.onoff2 = tk.BooleanVar(value=False)

        self.last_time = time.perf_counter()
        self.sim_accum_ms = 0.0

        self._build_ui()

        init_leds(self.led_count.get())
        ledsim.sim_set_loop_us(self.loop_us.get())
        self._sync_can_all()

        self.after(0, self._tick)

    def _build_ui(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        controls = ttk.Frame(self, padding=8)
        controls.grid(row=0, column=0, sticky="ns")

        canvas_frame = ttk.Frame(self, padding=8)
        canvas_frame.grid(row=0, column=1, sticky="nsew")
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, bg="#111111")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        def slider(label, var, frm, to_, cmd):
            ttk.Label(frm, text=label).pack(anchor="w")
            scale = tk.Scale(
                frm,
                from_=0,
                to=to_,
                orient="horizontal",
                showvalue=True,
                variable=var,
                command=cmd,
            )
            scale.pack(fill="x", pady=(0, 6))
            return scale

        def intbox(label, var, frm, from_, to_, cmd):
            row = ttk.Frame(frm)
            row.pack(fill="x", pady=(0, 6))
            ttk.Label(row, text=label).pack(side="left")
            spin = ttk.Spinbox(row, from_=from_, to=to_, textvariable=var, width=6, command=cmd)
            spin.pack(side="right")
            return spin

        intbox("LED Count", self.led_count, controls, 1, 600, self._on_led_count)
        intbox("Loop us", self.loop_us, controls, 100, 20000, self._on_loop_us)
        ttk.Checkbutton(controls, text="Real-time sync", variable=self.realtime).pack(anchor="w", pady=(0, 8))

        ttk.Separator(controls).pack(fill="x", pady=6)

        intbox("Mode", self.mode, controls, 0, 255, self._on_mode)
        ttk.Checkbutton(controls, text="On/Off", variable=self.onoff, command=self._on_onoff).pack(anchor="w", pady=(0, 6))

        slider("R", self.r, controls, 255, self._on_rgb)
        slider("G", self.g, controls, 255, self._on_rgb)
        slider("B", self.b, controls, 255, self._on_rgb)

        slider("Brightness", self.brig, controls, 255, self._on_brig)
        intbox("Param0", self.param0, controls, 0, 255, self._on_param0)
        intbox("Param1", self.param1, controls, 0, 255, self._on_param1)

        ttk.Separator(controls).pack(fill="x", pady=6)

        ttk.Checkbutton(controls, text="Color2 On/Off", variable=self.onoff2, command=self._on_onoff2).pack(anchor="w", pady=(0, 6))
        slider("R2", self.r2, controls, 255, self._on_rgb2)
        slider("G2", self.g2, controls, 255, self._on_rgb2)
        slider("B2", self.b2, controls, 255, self._on_rgb2)
        slider("Brightness2", self.brig2, controls, 255, self._on_brig2)

        ttk.Button(controls, text="Mode Refresh", command=self._on_mode_refresh).pack(fill="x", pady=(6, 0))

    def _on_led_count(self):
        init_leds(self.led_count.get())

    def _on_loop_us(self):
        ledsim.sim_set_loop_us(self.loop_us.get())

    def _on_mode(self):
        ledsim.set_can_mode(self.mode.get())
        ledsim.set_mode_refresh(True)

    def _on_onoff(self):
        ledsim.set_can_onoff(self.onoff.get())

    def _on_rgb(self, *_):
        ledsim.set_can_rgb(clamp_u8(self.r.get()), clamp_u8(self.g.get()), clamp_u8(self.b.get()))

    def _on_brig(self, *_):
        ledsim.set_can_brightness(clamp_u8(self.brig.get()))

    def _on_param0(self):
        ledsim.set_can_param0(clamp_u8(self.param0.get()))

    def _on_param1(self):
        ledsim.set_can_param1(clamp_u8(self.param1.get()))

    def _on_onoff2(self):
        ledsim.set_can2_onoff(1 if self.onoff2.get() else 0)

    def _on_rgb2(self, *_):
        ledsim.set_can2_rgb(clamp_u8(self.r2.get()), clamp_u8(self.g2.get()), clamp_u8(self.b2.get()))

    def _on_brig2(self, *_):
        ledsim.set_can2_brightness(clamp_u8(self.brig2.get()))

    def _on_mode_refresh(self):
        ledsim.set_mode_refresh(True)

    def _sync_can_all(self):
        self._on_mode()
        self._on_onoff()
        self._on_rgb()
        self._on_brig()
        self._on_param0()
        self._on_param1()
        self._on_onoff2()
        self._on_rgb2()
        self._on_brig2()

    def _draw_leds(self):
        self.canvas.delete("all")
        leds = ledsim.get_leds()
        if not leds:
            return

        w = max(1, self.canvas.winfo_width())
        h = max(1, self.canvas.winfo_height())
        n = len(leds)

        aspect = w / h if h > 0 else 1.0
        cols = max(1, int(math.ceil(math.sqrt(n * aspect))))
        rows = max(1, int(math.ceil(n / cols)))

        cell_w = w / cols
        cell_h = h / rows
        size = int(max(2, min(cell_w, cell_h) * 0.85))
        spacing = max(1, int(size * 0.15))
        grid_w = cols * size + (cols - 1) * spacing
        grid_h = rows * size + (rows - 1) * spacing
        pad_x = max(0, int((w - grid_w) / 2))
        pad_y = max(0, int((h - grid_h) / 2))

        x = pad_x
        y = pad_y
        for i, c in enumerate(leds):
            color = f"#{c.r:02x}{c.g:02x}{c.b:02x}"
            self.canvas.create_rectangle(x, y, x + size, y + size, fill=color, outline="")
            x += size + spacing
            if (i + 1) % cols == 0:
                x = pad_x
                y += size + spacing

    def _tick(self):
        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now

        if self.realtime.get():
            self.sim_accum_ms += dt * 1000.0
            loop_ms = max(0.001, ledsim.sim_loop_us() / 1000.0)
            steps = int(self.sim_accum_ms / loop_ms)
            if steps > 0:
                self.sim_accum_ms -= steps * loop_ms
                for _ in range(steps):
                    t0 = ledsim.sim_time_ms()
                    ledsim.run_current_mode()
                    t1 = ledsim.sim_time_ms()
                    delay_ms = t1 - t0
                    if delay_ms > 0:
                        # delay() already advanced simulated time; consume it from the accumulator
                        self.sim_accum_ms = max(0.0, self.sim_accum_ms - delay_ms)
                    else:
                        ledsim.sim_step_loop()
        else:
            t0 = ledsim.sim_time_ms()
            ledsim.run_current_mode()
            t1 = ledsim.sim_time_ms()
            if t1 == t0:
                ledsim.sim_step_loop()

        self._draw_leds()
        self.after(16, self._tick)


if __name__ == "__main__":
    app = LedSimApp()
    app.mainloop()
