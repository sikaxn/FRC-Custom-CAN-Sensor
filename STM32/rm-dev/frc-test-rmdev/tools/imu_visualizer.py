#!/usr/bin/env python3
"""Live RoboMaster Type C BMI088 visualizer over the STM32 USB CDC port.

Firmware protocol (ASCII, comma separated, CRLF terminated):

    HELLO,1,RMDEV-C,BMI088,<telemetry_hz>,<accel_range_g>,<gyro_range_dps>
    IMU,1,<seq>,<time_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp_centi_c>
    STAT,1,<up_ms>,<can1_rx>,<can2_rx>,<rx_drop>,<tx>,<tx_retry>,
           <tx_qdrop>,<imu_ok>,<imu_err>
    PWR,1,<time_ms>,<input_mv>,<battery_adc_raw>,<vref_adc_raw>

The IMU values are signed raw BMI088 counts. This client converts them using
the ranges announced by HELLO (and defaults to +/-3 g and +/-2000 deg/s).

Install:
    python -m pip install pygame pyserial

Run:
    python tools/imu_visualizer.py
    python tools/imu_visualizer.py --port COM31

Controls:
    R       Re-center the displayed attitude
    C       Clear the graphs
    Escape  Quit
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Union

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

try:
    import pygame
except ImportError as exc:  # pragma: no cover - exercised only on missing dependency
    raise SystemExit("pygame is required: python -m pip install pygame pyserial") from exc

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - exercised only on missing dependency
    raise SystemExit("pyserial is required: python -m pip install pygame pyserial") from exc


STM32_VID = 0x0483
STM32_CDC_PID = 0x5740
PROTOCOL_VERSION = 1
DEFAULT_BAUD = 115200  # Nominal for CDC ACM; USB itself is not baud limited.
MAX_LINE_BUFFER = 8192
POWER_STALE_SECONDS = 2.5
INPUT_VALID_MIN_V = 8.0
INPUT_VALID_MAX_V = 28.0
INPUT_GAUGE_MAX_V = 32.0

BACKGROUND = (12, 17, 27)
PANEL = (21, 29, 43)
PANEL_ALT = (25, 35, 51)
GRID = (45, 58, 77)
TEXT = (226, 234, 245)
MUTED = (135, 151, 173)
ACCENT = (58, 190, 255)
GREEN = (75, 222, 151)
AMBER = (255, 190, 82)
RED = (255, 93, 108)
AXIS_COLORS = ((255, 99, 121), (91, 225, 153), (74, 171, 255))


@dataclass(frozen=True)
class HelloFrame:
    board: str
    sensor: str
    telemetry_hz: int
    accel_range_g: float
    gyro_range_dps: float


@dataclass(frozen=True)
class ImuFrame:
    sequence: int
    timestamp_ms: int
    accel_raw: tuple[int, int, int]
    gyro_raw: tuple[int, int, int]
    temperature_c: float
    received_at: float


@dataclass(frozen=True)
class StatFrame:
    uptime_ms: int
    can1_rx: int
    can2_rx: int
    rx_drop: int
    can_tx: int
    can_tx_retry: int
    can_tx_queue_drop: int
    imu_ok: int
    imu_err: int


@dataclass(frozen=True)
class PowerFrame:
    timestamp_ms: int
    input_mv: int
    battery_adc_raw: int
    vref_adc_raw: int
    received_at: float

    @property
    def input_v(self) -> float:
        return self.input_mv / 1000.0


@dataclass(frozen=True)
class ConnectionEvent:
    state: str
    port: Optional[str]
    detail: str = ""


ProtocolFrame = Union[HelloFrame, ImuFrame, StatFrame, PowerFrame]


def _bounded_int(text: str, low: int, high: int) -> int:
    value = int(text, 10)
    if not low <= value <= high:
        raise ValueError(f"{value} outside [{low}, {high}]")
    return value


def parse_protocol_line(line: str, received_at: Optional[float] = None) -> Optional[ProtocolFrame]:
    """Parse one complete protocol line; return None for unknown or malformed data."""
    fields = [field.strip() for field in line.strip().split(",")]
    if len(fields) < 2:
        return None

    try:
        version = _bounded_int(fields[1], 1, 255)
        if version != PROTOCOL_VERSION:
            return None

        if fields[0] == "HELLO" and len(fields) == 7:
            return HelloFrame(
                board=fields[2],
                sensor=fields[3],
                telemetry_hz=_bounded_int(fields[4], 1, 1000),
                accel_range_g=float(_bounded_int(fields[5], 1, 100)),
                gyro_range_dps=float(_bounded_int(fields[6], 1, 10000)),
            )

        if fields[0] == "IMU" and len(fields) == 11:
            accel = tuple(_bounded_int(value, -32768, 32767) for value in fields[4:7])
            gyro = tuple(_bounded_int(value, -32768, 32767) for value in fields[7:10])
            temperature_centi = _bounded_int(fields[10], -10000, 20000)
            return ImuFrame(
                sequence=_bounded_int(fields[2], 0, 0xFFFFFFFF),
                timestamp_ms=_bounded_int(fields[3], 0, 0xFFFFFFFF),
                accel_raw=(accel[0], accel[1], accel[2]),
                gyro_raw=(gyro[0], gyro[1], gyro[2]),
                temperature_c=temperature_centi / 100.0,
                received_at=time.monotonic() if received_at is None else received_at,
            )

        if fields[0] == "STAT" and len(fields) == 11:
            values = [_bounded_int(value, 0, 0xFFFFFFFF) for value in fields[2:]]
            return StatFrame(
                uptime_ms=values[0],
                can1_rx=values[1],
                can2_rx=values[2],
                rx_drop=values[3],
                can_tx=values[4],
                can_tx_retry=values[5],
                can_tx_queue_drop=values[6],
                imu_ok=values[7],
                imu_err=values[8],
            )

        if fields[0] == "PWR" and len(fields) == 6:
            return PowerFrame(
                timestamp_ms=_bounded_int(fields[2], 0, 0xFFFFFFFF),
                input_mv=_bounded_int(fields[3], 0, 60000),
                battery_adc_raw=_bounded_int(fields[4], 0, 4095),
                vref_adc_raw=_bounded_int(fields[5], 0, 4095),
                received_at=time.monotonic() if received_at is None else received_at,
            )
    except (ValueError, TypeError, OverflowError):
        return None

    return None


def extract_complete_lines(buffer: bytearray, chunk: bytes) -> list[str]:
    """Append a serial chunk and return only complete newline-terminated lines."""
    buffer.extend(chunk)
    if len(buffer) > MAX_LINE_BUFFER and b"\n" not in buffer:
        buffer.clear()
        return []

    lines: list[str] = []
    while True:
        newline = buffer.find(b"\n")
        if newline < 0:
            break
        raw = bytes(buffer[:newline])
        del buffer[: newline + 1]
        lines.append(raw.rstrip(b"\r").decode("ascii", errors="replace"))
    return lines


def stm32_cdc_ports() -> list[object]:
    return sorted(
        (
            port
            for port in list_ports.comports()
            if port.vid == STM32_VID and port.pid == STM32_CDC_PID
        ),
        key=lambda port: port.device,
    )


class FrameEmitter:
    def __init__(self, output: "queue.Queue[object]") -> None:
        self.output = output
        self.dropped_events = 0

    def emit(self, event: object) -> None:
        try:
            self.output.put_nowait(event)
        except queue.Full:
            try:
                self.output.get_nowait()
            except queue.Empty:
                pass
            try:
                self.output.put_nowait(event)
            except queue.Full:
                self.dropped_events += 1


class SerialReader(threading.Thread, FrameEmitter):
    def __init__(
        self,
        output: "queue.Queue[object]",
        stop_event: threading.Event,
        requested_port: Optional[str],
        baud: int,
    ) -> None:
        threading.Thread.__init__(self, name="stm32-cdc-reader", daemon=True)
        FrameEmitter.__init__(self, output)
        self.stop_event = stop_event
        self.requested_port = requested_port
        self.baud = baud
        self.parse_errors = 0
        self._last_state: Optional[tuple[str, Optional[str], str]] = None

    def state(self, name: str, port: Optional[str], detail: str = "") -> None:
        signature = (name, port, detail)
        if signature != self._last_state:
            self._last_state = signature
            self.emit(ConnectionEvent(name, port, detail))

    def resolve_port(self) -> Optional[str]:
        if self.requested_port:
            return self.requested_port
        matches = stm32_cdc_ports()
        if not matches:
            return None
        detail = ""
        if len(matches) > 1:
            detail = f"{len(matches)} matching boards; selected {matches[0].device}"
        self.state("connecting", matches[0].device, detail)
        return matches[0].device

    def run(self) -> None:
        while not self.stop_event.is_set():
            device = self.resolve_port()
            if device is None:
                self.state("waiting", None, "Waiting for USB VID 0483 / PID 5740")
                self.stop_event.wait(0.75)
                continue

            try:
                with serial.Serial(
                    device,
                    self.baud,
                    timeout=0.10,
                    write_timeout=0.10,
                ) as port:
                    try:
                        port.dtr = True
                    except (OSError, serial.SerialException):
                        pass
                    self.state("connected", device, "USB CDC connected")
                    self._read_port(port, device)
            except (OSError, serial.SerialException) as exc:
                self.state("disconnected", device, str(exc))
                self.stop_event.wait(0.75)

    def _read_port(self, port: serial.Serial, device: str) -> None:
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                waiting = min(max(port.in_waiting, 1), 4096)
                chunk = port.read(waiting)
            except (OSError, serial.SerialException):
                raise

            if not chunk:
                continue

            for line in extract_complete_lines(buffer, chunk):
                frame = parse_protocol_line(line)
                if frame is None:
                    self.parse_errors += 1
                else:
                    self.emit(frame)


class DemoReader(threading.Thread, FrameEmitter):
    """Synthetic source for UI testing without hardware."""

    def __init__(self, output: "queue.Queue[object]", stop_event: threading.Event) -> None:
        threading.Thread.__init__(self, name="imu-demo-reader", daemon=True)
        FrameEmitter.__init__(self, output)
        self.stop_event = stop_event
        self.parse_errors = 0

    def run(self) -> None:
        start = time.monotonic()
        sequence = 0
        last_stat = start
        self.emit(
            ConnectionEvent(
                "connected", "DEMO", "Synthetic 100 Hz BMI088 + 1 Hz power"
            )
        )
        self.emit(HelloFrame("RMDEV-C", "BMI088", 100, 3.0, 2000.0))

        while not self.stop_event.is_set():
            now = time.monotonic()
            t = now - start
            roll = math.radians(24.0 * math.sin(t * 0.75))
            pitch = math.radians(17.0 * math.sin(t * 0.47))
            roll_rate = math.radians(18.0 * math.cos(t * 0.75))
            pitch_rate = math.radians(8.0 * math.cos(t * 0.47))
            yaw_rate = math.radians(12.0)

            accel_g = (
                -math.sin(pitch),
                math.sin(roll) * math.cos(pitch),
                math.cos(roll) * math.cos(pitch),
            )
            gyro_dps = tuple(
                math.degrees(value) for value in (roll_rate, pitch_rate, yaw_rate)
            )
            sequence += 5
            self.emit(
                ImuFrame(
                    sequence=sequence,
                    timestamp_ms=int(t * 1000.0),
                    accel_raw=tuple(
                        int(max(-32768, min(32767, value * 32768.0 / 3.0)))
                        for value in accel_g
                    ),
                    gyro_raw=tuple(
                        int(max(-32768, min(32767, value * 32768.0 / 2000.0)))
                        for value in gyro_dps
                    ),
                    temperature_c=38.0 + 3.0 * (1.0 - math.exp(-t / 20.0)),
                    received_at=now,
                )
            )

            if now - last_stat >= 1.0:
                input_v = 14.95 + 0.08 * math.sin(t * 0.31)
                self.emit(
                    StatFrame(
                        uptime_ms=int(t * 1000.0),
                        can1_rx=int(t * 2),
                        can2_rx=int(t),
                        rx_drop=0,
                        can_tx=0,
                        can_tx_retry=0,
                        can_tx_queue_drop=0,
                        imu_ok=sequence,
                        imu_err=0,
                    )
                )
                self.emit(
                    PowerFrame(
                        timestamp_ms=int(t * 1000.0),
                        input_mv=int(round(input_v * 1000.0)),
                        battery_adc_raw=int(round(input_v * 4095.0 / (3.30 * 11.0))),
                        vref_adc_raw=1502,
                        received_at=now,
                    )
                )
                last_stat = now
            self.stop_event.wait(0.01)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class AttitudeFilter:
    """Gyro integration with accelerometer correction for roll and pitch."""

    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.zero = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.last_timestamp_ms: Optional[int] = None
        self.initialized = False

    def reset(self) -> None:
        self.__init__()

    def recenter(self) -> None:
        self.zero = [self.roll, self.pitch, self.yaw]

    def update(
        self,
        frame: ImuFrame,
        accel_g: tuple[float, float, float],
        gyro_dps: tuple[float, float, float],
    ) -> None:
        ax, ay, az = accel_g
        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        if magnitude < 1.0e-6:
            return

        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if not self.initialized:
            self.roll = acc_roll
            self.pitch = acc_pitch
            self.yaw = 0.0
            self.last_timestamp_ms = frame.timestamp_ms
            self.initialized = True
            return

        assert self.last_timestamp_ms is not None
        delta_ms = (frame.timestamp_ms - self.last_timestamp_ms) & 0xFFFFFFFF
        self.last_timestamp_ms = frame.timestamp_ms
        dt = delta_ms / 1000.0
        if dt <= 0.0 or dt > 0.25:
            return

        corrected = [
            gyro_dps[index] - self.gyro_bias[index] for index in range(3)
        ]
        gyro_magnitude = math.sqrt(sum(value * value for value in corrected))
        if abs(magnitude - 1.0) < 0.08 and gyro_magnitude < 3.0:
            for index in range(3):
                self.gyro_bias[index] = (
                    0.997 * self.gyro_bias[index] + 0.003 * gyro_dps[index]
                )
                corrected[index] = gyro_dps[index] - self.gyro_bias[index]

        gx, gy, gz = (math.radians(value) for value in corrected)
        time_constant = 0.55
        alpha = time_constant / (time_constant + dt)
        self.roll = wrap_angle(alpha * (self.roll + gx * dt) + (1.0 - alpha) * acc_roll)
        self.pitch = wrap_angle(
            alpha * (self.pitch + gy * dt) + (1.0 - alpha) * acc_pitch
        )
        self.yaw = wrap_angle(self.yaw + gz * dt)

    @property
    def displayed(self) -> tuple[float, float, float]:
        return tuple(
            wrap_angle(value - offset)
            for value, offset in zip((self.roll, self.pitch, self.yaw), self.zero)
        )


@dataclass(frozen=True)
class HistoryPoint:
    received_at: float
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    temperature_c: float


def rotate_xyz(
    point: tuple[float, float, float], roll: float, pitch: float, yaw: float
) -> tuple[float, float, float]:
    x, y, z = point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    y, z = y * cr - z * sr, y * sr + z * cr
    x, z = x * cp + z * sp, -x * sp + z * cp
    x, y = x * cy - y * sy, x * sy + y * cy
    return x, y, z


class Visualizer:
    def __init__(
        self,
        event_queue: "queue.Queue[object]",
        reader: Union[SerialReader, DemoReader],
        duration: float,
        screenshot: Optional[Path],
    ) -> None:
        pygame.init()
        pygame.display.set_caption("RoboMaster Type C — BMI088 USB Visualizer")
        self.screen = pygame.display.set_mode((1280, 800), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.events = event_queue
        self.reader = reader
        self.duration = duration
        self.screenshot = screenshot
        self.started_at = time.monotonic()
        self.running = True

        self.font_title = pygame.font.SysFont("Segoe UI Semibold", 27)
        self.font_heading = pygame.font.SysFont("Segoe UI Semibold", 18)
        self.font_large = pygame.font.SysFont("Consolas", 31)
        self.font_value = pygame.font.SysFont("Consolas", 22)
        self.font_body = pygame.font.SysFont("Segoe UI", 16)
        self.font_small = pygame.font.SysFont("Segoe UI", 13)

        self.connection = ConnectionEvent("waiting", None, "")
        self.hello = HelloFrame("RMDEV-C", "BMI088", 100, 3.0, 2000.0)
        self.latest: Optional[ImuFrame] = None
        self.latest_power: Optional[PowerFrame] = None
        self.stats: Optional[StatFrame] = None
        self.accel_g = (0.0, 0.0, 0.0)
        self.gyro_dps = (0.0, 0.0, 0.0)
        self.attitude = AttitudeFilter()
        self.history: deque[HistoryPoint] = deque(maxlen=2000)
        self.frame_times: deque[float] = deque(maxlen=500)

    def run(self) -> None:
        while self.running:
            self._pygame_events()
            self._protocol_events()
            self._trim_history()
            self._draw()
            pygame.display.flip()
            self.clock.tick(60)

            if self.duration > 0.0 and time.monotonic() - self.started_at >= self.duration:
                self.running = False

        if self.screenshot:
            self.screenshot.parent.mkdir(parents=True, exist_ok=True)
            pygame.image.save(self.screen, str(self.screenshot))

    def _pygame_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_r:
                    self.attitude.recenter()
                elif event.key == pygame.K_c:
                    self.history.clear()

    def _protocol_events(self) -> None:
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break

            if isinstance(event, ConnectionEvent):
                self.connection = event
            elif isinstance(event, HelloFrame):
                self.hello = event
            elif isinstance(event, StatFrame):
                self.stats = event
            elif isinstance(event, PowerFrame):
                self.latest_power = event
            elif isinstance(event, ImuFrame):
                self.latest = event
                accel_scale = self.hello.accel_range_g / 32768.0
                gyro_scale = self.hello.gyro_range_dps / 32768.0
                self.accel_g = tuple(value * accel_scale for value in event.accel_raw)
                self.gyro_dps = tuple(value * gyro_scale for value in event.gyro_raw)
                self.attitude.update(event, self.accel_g, self.gyro_dps)
                self.history.append(
                    HistoryPoint(
                        event.received_at,
                        self.accel_g,
                        self.gyro_dps,
                        event.temperature_c,
                    )
                )
                self.frame_times.append(event.received_at)

    def _trim_history(self) -> None:
        cutoff = time.monotonic() - 12.0
        while self.history and self.history[0].received_at < cutoff:
            self.history.popleft()
        rate_cutoff = time.monotonic() - 1.0
        while self.frame_times and self.frame_times[0] < rate_cutoff:
            self.frame_times.popleft()

    def _draw(self) -> None:
        self.screen.fill(BACKGROUND)
        width, height = self.screen.get_size()
        self._draw_header(pygame.Rect(0, 0, width, 68))

        margin = 18
        graph_height = max(165, min(220, int(height * 0.25)))
        content_top = 82
        content_bottom = height - graph_height - margin * 2
        left_width = int(width * 0.56)
        cube_rect = pygame.Rect(
            margin,
            content_top,
            left_width - margin - 5,
            max(300, content_bottom - content_top),
        )
        readout_rect = pygame.Rect(
            left_width + 5,
            content_top,
            width - left_width - margin - 5,
            cube_rect.height,
        )
        graphs_top = height - graph_height - margin
        graph_gap = 12
        graph_width = (width - margin * 2 - graph_gap) // 2
        accel_graph = pygame.Rect(margin, graphs_top, graph_width, graph_height)
        gyro_graph = pygame.Rect(
            margin + graph_width + graph_gap,
            graphs_top,
            width - margin * 2 - graph_width - graph_gap,
            graph_height,
        )

        self._panel(cube_rect)
        self._panel(readout_rect)
        self._panel(accel_graph)
        self._panel(gyro_graph)
        self._draw_cube(cube_rect)
        self._draw_readouts(readout_rect)
        self._draw_graph(
            accel_graph,
            "Acceleration history",
            [point.accel_g for point in self.history],
            minimum_scale=1.2,
            unit="g",
        )
        self._draw_graph(
            gyro_graph,
            "Angular-rate history",
            [point.gyro_dps for point in self.history],
            minimum_scale=10.0,
            unit="°/s",
        )

    def _panel(self, rect: pygame.Rect) -> None:
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=14)
        pygame.draw.rect(self.screen, GRID, rect, width=1, border_radius=14)

    def _draw_header(self, rect: pygame.Rect) -> None:
        pygame.draw.rect(self.screen, PANEL_ALT, rect)
        pygame.draw.line(self.screen, GRID, (0, rect.bottom), (rect.width, rect.bottom))
        self._text(
            "BMI088  /  USB IMU",
            self.font_title,
            TEXT,
            (22, rect.centery),
            "midleft",
        )

        now = time.monotonic()
        age = math.inf if self.latest is None else now - self.latest.received_at
        state = self.connection.state
        if state == "connected" and age <= 0.35:
            status, color = "LIVE", GREEN
        elif state == "connected":
            status, color = "STALE", AMBER
        elif state in {"connecting", "waiting"}:
            status, color = "WAITING", AMBER
        else:
            status, color = "DISCONNECTED", RED

        status_x = rect.width - 22
        self._text(status, self.font_heading, color, (status_x, 22), "topright")
        port_text = self.connection.port or "auto-detecting STM32 CDC"
        rate = len(self.frame_times)
        self._text(
            f"{port_text}   •   {rate:3d} frames/s",
            self.font_small,
            MUTED,
            (status_x, 47),
            "midright",
        )
        pygame.draw.circle(self.screen, color, (status_x - 82, 23), 5)

    def _draw_cube(self, rect: pygame.Rect) -> None:
        self._text(
            "Fused attitude",
            self.font_heading,
            TEXT,
            (rect.x + 18, rect.y + 15),
        )
        self._text(
            "Accelerometer-corrected roll/pitch; gyro-integrated yaw",
            self.font_small,
            MUTED,
            (rect.x + 18, rect.y + 42),
        )

        if self.latest is None:
            self._text(
                "Waiting for IMU frames…",
                self.font_large,
                MUTED,
                rect.center,
                "center",
            )
            return

        roll, pitch, yaw = self.attitude.displayed
        center = (rect.centerx, rect.centery + 8)
        size = min(rect.width, rect.height) * 0.40
        camera = 5.0

        vertices = [
            (-1.6, -1.0, -0.18),
            (1.6, -1.0, -0.18),
            (1.6, 1.0, -0.18),
            (-1.6, 1.0, -0.18),
            (-1.6, -1.0, 0.18),
            (1.6, -1.0, 0.18),
            (1.6, 1.0, 0.18),
            (-1.6, 1.0, 0.18),
        ]
        rotated = [rotate_xyz(vertex, roll, pitch, yaw) for vertex in vertices]

        def project(point: tuple[float, float, float]) -> tuple[int, int]:
            x, y, z = point
            factor = size / (camera - z)
            return int(center[0] + x * factor), int(center[1] - y * factor)

        projected = [project(vertex) for vertex in rotated]
        faces = [
            ((0, 1, 2, 3), (37, 51, 72)),
            ((4, 5, 6, 7), (42, 115, 150)),
            ((0, 1, 5, 4), (28, 77, 104)),
            ((2, 3, 7, 6), (32, 87, 116)),
            ((1, 2, 6, 5), (35, 96, 128)),
            ((0, 3, 7, 4), (25, 68, 93)),
        ]
        faces.sort(key=lambda item: sum(rotated[index][2] for index in item[0]) / 4.0)
        for indices, color in faces:
            polygon = [projected[index] for index in indices]
            pygame.draw.polygon(self.screen, color, polygon)
            pygame.draw.aalines(self.screen, (105, 191, 222), True, polygon)

        self._text("RMDEV-C", self.font_heading, TEXT, center, "center")

        origin = project((0.0, 0.0, 0.0))
        for axis, label, color in (
            ((2.25, 0.0, 0.0), "X", AXIS_COLORS[0]),
            ((0.0, 1.65, 0.0), "Y", AXIS_COLORS[1]),
            ((0.0, 0.0, 1.45), "Z", AXIS_COLORS[2]),
        ):
            endpoint = project(rotate_xyz(axis, roll, pitch, yaw))
            pygame.draw.line(self.screen, color, origin, endpoint, 3)
            pygame.draw.circle(self.screen, color, endpoint, 4)
            self._text(label, self.font_heading, color, endpoint, "midleft")

        degrees = tuple(math.degrees(value) for value in (roll, pitch, yaw))
        labels = (
            f"ROLL  {degrees[0]:+7.2f}°",
            f"PITCH {degrees[1]:+7.2f}°",
            f"YAW   {degrees[2]:+7.2f}°",
        )
        y = rect.bottom - 72
        for index, label in enumerate(labels):
            x = rect.x + 28 + index * max(150, (rect.width - 50) // 3)
            self._text(label, self.font_value, AXIS_COLORS[index], (x, y))
        self._text(
            "R: re-center    C: clear history    Esc: quit",
            self.font_small,
            MUTED,
            (rect.centerx, rect.bottom - 25),
            "center",
        )

    def _draw_readouts(self, rect: pygame.Rect) -> None:
        padding = 16
        inner = rect.inflate(-padding * 2, -padding * 2)
        vector_height = max(110, min(130, int(inner.height * 0.27)))
        accel_rect = pygame.Rect(inner.x, inner.y, inner.width, vector_height)
        gyro_rect = pygame.Rect(
            inner.x, accel_rect.bottom + 8, inner.width, vector_height
        )
        status_rect = pygame.Rect(
            inner.x,
            gyro_rect.bottom + 8,
            inner.width,
            inner.bottom - gyro_rect.bottom - 8,
        )

        self._vector_section(
            accel_rect,
            "Acceleration",
            self.accel_g,
            "g",
            self.latest.accel_raw if self.latest else (0, 0, 0),
        )
        self._vector_section(
            gyro_rect,
            "Angular rate",
            self.gyro_dps,
            "°/s",
            self.latest.gyro_raw if self.latest else (0, 0, 0),
        )
        self._status_section(status_rect)

    def _vector_section(
        self,
        rect: pygame.Rect,
        title: str,
        values: tuple[float, float, float],
        unit: str,
        raw: tuple[int, int, int],
    ) -> None:
        pygame.draw.rect(self.screen, PANEL_ALT, rect, border_radius=10)
        self._text(title, self.font_heading, TEXT, (rect.x + 12, rect.y + 9))
        column_width = rect.width // 3
        for index, axis in enumerate("XYZ"):
            x = rect.x + index * column_width + column_width // 2
            self._text(
                axis,
                self.font_small,
                AXIS_COLORS[index],
                (x, rect.y + 43),
                "center",
            )
            self._text(
                f"{values[index]:+8.3f}",
                self.font_value,
                TEXT,
                (x, rect.y + 70),
                "center",
            )
            self._text(
                f"{raw[index]:+6d} raw",
                self.font_small,
                MUTED,
                (x, rect.y + 96),
                "center",
            )
        self._text(unit, self.font_small, MUTED, (rect.right - 10, rect.y + 10), "topright")

    def _status_section(self, rect: pygame.Rect) -> None:
        pygame.draw.rect(self.screen, PANEL_ALT, rect, border_radius=10)
        if rect.width < 80 or rect.height < 55:
            return

        temperature = self.latest.temperature_c if self.latest else math.nan
        compact = rect.height < 190
        gap = 18
        column_width = (rect.width - gap) // 2
        temp_rect = pygame.Rect(rect.x, rect.y, column_width, rect.height)
        power_rect = pygame.Rect(
            temp_rect.right + gap, rect.y, rect.right - temp_rect.right - gap, rect.height
        )
        divider_x = temp_rect.right + gap // 2
        pygame.draw.line(
            self.screen,
            GRID,
            (divider_x, rect.y + 10),
            (divider_x, rect.y + (68 if compact else 119)),
        )

        self._text(
            "IMU Temperature",
            self.font_heading,
            TEXT,
            (temp_rect.x + 12, temp_rect.y + 9),
        )
        temp_text = "--.-- °C" if math.isnan(temperature) else f"{temperature:5.2f} °C"
        if math.isnan(temperature):
            temp_color = MUTED
        elif abs(temperature - 50.0) <= 2.0:
            temp_color = GREEN
        elif temperature > 60.0:
            temp_color = RED
        else:
            temp_color = AMBER
        self._text(
            temp_text,
            self.font_large,
            temp_color,
            (temp_rect.x + 12, temp_rect.y + 39),
        )

        power_state, power_color = self._power_status()
        voltage = self.latest_power.input_v if self.latest_power else math.nan
        voltage_text = "--.-- V" if math.isnan(voltage) else f"{voltage:5.2f} V"
        voltage_color = MUTED if power_state in {"NO DATA", "STALE"} else power_color
        self._text(
            "External Input",
            self.font_heading,
            TEXT,
            (power_rect.x + 3, power_rect.y + 9),
        )
        self._text(
            power_state,
            self.font_small,
            power_color,
            (power_rect.right - 9, power_rect.y + 12),
            "topright",
        )
        self._text(
            voltage_text,
            self.font_large,
            voltage_color,
            (power_rect.x + 3, power_rect.y + 39),
        )

        if not compact:
            temp_bar = pygame.Rect(
                temp_rect.x + 12, temp_rect.y + 79, temp_rect.width - 24, 10
            )
            pygame.draw.rect(self.screen, GRID, temp_bar, border_radius=5)
            if not math.isnan(temperature):
                fraction = max(0.0, min(1.0, (temperature - 20.0) / 40.0))
                fill = temp_bar.copy()
                fill.width = max(1, int(temp_bar.width * fraction))
                pygame.draw.rect(self.screen, temp_color, fill, border_radius=5)
            target_x = temp_bar.x + int(temp_bar.width * 0.75)
            pygame.draw.line(
                self.screen,
                TEXT,
                (target_x, temp_bar.y - 3),
                (target_x, temp_bar.bottom + 3),
                2,
            )
            self._text("20", self.font_small, MUTED, (temp_bar.x, temp_bar.bottom + 3))
            self._text(
                "50",
                self.font_small,
                TEXT,
                (target_x, temp_bar.bottom + 3),
                "midtop",
            )
            self._text(
                "60",
                self.font_small,
                MUTED,
                (temp_bar.right, temp_bar.bottom + 3),
                "topright",
            )
            self._text(
                "heater target 50 °C",
                self.font_small,
                MUTED,
                (temp_rect.x + 12, temp_rect.y + 108),
            )

            power_bar = pygame.Rect(
                power_rect.x + 3, power_rect.y + 79, power_rect.width - 12, 10
            )
            pygame.draw.rect(self.screen, GRID, power_bar, border_radius=5)
            valid_start = power_bar.x + int(
                power_bar.width * INPUT_VALID_MIN_V / INPUT_GAUGE_MAX_V
            )
            valid_end = power_bar.x + int(
                power_bar.width * INPUT_VALID_MAX_V / INPUT_GAUGE_MAX_V
            )
            pygame.draw.rect(
                self.screen,
                (34, 83, 70),
                pygame.Rect(valid_start, power_bar.y, valid_end - valid_start, power_bar.height),
            )
            pygame.draw.rect(
                self.screen,
                (90, 42, 52),
                pygame.Rect(valid_end, power_bar.y, power_bar.right - valid_end, power_bar.height),
                border_top_right_radius=5,
                border_bottom_right_radius=5,
            )
            if not math.isnan(voltage):
                voltage_x = power_bar.x + int(
                    power_bar.width
                    * max(0.0, min(INPUT_GAUGE_MAX_V, voltage))
                    / INPUT_GAUGE_MAX_V
                )
                pygame.draw.line(
                    self.screen,
                    power_color,
                    (voltage_x, power_bar.y - 4),
                    (voltage_x, power_bar.bottom + 4),
                    3,
                )
            self._text(
                "0",
                self.font_small,
                MUTED,
                (power_bar.x, power_bar.bottom + 3),
            )
            self._text(
                "8",
                self.font_small,
                GREEN,
                (valid_start, power_bar.bottom + 3),
                "midtop",
            )
            self._text(
                "28",
                self.font_small,
                GREEN,
                (valid_end, power_bar.bottom + 3),
                "midtop",
            )
            if self.latest_power:
                raw_text = (
                    f"ADC {self.latest_power.battery_adc_raw}  •  "
                    f"VREF {self.latest_power.vref_adc_raw}"
                )
            else:
                raw_text = "Waiting for 1 Hz PWR…"
            self._text(
                raw_text,
                self.font_small,
                MUTED,
                (power_rect.x + 3, power_rect.y + 108),
            )

        stats_y = rect.y + (126 if not compact else 72)
        if self.stats:
            uptime = self.stats.uptime_ms / 1000.0
            line1 = (
                f"uptime {uptime:7.1f}s   imu ok {self.stats.imu_ok:,}   "
                f"errors {self.stats.imu_err:,}"
            )
            line2 = (
                f"CAN1 rx {self.stats.can1_rx:,}   CAN2 rx {self.stats.can2_rx:,}   "
                f"drops {self.stats.rx_drop + self.stats.can_tx_queue_drop:,}"
            )
        else:
            line1 = "Waiting for STAT frame…"
            line2 = ""
        protocol_y = rect.bottom - 22
        if stats_y < protocol_y:
            self._text(line1, self.font_small, TEXT, (rect.x + 12, stats_y))
        if stats_y + 22 < protocol_y:
            self._text(line2, self.font_small, MUTED, (rect.x + 12, stats_y + 22))
        if rect.height >= 112:
            self._text(
                f"protocol v{PROTOCOL_VERSION}  •  {self.hello.telemetry_hz} Hz  •  "
                f"parser errors {self.reader.parse_errors}",
                self.font_small,
                MUTED,
                (rect.x + 12, protocol_y),
            )

    def _power_status(self) -> tuple[str, tuple[int, int, int]]:
        if self.latest_power is None:
            return "NO DATA", MUTED
        if time.monotonic() - self.latest_power.received_at > POWER_STALE_SECONDS:
            return "STALE", AMBER
        voltage = self.latest_power.input_v
        if voltage < INPUT_VALID_MIN_V:
            return "USB / SWD ONLY", AMBER
        if voltage <= INPUT_VALID_MAX_V:
            return "VALID", GREEN
        return "OVERVOLTAGE", RED

    def _draw_graph(
        self,
        rect: pygame.Rect,
        title: str,
        values: list[tuple[float, float, float]],
        minimum_scale: float,
        unit: str,
    ) -> None:
        self._text(title, self.font_heading, TEXT, (rect.x + 14, rect.y + 10))
        plot = pygame.Rect(rect.x + 46, rect.y + 40, rect.width - 62, rect.height - 61)
        if plot.width <= 2 or plot.height <= 2:
            return

        for division in range(5):
            y = plot.y + division * plot.height // 4
            pygame.draw.line(self.screen, GRID, (plot.x, y), (plot.right, y), 1)

        peak = minimum_scale
        for triple in values:
            peak = max(peak, *(abs(value) for value in triple))
        scale = peak * 1.12
        zero_y = plot.centery
        pygame.draw.line(self.screen, (76, 91, 112), (plot.x, zero_y), (plot.right, zero_y), 1)

        self._text(f"+{scale:.1f}", self.font_small, MUTED, (plot.x - 7, plot.y), "topright")
        self._text("0", self.font_small, MUTED, (plot.x - 7, zero_y), "midright")
        self._text(f"-{scale:.1f}", self.font_small, MUTED, (plot.x - 7, plot.bottom), "bottomright")
        self._text(unit, self.font_small, MUTED, (rect.right - 12, rect.y + 12), "topright")

        if len(values) >= 2:
            count = len(values)
            for axis in range(3):
                points = []
                for index, triple in enumerate(values):
                    x = plot.x + int(index * (plot.width - 1) / max(1, count - 1))
                    y = zero_y - int((triple[axis] / scale) * (plot.height / 2.0))
                    points.append((x, max(plot.y, min(plot.bottom, y))))
                pygame.draw.aalines(self.screen, AXIS_COLORS[axis], False, points)

        legend_x = rect.right - 105
        for axis, label in enumerate("XYZ"):
            self._text(
                label,
                self.font_small,
                AXIS_COLORS[axis],
                (legend_x + axis * 28, rect.y + 17),
                "center",
            )

    def _text(
        self,
        value: str,
        font: pygame.font.Font,
        color: tuple[int, int, int],
        position: tuple[int, int],
        anchor: str = "topleft",
    ) -> None:
        rendered = font.render(value, True, color)
        rectangle = rendered.get_rect()
        setattr(rectangle, anchor, position)
        self.screen.blit(rendered, rectangle)


def run_self_test() -> None:
    now = 123.0
    hello = parse_protocol_line("HELLO,1,RMDEV-C,BMI088,100,3,2000", now)
    assert isinstance(hello, HelloFrame)
    assert hello.telemetry_hz == 100

    imu = parse_protocol_line(
        "IMU,1,6751,13500,-1,-1183,10767,-5,-7,-2,3588", now
    )
    assert isinstance(imu, ImuFrame)
    assert imu.accel_raw == (-1, -1183, 10767)
    assert imu.gyro_raw == (-5, -7, -2)
    assert imu.temperature_c == 35.88
    assert imu.received_at == now

    stat = parse_protocol_line("STAT,1,13460,0,0,0,0,0,0,6731,0", now)
    assert isinstance(stat, StatFrame)
    assert stat.imu_ok == 6731 and stat.imu_err == 0

    power = parse_protocol_line("PWR,1,13500,14950,1689,1502", now)
    assert isinstance(power, PowerFrame)
    assert power.timestamp_ms == 13500
    assert power.input_mv == 14950 and power.input_v == 14.95
    assert power.battery_adc_raw == 1689 and power.vref_adc_raw == 1502
    assert power.received_at == now

    assert parse_protocol_line("IMU,2,1,2,3,4,5,6,7,8,9", now) is None
    assert parse_protocol_line("IMU,1,1,2,40000,4,5,6,7,8,9", now) is None
    assert parse_protocol_line("PWR,2,1,14950,1689,1502", now) is None
    assert parse_protocol_line("PWR,1,1,60001,1689,1502", now) is None
    assert parse_protocol_line("PWR,1,1,14950,4096,1502", now) is None
    assert parse_protocol_line("PWR,1,1,14950,1689,-1", now) is None
    assert parse_protocol_line("garbage", now) is None

    buffer = bytearray()
    assert extract_complete_lines(buffer, b"IMU,1,1") == []
    assert extract_complete_lines(buffer, b",2,3,4,5,6,7,8,9,10,11\r\nNEXT\n") == [
        "IMU,1,1,2,3,4,5,6,7,8,9,10,11",
        "NEXT",
    ]
    print("imu_visualizer protocol self-test: PASS")


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        vid_pid = (
            f"{port.vid:04X}:{port.pid:04X}"
            if port.vid is not None and port.pid is not None
            else "----:----"
        )
        marker = "  <STM32 CDC>" if port.vid == STM32_VID and port.pid == STM32_CDC_PID else ""
        print(f"{port.device:8s} {vid_pid}  {port.description}{marker}")


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        help="Serial port override (for example COM31); default auto-detects 0483:5740",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--demo", action="store_true", help="Use synthetic IMU data")
    parser.add_argument("--self-test", action="store_true", help="Test the protocol parser and exit")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Exit after N seconds (primarily useful with --demo)",
    )
    parser.add_argument(
        "--screenshot",
        type=Path,
        help="Save the final Pygame frame as an image",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = argument_parser().parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    if args.list:
        list_serial_ports()
        return 0

    event_queue: "queue.Queue[object]" = queue.Queue(maxsize=512)
    stop_event = threading.Event()
    reader: Union[SerialReader, DemoReader]
    if args.demo:
        reader = DemoReader(event_queue, stop_event)
    else:
        reader = SerialReader(event_queue, stop_event, args.port, args.baud)

    reader.start()
    visualizer = Visualizer(event_queue, reader, args.duration, args.screenshot)
    try:
        visualizer.run()
    finally:
        stop_event.set()
        reader.join(timeout=2.0)
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
