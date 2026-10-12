"""
Rocket telemetry ground station — FC panel (LoRa bridge: TDD two-way link,
settings read/apply/save), live altitude/velocity plots, and a live OSM map
of the rocket position (tiles fetched on a background thread, cached in RAM).
Uses PySide6 (Qt) for the GUI. Install with:  pip install PySide6 pyserial
"""

import sys
import os
import csv
import json
import math
import queue
import struct
import threading
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import serial
import serial.tools.list_ports
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QComboBox,
    QLineEdit,
    QPushButton,
    QMessageBox,
    QFrame,
    QGroupBox,
)
from PySide6.QtCore import QTimer, Qt, QRectF, QPointF
from PySide6.QtGui import QFont, QPainter, QPen, QBrush, QColor, QPixmap, QPolygonF

# ---------- CONFIG ----------
DEFAULT_BAUD = 57600
DEBUG_PRINT = True

# Packet format: extended with attitude (roll/pitch/yaw centidegrees)
PACKET_FMT_OLD = "<2sIihiiBB"   # 22 bytes, no attitude
PACKET_FMT = "<2sIihiiBhhhB"    # 28 bytes, with roll_centi, pitch_centi, yaw_centi
PACKET_SIZE_OLD = struct.calcsize(PACKET_FMT_OLD)
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # use max for buffer
STATE_NAMES = [
    "READY", "CALIBRATING", "INIT", "ARMED", "BOOST", "APOGEE",
    "DEPLOY_DROGUE", "DROGUE_DESCENT", "DEPLOY_MAIN", "MAIN_DESCENT", "FINISHED"
]

# FC flight-config keys (must match CFG_KEYS in RocketComputer.ino)
# (key, type, label) — type: u32 = integer, f32 = decimal, bool = 0/1
SETTINGS_KEYS = [
    ("launch_alt_gain_m", "f32", "Launch alt gain (m)"),
    ("launch_anet_thresh", "f32", "Launch accel thresh (m/s²)"),
    ("launch_confirm_ms", "u32", "Launch confirm (ms)"),
    ("min_time_after_arm_ms", "u32", "Min time after arm (ms)"),
    ("armed_idle_timeout_ms", "u32", "Armed idle timeout (ms)"),
    ("apogee_min_ms", "u32", "Apogee min (ms)"),
    ("apogee_confirm_ms", "u32", "Apogee confirm (ms)"),
    ("apogee_vneg_thresh", "f32", "Apogee vneg thresh (m/s)"),
    ("drogue_min_ms", "u32", "Drogue min (ms)"),
    ("drogue_max_ms", "u32", "Drogue max (ms)"),
    ("main_deploy_alt_m", "f32", "Main deploy alt (m)"),
    ("main_min_ms", "u32", "Main min (ms)"),
    ("main_max_ms", "u32", "Main max (ms)"),
    ("airstart_enabled", "bool", "Airstart enabled"),
    ("airstart_ms_after_launch", "u32", "Airstart delay (ms)"),
    ("ground_baro_stable_ms", "u32", "Ground baro stable (ms)"),
    ("ground_baro_thresh_pa", "f32", "Ground baro thresh (Pa)"),
    ("ground_alt_near_m", "f32", "Ground alt near (m)"),
    ("ground_alt_hold_ms", "u32", "Ground alt hold (ms)"),
    ("pyro_pulse_ms", "u32", "Pyro pulse (ms)"),
]

# States in which the FC accepts SET (two-way TDD link, settings unlocked)
SETTINGS_STATES = (0, 2)  # READY, INIT


# CSV logging
DATA_DIR = Path("data")
CSV_HEADER = ["timestamp_utc", "time_ms", "alt_m", "vel_m_s", "lat", "lon", "state", "state_name", "roll_deg", "pitch_deg", "yaw_deg"]


class AttitudeWidget(QWidget):
    """Simple attitude indicator: horizon line (roll) + pitch offset. Roll/pitch in degrees."""
    def __init__(self, size=140, parent=None):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self.roll = 0.0   # degrees
        self.pitch = 0.0  # degrees
        self.size = size

    def set_attitude(self, roll: float, pitch: float):
        self.roll = roll if roll is not None else 0.0
        self.pitch = pitch if pitch is not None else 0.0
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx, cy = self.size / 2, self.size / 2
        r = (self.size / 2) - 10
        roll_rad = math.radians(-self.roll)
        pitch_scale = 2.5  # pixels per degree
        dy = self.pitch * pitch_scale  # nose up = horizon moves down
        dx = r * math.cos(roll_rad)
        dy_h = r * math.sin(roll_rad)
        x1, y1 = cx - dx, cy - dy_h + dy
        x2, y2 = cx + dx, cy + dy_h + dy
        p.setPen(QPen(QColor(70, 70, 70), 2))
        p.setBrush(QBrush(QColor(240, 240, 245)))
        p.drawEllipse(5, 5, self.size - 10, self.size - 10)
        p.setPen(QPen(QColor(0, 0, 0), 3))
        p.drawLine(int(x1), int(y1), int(x2), int(y2))
        p.setPen(QPen(QColor(220, 20, 20), 2))
        p.setBrush(QBrush(QColor(220, 20, 20)))
        nose_y = cy - int(r * 0.65) - int(dy)
        p.drawEllipse(int(cx - 5), int(nose_y - 5), 10, 10)
        p.setPen(QPen(QColor(50, 50, 50), 2))
        p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        p.drawEllipse(5, 5, self.size - 10, self.size - 10)


# ---------------- Live plots ----------------
class LivePlotWidget(QWidget):
    """Rolling time-series: altitude (blue, left axis) and velocity (red, right axis)."""

    def __init__(self, window_s=120.0, max_points=7200, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(230)
        self._window_s = window_s
        self._samples = []       # (t_s, alt_m|None, vel_m/s|None)
        self._max_points = max_points

    def add_sample(self, t_s, alt, vel):
        self._samples.append((t_s, alt, vel))
        # trim: by count and by age (keep a bit beyond the display window)
        if len(self._samples) > self._max_points:
            self._samples = self._samples[-self._max_points // 2:]
        cutoff = t_s - self._window_s * 3
        while len(self._samples) > 2 and self._samples[0][0] < cutoff:
            self._samples.pop(0)
        self.update()

    def clear(self):
        self._samples = []
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(255, 255, 255))
        if len(self._samples) < 2:
            p.setPen(QColor(130, 130, 130))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for telemetry…")
            return
        w, h = self.width(), self.height()
        ml, mr, mt, mb = 46, 46, 26, 24
        cw, ch = w - ml - mr, h - mt - mb
        t_end = self._samples[-1][0]
        t0 = t_end - self._window_s
        win = [s for s in self._samples if s[0] >= t0]
        if len(win) < 2:
            win = self._samples[-2:]
        t0, t_end = win[0][0], win[-1][0]
        span = max(1e-6, t_end - t0)
        alts = [s[1] for s in win if s[1] is not None]
        vels = [s[2] for s in win if s[2] is not None]
        if not alts:
            p.setPen(QColor(130, 130, 130))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for telemetry…")
            return
        a_lo, a_hi = min(alts), max(alts)
        if a_hi - a_lo < 1.0:
            a_hi = a_lo + 1.0
        if vels:
            v_lo, v_hi = min(vels), max(vels)
            if v_hi - v_lo < 1.0:
                v_hi = v_lo + 1.0
        else:
            v_lo, v_hi = 0.0, 1.0

        def xa(t):
            return ml + (t - t0) / span * cw

        def ya(a):
            return mt + ch - (a - a_lo) / (a_hi - a_lo) * ch

        def yv(v):
            return mt + ch - (v - v_lo) / (v_hi - v_lo) * ch

        # grid + axis labels
        p.setFont(QFont("Segoe UI", 8))
        p.setPen(QColor(228, 228, 228))
        for i in range(5):
            y = mt + ch * i / 4
            p.drawLine(int(ml), int(y), int(w - mr), int(y))
        p.setPen(QColor(100, 100, 100))
        for i in range(5):
            y = mt + ch * i / 4
            av = a_hi - (a_hi - a_lo) * i / 4
            p.drawText(2, int(y) + 3, f"{av:.0f} m")
            vv = v_hi - (v_hi - v_lo) * i / 4
            p.drawText(int(w - mr + 4), int(y) + 3, f"{vv:.0f}")
        for i in range(6):
            tv = t0 + span * i / 5
            p.setPen(QColor(228, 228, 228))
            p.drawLine(int(xa(tv)), int(mt), int(xa(tv)), int(mt + ch))
            p.setPen(QColor(100, 100, 100))
            p.drawText(int(xa(tv)) - 12, h - 8, f"{tv:.0f}s")
        # series
        alt_pts = []
        vel_pts = []
        for (t, a, v) in win:
            if a is not None:
                alt_pts.append(QPointF(xa(t), ya(a)))
            if v is not None:
                vel_pts.append(QPointF(xa(t), yv(v)))
        if len(alt_pts) >= 2:
            p.setPen(QPen(QColor(0, 102, 204), 2))
            p.drawPolyline(QPolygonF(alt_pts))
        if len(vel_pts) >= 2:
            p.setPen(QPen(QColor(204, 51, 0), 2))
            p.drawPolyline(QPolygonF(vel_pts))
        # legend
        p.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        p.setPen(QColor(0, 102, 204))
        p.drawText(int(ml) + 8, int(mt) + 14, "ALTITUDE (m)")
        p.setPen(QColor(204, 51, 0))
        p.drawText(int(ml) + 110, int(mt) + 14, "VELOCITY (m/s, right axis)")


# ---------------- Map (OpenStreetMap tiles, RAM cache, background fetch) ----------------
TILE_SIZE = 256
MAP_ZOOM = 16
MAP_GRID = 3           # 3x3 tiles at z16 ≈ ±0.7–1 km square
OSM_USER_AGENT = "RocketGroundStation/1.0 (amateur rocketry ground station, personal use)"


def lonlat_to_global_px(lat, lon, zoom):
    """Slippy-map projection: lat/lon -> global pixel coords at zoom."""
    n = 2.0 ** zoom * TILE_SIZE
    x = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def fetch_pc_location(timeout=6.0):
    """Best-effort geo-IP lookup of the ground station's rough location. None on failure."""
    for url in ("https://ipapi.co/json/", "http://ip-api.com/json/"):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": OSM_USER_AGENT})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                d = json.loads(r.read().decode("utf-8", "replace"))
            lat = d.get("lat", d.get("latitude"))
            lon = d.get("lon", d.get("longitude"))
            if lat is not None and lon is not None:
                return float(lat), float(lon)
        except Exception:
            continue
    return None


class TileFetcher:
    """Downloads OSM tiles on a worker thread into a RAM cache; GUI drains results."""

    def __init__(self):
        self._lock = threading.Lock()
        self._cache = {}               # (z, x, y) -> png bytes
        self._pending = set()
        self._queue = queue.Queue()
        self._results = queue.Queue()  # GUI drains: (z, x, y, png bytes)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def request(self, z, x, y):
        key = (z, x, y)
        with self._lock:
            if key in self._cache or key in self._pending:
                return
            self._pending.add(key)
        self._queue.put(key)

    def drain(self):
        out = []
        while True:
            try:
                out.append(self._results.get_nowait())
            except queue.Empty:
                break
        return out

    def stop(self):
        self._stop.set()

    def _worker(self):
        while not self._stop.is_set():
            try:
                z, x, y = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            data = None
            try:
                url = f"https://tile.openstreetmap.org/{z}/{x}/{y}.png"
                req = urllib.request.Request(url, headers={"User-Agent": OSM_USER_AGENT})
                with urllib.request.urlopen(req, timeout=10) as r:
                    data = r.read()
            except Exception:
                pass
            with self._lock:
                self._pending.discard((z, x, y))
                if data:
                    self._cache[(z, x, y)] = data
            if data:
                self._results.put((z, x, y, data))


class MapWidget(QWidget):
    """Live map around the launch pad: cached OSM tiles + rocket track + position."""

    def __init__(self, fetcher: TileFetcher, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(320)
        self._fetcher = fetcher
        self._pixmaps = {}     # (z, x, y) -> QPixmap
        self._origin = None    # (tile_x, tile_y) of grid NW corner
        self._zoom = MAP_ZOOM
        self._center = None
        self._track = []       # (lat, lon), capped
        self._status = "Waiting for location (PC geo-IP or first GPS fix)…"

    def set_center(self, lat, lon):
        gx, gy = lonlat_to_global_px(lat, lon, self._zoom)
        cx, cy = int(gx // TILE_SIZE), int(gy // TILE_SIZE)
        self._origin = (cx - MAP_GRID // 2, cy - MAP_GRID // 2)
        self._center = (lat, lon)
        self._status = "Loading map tiles…"
        ox, oy = self._origin
        for dy in range(MAP_GRID):
            for dx in range(MAP_GRID):
                self._fetcher.request(self._zoom, ox + dx, oy + dy)
        self.update()

    def add_position(self, lat, lon):
        self._track.append((lat, lon))
        if len(self._track) > 3000:
            self._track = self._track[-1500:]
        self.update()

    def last_position(self):
        return self._track[-1] if self._track else None

    def clear_track(self):
        self._track = []
        self.update()

    def ingest_tiles(self, tiles):
        changed = False
        for (z, x, y, data) in tiles:
            pm = QPixmap()
            if pm.loadFromData(data):
                self._pixmaps[(z, x, y)] = pm
                changed = True
        if changed:
            self._status = ""
            self.update()

    def has_tiles(self):
        return any(k[0] == self._zoom for k in self._pixmaps)

    def _geo_to_screen(self, lat, lon, scale, ox, oy):
        gx, gy = lonlat_to_global_px(lat, lon, self._zoom)
        sx = ox + (gx - self._origin[0] * TILE_SIZE) * scale
        sy = oy + (gy - self._origin[1] * TILE_SIZE) * scale
        return sx, sy

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(233, 236, 233))
        if self._origin is None:
            p.setPen(QColor(110, 110, 110))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._status)
            return
        grid_px = MAP_GRID * TILE_SIZE
        scale = min(self.width(), self.height()) / grid_px
        ox = (self.width() - grid_px * scale) / 2
        oy = (self.height() - grid_px * scale) / 2
        tx0, ty0 = self._origin
        for dy in range(MAP_GRID):
            for dx in range(MAP_GRID):
                key = (self._zoom, tx0 + dx, ty0 + dy)
                pm = self._pixmaps.get(key)
                rect = QRectF(ox + dx * TILE_SIZE * scale, oy + dy * TILE_SIZE * scale,
                              TILE_SIZE * scale, TILE_SIZE * scale)
                if pm is not None:
                    p.drawPixmap(rect, pm, QRectF(0, 0, TILE_SIZE, TILE_SIZE))
                else:
                    p.fillRect(rect, QColor(222, 226, 222))
        if not self.has_tiles():
            p.setPen(QColor(110, 110, 110))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                       self._status or "Map unavailable (offline?) — tiles failed to load")
        if len(self._track) >= 2:
            p.setPen(QPen(QColor(255, 210, 0), 2))
            pts = [QPointF(*self._geo_to_screen(la, lo, scale, ox, oy)) for (la, lo) in self._track]
            p.drawPolyline(QPolygonF(pts))
        if self._track:
            sx, sy = self._geo_to_screen(*self._track[-1], scale, ox, oy)
            p.setPen(QPen(QColor(200, 30, 30), 2))
            p.setBrush(QBrush(QColor(220, 40, 40)))
            p.drawEllipse(QPointF(sx, sy), 5, 5)
        # OSM attribution (required by the tile license)
        p.setPen(QColor(90, 90, 90))
        p.setFont(QFont("Segoe UI", 8))
        p.drawText(6, self.height() - 6, "© OpenStreetMap contributors")


def get_serial_ports():
    """Return list of (device_name, description) for COM/serial ports. OS-aware."""
    ports = list(serial.tools.list_ports.comports())
    result = []
    for p in ports:
        device = p.device
        desc = p.description or ""
        result.append((device, f"{device} — {desc}"))
    return sorted(result, key=lambda x: x[0])


class TelemetryPanel(QFrame):
    """One telemetry source (FC1 or FC2): port selection, connect, HUD, ARM."""

    def __init__(self, title: str, baud_default: int = DEFAULT_BAUD, parent=None):
        super().__init__(parent)
        self.title = title
        self.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        self.setLineWidth(1)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)

        self.time_ms = None
        self.alt = None
        self.vel = None
        self.lat = None
        self.lon = None
        self.state = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.log_file = None  # when set, writer thread writes each packet to this file and flushes
        self.last_raw_hex = ""
        self.last_error_msg = ""
        self.last_packet_time = None
        self.connection_lost = False
        self.connection_timeout_sec = 3.0
        self._status_text = "Disconnected"  # set by reader thread; displayed in main thread

        self.ser = None
        self._stop = threading.Event()
        self._thread = None

        # ACK/NACK tracking for sent commands (filled by reader thread)
        self._ack_lock = threading.Lock()
        self._ack_lines = []
        self._ack_event = threading.Event()
        self._cmd_busy = False

        # FC settings (CFG lines from FC)
        self._cfg_lock = threading.Lock()
        self._cfg_values = {}      # key -> str(value) as last read from FC
        self._cfg_dirty = False    # reader thread signals update_ui to refresh fields
        self._cfg_fields = {}      # key -> QLineEdit

        # COM port on its own line
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(280)
        self.port_combo.setEditable(False)
        port_row.addWidget(self.port_combo)
        port_row.addStretch()
        layout.addLayout(port_row)

        # Baud rate on its own line
        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("Baud:"))
        self.baud_edit = QLineEdit()
        self.baud_edit.setMaximumWidth(100)
        self.baud_edit.setText(str(baud_default))
        baud_row.addWidget(self.baud_edit)
        baud_row.addStretch()
        layout.addLayout(baud_row)

        # Status and buttons
        btn_row = QHBoxLayout()
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #666;")
        btn_row.addWidget(self.status_label)
        btn_row.addStretch()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect)
        btn_row.addWidget(self.connect_btn)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect)
        self.disconnect_btn.setEnabled(False)
        btn_row.addWidget(self.disconnect_btn)
        self.start_btn = QPushButton("START")
        self.start_btn.clicked.connect(self.send_start_command)
        self.start_btn.setEnabled(False)
        self.start_btn.setToolTip("Send START: begin sensor init + 10 s calibration (from READY)")
        btn_row.addWidget(self.start_btn)
        self.rearm_btn = QPushButton("ARM")
        self.rearm_btn.clicked.connect(self.send_rearm_command)
        self.rearm_btn.setEnabled(False)
        self.rearm_btn.setToolTip("Send ARM: INIT → ARMED (explicit command only, no auto-arm). Also re-arms after idle timeout returns FC to INIT")
        btn_row.addWidget(self.rearm_btn)
        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.clicked.connect(self.send_disarm_command)
        self.disarm_btn.setEnabled(False)
        self.disarm_btn.setToolTip("Send DISARM: ARMED → INIT (safe disarm on pad)")
        btn_row.addWidget(self.disarm_btn)
        layout.addLayout(btn_row)

        # State (big)
        state_label = QLabel("STATE")
        state_label.setFont(QFont("Segoe UI", 12))
        layout.addWidget(state_label)
        self.state_value = QLabel("--")
        self.state_value.setFont(QFont("Segoe UI", 36, QFont.Weight.Bold))
        self.state_value.setStyleSheet("color: #0066cc;")
        layout.addWidget(self.state_value)

        # Metrics grid
        grid = QGridLayout()
        big_font = QFont("Segoe UI", 24, QFont.Weight.Bold)
        label_font = QFont("Segoe UI", 11)
        self.alt_value = QLabel("-- m")
        self.alt_value.setFont(big_font)
        self.vel_value = QLabel("-- m/s")
        self.vel_value.setFont(big_font)
        self.lat_value = QLabel("--")
        self.lat_value.setFont(big_font)
        self.lon_value = QLabel("--")
        self.lon_value.setFont(big_font)
        grid.addWidget(QLabel("ALTITUDE"), 0, 0)
        grid.addWidget(self.alt_value, 1, 0)
        grid.addWidget(QLabel("VELOCITY"), 0, 1)
        grid.addWidget(self.vel_value, 1, 1)
        grid.addWidget(QLabel("LATITUDE"), 2, 0)
        grid.addWidget(self.lat_value, 3, 0)
        grid.addWidget(QLabel("LONGITUDE"), 2, 1)
        grid.addWidget(self.lon_value, 3, 1)
        for i in range(4):
            grid.setRowStretch(i, 0)
        layout.addLayout(grid)

        # Attitude (roll/pitch/yaw + indicator)
        att_row = QHBoxLayout()
        att_row.addWidget(QLabel("ATTITUDE"))
        self.roll_value = QLabel("--°")
        self.pitch_value = QLabel("--°")
        self.yaw_value = QLabel("--°")
        att_row.addWidget(QLabel("R:"))
        att_row.addWidget(self.roll_value)
        att_row.addWidget(QLabel("P:"))
        att_row.addWidget(self.pitch_value)
        att_row.addWidget(QLabel("Y:"))
        att_row.addWidget(self.yaw_value)
        att_row.addStretch()
        self.attitude_widget = AttitudeWidget(120)
        att_row.addWidget(self.attitude_widget)
        layout.addLayout(att_row)

        # Debug row
        debug_layout = QGridLayout()
        debug_layout.addWidget(QLabel("Last packet (hex):"), 0, 0)
        self.last_hex_label = QLabel("")
        self.last_hex_label.setFont(QFont("Consolas", 9))
        self.last_hex_label.setWordWrap(True)
        debug_layout.addWidget(self.last_hex_label, 0, 1)
        debug_layout.addWidget(QLabel("Error:"), 1, 0)
        self.last_err_label = QLabel("")
        self.last_err_label.setStyleSheet("color: #a00;")
        self.last_err_label.setWordWrap(True)
        debug_layout.addWidget(self.last_err_label, 1, 1)
        debug_layout.addWidget(QLabel("Since last packet:"), 2, 0)
        self.watchdog_label = QLabel("--")
        self.watchdog_label.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        debug_layout.addWidget(self.watchdog_label, 2, 1)
        layout.addLayout(debug_layout)

        # ---- FC settings (two-way TDD link: readable/editable in READY/INIT only) ----
        self.settings_box = QGroupBox("FC Settings (two-way until ARMED — editable in READY/INIT only)")
        settings_layout = QVBoxLayout()
        cfg_btns = QHBoxLayout()
        self.cfg_read_btn = QPushButton("Read from FC")
        self.cfg_read_btn.clicked.connect(self.read_settings)
        cfg_btns.addWidget(self.cfg_read_btn)
        self.cfg_apply_btn = QPushButton("Apply changes")
        self.cfg_apply_btn.clicked.connect(self.apply_settings)
        cfg_btns.addWidget(self.cfg_apply_btn)
        self.cfg_save_btn = QPushButton("Save to flash")
        self.cfg_save_btn.clicked.connect(self.save_settings)
        cfg_btns.addWidget(self.cfg_save_btn)
        cfg_btns.addStretch()
        settings_layout.addLayout(cfg_btns)
        cfg_grid = QGridLayout()
        half = (len(SETTINGS_KEYS) + 1) // 2
        for i, (key, _typ, label) in enumerate(SETTINGS_KEYS):
            col = 0 if i < half else 2
            row = i if i < half else i - half
            cfg_grid.addWidget(QLabel(label), row, col)
            edit = QLineEdit()
            edit.setMaximumWidth(90)
            self._cfg_fields[key] = edit
            cfg_grid.addWidget(edit, row, col + 1)
        settings_layout.addLayout(cfg_grid)
        self.settings_box.setLayout(settings_layout)
        self.settings_box.setEnabled(False)
        layout.addWidget(self.settings_box)

        self._refresh_ports()

    def _refresh_ports(self):
        ports = get_serial_ports()
        current = self.port_combo.currentText()
        self.port_combo.clear()
        disp_list = [disp for _, disp in ports]
        self.port_combo.addItems(disp_list)
        if disp_list and not current and ports:
            self.port_combo.setCurrentIndex(0)
        elif current in disp_list:
            self.port_combo.setCurrentText(current)

    def _set_error(self, msg: str):
        self.last_error_msg = msg
        self._status_text = f"Error: {msg[:60]}..."

    def connect(self):
        disp = self.port_combo.currentText().strip()
        device = disp.split(" — ")[0].strip() if " — " in disp else disp
        if not device:
            QMessageBox.critical(self, "No port", "Select a COM/serial port.")
            return
        try:
            baud = int(self.baud_edit.text().strip())
        except ValueError:
            QMessageBox.critical(self, "Invalid baud", "Baud must be an integer.")
            return
        try:
            self.ser = serial.Serial(device, baudrate=baud, timeout=1.0)
        except Exception as e:
            QMessageBox.critical(self, "Connection error", f"Could not open {device} @ {baud}:\n{e}")
            return
        self._status_text = f"Connected {device} @ {baud}"
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.start_btn.setEnabled(True)
        self.rearm_btn.setEnabled(True)
        self.disarm_btn.setEnabled(True)
        self._stop.clear()
        self.last_packet_time = time.time()
        self.connection_lost = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        # Auto-read FC settings over the two-way link (LoRa bridge panel only;
        # legacy direct-serial panels don't answer CFG?)
        if "LoRa" in self.title:
            QTimer.singleShot(800, self.read_settings)

    def disconnect(self):
        self._stop.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._status_text = "Disconnected"
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.rearm_btn.setEnabled(False)
        self.disarm_btn.setEnabled(False)
        self.last_packet_time = None
        self.connection_lost = True

    def _reader_loop(self):
        buf = b""
        while not self._stop.is_set():
            # Port may have been closed by disconnect() or device unplugged
            if self.ser is None or not self.ser.is_open:
                break
            try:
                chunk = self.ser.read(64)
            except OSError as e:
                # errno 9 = EBADF (bad file descriptor): port closed or device unplugged
                if self._stop.is_set():
                    break
                if getattr(e, "errno", None) == 9:
                    self._status_text = "Port closed or device disconnected"
                else:
                    self._set_error(f"Serial: {e}")
                break
            except serial.SerialException as e:
                if self._stop.is_set():
                    break
                # Often wraps OSError(9) when port is closed or device unplugged
                msg = str(e).lower()
                if "bad file descriptor" in msg or "errno 9" in msg:
                    self._status_text = "Port closed or device disconnected"
                else:
                    self._set_error(f"Serial: {e}")
                break
            except Exception as e:
                if self._stop.is_set():
                    break
                self._set_error(f"Serial read: {e}")
                break
            if not chunk:
                continue
            buf += chunk
            # Stream is a sequence of records: 28-byte "RC" binary telemetry packets
            # (or legacy 22-byte) and newline-terminated text lines (ACK/NACK/CFG/PONG
            # from the FC via the bridge, bridge banners on connect).
            while True:
                if buf[:2] == b"RC":
                    use_extended = len(buf) >= PACKET_SIZE
                    pkt_size = PACKET_SIZE if use_extended else PACKET_SIZE_OLD
                    if len(buf) < pkt_size:
                        break
                    packet = buf[:pkt_size]
                    buf = buf[pkt_size:]
                    self.last_raw_hex = packet.hex(" ")
                    if packet[0:2] != b"RC":
                        self._set_error("Bad header")
                        continue
                    try:
                        if use_extended:
                            unpacked = struct.unpack(PACKET_FMT, packet)
                            magic, time_ms, alt_cm, vel_cmps, lat_1e7, lon_1e7, state, roll_centi, pitch_centi, yaw_centi, checksum = unpacked
                        else:
                            unpacked = struct.unpack(PACKET_FMT_OLD, packet)
                            magic, time_ms, alt_cm, vel_cmps, lat_1e7, lon_1e7, state, checksum = unpacked
                            roll_centi = pitch_centi = yaw_centi = None
                    except struct.error as e:
                        self._set_error(str(e))
                        continue
                    if magic != b"RC":
                        continue
                    calc_cs = 0
                    for i in range(len(packet) - 1):
                        calc_cs ^= packet[i]
                    if calc_cs != checksum:
                        self._set_error("Checksum mismatch")
                        continue
                    self.last_packet_time = time.time()
                    self.connection_lost = False
                    self.last_error_msg = ""
                    self._status_text = "Receiving OK"
                    self.time_ms = time_ms
                    self.alt = alt_cm / 100.0
                    self.vel = vel_cmps / 100.0
                    self.lat = lat_1e7 / 1e7 if lat_1e7 != -2147483648 else None
                    self.lon = lon_1e7 / 1e7 if lon_1e7 != -2147483648 else None
                    self.state = state
                    if roll_centi is not None:
                        self.roll = roll_centi / 100.0
                        self.pitch = pitch_centi / 100.0
                        self.yaw = yaw_centi / 100.0
                    else:
                        self.roll = self.pitch = self.yaw = None
                    if self.log_file is not None:
                        try:
                            ts_utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
                            state_name = STATE_NAMES[state] if 0 <= state < len(STATE_NAMES) else ""
                            row = [
                                ts_utc, time_ms,
                                self.alt if self.alt is not None else "",
                                self.vel if self.vel is not None else "",
                                self.lat if self.lat is not None else "",
                                self.lon if self.lon is not None else "",
                                state, state_name,
                                self.roll if self.roll is not None else "",
                                self.pitch if self.pitch is not None else "",
                                self.yaw if self.yaw is not None else "",
                            ]
                            w = csv.writer(self.log_file, lineterminator="\n")
                            w.writerow(row)
                            self.log_file.flush()
                        except Exception:
                            pass
                else:
                    nl = buf.find(b"\n")
                    if nl == -1:
                        # No complete line yet; keep only a trailing window for resync
                        if len(buf) > PACKET_SIZE:
                            buf = buf[-PACKET_SIZE:]
                        break
                    line = buf[:nl]
                    buf = buf[nl + 1:]
                    if line:
                        self._handle_line(line)
        self._status_text = "Disconnected (reader stopped)"

    def _handle_line(self, raw: bytes):
        """Handle a text line from the FC/bridge: ACK/NACK, PONG, CFG responses."""
        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            return
        if not line:
            return
        if line.startswith("ACK") or line.startswith("NACK") or line == "PONG":
            with self._ack_lock:
                self._ack_lines.append(line)
                self._ack_event.set()
        elif line.startswith("CFG "):
            parts = line.split(" ", 2)
            if len(parts) == 3:
                with self._cfg_lock:
                    self._cfg_values[parts[1]] = parts[2]
                    self._cfg_dirty = True
        elif line == "CFG END":
            with self._cfg_lock:
                self._cfg_dirty = True
        # anything else (bridge banner, etc.) is ignored

    # ---------------- Commands (ACK-aware, two-way until FC is ARMED) ----------------
    def _write_line(self, text: str):
        self.ser.write(text.encode("ascii") + b"\n")

    def _wait_for_ack(self, expect_prefix="ACK", timeout=2.0):
        """Collect reply lines until one matches expect_prefix (or a NACK) arrives."""
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            self._ack_event.wait(min(remaining, 0.2))
            with self._ack_lock:
                lines = list(self._ack_lines)
                self._ack_lines.clear()
            self._ack_event.clear()
            for ln in lines:
                if ln.startswith(expect_prefix) or ln.startswith("NACK"):
                    return ln
        return None

    def send_command_blocking(self, cmd, expect_prefix="ACK", timeout=2.0, retries=3):
        """Send a command line, wait for its reply. Returns (ok, response_line|error)."""
        if self.ser is None or not self.ser.is_open:
            return False, "not connected"
        with self._ack_lock:
            self._ack_lines.clear()
        self._ack_event.clear()
        for _ in range(retries):
            try:
                self._write_line(cmd)
            except Exception as e:
                return False, str(e)
            resp = self._wait_for_ack(expect_prefix=expect_prefix, timeout=timeout)
            if resp is not None:
                return (resp.startswith("ACK") or resp == "PONG"), resp
        return False, "no reply (timeout)"

    def _run_cmd(self, cmd, label):
        if self.ser is None or not self.ser.is_open:
            QMessageBox.critical(self, "Not Connected", f"Cannot send {label}: {self.title} not connected.")
            return
        if self._cmd_busy:
            self._status_text = "Busy — previous command still running"
            return
        self._cmd_busy = True
        self._status_text = f"{label} sent — waiting for ACK..."

        def worker():
            ok, resp = self.send_command_blocking(cmd)
            self._status_text = f"{label} acknowledged" if ok else f"{label} failed: {resp}"
            self._cmd_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def send_start_command(self):
        self._run_cmd("START", "START")

    def send_rearm_command(self):
        self._run_cmd("ARM", "ARM")

    def send_disarm_command(self):
        self._run_cmd("DISARM", "DISARM")

    # ---------------- FC settings ----------------
    def read_settings(self):
        if self.ser is None or not self.ser.is_open or self._cmd_busy:
            return
        self._cmd_busy = True
        self._status_text = "Reading settings from FC..."

        def worker():
            with self._cfg_lock:
                self._cfg_values.clear()
            ok, resp = self.send_command_blocking("CFG?", expect_prefix="CFG END", timeout=2.0, retries=2)
            with self._cfg_lock:
                n = len(self._cfg_values)
                self._cfg_dirty = True
            self._status_text = f"Settings read ({n} keys)" if ok else f"Settings read failed: {resp}"
            self._cmd_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def apply_settings(self):
        if self.ser is None or not self.ser.is_open or self._cmd_busy:
            return
        changes = []
        for key, typ, _label in SETTINGS_KEYS:
            edit = self._cfg_fields.get(key)
            if edit is None:
                continue
            val = edit.text().strip()
            with self._cfg_lock:
                old = self._cfg_values.get(key)
            if old is None or val != old:
                changes.append((key, typ, val))
        if not changes:
            self._status_text = "No setting changes to apply"
            return
        # local sanity check before hitting the FC
        for key, typ, val in changes:
            try:
                if typ == "u32":
                    int(val)
                elif typ == "f32":
                    float(val)
                elif typ == "bool":
                    if val.lower() not in ("0", "1", "true", "false", "on", "off"):
                        raise ValueError
            except ValueError:
                QMessageBox.critical(self, "Invalid value", f"{key}: '{val}' is not a valid {typ} value.")
                return
        self._cmd_busy = True
        self._status_text = f"Applying {len(changes)} setting(s)..."

        def worker():
            fails = []
            for key, typ, val in changes:
                if typ == "bool":
                    val = "1" if val.lower() in ("1", "true", "on") else "0"
                ok, resp = self.send_command_blocking(f"SET {key} {val}")
                if not ok:
                    fails.append(f"{key}: {resp}")
            self._status_text = ("Apply failed — " + "; ".join(fails[:3])) if fails else f"Applied {len(changes)} setting(s)"
            self._cmd_busy = False
            self.read_settings()  # refresh fields from FC ground truth

        threading.Thread(target=worker, daemon=True).start()

    def save_settings(self):
        if self.ser is None or not self.ser.is_open or self._cmd_busy:
            return
        self._cmd_busy = True
        self._status_text = "Saving settings to FC flash..."

        def worker():
            ok, resp = self.send_command_blocking("SAVECFG")
            self._status_text = "Settings saved to flash" if ok else f"Save failed: {resp}"
            self._cmd_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def update_ui(self):
        status_display = self._status_text
        # FC (LoRa bridge) sends telemetry only after START; hint when connected but no data
        if "LoRa" in self.title and self.ser is not None and self.ser.is_open and self.last_packet_time is None:
            if "Connected" in self._status_text and "Error" not in self._status_text:
                status_display = "Connected — no data. Send START, then wait for calibration."
        self.status_label.setText(status_display)
        if "Error" in self._status_text or "failed" in self._status_text:
            self.status_label.setStyleSheet("color: #a00;")
        elif "Connected" in status_display or "Receiving" in self._status_text or "acknowledged" in self._status_text:
            self.status_label.setStyleSheet("color: #060;")
        else:
            self.status_label.setStyleSheet("color: #666;")

        # Settings: refresh fields when the reader thread reports new CFG data;
        # editable only in READY/INIT (FC rejects SET anywhere else anyway)
        with self._cfg_lock:
            dirty = self._cfg_dirty
            self._cfg_dirty = False
            values = dict(self._cfg_values)
        if dirty:
            for key, edit in self._cfg_fields.items():
                if key in values:
                    edit.setText(values[key])
        settings_ok = (self.ser is not None and self.ser.is_open
                       and self.state is not None and self.state in SETTINGS_STATES)
        self.settings_box.setEnabled(settings_ok)
        self.alt_value.setText(self._fmt_num(self.alt, "m"))
        self.vel_value.setText(self._fmt_num(self.vel, "m/s"))
        self.lat_value.setText(self._fmt_float(self.lat))
        self.lon_value.setText(self._fmt_float(self.lon))
        if self.state is not None and 0 <= self.state < len(STATE_NAMES):
            self.state_value.setText(STATE_NAMES[self.state])
        else:
            self.state_value.setText("--")
        if self.roll is not None:
            self.roll_value.setText(f"{self.roll:.1f}°")
            self.pitch_value.setText(f"{self.pitch:.1f}°")
            self.yaw_value.setText(f"{self.yaw:.1f}°")
            self.attitude_widget.set_attitude(self.roll, self.pitch)
        else:
            self.roll_value.setText("--°")
            self.pitch_value.setText("--°")
            self.yaw_value.setText("--°")
            self.attitude_widget.set_attitude(None, None)
        self.last_hex_label.setText(self.last_raw_hex)
        self.last_err_label.setText(self.last_error_msg)

        if self.ser is not None and self.ser.is_open and self.last_packet_time is not None:
            elapsed = time.time() - self.last_packet_time
            if elapsed > self.connection_timeout_sec:
                if not self.connection_lost:
                    self.connection_lost = True
                self.watchdog_label.setText(f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d} (LOST!)")
                self.watchdog_label.setStyleSheet("color: #f00; font-weight: bold;")
            else:
                if self.connection_lost:
                    self.connection_lost = False
                self.watchdog_label.setText(f"{elapsed:.2f} s")
                self.watchdog_label.setStyleSheet("color: #0a0; font-weight: bold;")
        else:
            self.watchdog_label.setText("--")
            self.watchdog_label.setStyleSheet("color: #666;")

    @staticmethod
    def _fmt_num(v, suffix=""):
        if v is None:
            return f"-- {suffix}".strip()
        return f"{v:.2f} {suffix}".strip()

    @staticmethod
    def _fmt_float(v):
        if v is None:
            return "--"
        return f"{v:.6f}"

    def set_log_file(self, f):
        self.log_file = f

    def stop_logging(self):
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None


def cleanup_empty_data_dirs():
    """Remove empty directories under data/ so they don't accumulate."""
    if not DATA_DIR.exists():
        return
    for p in list(DATA_DIR.iterdir()):
        if p.is_dir() and not any(p.iterdir()):
            try:
                p.rmdir()
            except Exception:
                pass


class GroundStationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Telemetry HUD — FC")
        self.setMinimumSize(1200, 720)
        self.resize(1500, 860)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)

        cleanup_empty_data_dirs()

        top_bar = QHBoxLayout()
        refresh_btn = QPushButton("Refresh COM ports")
        refresh_btn.clicked.connect(self._refresh_ports)
        top_bar.addWidget(refresh_btn)
        self._log_btn = QPushButton("Start logging")
        self._log_btn.clicked.connect(self._toggle_logging)
        self._logging_active = False
        top_bar.addWidget(self._log_btn)
        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        content = QHBoxLayout()
        self.panel = TelemetryPanel("FC (LoRa → Serial)", DEFAULT_BAUD)
        content.addWidget(self.panel, 1)

        right_col = QVBoxLayout()
        right_col.addWidget(QLabel("ALTITUDE / VELOCITY — LIVE"))
        self.plot = LivePlotWidget(window_s=120.0)
        right_col.addWidget(self.plot)
        map_row = QHBoxLayout()
        map_row.addWidget(QLabel("MAP — LIVE (±~1 km)"))
        map_row.addStretch()
        self._center_btn = QPushButton("Center on rocket")
        self._center_btn.clicked.connect(self._center_on_rocket)
        map_row.addWidget(self._center_btn)
        right_col.addLayout(map_row)
        self.fetcher = TileFetcher()
        self.map = MapWidget(self.fetcher)
        right_col.addWidget(self.map, 1)
        content.addLayout(right_col, 1)
        main_layout.addLayout(content)

        # Map centering state: geo-IP result (or first GPS fix) claims the center once
        self._geo_queue = queue.Queue()
        self._map_centered = False
        self._last_sample_t = None
        threading.Thread(target=self._locate_pc, daemon=True).start()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._schedule_ui_update)
        self._timer.start(100)

    def _locate_pc(self):
        """Background: find rough PC location at startup so the map is ready before GPS lock."""
        loc = fetch_pc_location()
        if loc is not None:
            self._geo_queue.put(loc)

    def _center_on_rocket(self):
        pos = self.map.last_position()
        if pos is not None:
            self.map.set_center(*pos)
            self._map_centered = True

    def _refresh_ports(self):
        self.panel._refresh_ports()

    def _toggle_logging(self):
        if self._logging_active:
            self.panel.stop_logging()
            self._logging_active = False
            self._log_btn.setText("Start logging")
            return
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        start_utc = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        d = DATA_DIR / start_utc
        d.mkdir(parents=True, exist_ok=True)
        f = None
        try:
            f = open(d / "data.csv", "w", newline="")
            csv.writer(f, lineterminator="\n").writerow(CSV_HEADER)
            f.flush()
            self.panel.set_log_file(f)
            self._logging_active = True
            self._log_btn.setText("Stop logging")
        except Exception as e:
            QMessageBox.critical(self, "Logging Error", f"Could not start logging: {e}")
            if f and not f.closed:
                try:
                    f.close()
                except Exception:
                    pass

    def _schedule_ui_update(self):
        self.panel.update_ui()
        # Feed plot + map with fresh telemetry samples
        if self.panel.time_ms is not None and self.panel.time_ms != self._last_sample_t:
            self._last_sample_t = self.panel.time_ms
            self.plot.add_sample(self.panel.time_ms / 1000.0, self.panel.alt, self.panel.vel)
            if self.panel.lat is not None and self.panel.lon is not None:
                if not self._map_centered:
                    self._map_centered = True
                    self.map.set_center(self.panel.lat, self.panel.lon)
                self.map.add_position(self.panel.lat, self.panel.lon)
        # Drain background results (map tiles, PC location) — never blocks the GUI
        self.map.ingest_tiles(self.fetcher.drain())
        try:
            loc = self._geo_queue.get_nowait()
        except queue.Empty:
            loc = None
        if loc is not None and not self._map_centered:
            self._map_centered = True
            self.map.set_center(*loc)

    def disconnect_all(self):
        if self._logging_active:
            self.panel.stop_logging()
            self._logging_active = False
            self._log_btn.setText("Start logging")
        self.panel.disconnect()

    def closeEvent(self, event):
        self.fetcher.stop()
        self.disconnect_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = GroundStationWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
