"""
Rocket telemetry ground station — dual FC1/FC2 panels.
Uses PySide6 (Qt) for the GUI. Install with:  pip install PySide6 pyserial
"""

import sys
import os
import csv
import threading
import serial
import serial.tools.list_ports
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

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
)
from PySide6.QtCore import QTimer, Qt, QRectF
from PySide6.QtGui import QFont, QPainter, QPen, QBrush, QColor
import math

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
        self.start_btn.setToolTip("Send START to begin FC init and calibration (READY → CALIBRATING)")
        btn_row.addWidget(self.start_btn)
        self.arm_btn = QPushButton("🔴 ARM")
        self.arm_btn.clicked.connect(self.send_arm_command)
        self.arm_btn.setEnabled(False)
        btn_row.addWidget(self.arm_btn)
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
        self.arm_btn.setEnabled(True)
        self._stop.clear()
        self.last_packet_time = time.time()
        self.connection_lost = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

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
        self.arm_btn.setEnabled(False)
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
            while len(buf) >= PACKET_SIZE_OLD:
                if len(buf) >= 2 and buf[0:2] != b"RC":
                    pos = buf.find(b"RC", 1)
                    if pos == -1:
                        buf = buf[-1:] if len(buf) > PACKET_SIZE else buf
                        break
                    buf = buf[pos:]
                    if len(buf) < PACKET_SIZE_OLD:
                        break
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
        self._status_text = "Disconnected (reader stopped)"

    def send_start_command(self):
        if self.ser is None or not self.ser.is_open:
            QMessageBox.critical(self, "Not Connected", f"Cannot send START: {self.title} not connected.")
            return
        try:
            self.ser.write(b"START\n")
            if DEBUG_PRINT:
                print(f"[{self.title}] START sent")
            self._status_text = "START sent"
        except Exception as e:
            QMessageBox.critical(self, "Send Error", str(e))

    def send_arm_command(self):
        if self.ser is None or not self.ser.is_open:
            QMessageBox.critical(self, "Not Connected", f"Cannot send ARM: {self.title} not connected.")
            return
        try:
            self.ser.write(b"ARM\n")
            if DEBUG_PRINT:
                print(f"[{self.title}] ARM sent")
            self._status_text = "ARM sent"
        except Exception as e:
            QMessageBox.critical(self, "Send Error", str(e))

    def update_ui(self):
        status_display = self._status_text
        # FC2 gets telemetry only after START is sent to FC2 over USB; hint when connected but no data
        if "FC2" in self.title and self.ser is not None and self.ser.is_open and self.last_packet_time is None:
            if "Connected" in self._status_text and "Error" not in self._status_text:
                status_display = "Connected — no data. Send START to FC2 over USB (115200), then wait for calibration."
        self.status_label.setText(status_display)
        if "Error" in self._status_text:
            self.status_label.setStyleSheet("color: #a00;")
        elif "Connected" in status_display or "Receiving" in self._status_text or "ARM sent" in self._status_text or "START sent" in self._status_text:
            self.status_label.setStyleSheet("color: #060;")
        else:
            self.status_label.setStyleSheet("color: #666;")
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


class DualTelemetryHUD(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Telemetry HUD — FC1 | FC2")
        self.setMinimumSize(1200, 640)
        self.resize(1400, 720)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)

        cleanup_empty_data_dirs()

        top_bar = QHBoxLayout()
        refresh_btn = QPushButton("Refresh COM ports")
        refresh_btn.clicked.connect(self._refresh_all_ports)
        top_bar.addWidget(refresh_btn)
        self._log_btn = QPushButton("Start logging")
        self._log_btn.clicked.connect(self._toggle_logging)
        self._logging_active = False
        top_bar.addWidget(self._log_btn)
        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        content = QHBoxLayout()
        self.panel_fc1 = TelemetryPanel("FC1 (Serial)", DEFAULT_BAUD)
        self.panel_fc2 = TelemetryPanel("FC2/FC3 (NRF24 or LoRa → Serial)", DEFAULT_BAUD)
        content.addWidget(self.panel_fc1)
        content.addWidget(self.panel_fc2)
        main_layout.addLayout(content)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._schedule_ui_update)
        self._timer.start(100)

    def _refresh_all_ports(self):
        self.panel_fc1._refresh_ports()
        self.panel_fc2._refresh_ports()

    def _toggle_logging(self):
        if self._logging_active:
            self.panel_fc1.stop_logging()
            self.panel_fc2.stop_logging()
            self._logging_active = False
            self._log_btn.setText("Start logging")
            return
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        start_utc = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        dirs = [DATA_DIR / f"{start_utc}--FC_1", DATA_DIR / f"{start_utc}--FC_2"]
        for d in dirs:
            d.mkdir(parents=True, exist_ok=True)
        files = [dirs[0] / "data.csv", dirs[1] / "data.csv"]
        f1 = f2 = None
        try:
            f1 = open(files[0], "w", newline="")
            f2 = open(files[1], "w", newline="")
            csv.writer(f1, lineterminator="\n").writerow(CSV_HEADER)
            csv.writer(f2, lineterminator="\n").writerow(CSV_HEADER)
            f1.flush()
            f2.flush()
            self.panel_fc1.set_log_file(f1)
            self.panel_fc2.set_log_file(f2)
            self._logging_active = True
            self._log_btn.setText("Stop logging")
        except Exception as e:
            QMessageBox.critical(self, "Logging Error", f"Could not start logging: {e}")
            for f in (f1, f2):
                if f and not f.closed:
                    try:
                        f.close()
                    except Exception:
                        pass

    def _schedule_ui_update(self):
        self.panel_fc1.update_ui()
        self.panel_fc2.update_ui()

    def disconnect_all(self):
        if self._logging_active:
            self.panel_fc1.stop_logging()
            self.panel_fc2.stop_logging()
            self._logging_active = False
            self._log_btn.setText("Start logging")
        self.panel_fc1.disconnect()
        self.panel_fc2.disconnect()

    def closeEvent(self, event):
        self.disconnect_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = DualTelemetryHUD()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
