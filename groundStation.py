import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct
import time

# ---------- CONFIG ----------
DEFAULT_BAUD = 57600
DEBUG_PRINT = True

# IMPORTANT: adjust these to match your C struct
# C struct (packed, little-endian):
#   uint8_t  preamble[2];   // 'R','C'
#   uint32_t time_ms;       // millis() since t0_ms or since boot
#   int32_t  alt_cm;        // alt_f in centimeters (m * 100)
#   int16_t  vel_cmps;      // v_est in cm/s (m/s * 100)
#   int32_t  lat_1e7;       // latitude degrees * 1e7
#   int32_t  lon_1e7;       // longitude degrees * 1e7
#   uint8_t  state;         // FlightState as uint8_t
#   uint8_t  checksum;      // XOR of all previous bytes
#
# Python format:
#   "<2sIihiiBB"
#   2s = 2-byte string (preamble)
#   I  = uint32_t (time_ms)
#   i  = int32_t (alt_cm)
#   h  = int16_t (vel_cmps)
#   i  = int32_t (lat_1e7)
#   i  = int32_t (lon_1e7)
#   B  = uint8_t (state)
#   B  = uint8_t (checksum)
#
PACKET_FMT = "<2sIihiiBB"
PACKET_SIZE = struct.calcsize(PACKET_FMT)
# ----------------------------


class TelemetryHUD:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Telemetry HUD (Binary)")
        self.root.geometry("900x680")

        # Latest decoded values
        self.time_ms = None
        self.alt = None
        self.vel = None
        self.lat = None
        self.lon = None
        self.state = None

        # Debug info
        self.last_raw_hex = ""
        self.last_error_msg = ""

        # Connection tracking
        self.last_packet_time = None
        self.connection_lost = False
        self.connection_timeout_sec = 3.0  # Consider connection lost after 3 seconds
        
        # Background overlay for connection loss indicator
        self.bg_overlay = None

        self.ser = None
        self._stop = threading.Event()
        self._thread = None

        self._build_ui()
        self._schedule_ui_update()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="COM Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=self._auto_detect_port() or "COM5")
        ttk.Entry(top, textvariable=self.port_var, width=12).pack(side=tk.LEFT, padx=(6, 16))

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=(6, 16))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=(10, 0))

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.pack(side=tk.RIGHT)

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.RIGHT, padx=(0, 10))
        
        # ARM button (big and prominent)
        self.arm_btn = ttk.Button(top, text="🔴 ARM", command=self.send_arm_command, state=tk.DISABLED)
        self.arm_btn.pack(side=tk.RIGHT, padx=(0, 10))

        # Flight State Display (big and bold at the top)
        state_frame = ttk.Frame(self.root, padding=10)
        state_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(state_frame, text="FLIGHT STATE", font=("Segoe UI", 16)).pack()
        self.state_text = tk.StringVar(value="--")
        state_label = tk.Label(state_frame, textvariable=self.state_text, 
                              font=("Segoe UI", 56, "bold"), 
                              foreground="#0066cc",
                              bg=self.root.cget("bg"))
        state_label.pack()

        # Big HUD center
        center = ttk.Frame(self.root, padding=12)
        center.pack(expand=True, fill=tk.BOTH)

        big_font = ("Segoe UI", 42, "bold")
        label_font = ("Segoe UI", 18)

        center.columnconfigure(0, weight=1)
        center.columnconfigure(1, weight=1)
        center.rowconfigure(0, weight=1)
        center.rowconfigure(1, weight=1)

        # Altitude
        alt_frame = ttk.Frame(center, padding=10)
        alt_frame.grid(row=0, column=0, sticky="nsew")
        ttk.Label(alt_frame, text="ALTITUDE", font=label_font).pack()
        self.alt_text = tk.StringVar(value="-- m")
        ttk.Label(alt_frame, textvariable=self.alt_text, font=big_font).pack()

        # Velocity
        vel_frame = ttk.Frame(center, padding=10)
        vel_frame.grid(row=0, column=1, sticky="nsew")
        ttk.Label(vel_frame, text="VELOCITY", font=label_font).pack()
        self.vel_text = tk.StringVar(value="-- m/s")
        ttk.Label(vel_frame, textvariable=self.vel_text, font=big_font).pack()

        # Latitude
        lat_frame = ttk.Frame(center, padding=10)
        lat_frame.grid(row=1, column=0, sticky="nsew")
        ttk.Label(lat_frame, text="LATITUDE", font=label_font).pack()
        self.lat_text = tk.StringVar(value="--")
        ttk.Label(lat_frame, textvariable=self.lat_text, font=big_font).pack()

        # Longitude
        lon_frame = ttk.Frame(center, padding=10)
        lon_frame.grid(row=1, column=1, sticky="nsew")
        ttk.Label(lon_frame, text="LONGITUDE", font=label_font).pack()
        self.lon_text = tk.StringVar(value="--")
        ttk.Label(lon_frame, textvariable=self.lon_text, font=big_font).pack()

        # Bottom debug area
        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)

        debug_bottom = ttk.Frame(bottom)
        debug_bottom.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(debug_bottom, text="Last raw packet (hex):").grid(row=0, column=0, sticky="w")
        self.last_hex_var = tk.StringVar(value="")
        ttk.Label(debug_bottom, textvariable=self.last_hex_var, foreground="#333").grid(
            row=0, column=1, sticky="w"
        )

        ttk.Label(debug_bottom, text="Last error:").grid(row=1, column=0, sticky="w")
        self.last_err_var = tk.StringVar(value="")
        ttk.Label(debug_bottom, textvariable=self.last_err_var, foreground="#a00").grid(
            row=1, column=1, sticky="w"
        )
        
        # Watchdog timer
        ttk.Label(debug_bottom, text="Time since last packet:").grid(row=2, column=0, sticky="w")
        self.watchdog_var = tk.StringVar(value="--")
        self.watchdog_label = ttk.Label(debug_bottom, textvariable=self.watchdog_var, foreground="#a00", font=("Segoe UI", 12, "bold"))
        self.watchdog_label.grid(row=2, column=1, sticky="w")

        debug_bottom.columnconfigure(1, weight=1)

    def _auto_detect_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = (p.description or "").lower()
            if "usb" in desc or "uart" in desc:
                return p.device
        return None

    def connect(self):
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud must be an integer.")
            return

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)
        except Exception as e:
            messagebox.showerror("Connection error", f"Could not open {port} @ {baud}:\n{e}")
            return

        self.status_var.set(f"Connected to {port} @ {baud}")
        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL)
        self.arm_btn.configure(state=tk.NORMAL)

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
        self.status_var.set("Disconnected")
        self.connect_btn.configure(state=tk.NORMAL)
        self.disconnect_btn.configure(state=tk.DISABLED)
        self.arm_btn.configure(state=tk.DISABLED)
        self.last_packet_time = None
        self.connection_lost = True

    def _reader_loop(self):
        if DEBUG_PRINT:
            print("=== Reader thread started ===")
            print(f"Expecting packet size: {PACKET_SIZE} bytes, format: {PACKET_FMT}")

        buf = b""

        while not self._stop.is_set():
            try:
                chunk = self.ser.read(64)  # grab up to 64 bytes at a time
            except Exception as e:
                self._set_error(f"Serial read error: {e}")
                break

            if not chunk:
                continue

            buf += chunk

            # Try to find packets in the buffer
            # First, search for preamble "RC" to synchronize
            while len(buf) >= PACKET_SIZE:
                # Look for preamble at the start
                if len(buf) >= 2 and buf[0:2] != b"RC":
                    # Preamble not found, search for it in the buffer
                    preamble_pos = buf.find(b"RC", 1)
                    if preamble_pos == -1:
                        # No preamble found, keep only the last byte (might be part of 'R')
                        if len(buf) > PACKET_SIZE:
                            buf = buf[-1:]
                        break
                    else:
                        # Found preamble, discard bytes before it
                        buf = buf[preamble_pos:]
                        if len(buf) < PACKET_SIZE:
                            break
                
                # Now we should have a packet starting with "RC"
                if len(buf) < PACKET_SIZE:
                    break
                    
                packet = buf[:PACKET_SIZE]
                buf = buf[PACKET_SIZE:]

                self.last_raw_hex = packet.hex(" ")
                if DEBUG_PRINT:
                    print(f"[RAW PACKET] len={len(packet)}  hex={self.last_raw_hex}")
                    print(f"[RAW PACKET] First 2 bytes: {packet[0:2]} (should be b'RC')")

                # Check preamble first before unpacking
                if len(packet) >= 2 and packet[0:2] != b"RC":
                    msg = f"Bad header (got {packet[0:2]!r}, expected b'RC')"
                    self._set_error(msg)
                    if DEBUG_PRINT:
                        print("[WARN]", msg)
                    continue

                # Decode with struct
                try:
                    unpacked = struct.unpack(PACKET_FMT, packet)
                    magic, time_ms, alt_cm, vel_cmps, lat_1e7, lon_1e7, state, checksum = unpacked
                    if DEBUG_PRINT:
                        print(f"[DEBUG] Unpacked values: magic={magic!r}, time_ms={time_ms}, alt_cm={alt_cm}, vel_cmps={vel_cmps}, state={state}")
                except struct.error as e:
                    msg = f"struct.unpack error: {e}"
                    self._set_error(msg)
                    if DEBUG_PRINT:
                        print("[WARN]", msg)
                        print(f"[DEBUG] Packet bytes: {[hex(b) for b in packet]}")
                    continue
                except Exception as e:
                    msg = f"Unexpected error during unpack: {e}"
                    self._set_error(msg)
                    if DEBUG_PRINT:
                        print("[WARN]", msg)
                        print(f"[DEBUG] Packet bytes: {[hex(b) for b in packet]}")
                    continue

                # Validate unpacked values
                if magic != b"RC":
                    msg = f"Bad header after unpack (got {magic!r}, expected b'RC')"
                    self._set_error(msg)
                    if DEBUG_PRINT:
                        print("[WARN]", msg)
                    continue

                # Validate checksum (XOR of all bytes except checksum)
                calc_checksum = 0
                for i in range(len(packet) - 1):
                    calc_checksum ^= packet[i]
                
                if calc_checksum != checksum:
                    msg = f"Checksum mismatch (got {checksum:02x}, expected {calc_checksum:02x})"
                    self._set_error(msg)
                    if DEBUG_PRINT:
                        print("[WARN]", msg)
                    continue

                # Validate that we got valid numeric values
                if time_ms is None or alt_cm is None or vel_cmps is None:
                    msg = f"Invalid unpacked values: time_ms={time_ms}, alt_cm={alt_cm}, vel_cmps={vel_cmps}"
                    self._set_error(msg)
                    if DEBUG_PRINT:
                        print("[WARN]", msg)
                    continue

                # Convert units to something human-friendly
                alt_m = alt_cm / 100.0
                vel_ms = vel_cmps / 100.0
                lat = lat_1e7 / 1e7 if lat_1e7 != -2147483648 else None  # INT32_MIN sentinel
                lon = lon_1e7 / 1e7 if lon_1e7 != -2147483648 else None  # INT32_MIN sentinel

                # Success - update last packet time
                self.last_packet_time = time.time()
                self.connection_lost = False
                self.last_error_msg = ""
                self.status_var.set("Receiving data OK")

                self.time_ms = time_ms
                self.alt = alt_m
                self.vel = vel_ms
                self.lat = lat
                self.lon = lon
                self.state = state

                if DEBUG_PRINT:
                    state_names = ["INIT", "ARMED", "BOOST", "APOGEE", "DEPLOY_DROGUE", "DEPLOY_MAIN", "FINISHED"]
                    state_name = state_names[state] if state < len(state_names) else f"UNKNOWN({state})"
                    # Safe formatting - handle None values
                    time_str = f"{time_ms}ms" if time_ms is not None else "N/A"
                    alt_str = f"{alt_m:.2f}" if alt_m is not None else "N/A"
                    vel_str = f"{vel_ms:.2f}" if vel_ms is not None else "N/A"
                    lat_str = f"{lat:.6f}" if lat is not None else "N/A"
                    lon_str = f"{lon:.6f}" if lon is not None else "N/A"
                    print(f"[DECODED] t={time_str}, alt={alt_str} m, vel={vel_str} m/s, "
                          f"lat={lat_str}, lon={lon_str}, state={state_name}")

        if DEBUG_PRINT:
            print("=== Reader thread exiting ===")
        self.status_var.set("Disconnected (reader stopped)")

    def _set_error(self, msg: str):
        self.last_error_msg = msg
        self.status_var.set(f"Error: {msg[:80]}...")

    def send_arm_command(self):
        """Send ARM command to the rocket via telemetry serial"""
        if self.ser is None or not self.ser.is_open:
            messagebox.showerror("Not Connected", "Cannot send ARM command: not connected to telemetry")
            return
        
        try:
            # Send ARM command with newline
            self.ser.write(b"ARM\n")
            if DEBUG_PRINT:
                print("[SENT] ARM command via telemetry")
            self.status_var.set("ARM command sent")
        except Exception as e:
            messagebox.showerror("Send Error", f"Failed to send ARM command:\n{e}")

    def _schedule_ui_update(self):
        # Called in main thread periodically
        self.alt_text.set(self._fmt_num(self.alt, "m"))
        self.vel_text.set(self._fmt_num(self.vel, "m/s"))
        self.lat_text.set(self._fmt_float(self.lat))
        self.lon_text.set(self._fmt_float(self.lon))
        
        # Update flight state display
        state_names = ["INIT", "ARMED", "BOOST", "APOGEE", "DEPLOY_DROGUE", "DEPLOY_MAIN", "FINISHED"]
        if self.state is not None and 0 <= self.state < len(state_names):
            self.state_text.set(state_names[self.state])
        else:
            self.state_text.set("--")

        self.last_hex_var.set(self.last_raw_hex)
        self.last_err_var.set(self.last_error_msg)
        
        # Check connection status and update watchdog timer
        if self.ser is not None and self.ser.is_open and self.last_packet_time is not None:
            elapsed = time.time() - self.last_packet_time
            if elapsed > self.connection_timeout_sec:
                # Connection lost
                if not self.connection_lost:
                    self.connection_lost = True
                    # Change root background to red
                    self.root.configure(bg="#ffcccc")
                    # Create a semi-transparent red overlay if it doesn't exist
                    if self.bg_overlay is None:
                        self.bg_overlay = tk.Frame(self.root, bg="#ffcccc", highlightthickness=0)
                        self.bg_overlay.place(x=0, y=0, relwidth=1, relheight=1)
                        self.bg_overlay.lower()  # Put it behind other widgets
                
                # Update watchdog timer
                minutes = int(elapsed // 60)
                seconds = int(elapsed % 60)
                self.watchdog_var.set(f"{minutes:02d}:{seconds:02d} (CONNECTION LOST!)")
                self.watchdog_label.configure(foreground="#ff0000")
            else:
                # Connection OK
                if self.connection_lost:
                    self.connection_lost = False
                    # Remove red background overlay
                    if self.bg_overlay is not None:
                        self.bg_overlay.destroy()
                        self.bg_overlay = None
                    # Restore normal background
                    self.root.configure(bg="SystemButtonFace")
                
                # Update watchdog timer (show time since last packet)
                self.watchdog_var.set(f"{elapsed:.2f} s")
                self.watchdog_label.configure(foreground="#00aa00")
        else:
            # Not connected
            self.watchdog_var.set("--")
            self.watchdog_label.configure(foreground="#666")
            if self.connection_lost:
                self.connection_lost = False
                if self.bg_overlay is not None:
                    self.bg_overlay.destroy()
                    self.bg_overlay = None
                self.root.configure(bg="SystemButtonFace")

        self.root.after(100, self._schedule_ui_update)

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

def main():
    root = tk.Tk()
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass

    app = TelemetryHUD(root)

    def on_close():
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
