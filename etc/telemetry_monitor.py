"""
K417 Passive Telemetry Monitor
===============================
Monitor Arduino control packets and drone responses with beautiful tkinter UI.

Features:
  - Display real-time stick values from Arduino's UDP packets
  - Read Arduino telemetry via serial (IMU angles, flex sensors, etc)
  - Show drone video stream (read-only)
  - Send drone commands to Arduino via serial
  - Distance estimator integration (YOLO)

Usage:
  python telemetry_monitor.py
"""

from __future__ import annotations

import os
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
import queue
import logging

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    import numpy as np
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from distance_estimator_v2 import AsyncDistanceEstimator
    DIST_EST_AVAILABLE = True
except ImportError:
    DIST_EST_AVAILABLE = False
    AsyncDistanceEstimator = None

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("telemetry")


# ============================================================================
# Protocol Constants
# ============================================================================

DRONE_IP = "192.168.169.1"
DRONE_PORT = 8800
LOCAL_LISTEN_PORT = 8891

STICK_MIN = 40
STICK_MID = 128
STICK_MAX = 220

CMD_NONE = 0x00
CMD_TAKEOFF = 0x01
CMD_LAND = 0x02
CMD_STOP = 0x02
CMD_CALIBRATE = 0x04
CMD_CAM_UP = 0x05
CMD_CAM_DOWN = 0x06

CMD_NAMES = {
    0x00: "NONE",
    0x01: "TAKEOFF",
    0x02: "LAND/STOP",
    0x04: "CALIBRATE",
    0x05: "CAM_UP",
    0x06: "CAM_DOWN",
}

K417_PACKET_SIZE = 124


# ============================================================================
# Control Packet Listener (UDP)
# ============================================================================

class ControlPacketListener:
    """Listen for UDP control packets from Arduino to drone."""

    def __init__(self, listen_port=LOCAL_LISTEN_PORT):
        self.listen_port = listen_port
        self.socket = None
        self.running = False
        self.thread = None
        self.latest_packet = None

    def start(self):
        if self.running:
            return
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(("0.0.0.0", self.listen_port))
            self.socket.settimeout(1.0)
            logger.info(f"UDP listener on port {self.listen_port}")
        except Exception as e:
            logger.error(f"UDP socket error: {e}")
            return

        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.socket:
            self.socket.close()

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(K417_PACKET_SIZE + 32)
                if len(data) >= K417_PACKET_SIZE:
                    self.latest_packet = self._decode_packet(data[:K417_PACKET_SIZE])
            except socket.timeout:
                pass
            except Exception:
                pass

    def _decode_packet(self, pkt: bytes):
        """Decode K417 124-byte control packet."""
        roll = pkt[20] if len(pkt) > 20 else STICK_MID
        pitch = pkt[21] if len(pkt) > 21 else STICK_MID
        throttle = pkt[22] if len(pkt) > 22 else STICK_MID
        yaw = pkt[23] if len(pkt) > 23 else STICK_MID
        cmd = pkt[24] if len(pkt) > 24 else CMD_NONE
        flags = pkt[25] if len(pkt) > 25 else 0

        headless = (flags & 0x03) == 0x03
        flip = (flags & 0x08) != 0

        return {
            "roll": roll,
            "pitch": pitch,
            "throttle": throttle,
            "yaw": yaw,
            "command": cmd,
            "headless": headless,
            "flip": flip,
        }

    def get_latest(self):
        """Get latest packet snapshot."""
        return self.latest_packet


# ============================================================================
# Serial Telemetry Reader (from Arduino)
# ============================================================================

class SerialTelemetryReader:
    """Read telemetry data from Arduino via serial."""

    def __init__(self, port=None, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.thread = None
        self.latest_data = {}
        self.data_lock = threading.Lock()
        self.status = "Disconnected"
        self.connected = False

    def auto_connect(self):
        """Try to find Arduino on common ports."""
        if not SERIAL_AVAILABLE:
            self.status = "pyserial not installed"
            return False

        ports = ["COM3", "COM4", "COM5", "COM6"]

        for port in ports:
            try:
                self.ser = serial.Serial(port, self.baud, timeout=0.5)
                time.sleep(1.0)
                self.status = f"Connected to {port}"
                self.connected = True
                self.running = True
                self.thread = threading.Thread(target=self._read_loop, daemon=True)
                self.thread.start()
                logger.info(f"Serial connected: {port}")
                return True
            except Exception:
                pass

        self.status = "No Arduino found"
        return False

    def send_command(self, cmd: str) -> bool:
        """Send command to Arduino."""
        if not self.ser or not self.ser.is_open:
            self.status = "Serial not connected"
            return False

        try:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
            logger.info(f"Serial TX: {cmd}")
            return True
        except Exception as e:
            logger.error(f"Serial send failed: {e}")
            self.status = f"Error: {str(e)[:30]}"
            return False

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.connected = False

    def _read_loop(self):
        """Background: read and parse telemetry from Arduino."""
        line_buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    ch = self.ser.read(1).decode("utf-8", errors="ignore")
                    if ch == "\n":
                        if line_buffer.strip():
                            self._parse_line(line_buffer.strip())
                        line_buffer = ""
                    else:
                        line_buffer += ch
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

    def _parse_line(self, line: str):
        """Parse telemetry line from Arduino (CSV format from STREAM command)."""
        try:
            parts = line.split(",")
            if len(parts) < 8:
                return

            with self.data_lock:
                self.latest_data = {
                    "timestamp": float(parts[0]),
                    "A3_raw": int(parts[1]),
                    "A2_raw": int(parts[2]),
                    "A1_raw": int(parts[3]),
                    "A0_raw": int(parts[4]),
                    "yaw_deg": float(parts[5]),
                    "pitch_deg": float(parts[6]),
                    "roll_deg": float(parts[7]),
                    "throttle_stick": float(parts[8]) if len(parts) > 8 else 0,
                    "yaw_stick": float(parts[9]) if len(parts) > 9 else 0,
                    "pitch_stick": float(parts[10]) if len(parts) > 10 else 0,
                    "roll_stick": float(parts[11]) if len(parts) > 11 else 0,
                }
        except (ValueError, IndexError):
            pass

    def get_data(self):
        """Get latest telemetry snapshot."""
        with self.data_lock:
            return dict(self.latest_data)


# ============================================================================
# K417VideoAdapter
# ============================================================================

class K417VideoAdapter:
    """Asynchronous video receiver for K417 drone over WiFi."""

    def __init__(self, drone_ip=DRONE_IP, drone_port=DRONE_PORT):
        self.drone_ip = drone_ip
        self.drone_port = drone_port
        self.udp_socket = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=2)
        self.frame_count = 0
        self.thread = None

    def start(self):
        if self.running:
            return
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**24)
            self.udp_socket.settimeout(2.0)
            self.udp_socket.bind(("0.0.0.0", self.drone_port))
            logger.info(f"Video listen port {self.drone_port}")
        except Exception as e:
            logger.error(f"Video socket: {e}")
            return

        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.udp_socket:
            self.udp_socket.close()

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(65536)
                if len(data) > 0:
                    try:
                        self.frame_queue.put_nowait(data)
                        self.frame_count += 1
                    except queue.Full:
                        pass
            except socket.timeout:
                pass
            except Exception:
                pass

    def get_frame(self):
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None


# ============================================================================
# Main GUI (Tkinter)
# ============================================================================

class TelemetryMonitorGUI:
    """Beautiful tkinter GUI inspired by control_video_v6.py."""

    COLORS = {
        "bg_main": "#1e1e2e",
        "bg_panel": "#2d2d44",
        "fg_text": "#c5c5d1",
        "fg_accent": "#50fa7b",
        "fg_warn": "#ff79c6",
    }

    def __init__(self, root):
        self.root = root
        self.root.title("K417 Telemetry Monitor")
        self.root.geometry("1200x750")
        
        # Set dark theme colors
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=self.COLORS["bg_main"])
        style.configure("TLabel", background=self.COLORS["bg_main"],
                       foreground=self.COLORS["fg_text"])
        style.configure("TLabelframe", background=self.COLORS["bg_main"],
                       foreground=self.COLORS["fg_text"])
        style.configure("TLabelframe.Label", background=self.COLORS["bg_main"],
                       foreground=self.COLORS["fg_accent"])

        self.root.configure(bg=self.COLORS["bg_main"])

        # Components
        self.control_listener = ControlPacketListener()
        self.serial_reader = SerialTelemetryReader()
        self.video_adapter = K417VideoAdapter()
        self.dist_est = None
        if DIST_EST_AVAILABLE:
            self.dist_est = AsyncDistanceEstimator()

        self.video_enabled = False
        self.video_thread = None
        self.current_photo = None

        # Build UI
        self._build_ui()

        # Start components
        self.control_listener.start()
        self.serial_reader.auto_connect()

        # Start update loop
        self._update_loop()

        # Shutdown handler
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        """Build tkinter UI."""
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Title
        title_frame = ttk.Frame(main)
        title_frame.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(title_frame, text="K417 Telemetry Monitor",
                 font=("Courier", 16, "bold")).pack(side=tk.LEFT)
        ttk.Label(title_frame, text="(Read-Only Mode - All commands via serial)",
                 font=("Courier", 10)).pack(side=tk.LEFT, padx=(20, 0))

        # Content: left=data, right=video
        content = ttk.Frame(main)
        content.pack(fill=tk.BOTH, expand=True)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=0, minsize=380)
        content.rowconfigure(0, weight=1)

        # LEFT: UDP + Serial data
        self._build_data_panel(content).grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        # RIGHT: Video
        self._build_video_panel(content).grid(row=0, column=1, sticky="nsew")

        # BOTTOM: Status & commands
        self._build_control_panel(main).pack(fill=tk.X, pady=(8, 0))

    def _build_data_panel(self, parent):
        """Left panel: UDP and serial data."""
        frame = ttk.Frame(parent)

        # UDP Packets
        udp_frame = ttk.LabelFrame(frame, text="Arduino UDP Packets (Control)")
        udp_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        self.udp_text = tk.Text(udp_frame, height=15, width=60,
                               bg=self.COLORS["bg_panel"],
                               fg=self.COLORS["fg_accent"],
                               font=("Courier", 9),
                               relief=tk.FLAT)
        self.udp_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.udp_text.config(state=tk.DISABLED)

        # Serial Telemetry
        ser_frame = ttk.LabelFrame(frame, text="Arduino Telemetry (Serial 115200)")
        ser_frame.pack(fill=tk.BOTH, expand=True)

        self.ser_text = tk.Text(ser_frame, height=15, width=60,
                               bg=self.COLORS["bg_panel"],
                               fg=self.COLORS["fg_text"],
                               font=("Courier", 9),
                               relief=tk.FLAT)
        self.ser_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.ser_text.config(state=tk.DISABLED)

        return frame

    def _build_video_panel(self, parent):
        """Right panel: video stream."""
        frame = ttk.LabelFrame(parent, text="Video Stream")

        self.video_canvas = tk.Canvas(frame, bg="black", width=380, height=600,
                                     relief=tk.FLAT)
        self.video_canvas.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Placeholder text
        self.video_canvas.create_text(190, 300,
                                     text="Press 'V' to start video",
                                     fill="gray", font=("Courier", 12))

        return frame

    def _build_control_panel(self, parent):
        """Bottom panel: status and quick commands."""
        frame = ttk.Frame(parent)

        # Status bar (top)
        status_f = ttk.Frame(frame)
        status_f.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(status_f, text="Status:").pack(side=tk.LEFT, padx=(0, 5))
        self.status_label = ttk.Label(status_f, text="Initializing…",
                                     foreground="gray")
        self.status_label.pack(side=tk.LEFT)

        # Quick commands
        cmd_f = ttk.LabelFrame(frame, text="Drone Commands")
        cmd_f.pack(fill=tk.X)

        btn_frame = ttk.Frame(cmd_f)
        btn_frame.pack(fill=tk.X, padx=4, pady=4)

        cmds = [
            ("Takeoff", "T"),
            ("Land", "L"),
            ("Stop", "SPACE"),
            ("Calibrate", "F5"),
            ("Zero", "O"),
            ("Video", "V"),
            ("Help", "H"),
        ]

        for label, cmd in cmds:
            btn = ttk.Button(btn_frame, text=label,
                            command=lambda c=cmd: self._send_cmd(c),
                            width=10)
            btn.pack(side=tk.LEFT, padx=2)

        return frame

    def _send_cmd(self, cmd):
        """Send command to Arduino."""
        if cmd == "V":
            self.video_enabled = not self.video_enabled
            if self.video_enabled:
                self.video_adapter.start()
                self.video_thread = threading.Thread(target=self._video_loop,
                                                    daemon=True)
                self.video_thread.start()
                self.status_label.configure(
                    text="Video: ON | Press Q in video window to stop")
            else:
                self.video_adapter.stop()
                self.video_canvas.delete("all")
                self.video_canvas.create_text(190, 300,
                                             text="Press 'V' to start video",
                                             fill="gray", font=("Courier", 12))
        elif cmd == "H":
            self._show_help()
        else:
            self.serial_reader.send_command(cmd)

    def _show_help(self):
        """Show help dialog."""
        help_text = """K417 Telemetry Monitor - Keyboard Shortcuts

SERIAL COMMANDS (auto-sent to Arduino):
  T     Takeoff
  L     Land
  SPACE Stop
  F5    Calibrate
  O     Zero (capture orientation)
  C     Camera Down
  X     Headless toggle
  1-4   Flip directions

DISPLAY:
  V     Toggle video stream
  D     Toggle distance estimator
  Q     Quit monitor"""

        import tkinter.messagebox
        tkinter.messagebox.showinfo("Help", help_text)

    def _video_loop(self):
        """Background: update video canvas."""
        if not CV2_AVAILABLE or not PIL_AVAILABLE:
            logger.error("OpenCV or PIL not available")
            return

        placeholder = np.zeros((400, 380, 3), dtype=np.uint8)
        placeholder[:] = (30, 30, 40)
        last_frame = placeholder

        while self.video_enabled:
            frame_data = self.video_adapter.get_frame()
            if frame_data:
                try:
                    frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8),
                                        cv2.IMREAD_COLOR)
                    if frame is not None:
                        last_frame = frame
                        if self.dist_est and hasattr(self.dist_est, 'submit'):
                            self.dist_est.submit(frame)
                except Exception:
                    pass

            # Resize and display
            h, w = last_frame.shape[:2]
            aspect = w / h
            new_w = 380
            new_h = int(new_w / aspect)
            if new_h > 600:
                new_h = 600
                new_w = int(new_h * aspect)

            display = cv2.resize(last_frame, (new_w, new_h))
            display = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(display)
            photo = ImageTk.PhotoImage(im)

            try:
                self.video_canvas.delete("all")
                self.video_canvas.create_image(190, 300, image=photo, anchor=tk.CENTER)
                self.current_photo = photo
                self.root.update_idletasks()
            except Exception:
                break

            time.sleep(0.033)

    def _update_loop(self):
        """Main update loop: refresh displays."""
        # UDP packets
        pkt = self.control_listener.get_latest()
        if pkt:
            bar_roll = "█" * int((pkt['roll'] - STICK_MIN) / (STICK_MAX - STICK_MIN) * 10)
            bar_pitch = "█" * int((pkt['pitch'] - STICK_MIN) / (STICK_MAX - STICK_MIN) * 10)
            bar_thr = "█" * int((pkt['throttle'] - STICK_MIN) / (STICK_MAX - STICK_MIN) * 10)
            bar_yaw = "█" * int((pkt['yaw'] - STICK_MIN) / (STICK_MAX - STICK_MIN) * 10)

            text = f"""STICK VALUES:
  Roll:     {pkt['roll']:3d}  {bar_roll}
  Pitch:    {pkt['pitch']:3d}  {bar_pitch}
  Throttle: {pkt['throttle']:3d}  {bar_thr}
  Yaw:      {pkt['yaw']:3d}  {bar_yaw}

COMMAND: {CMD_NAMES.get(pkt['command'], f"UNK({pkt['command']:02X})")}

FLAGS:
  Headless: {'✓ ON' if pkt['headless'] else '✗ OFF'}
  Flip:     {'✓ YES' if pkt['flip'] else '✗ NO'}"""

            self.udp_text.config(state=tk.NORMAL)
            self.udp_text.delete("1.0", tk.END)
            self.udp_text.insert("1.0", text)
            self.udp_text.config(state=tk.DISABLED)
        else:
            self.udp_text.config(state=tk.NORMAL)
            self.udp_text.delete("1.0", tk.END)
            self.udp_text.insert("1.0", "Waiting for Arduino UDP packets...")
            self.udp_text.config(state=tk.DISABLED)

        # Serial telemetry
        data = self.serial_reader.get_data()
        if data:
            text = f"""IMU ANGLES:
  Yaw:   {data.get('yaw_deg', 0):7.1f}°
  Pitch: {data.get('pitch_deg', 0):7.1f}°
  Roll:  {data.get('roll_deg', 0):7.1f}°

FLEX SENSORS (raw ADC):
  A3 (down):  {data.get('A3_raw', 0):4d}
  A2 (up):    {data.get('A2_raw', 0):4d}
  A1:         {data.get('A1_raw', 0):4d}
  A0:         {data.get('A0_raw', 0):4d}

STICK OUTPUT:
  Throttle: {data.get('throttle_stick', 0):7.1f}
  Yaw:      {data.get('yaw_stick', 0):7.1f}
  Pitch:    {data.get('pitch_stick', 0):7.1f}
  Roll:     {data.get('roll_stick', 0):7.1f}

Time: {data.get('timestamp', 0):.3f}s"""

            self.ser_text.config(state=tk.NORMAL)
            self.ser_text.delete("1.0", tk.END)
            self.ser_text.insert("1.0", text)
            self.ser_text.config(state=tk.DISABLED)
        else:
            msg = f"Serial: {self.serial_reader.status}\n\nEnable telemetry:\n  Send 'STREAM ON' command"
            self.ser_text.config(state=tk.NORMAL)
            self.ser_text.delete("1.0", tk.END)
            self.ser_text.insert("1.0", msg)
            self.ser_text.config(state=tk.DISABLED)

        # Status bar
        udp_status = "UDP: ✓" if pkt else "UDP: ✗"
        ser_status = "Serial: ✓" if self.serial_reader.connected else "Serial: ✗"
        vid_status = "Video: ✓" if self.video_enabled else "Video: ✗"
        self.status_label.configure(
            text=f"{udp_status} | {ser_status} | {vid_status}")

        self.root.after(500, self._update_loop)

    def _on_close(self):
        """Shutdown."""
        self.video_enabled = False
        self.control_listener.stop()
        self.serial_reader.stop()
        self.video_adapter.stop()
        if self.dist_est:
            self.dist_est.stop()
        self.root.destroy()


# ============================================================================
# Main
# ============================================================================

def main():
    if not CV2_AVAILABLE or not PIL_AVAILABLE:
        print("ERROR: Missing required packages")
        print("Install with: pip install opencv-python Pillow pyserial")
        return

    root = tk.Tk()
    app = TelemetryMonitorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
