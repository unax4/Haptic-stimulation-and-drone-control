#!/usr/bin/env python3
"""
k417_controller.py  –  Karuisrc K417 WiFi Drone Controller
============================================================
Single-file controller for the Karuisrc K417 drone (and compatible
WiFi-UAV family: E58, LH-X20, …).

Protocol details are reverse-engineered from Android app packet captures
and are fully embedded here — no external dependencies required beyond
the Python standard library.

Usage
-----
    python k417_controller.py

The GUI lets you:
  • Configure drone IP / port / packet rate on-the-fly
  • Takeoff / Land / Emergency stop
  • Control throttle, yaw, pitch, roll via sliders OR keyboard
  • Toggle headless mode
  • Calibrate IMU
  • Watch live packet hex output
  • Enable/disable debug logging

Keyboard bindings (when the window is focused)
------------------------------------------------
  W / S   – Throttle up / down
  A / D   – Yaw left / right
  ↑ / ↓   – Pitch forward / backward
  ← / →   – Roll left / right
  T       – Takeoff
  L       – Land
  Space   – Emergency stop
  H       – Toggle headless mode
  C       – Calibrate

Requirements
------------
  Python 3.8+   (tkinter is included in the standard library)
  No pip packages needed.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import logging

# ──────────────────────────────────────────────────────────────────────────────
# Logging
# ──────────────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.DEBUG,
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("k417")


# ──────────────────────────────────────────────────────────────────────────────
# Protocol constants  (from wifi_uav_packets.py + wifi_uav_rc_protocol_adapter)
# ──────────────────────────────────────────────────────────────────────────────

# Drone defaults — override in the GUI
DEFAULT_IP   = "192.168.169.1"
DEFAULT_PORT = 8800

# Stick raw value range (reverse-engineered from packet dumps)
STICK_MIN = 40
STICK_MID = 128
STICK_MAX = 220

# Fixed header shared by every control packet
_HDR = bytes([0xEF, 0x02, 0x7C, 0x00, 0x02, 0x02,
              0x00, 0x01, 0x02, 0x00, 0x00, 0x00])

# Static suffixes
_C1_SUFFIX  = bytes([0x00, 0x00, 0x14, 0x00, 0x66, 0x14])
_CTRL_PAD   = bytes(10)                               # 10 × 0x00 after controls
_CKSUM_SFX  = bytes([0x99]) + bytes(44) + bytes([0x32, 0x4B, 0x14, 0x2D, 0x00, 0x00])
_C2_SUFFIX  = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                     0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
                     0xFF, 0xFF, 0xFF, 0xFF])
_C3_SUFFIX  = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x03, 0x00, 0x00, 0x00, 0x10, 0x00,
                     0x00, 0x00])

# Command byte values
CMD_NONE        = 0x00
CMD_TAKEOFF     = 0x01
CMD_LAND        = 0x02
CMD_STOP        = 0x02   # same byte as land
CMD_CALIBRATE   = 0x04

HEADLESS_OFF = 0x02
HEADLESS_ON  = 0x03


def build_packet(roll: int, pitch: int, throttle: int, yaw: int,
                 command: int, headless: int,
                 c1: int, c2: int, c3: int) -> bytes:
    """
    Assemble one WiFi-UAV control packet (identical layout to
    WifiUavRcProtocolAdapter.build_control_packet).
    """
    b_c1 = c1.to_bytes(2, "little")
    b_c2 = c2.to_bytes(2, "little")
    b_c3 = c3.to_bytes(2, "little")

    controls = [
        roll     & 0xFF,
        pitch    & 0xFF,
        throttle & 0xFF,
        yaw      & 0xFF,
        command  & 0xFF,
        headless & 0xFF,
    ]

    checksum = 0
    for b in controls:
        checksum ^= b

    pkt = bytearray()
    pkt += _HDR
    pkt += b_c1 + _C1_SUFFIX
    pkt += bytes(controls)
    pkt += _CTRL_PAD
    pkt.append(checksum)
    pkt += _CKSUM_SFX
    pkt += b_c2 + _C2_SUFFIX
    pkt += b_c3 + _C3_SUFFIX
    return bytes(pkt)


# ──────────────────────────────────────────────────────────────────────────────
# DroneState  –  all mutable flight state in one place
# ──────────────────────────────────────────────────────────────────────────────
class DroneState:
    def __init__(self):
        self._lock = threading.Lock()

        # raw stick values [STICK_MIN … STICK_MAX], float for smooth ramp
        self.throttle: float = STICK_MID
        self.yaw:      float = STICK_MID
        self.pitch:    float = STICK_MID
        self.roll:     float = STICK_MID

        # directional inputs  (-1, 0, +1)
        self.d_throttle = 0
        self.d_yaw      = 0
        self.d_pitch    = 0
        self.d_roll     = 0

        # one-shot flags
        self.takeoff_flag     = False
        self.land_flag        = False
        self.stop_flag        = False
        self.calibrate_flag   = False

        # persistent flags
        self.headless = False

        # rolling 16-bit packet counters
        self._c1 = 0x0000
        self._c2 = 0x0001
        self._c3 = 0x0002

        # profile
        self.accel_rate = 2.0   # ratio of half-range per second
        self.decel_rate = 4.0
        self.expo       = 0.5

    # ── axis update (incremental w/ accel/decel) ─────────────────────────
    def update(self, dt: float):
        with self._lock:
            half = STICK_MAX - STICK_MID
            full = STICK_MAX - STICK_MIN
            accel = self.accel_rate * half * dt
            decel = self.decel_rate * half * dt

            for attr, dir_attr in [
                ("throttle", "d_throttle"),
                ("yaw",      "d_yaw"),
                ("pitch",    "d_pitch"),
                ("roll",     "d_roll"),
            ]:
                cur = getattr(self, attr)
                direction = getattr(self, dir_attr)

                if direction > 0:
                    dist = STICK_MAX - cur
                    inc  = accel * (1 + self.expo * dist / half)
                    cur  = min(STICK_MAX, cur + inc)
                elif direction < 0:
                    dist = cur - STICK_MIN
                    inc  = accel * (1 + self.expo * dist / half)
                    cur  = max(STICK_MIN, cur - inc)
                else:
                    if cur > STICK_MID:
                        cur = max(STICK_MID, cur - decel)
                    elif cur < STICK_MID:
                        cur = min(STICK_MID, cur + decel)

                setattr(self, attr, cur)

    def next_counters(self):
        with self._lock:
            c1, c2, c3 = self._c1, self._c2, self._c3
            self._c1 = (self._c1 + 1) & 0xFFFF
            self._c2 = (self._c2 + 1) & 0xFFFF
            self._c3 = (self._c3 + 1) & 0xFFFF
        return c1, c2, c3

    def consume_flags(self):
        """Return current command byte and clear one-shot flags."""
        with self._lock:
            if self.takeoff_flag:
                cmd = CMD_TAKEOFF
                self.takeoff_flag = False
            elif self.stop_flag:
                cmd = CMD_STOP
                self.stop_flag = False
            elif self.land_flag:
                cmd = CMD_LAND
                self.land_flag = False
            elif self.calibrate_flag:
                cmd = CMD_CALIBRATE
                self.calibrate_flag = False
            else:
                cmd = CMD_NONE

            hless = HEADLESS_ON if self.headless else HEADLESS_OFF
        return cmd, hless

    def snapshot(self):
        with self._lock:
            return {
                "throttle": self.throttle,
                "yaw":      self.yaw,
                "pitch":    self.pitch,
                "roll":     self.roll,
            }

    def set_direct(self, throttle, yaw, pitch, roll):
        """Set stick values directly from slider (STICK_MIN … STICK_MAX)."""
        with self._lock:
            self.throttle = max(STICK_MIN, min(STICK_MAX, throttle))
            self.yaw      = max(STICK_MIN, min(STICK_MAX, yaw))
            self.pitch    = max(STICK_MIN, min(STICK_MAX, pitch))
            self.roll     = max(STICK_MIN, min(STICK_MAX, roll))


# ──────────────────────────────────────────────────────────────────────────────
# FlightController  –  background thread sending UDP packets
# ──────────────────────────────────────────────────────────────────────────────
class FlightController:
    def __init__(self, state: DroneState, log_q: queue.Queue):
        self.state  = state
        self.log_q  = log_q

        self.drone_ip   = DEFAULT_IP
        self.drone_port = DEFAULT_PORT
        self.rate       = 80.0           # Hz

        self._running = False
        self._thread  = None
        self._sock    = None
        self.debug    = False

    # ── network ──────────────────────────────────────────────────────────
    def _open_socket(self):
        if self._sock:
            try: self._sock.close()
            except Exception: pass
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.5)

    def _send(self, pkt: bytes):
        try:
            self._sock.sendto(pkt, (self.drone_ip, self.drone_port))
        except OSError as e:
            logger.warning("Send error: %s", e)

    # ── loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        self._open_socket()
        interval = 1.0 / self.rate
        prev = time.time()
        pkt_num = 0

        while self._running:
            now = time.time()
            dt  = now - prev
            prev = now

            self.state.update(dt)
            cmd, headless = self.state.consume_flags()
            c1, c2, c3   = self.state.next_counters()
            snap         = self.state.snapshot()

            pkt = build_packet(
                roll     = int(snap["roll"]),
                pitch    = int(snap["pitch"]),
                throttle = int(snap["throttle"]),
                yaw      = int(snap["yaw"]),
                command  = cmd,
                headless = headless,
                c1=c1, c2=c2, c3=c3,
            )
            self._send(pkt)
            pkt_num += 1

            if self.debug:
                hex_str = " ".join(f"{b:02x}" for b in pkt[:24])
                msg = (f"#{pkt_num:06d}  "
                       f"T:{int(snap['throttle']):3d} "
                       f"Y:{int(snap['yaw']):3d} "
                       f"P:{int(snap['pitch']):3d} "
                       f"R:{int(snap['roll']):3d}  "
                       f"cmd={cmd:#04x}  [{hex_str}…]")
                try:
                    self.log_q.put_nowait(msg)
                except queue.Full:
                    pass

            elapsed = time.time() - now
            sleep_t = max(0.0, interval - elapsed)
            time.sleep(sleep_t)

        if self._sock:
            self._sock.close()
            self._sock = None

    # ── public API ────────────────────────────────────────────────────────
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="FlightCtrl")
        self._thread.start()
        logger.info("FlightController started  %s:%d  @ %.0f Hz",
                    self.drone_ip, self.drone_port, self.rate)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("FlightController stopped")

    def reconnect(self, ip: str, port: int, rate: float):
        """Apply new connection settings — restarts the background thread."""
        was_running = self._running
        if was_running:
            self.stop()
        self.drone_ip   = ip
        self.drone_port = port
        self.rate       = rate
        if was_running:
            self.start()


# ──────────────────────────────────────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────────────────────────────────────
DARK_BG     = "#0d0f14"
PANEL_BG    = "#13161e"
ACCENT      = "#00e5ff"
ACCENT2     = "#ff4081"
TEXT_MAIN   = "#e8eaf6"
TEXT_DIM    = "#546e7a"
BTN_TAKE    = "#00c853"
BTN_LAND    = "#ff6d00"
BTN_STOP    = "#d50000"
BTN_HEAD    = "#7c4dff"
BTN_CAL     = "#0091ea"

FONT_MONO   = ("Courier New", 10)
FONT_LABEL  = ("Courier New", 9, "bold")
FONT_BTN    = ("Courier New", 10, "bold")
FONT_BIG    = ("Courier New", 14, "bold")
FONT_TITLE  = ("Courier New", 18, "bold")


class K417GUI:
    def __init__(self, root: tk.Tk):
        self.root  = root
        self.state = DroneState()
        self.log_q: queue.Queue = queue.Queue(maxsize=200)
        self.ctrl  = FlightController(self.state, self.log_q)

        # keyboard direction map
        self._keys: set[str] = set()

        self._build_ui()
        self._bind_keys()

        # start control loop
        self.ctrl.start()

        # periodic UI refresh
        self._tick()

    # ─────────────────────────────────────────── UI construction ─────────
    def _build_ui(self):
        root = self.root
        root.title("Karuisrc K417  //  WiFi Drone Controller")
        root.configure(bg=DARK_BG)
        root.resizable(False, False)
        root.geometry("880x760")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame",      background=DARK_BG)
        style.configure("Panel.TFrame", background=PANEL_BG)
        style.configure("TLabel",
                         background=DARK_BG, foreground=TEXT_MAIN,
                         font=FONT_LABEL)
        style.configure("Dim.TLabel",
                         background=DARK_BG, foreground=TEXT_DIM,
                         font=FONT_LABEL)
        style.configure("Panel.TLabel",
                         background=PANEL_BG, foreground=TEXT_MAIN,
                         font=FONT_LABEL)
        style.configure("TEntry",
                         fieldbackground="#1c2130", foreground=TEXT_MAIN,
                         insertcolor=ACCENT, font=FONT_MONO)
        style.configure("Horizontal.TScale",
                         background=PANEL_BG, troughcolor="#1c2130",
                         sliderrelief="flat")

        # ── TITLE ──────────────────────────────────────────────────────
        title_frame = tk.Frame(root, bg=DARK_BG)
        title_frame.pack(fill="x", padx=20, pady=(16, 4))

        tk.Label(title_frame, text="K417", fg=ACCENT, bg=DARK_BG,
                 font=("Courier New", 28, "bold")).pack(side="left")
        tk.Label(title_frame, text="  DRONE CONTROLLER",
                 fg=TEXT_DIM, bg=DARK_BG,
                 font=("Courier New", 13, "bold")).pack(side="left", pady=6)
        self._status_label = tk.Label(title_frame, text="● STOPPED",
                                      fg=ACCENT2, bg=DARK_BG,
                                      font=FONT_LABEL)
        self._status_label.pack(side="right", padx=8)

        sep = tk.Frame(root, height=1, bg=ACCENT)
        sep.pack(fill="x", padx=20, pady=(0, 12))

        # ── MAIN COLUMNS ──────────────────────────────────────────────
        cols = tk.Frame(root, bg=DARK_BG)
        cols.pack(fill="both", expand=True, padx=20)

        left  = tk.Frame(cols, bg=DARK_BG)
        left.pack(side="left", fill="both", expand=True, padx=(0, 10))

        right = tk.Frame(cols, bg=DARK_BG)
        right.pack(side="right", fill="both", expand=True)

        # ── LEFT: Connection settings ──────────────────────────────────
        self._build_connection_panel(left)

        # ── LEFT: Control profile ──────────────────────────────────────
        self._build_profile_panel(left)

        # ── LEFT: Command buttons ──────────────────────────────────────
        self._build_commands_panel(left)

        # ── LEFT: Keyboard legend ──────────────────────────────────────
        self._build_keyboard_legend(left)

        # ── RIGHT: Stick sliders ───────────────────────────────────────
        self._build_sticks_panel(right)

        # ── BOTTOM: Debug log ──────────────────────────────────────────
        self._build_log_panel(root)

    def _panel(self, parent, title: str):
        """Utility: labelled dark panel frame."""
        outer = tk.Frame(parent, bg=DARK_BG)
        outer.pack(fill="x", pady=6)
        tk.Label(outer, text=f"  {title}  ", fg=ACCENT, bg=DARK_BG,
                 font=("Courier New", 9, "bold")).pack(anchor="w")
        frame = tk.Frame(outer, bg=PANEL_BG, padx=12, pady=10)
        frame.pack(fill="x")
        return frame

    # ── Connection panel ──────────────────────────────────────────────
    def _build_connection_panel(self, parent):
        f = self._panel(parent, "CONNECTION")

        def row(label, default):
            r = tk.Frame(f, bg=PANEL_BG)
            r.pack(fill="x", pady=3)
            tk.Label(r, text=label, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, width=12, anchor="w").pack(side="left")
            var = tk.StringVar(value=default)
            e = tk.Entry(r, textvariable=var, width=20,
                         bg="#1c2130", fg=TEXT_MAIN,
                         insertbackground=ACCENT, font=FONT_MONO,
                         relief="flat", bd=2)
            e.pack(side="left", padx=4)
            return var

        self._ip_var   = row("Drone IP",    DEFAULT_IP)
        self._port_var = row("Port (UDP)",  str(DEFAULT_PORT))
        self._rate_var = row("Rate (Hz)",   "80")

        btn_row = tk.Frame(f, bg=PANEL_BG)
        btn_row.pack(fill="x", pady=(8, 0))
        tk.Button(btn_row, text="CONNECT / APPLY",
                  bg="#0d47a1", fg=TEXT_MAIN,
                  activebackground="#1565c0", activeforeground="white",
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._apply_connection
                  ).pack(side="left", padx=2)
        tk.Button(btn_row, text="DISCONNECT",
                  bg="#37474f", fg=TEXT_MAIN,
                  activebackground="#546e7a", activeforeground="white",
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._disconnect
                  ).pack(side="left", padx=2)

    # ── Profile panel ─────────────────────────────────────────────────
    def _build_profile_panel(self, parent):
        f = self._panel(parent, "CONTROL PROFILE")

        def param_row(label, from_, to, initial, attr):
            r = tk.Frame(f, bg=PANEL_BG)
            r.pack(fill="x", pady=3)
            tk.Label(r, text=label, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, width=16, anchor="w").pack(side="left")
            var = tk.DoubleVar(value=initial)
            lbl = tk.Label(r, textvariable=var, fg=ACCENT, bg=PANEL_BG,
                           font=FONT_MONO, width=5)
            lbl.pack(side="right")

            def on_change(*_):
                setattr(self.state, attr, round(var.get(), 2))
                lbl.config(text=f"{var.get():.2f}")

            s = tk.Scale(r, variable=var, from_=from_, to=to,
                         orient="horizontal", resolution=0.1,
                         length=160,
                         bg=PANEL_BG, fg=TEXT_MAIN,
                         troughcolor="#1c2130",
                         highlightthickness=0,
                         activebackground=ACCENT,
                         showvalue=False, command=on_change)
            s.pack(side="left", padx=6)

        param_row("Accel rate",  0.5, 8.0, 2.0, "accel_rate")
        param_row("Decel rate",  0.5, 8.0, 4.0, "decel_rate")
        param_row("Expo curve",  0.0, 2.0, 0.5, "expo")

    # ── Command buttons panel ──────────────────────────────────────────
    def _build_commands_panel(self, parent):
        f = self._panel(parent, "COMMANDS")

        def btn(text, color, cmd, row, col):
            b = tk.Button(f, text=text,
                          bg=color, fg="white",
                          activebackground=color, activeforeground="white",
                          font=FONT_BTN, relief="flat", cursor="hand2",
                          width=12, height=1,
                          command=cmd)
            b.grid(row=row, column=col, padx=4, pady=4)
            return b

        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)

        btn("⬆  TAKEOFF", BTN_TAKE, self._cmd_takeoff, 0, 0)
        btn("⬇  LAND",    BTN_LAND, self._cmd_land,    0, 1)
        self._btn_stop = btn("✕  STOP",    BTN_STOP, self._cmd_stop,    1, 0)
        self._btn_head = btn("⧖  HEADLESS",BTN_HEAD, self._toggle_headless, 1, 1)
        btn("◎  CALIBRATE",BTN_CAL, self._cmd_calibrate, 2, 0)
        self._debug_btn = btn("⚙  DEBUG OFF", "#37474f", self._toggle_debug, 2, 1)

    # ── Keyboard legend ────────────────────────────────────────────────
    def _build_keyboard_legend(self, parent):
        f = self._panel(parent, "KEYBOARD")
        items = [
            ("W / S",   "Throttle ↑ ↓"),
            ("A / D",   "Yaw ← →"),
            ("↑ / ↓",   "Pitch fwd / bck"),
            ("← / →",   "Roll left / right"),
            ("T",       "Takeoff"),
            ("L",       "Land"),
            ("SPACE",   "Emergency stop"),
            ("H",       "Toggle headless"),
            ("C",       "Calibrate"),
        ]
        for i, (key, desc) in enumerate(items):
            r = i % 5
            c = (i // 5) * 2
            tk.Label(f, text=key, fg=ACCENT, bg=PANEL_BG,
                     font=FONT_MONO, width=9, anchor="e").grid(
                row=r, column=c, padx=(0, 4), pady=1, sticky="e")
            tk.Label(f, text=desc, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, anchor="w").grid(
                row=r, column=c+1, padx=(0, 20), pady=1, sticky="w")

    # ── Stick sliders panel ────────────────────────────────────────────
    def _build_sticks_panel(self, parent):
        f = self._panel(parent, "LIVE STICKS   (drag or use keyboard)")

        self._stick_vars: dict[str, tk.DoubleVar] = {}
        self._stick_sliders: dict[str, tk.Scale] = {}

        def make_stick(name: str, label: str, row: int):
            tk.Label(f, text=label, fg=ACCENT, bg=PANEL_BG,
                     font=FONT_LABEL, width=10, anchor="w").grid(
                row=row, column=0, padx=8, pady=6, sticky="w")

            var = tk.DoubleVar(value=STICK_MID)
            self._stick_vars[name] = var

            val_lbl = tk.Label(f, text="128", fg=TEXT_MAIN, bg=PANEL_BG,
                               font=FONT_MONO, width=4)
            val_lbl.grid(row=row, column=2, padx=8)

            def on_drag(v):
                val_lbl.config(text=str(int(float(v))))
                # direct set when user drags
                self.state.set_direct(
                    throttle=self._stick_vars["throttle"].get(),
                    yaw     =self._stick_vars["yaw"].get(),
                    pitch   =self._stick_vars["pitch"].get(),
                    roll    =self._stick_vars["roll"].get(),
                )

            s = tk.Scale(f, variable=var,
                         from_=STICK_MIN, to=STICK_MAX,
                         orient="horizontal", resolution=1, length=260,
                         bg=PANEL_BG, fg=TEXT_MAIN,
                         troughcolor="#1c2130",
                         highlightthickness=0,
                         activebackground=ACCENT,
                         showvalue=False, command=on_drag)
            s.grid(row=row, column=1, padx=8, pady=6)
            self._stick_sliders[name] = s

            # centre button
            def centre(n=name, lbl=val_lbl):
                self._stick_vars[n].set(STICK_MID)
                lbl.config(text="128")
                self.state.set_direct(**{k: self._stick_vars[k].get()
                                          for k in self._stick_vars})

            tk.Button(f, text="⊙", bg="#1c2130", fg=TEXT_DIM,
                      activebackground="#263238", font=FONT_MONO,
                      relief="flat", cursor="hand2", width=2,
                      command=centre).grid(row=row, column=3, padx=4)

        make_stick("throttle", "THROTTLE", 0)
        make_stick("yaw",      "YAW",      1)
        make_stick("pitch",    "PITCH",    2)
        make_stick("roll",     "ROLL",     3)

        # Centre all button
        tk.Button(f, text="CENTRE ALL", bg="#263238", fg=TEXT_DIM,
                  activebackground="#37474f", font=FONT_BTN,
                  relief="flat", cursor="hand2",
                  command=self._centre_all).grid(
            row=4, column=0, columnspan=4, pady=(12, 4), sticky="ew", padx=8)

        # raw hex display
        tk.Label(f, text="LAST PACKET (hex):", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_LABEL).grid(row=5, column=0, columnspan=4,
                                        sticky="w", padx=8, pady=(10, 2))
        self._hex_label = tk.Label(f, text="—", fg=ACCENT, bg="#0a0c11",
                                    font=("Courier New", 8),
                                    wraplength=360, justify="left",
                                    anchor="w")
        self._hex_label.grid(row=6, column=0, columnspan=4,
                              sticky="ew", padx=8, pady=(0, 8))

    # ── Debug log ──────────────────────────────────────────────────────
    def _build_log_panel(self, parent):
        sep = tk.Frame(parent, height=1, bg=TEXT_DIM)
        sep.pack(fill="x", padx=20, pady=(8, 0))

        lf = tk.Frame(parent, bg=DARK_BG)
        lf.pack(fill="both", expand=True, padx=20, pady=(4, 12))

        tk.Label(lf, text="PACKET LOG", fg=TEXT_DIM, bg=DARK_BG,
                 font=FONT_LABEL).pack(anchor="w")

        self._log_text = scrolledtext.ScrolledText(
            lf, height=6, bg="#0a0c11", fg="#37ff8b",
            font=("Courier New", 8), relief="flat",
            state="disabled", wrap="none")
        self._log_text.pack(fill="both", expand=True)

        tk.Button(lf, text="Clear", bg="#1c2130", fg=TEXT_DIM,
                  font=FONT_LABEL, relief="flat", cursor="hand2",
                  command=self._clear_log).pack(side="right", pady=4)

    # ─────────────────────────────────────────── key bindings ────────────
    def _bind_keys(self):
        self.root.bind("<KeyPress>",   self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)

    _AXIS_KEYS = {
        "w":      ("throttle", +1),
        "s":      ("throttle", -1),
        "a":      ("yaw",      -1),
        "d":      ("yaw",      +1),
        "Up":     ("pitch",    +1),
        "Down":   ("pitch",    -1),
        "Left":   ("roll",     -1),
        "Right":  ("roll",     +1),
    }

    def _on_key_press(self, event):
        k = event.keysym
        if k in self._keys:
            return   # already pressed

        self._keys.add(k)

        if k in self._AXIS_KEYS:
            axis, direction = self._AXIS_KEYS[k]
            setattr(self.state, f"d_{axis}", direction)
        elif k == "t":
            self._cmd_takeoff()
        elif k == "l":
            self._cmd_land()
        elif k == "space":
            self._cmd_stop()
        elif k == "h":
            self._toggle_headless()
        elif k == "c":
            self._cmd_calibrate()

    def _on_key_release(self, event):
        k = event.keysym
        self._keys.discard(k)

        if k in self._AXIS_KEYS:
            axis, _ = self._AXIS_KEYS[k]
            # only clear if no opposing key is pressed
            opposite_dir = -self._AXIS_KEYS[k][1]
            opposite_key = next(
                (kk for kk, v in self._AXIS_KEYS.items()
                 if v == (axis, opposite_dir)), None)
            if opposite_key not in self._keys:
                setattr(self.state, f"d_{axis}", 0)

    # ─────────────────────────────────────────── commands ────────────────
    def _cmd_takeoff(self):
        self.state.takeoff_flag = True
        self._log_event("CMD: TAKEOFF")

    def _cmd_land(self):
        self.state.land_flag = True
        self._log_event("CMD: LAND")

    def _cmd_stop(self):
        self.state.stop_flag = True
        self._log_event("CMD: EMERGENCY STOP")

    def _cmd_calibrate(self):
        self.state.calibrate_flag = True
        self._log_event("CMD: CALIBRATE")

    def _toggle_headless(self):
        self.state.headless = not self.state.headless
        state = "ON" if self.state.headless else "OFF"
        self._btn_head.config(
            bg=ACCENT if self.state.headless else BTN_HEAD,
            fg=DARK_BG if self.state.headless else "white")
        self._log_event(f"HEADLESS: {state}")

    def _toggle_debug(self):
        self.ctrl.debug = not self.ctrl.debug
        s = "ON" if self.ctrl.debug else "OFF"
        self._debug_btn.config(
            text=f"⚙  DEBUG {s}",
            bg=ACCENT if self.ctrl.debug else "#37474f",
            fg=DARK_BG if self.ctrl.debug else TEXT_MAIN)

    # ─────────────────────────────────────────── connection ──────────────
    def _apply_connection(self):
        try:
            ip   = self._ip_var.get().strip()
            port = int(self._port_var.get())
            rate = float(self._rate_var.get())
        except ValueError as e:
            self._log_event(f"ERROR: bad settings – {e}")
            return
        self.ctrl.reconnect(ip, port, rate)
        self._status_label.config(text="● CONNECTED", fg=BTN_TAKE)
        self._log_event(f"CONNECTED  {ip}:{port}  @ {rate} Hz")

    def _disconnect(self):
        self.ctrl.stop()
        self._status_label.config(text="● STOPPED", fg=ACCENT2)
        self._log_event("DISCONNECTED")

    # ─────────────────────────────────────────── helpers ─────────────────
    def _centre_all(self):
        for name in ("throttle", "yaw", "pitch", "roll"):
            self._stick_vars[name].set(STICK_MID)
            setattr(self.state, f"d_{name}", 0)
        self.state.set_direct(STICK_MID, STICK_MID, STICK_MID, STICK_MID)

    def _log_event(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        try:
            self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full:
            pass

    def _clear_log(self):
        self._log_text.config(state="normal")
        self._log_text.delete("1.0", "end")
        self._log_text.config(state="disabled")

    # ─────────────────────────────────────────── periodic tick ────────────
    def _tick(self):
        # drain log queue → text widget
        msgs = []
        try:
            while True:
                msgs.append(self.log_q.get_nowait())
        except queue.Empty:
            pass

        if msgs:
            self._log_text.config(state="normal")
            for m in msgs:
                self._log_text.insert("end", m + "\n")
            self._log_text.see("end")
            self._log_text.config(state="disabled")

        # update slider positions from live state
        snap = self.state.snapshot()
        for name in ("throttle", "yaw", "pitch", "roll"):
            self._stick_vars[name].set(snap[name])

        # update last-packet hex display (build a dummy preview packet)
        cmd, headless = CMD_NONE, HEADLESS_ON if self.state.headless else HEADLESS_OFF
        preview = build_packet(
            int(snap["roll"]), int(snap["pitch"]),
            int(snap["throttle"]), int(snap["yaw"]),
            cmd, headless, 0, 0, 0)
        hex_rows = [" ".join(f"{b:02x}" for b in preview[i:i+16])
                    for i in range(0, min(len(preview), 48), 16)]
        self._hex_label.config(text="\n".join(hex_rows))

        self.root.after(40, self._tick)   # ~25 fps UI refresh

    # ─────────────────────────────────────────── shutdown ─────────────────
    def on_close(self):
        self.ctrl.stop()
        self.root.destroy()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = K417GUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()