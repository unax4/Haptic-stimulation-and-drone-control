#!/usr/bin/env python3
"""
k417_imu_controller.py  –  Karuisrc K417 WiFi Drone Controller
===============================================================
Glove-based IMU controller for the Karuisrc K417 drone.

IMU (Arduino Nano RP2040) provides:
  - Yaw / Pitch / Roll via Mahony AHRS filter
  - Throttle via A2 (up) and A3 (down) finger flex sensors

Serial data format from Arduino:
  timestamp, A3, A2, A1, A0, ax, ay, az, gx, gy, gz

Keyboard bindings (command overrides — always active):
  T       – Takeoff
  L       – Land
  Space   – Emergency stop
  H       – Toggle headless mode
  C       – Calibrate drone IMU
  O       – Reset glove orientation (re-zero)
  F5      – Start glove calibration

Requirements:
  Python 3.8+   pip install pyserial numpy
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import logging
import colorsys

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ──────────────────────────────────────────────────────────────────────────────
# Logging
# ──────────────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.DEBUG,
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("k417")


# ──────────────────────────────────────────────────────────────────────────────
# Protocol constants
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_IP   = "192.168.169.1"
DEFAULT_PORT = 8800

STICK_MIN = 40
STICK_MID = 128
STICK_MAX = 220

_HDR = bytes([0xEF, 0x02, 0x7C, 0x00, 0x02, 0x02,
              0x00, 0x01, 0x02, 0x00, 0x00, 0x00])
_C1_SUFFIX  = bytes([0x00, 0x00, 0x14, 0x00, 0x66, 0x14])
_CTRL_PAD   = bytes(10)
_CKSUM_SFX  = bytes([0x99]) + bytes(44) + bytes([0x32, 0x4B, 0x14, 0x2D, 0x00, 0x00])
_C2_SUFFIX  = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                     0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
                     0xFF, 0xFF, 0xFF, 0xFF])
_C3_SUFFIX  = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x03, 0x00, 0x00, 0x00, 0x10, 0x00,
                     0x00, 0x00])

CMD_NONE      = 0x00
CMD_TAKEOFF   = 0x01
CMD_LAND      = 0x02
CMD_STOP      = 0x02
CMD_CALIBRATE = 0x04
HEADLESS_OFF  = 0x02
HEADLESS_ON   = 0x03

# ──────────────────────────────────────────────────────────────────────────────
# Video / Telemetry constants  (from wifi_uav_packets reverse-engineering)
# ──────────────────────────────────────────────────────────────────────────────
VIDEO_PORT         = 8800   # same socket as control — drone muxes on payload[1]
START_STREAM       = b"\xef\x00\x04\x00"

# Frame-request packets — must be sent once per received frame
_REQUEST_A = (
    b"\xef\x02\x58\x00\x02\x02"
    b"\x00\x01\x00\x00\x00\x00\x05\x00\x00\x00\x14\x00\x66\x14\x80\x80"
    b"\x80\x80\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x99"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x32\x4b\x14\x2d"
    b"\x00\x00"
)
_REQUEST_B = (
    b"\xef\x02\x6c\x00\x02\x02"
    b"\x00\x01\x02\x00\x00\x00\x09\x00\x00\x00\x14\x00\x66\x14\x80\x80"
    b"\x80\x80\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x99"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x32\x4b\x14\x2d"
    b"\x00\x00\x08\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x14\x00"
    b"\x00\x00\xff\xff\xff\xff\x09\x00\x00\x00\x00\x00\x00\x00\x03\x00"
    b"\x00\x00\x10\x00\x00\x00"
)

# JPEG segment helpers (ported from wifi_uav_jpeg.py in the reference project)
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"

_STD_LUM_QT = [
    16, 11, 10, 16, 24,  40,  51,  61,
    12, 12, 14, 19, 26,  58,  60,  55,
    14, 13, 16, 24, 40,  57,  69,  56,
    14, 17, 22, 29, 51,  87,  80,  62,
    18, 22, 37, 56, 68, 109, 103,  77,
    24, 35, 55, 64, 81, 104, 113,  92,
    49, 64, 78, 87,103, 121, 120, 101,
    72, 92, 95, 98,112, 100, 103,  99,
]
_STD_CHR_QT = [
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
]


def _dqt(table_id: int, table: list) -> bytes:
    payload = bytearray([table_id]) + bytearray(table)
    seg = bytearray(b"\xff\xdb")
    seg += (len(payload) + 2).to_bytes(2, "big")
    seg += payload
    return bytes(seg)


def _sof0(w: int, h: int) -> bytes:
    comps = bytes([1, 0x11, 0, 2, 0x11, 1, 3, 0x11, 1])
    length = (8 + 9).to_bytes(2, "big")
    return (b"\xff\xc0" + length + b"\x08" +
            h.to_bytes(2, "big") + w.to_bytes(2, "big") + b"\x03" + comps)


def _sos() -> bytes:
    payload = bytearray([3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0])
    length = (len(payload) + 2).to_bytes(2, "big")
    return b"\xff\xda" + length + bytes(payload)


def _build_jpeg_header(w: int = 640, h: int = 360) -> bytes:
    return (_SOI + _dqt(0, _STD_LUM_QT) + _dqt(1, _STD_CHR_QT) +
            _sof0(w, h) + _sos())


def build_packet(roll: int, pitch: int, throttle: int, yaw: int,
                 command: int, headless: int,
                 c1: int, c2: int, c3: int) -> bytes:
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
# Mahony AHRS — pure Python / numpy
# ──────────────────────────────────────────────────────────────────────────────
class MahonyFilter:
    """Quaternion-based Mahony AHRS (no magnetometer variant)."""

    def __init__(self, kp: float = 5.0, ki: float = 0.02):
        self.kp = kp
        self.ki = ki
        self.q  = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z
        self._eInt = [0.0, 0.0, 0.0]

        # Calibration
        self.gyro_bias    = [0.0, 0.0, 0.0]
        self.bias_samples: list[list[float]] = []
        self.calibrated   = False

        # Orientation offset (zero-reference quaternion, stored as conjugate)
        self._q_offset = [1.0, 0.0, 0.0, 0.0]

    # ── calibration ──────────────────────────────────────────────────────
    def add_gyro_sample(self, gx: float, gy: float, gz: float,
                        max_samples: int = 150) -> bool:
        """Accumulate gyro bias samples. Returns True when done (and every call after)."""
        if self.calibrated:
            return True   # already done — skip accumulation
        self.bias_samples.append([gx, gy, gz])
        if len(self.bias_samples) >= max_samples:
            n = len(self.bias_samples)
            self.gyro_bias = [
                sum(s[0] for s in self.bias_samples) / n,
                sum(s[1] for s in self.bias_samples) / n,
                sum(s[2] for s in self.bias_samples) / n,
            ]
            self.calibrated = True
            self.capture_offset()
            return True
        return False

    def capture_offset(self):
        """Store conjugate of current quaternion as the zero reference."""
        w, x, y, z = self.q
        self._q_offset = [w, -x, -y, -z]

    # ── update ───────────────────────────────────────────────────────────
    def update(self, ax: float, ay: float, az: float,
               gx: float, gy: float, gz: float, dt: float):
        if self.calibrated:
            gx -= self.gyro_bias[0]
            gy -= self.gyro_bias[1]
            gz -= self.gyro_bias[2]

        q = self.q
        norm_a = math.sqrt(ax*ax + ay*ay + az*az)
        if norm_a == 0.0:
            return
        ax /= norm_a; ay /= norm_a; az /= norm_a

        vx = 2.0 * (q[1]*q[3] - q[0]*q[2])
        vy = 2.0 * (q[0]*q[1] + q[2]*q[3])
        vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]

        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        self._eInt[0] += ex * self.ki * dt
        self._eInt[1] += ey * self.ki * dt
        self._eInt[2] += ez * self.ki * dt

        gx += self.kp * ex + self._eInt[0]
        gy += self.kp * ey + self._eInt[1]
        gz += self.kp * ez + self._eInt[2]

        hw = 0.5 * dt
        pa, pb, pc = q[0], q[1], q[2]
        q[0] += (-q[1]*gx - q[2]*gy - q[3]*gz) * hw
        q[1] += (pa*gx  + q[2]*gz - q[3]*gy)   * hw
        q[2] += (pa*gy  - pb*gz   + q[3]*gx)   * hw
        q[3] += (pa*gz  + pb*gy   - pc*gx)     * hw

        norm_q = math.sqrt(sum(v*v for v in q))
        self.q = [v / norm_q for v in q]

    # ── euler (relative to offset) ────────────────────────────────────────
    def get_euler_relative(self) -> tuple[float, float, float]:
        """Return (yaw, pitch, roll) in degrees relative to the zero offset."""
        qo = self._q_offset
        qa = self.q
        # qo * qa  (Hamilton product, qo is already the conjugate)
        w = qo[0]*qa[0] - qo[1]*qa[1] - qo[2]*qa[2] - qo[3]*qa[3]
        x = qo[0]*qa[1] + qo[1]*qa[0] + qo[2]*qa[3] - qo[3]*qa[2]
        y = qo[0]*qa[2] - qo[1]*qa[3] + qo[2]*qa[0] + qo[3]*qa[1]
        z = qo[0]*qa[3] + qo[1]*qa[2] - qo[2]*qa[1] + qo[3]*qa[0]

        # yaw (Z), pitch (Y), roll (X)  — ZYX Tait-Bryan
        sinr_cosp = 2.0 * (w*x + y*z)
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
        roll_r    = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w*y - z*x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_r = math.asin(sinp)

        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        yaw_r = math.atan2(siny_cosp, cosy_cosp)

        return (math.degrees(yaw_r),
                math.degrees(pitch_r),
                math.degrees(roll_r))


# ──────────────────────────────────────────────────────────────────────────────
# IMU → Drone axis mapper
# ──────────────────────────────────────────────────────────────────────────────
class IMUAxisMapper:
    """
    Converts Mahony Euler angles + flex sensors → raw stick values.

    Deadzone + sensitivity curve:
        if |angle| < deadzone  →  output = 0
        else: normalised = (|angle| − deadzone) / (max_angle − deadzone)
              then apply expo, then scale to stick range

    Throttle  (A2 = up, A3 = down — piezoresistive flex sensors):
        Each channel has a different resting ADC value and a different
        response range because piezoresistive sensors have no fixed span.

        Auto-calibration procedure (runs at startup / on re-calibrate):
          1. Collect FLEX_REST_SAMPLES readings per channel while the
             glove is at rest.
          2. Compute per-channel resting mean (rest_a2, rest_a3) and
             resting standard-deviation (std_a2, std_a3).
          3. At runtime, deflection = raw − rest_mean.
             The active threshold is FLEX_THRESH_STD × std  (default 3σ),
             so small ADC noise at rest never triggers throttle.
          4. Each channel's deflection is normalised by FLEX_NORM_SCALE
             (an expected "full flex" delta in ADC counts).  Users can
             tune this via the GUI slider.
          5. net = clamp(a2_norm − a3_norm, −1, 1)
             → positive  → throttle up,  negative → throttle down.
    """

    # Angular range beyond deadzone where stick reaches maximum
    MAX_ANGLE: float = 45.0

    # Flex auto-calibration
    FLEX_REST_SAMPLES: int   = 80      # samples collected at rest
    FLEX_THRESH_STD:   float = 3.0     # σ multiplier for dead-band
    FLEX_NORM_SCALE:   float = 150.0   # expected ADC delta for full flex

    def __init__(self):
        # --- tunable ---
        self.deadzone:       float = 8.0    # degrees
        self.sensitivity:    float = 1.0    # 0.1 … 2.0 multiplier
        self.expo:           float = 0.5    # 0 = linear, 1 = maximum expo
        self.flex_norm_scale: float = self.FLEX_NORM_SCALE  # GUI-adjustable

        # --- flex rest calibration (per channel: A2=index 2, A3=index 3) ---
        self._flex_rest_buf: list[list[float]] = [[] for _ in range(4)]
        self._flex_rest_mean = [512.0, 512.0, 512.0, 512.0]  # initial guess
        self._flex_rest_std  = [20.0,  20.0,  20.0,  20.0]
        self._flex_calibrated = False   # True once rest baseline is captured

        # --- throttle smoothing ---
        self._throttle_smooth: float = STICK_MID
        self._throttle_alpha:  float = 0.12   # low-pass coefficient

    # ── flex rest calibration ──────────────────────────────────────────────
    def add_flex_rest_sample(self, a0: float, a1: float,
                             a2: float, a3: float) -> bool:
        """
        Accumulate resting ADC samples for all channels.
        Returns True when calibration is complete (and every call after).
        Call this during the gyro-bias phase (glove held still).
        """
        if self._flex_calibrated:
            return True   # already done — skip accumulation

        for i, v in enumerate([a0, a1, a2, a3]):
            self._flex_rest_buf[i].append(v)

        if len(self._flex_rest_buf[2]) >= self.FLEX_REST_SAMPLES:
            for i in range(4):
                buf  = self._flex_rest_buf[i]
                n    = len(buf)
                mean = sum(buf) / n
                std  = math.sqrt(sum((x - mean)**2 for x in buf) / n)
                self._flex_rest_mean[i] = mean
                self._flex_rest_std[i]  = max(std, 5.0)
            self._flex_calibrated = True
            return True
        return False

    def reset_flex_calibration(self):
        """Clear resting baseline so it will be re-captured."""
        self._flex_rest_buf   = [[] for _ in range(4)]
        self._flex_calibrated = False

    # ── flex normalisation (deflection from rest) ────────────────────────
    def _flex_deflection(self, raw: float, idx: int) -> float:
        """
        Return signed normalised deflection in [−1, 1].
        Values within FLEX_THRESH_STD × std of rest are zeroed (dead-band).
        Positive = more flexed than rest, negative = more extended than rest.
        """
        delta = raw - self._flex_rest_mean[idx]
        thresh = self.FLEX_THRESH_STD * self._flex_rest_std[idx]
        if abs(delta) < thresh:
            return 0.0
        # Remove the threshold offset so output starts from 0
        signed = delta - math.copysign(thresh, delta)
        return max(-1.0, min(1.0, signed / self.flex_norm_scale))

    def _angle_to_stick(self, angle: float) -> float:
        """Map signed angle (degrees) → stick value in STICK_MIN…STICK_MAX."""
        sign  = 1.0 if angle >= 0 else -1.0
        mag   = abs(angle)
        if mag < self.deadzone:
            return float(STICK_MID)
        # Normalise 0→1 in the active zone
        norm = min(1.0, (mag - self.deadzone) / (self.MAX_ANGLE - self.deadzone))
        # Sensitivity (capped so we don't exceed limits)
        norm = min(1.0, norm * self.sensitivity)
        # Expo
        e = self.expo
        curved = norm * (1.0 - e) + norm**3 * e
        curved = max(0.0, min(1.0, curved))
        half = float(STICK_MAX - STICK_MID)
        return STICK_MID + sign * curved * half

    def compute(self, yaw_deg: float, pitch_deg: float, roll_deg: float,
                a2: float, a3: float) -> dict[str, float]:
        """
        Returns dict with keys: throttle, yaw, pitch, roll  — all in stick range.
        a2 = throttle-up sensor (A2), a3 = throttle-down sensor (A3).
        """
        # --- YPR → sticks ---
        stick_yaw   = self._angle_to_stick(yaw_deg)
        stick_pitch = self._angle_to_stick(pitch_deg)
        stick_roll  = self._angle_to_stick(roll_deg)

        # --- Throttle via differential flex (A2 up, A3 down) ---
        if not self._flex_calibrated:
            # Not yet calibrated — hold throttle at mid
            stick_throttle = self._throttle_smooth
        else:
            a2_d = self._flex_deflection(a2, 2)   # positive = finger flexed more
            a3_d = self._flex_deflection(a3, 3)   # positive = finger flexed more

            # A2 flexion → throttle up (+), A3 flexion → throttle down (−)
            net = a2_d - a3_d
            net = max(-1.0, min(1.0, net))

            # Expo on throttle (lighter)
            e      = self.expo * 0.6
            sign_t = 1.0 if net >= 0 else -1.0
            mag_t  = abs(net)
            curved_t = mag_t * (1.0 - e) + mag_t**3 * e

            raw_throttle = STICK_MID + sign_t * curved_t * (STICK_MAX - STICK_MID)
            raw_throttle = max(STICK_MIN, min(STICK_MAX, raw_throttle))

            # Low-pass smooth
            self._throttle_smooth += (
                (raw_throttle - self._throttle_smooth) * self._throttle_alpha
            )
            stick_throttle = self._throttle_smooth

        return {
            "throttle": stick_throttle,
            "yaw":      stick_yaw,
            "pitch":    stick_pitch,
            "roll":     stick_roll,
        }


# ──────────────────────────────────────────────────────────────────────────────
# DroneState
# ──────────────────────────────────────────────────────────────────────────────
class DroneState:
    def __init__(self):
        self._lock = threading.Lock()

        self.throttle: float = STICK_MID
        self.yaw:      float = STICK_MID
        self.pitch:    float = STICK_MID
        self.roll:     float = STICK_MID

        self.takeoff_flag   = False
        self.land_flag      = False
        self.stop_flag      = False
        self.calibrate_flag = False
        self.headless       = False

        self._c1 = 0x0000
        self._c2 = 0x0001
        self._c3 = 0x0002

    def set_imu(self, values: dict[str, float]):
        with self._lock:
            self.throttle = max(STICK_MIN, min(STICK_MAX, values["throttle"]))
            self.yaw      = max(STICK_MIN, min(STICK_MAX, values["yaw"]))
            self.pitch    = max(STICK_MIN, min(STICK_MAX, values["pitch"]))
            self.roll     = max(STICK_MIN, min(STICK_MAX, values["roll"]))

    def set_direct(self, throttle, yaw, pitch, roll):
        with self._lock:
            self.throttle = max(STICK_MIN, min(STICK_MAX, throttle))
            self.yaw      = max(STICK_MIN, min(STICK_MAX, yaw))
            self.pitch    = max(STICK_MIN, min(STICK_MAX, pitch))
            self.roll     = max(STICK_MIN, min(STICK_MAX, roll))

    def next_counters(self):
        with self._lock:
            c1, c2, c3 = self._c1, self._c2, self._c3
            self._c1 = (self._c1 + 1) & 0xFFFF
            self._c2 = (self._c2 + 1) & 0xFFFF
            self._c3 = (self._c3 + 1) & 0xFFFF
        return c1, c2, c3

    def consume_flags(self):
        with self._lock:
            if   self.takeoff_flag:   cmd, self.takeoff_flag   = CMD_TAKEOFF,  False
            elif self.stop_flag:      cmd, self.stop_flag       = CMD_STOP,     False
            elif self.land_flag:      cmd, self.land_flag       = CMD_LAND,     False
            elif self.calibrate_flag: cmd, self.calibrate_flag  = CMD_CALIBRATE,False
            else:                     cmd = CMD_NONE
            hless = HEADLESS_ON if self.headless else HEADLESS_OFF
        return cmd, hless

    def snapshot(self):
        with self._lock:
            return {k: getattr(self, k)
                    for k in ("throttle", "yaw", "pitch", "roll")}


# ──────────────────────────────────────────────────────────────────────────────
# FlightController  — UDP sender thread
# ──────────────────────────────────────────────────────────────────────────────
class FlightController:
    """
    Sends control packets at a fixed rate over UDP.

    Socket ownership model
    ──────────────────────
    FlightController does NOT create its own socket.  When video is active,
    VideoReceiver creates the socket (bound to an OS-chosen port) and injects
    it here via inject_socket().  That way both control packets AND the
    START_STREAM / frame-request packets leave from the same source port, so
    the drone sends video fragments back to that port — which VideoReceiver
    is already listening on.

    When video is NOT active, _ensure_own_socket() creates a minimal socket
    just for sending control packets (no video reception).
    """

    def __init__(self, state: DroneState, log_q: queue.Queue):
        self.state       = state
        self.log_q       = log_q
        self.drone_ip    = DEFAULT_IP
        self.drone_port  = DEFAULT_PORT
        self.rate        = 80.0
        self._running    = False
        self._thread     = None
        self._sock       : socket.socket | None = None
        self._sock_lock  = threading.Lock()
        self._own_socket = False   # True only when we created the socket ourselves
        self.debug       = False

    # ── socket injection ──────────────────────────────────────────────────
    def inject_socket(self, sock: socket.socket | None):
        """
        Replace the current socket with an externally-owned one.
        Pass None to drop the injected socket (FlightController will create
        its own on the next send if needed).
        """
        with self._sock_lock:
            if self._own_socket and self._sock:
                try: self._sock.close()
                except Exception: pass
            self._sock       = sock
            self._own_socket = False   # we don't own injected sockets

    def _ensure_own_socket(self):
        """Create a plain send-only socket if none is injected."""
        with self._sock_lock:
            if self._sock is not None:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock       = sock
            self._own_socket = True

    def _send(self, pkt: bytes):
        with self._sock_lock:
            if self._sock is None:
                return
            try:
                self._sock.sendto(pkt, (self.drone_ip, self.drone_port))
            except OSError as e:
                logger.warning("Send error: %s", e)

    def _loop(self):
        self._ensure_own_socket()
        interval = 1.0 / self.rate
        prev     = time.time()
        pkt_num  = 0

        while self._running:
            now = time.time()
            dt  = now - prev
            prev = now

            cmd, headless  = self.state.consume_flags()
            c1, c2, c3     = self.state.next_counters()
            snap           = self.state.snapshot()

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
            time.sleep(max(0.0, interval - elapsed))

        # Only close socket if we own it
        with self._sock_lock:
            if self._own_socket and self._sock:
                self._sock.close()
            self._sock       = None
            self._own_socket = False

    def start(self):
        if self._running: return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="FlightCtrl")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def reconnect(self, ip: str, port: int, rate: float):
        was = self._running
        if was: self.stop()
        self.drone_ip, self.drone_port, self.rate = ip, port, rate
        if was: self.start()


# ──────────────────────────────────────────────────────────────────────────────
# SerialReader  — Arduino serial thread
# ──────────────────────────────────────────────────────────────────────────────
class SerialReader:
    """
    Reads lines from Arduino:  timestamp,A3,A2,A1,A0,ax,ay,az,gx,gy,gz
    Passes parsed data to a callback.
    """

    def __init__(self, port: str, baud: int,
                 on_data,        # callback(a0,a1,a2,a3, ax,ay,az,gx,gy,gz)
                 on_status,      # callback(str)
                 log_q: queue.Queue):
        self.port     = port
        self.baud     = baud
        self.on_data  = on_data
        self.on_status= on_status
        self.log_q    = log_q
        self._running = False
        self._thread  = None

    def start(self):
        if not SERIAL_AVAILABLE:
            self.on_status("ERROR: pyserial not installed")
            return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="SerialReader")
        self._thread.start()

    def stop(self):
        self._running = False

    def _loop(self):
        self.on_status(f"Connecting {self.port}…")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
            self.on_status(f"✓ {self.port} @ {self.baud}")
        except Exception as e:
            self.on_status(f"✗ {e}")
            self._running = False
            return

        while self._running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line and "," in line:
                        self._parse(line)
            except Exception as e:
                self.on_status(f"Read error: {e}")
                time.sleep(0.5)

        ser.close()

    def _parse(self, line: str):
        parts = line.split(",")
        if len(parts) < 11:
            return
        try:
            vals = [float(v) for v in parts[:11]]
            # Format: timestamp, A3, A2, A1, A0, ax, ay, az, gx, gy, gz
            _ts  = vals[0]
            a3   = vals[3]
            a2   = vals[4]
            a1   = vals[1]
            a0   = vals[2]
            ax, ay, az, gx, gy, gz = vals[5:11]
            self.on_data(a0, a1, a2, a3, ax, ay, az, gx, gy, gz)
        except (ValueError, IndexError):
            pass


# ──────────────────────────────────────────────────────────────────────────────
# GloveController  — fuses IMU, flex, calibration → DroneState
# ──────────────────────────────────────────────────────────────────────────────
class GloveController:
    """
    Orchestrates:
      1. Mahony filter update
      2. Gyro bias calibration (auto, on startup) — also captures flex resting baseline
      3. Orientation zero-capture (user-triggered)
      4. IMUAxisMapper → DroneState
    """

    CALIB_SAMPLES = 150

    def __init__(self, state: DroneState, log_q: queue.Queue):
        self.state    = state
        self.log_q    = log_q
        self.ahrs     = MahonyFilter()
        self.mapper   = IMUAxisMapper()
        self._last_t  = time.time()

        # Status
        self.connected       = False
        self.calibrating     = True   # collecting gyro + flex rest baseline on startup
        self.calib_count     = 0
        self.enabled         = True   # can be toggled (pause IMU control)

        # Latest values (for GUI)
        self.yaw_deg   = 0.0
        self.pitch_deg = 0.0
        self.roll_deg  = 0.0
        self.a2_raw    = 0.0
        self.a3_raw    = 0.0
        self.throttle_pct = 0.0

        # Flex calibration status (separate from gyro)
        self.flex_calibrated = False
        self.flex_rest_mean  = [0.0, 0.0, 0.0, 0.0]  # for GUI display

    def reset_calibration(self):
        """Restart gyro bias + flex rest calibration."""
        self.ahrs = MahonyFilter()
        self.mapper.reset_flex_calibration()
        self.calibrating    = True
        self.calib_count    = 0
        self.flex_calibrated= False
        self._log("IMU: re-calibrating gyro bias + flex rest baseline…")

    def capture_zero(self):
        """Capture current orientation as the control zero."""
        self.ahrs.capture_offset()
        self._log("IMU: orientation zeroed ✓")

    def on_sensor_data(self, a0, a1, a2, a3,
                       ax_raw, ay_raw, az_raw,
                       gx_raw, gy_raw, gz_raw):
        # ── axis remapping ──
        ax = ay_raw;  ay = -ax_raw;  az = az_raw
        gx = gy_raw;  gy = -gx_raw;  gz = gz_raw
        gx_r = math.radians(gx)
        gy_r = math.radians(gy)
        gz_r = math.radians(gz)

        now = time.time()
        dt  = now - self._last_t
        self._last_t = now
        dt = min(dt, 0.05)   # guard against large gaps

        # Store raw values for GUI regardless of calibration state
        self.a2_raw = a2
        self.a3_raw = a3

        # ── gyro bias + flex rest calibration phase ──
        if self.calibrating:
            gyro_done = self.ahrs.add_gyro_sample(gx_r, gy_r, gz_r, self.CALIB_SAMPLES)
            flex_done = self.mapper.add_flex_rest_sample(a0, a1, a2, a3)
            self.calib_count += 1
            if gyro_done and flex_done:
                self.calibrating     = False
                self.flex_calibrated = True
                self.flex_rest_mean  = list(self.mapper._flex_rest_mean)
                self._log(
                    f"IMU: gyro bias calibrated ✓  "
                    f"flex rest A2={self.mapper._flex_rest_mean[2]:.0f} "
                    f"A3={self.mapper._flex_rest_mean[3]:.0f}  "
                    f"— point glove forward and press O to zero."
                )
            return

        # ── AHRS update ──
        self.ahrs.update(ax, ay, az, gx_r, gy_r, gz_r, dt)
        yaw, pitch, roll = self.ahrs.get_euler_relative()

        self.yaw_deg   = yaw
        self.pitch_deg = pitch
        self.roll_deg  = roll

        # ── compute sticks (A2 = throttle up, A3 = throttle down) ──
        sticks = self.mapper.compute(yaw, pitch, roll, a2, a3)
        self.throttle_pct = (sticks["throttle"] - STICK_MID) / (STICK_MAX - STICK_MID)

        if self.enabled:
            self.state.set_imu(sticks)

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        try:
            self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full:
            pass


# ──────────────────────────────────────────────────────────────────────────────
# TelemetryParser  — reads non-video packets from the drone UDP stream
# ──────────────────────────────────────────────────────────────────────────────
class TelemetryParser:
    """
    The K417 / WiFi-UAV family sends some non-video UDP datagrams on the same
    port 8800.  Video frames are identified by payload[1] == 0x01.  Any packet
    where payload[1] != 0x01 and len >= 8 is treated as a potential telemetry
    frame.

    Known byte positions (from community captures of similar drones):
      payload[4]  – battery level  (0–100, raw)
      payload[6]  – altitude / floor distance in cm (optical-flow or barometer)

    If the drone does NOT transmit altitude, the field will stay at 0 and the
    GUI will show "N/A".  This is graceful — no crash, just no data.
    """

    def __init__(self):
        self._lock       = threading.Lock()
        self.battery_pct : int   = -1    # -1 = never received
        self.altitude_cm : int   = -1    # -1 = never received
        self.rssi        : int   = -1
        self.raw_last    : bytes = b""

    def ingest(self, payload: bytes) -> bool:
        """
        Try to parse telemetry.  Returns True if data was updated.
        Only considers packets that are NOT video frames (payload[1] != 0x01).
        """
        if len(payload) < 8:
            return False
        if payload[1] == 0x01:
            return False   # video packet — skip

        with self._lock:
            self.raw_last = payload[:16]
            updated = False

            # battery is commonly at byte 4 on these drones
            if len(payload) > 4:
                bat = payload[4]
                if 0 <= bat <= 100:
                    self.battery_pct = bat
                    updated = True

            # altitude (optical flow / barometer) at byte 6, in cm
            if len(payload) > 6:
                alt = payload[6]
                self.altitude_cm = alt
                updated = True

        return updated

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "battery_pct": self.battery_pct,
                "altitude_cm": self.altitude_cm,
                "rssi":        self.rssi,
                "raw":         self.raw_last.hex(" ") if self.raw_last else "—",
            }


# ──────────────────────────────────────────────────────────────────────────────
# VideoReceiver  — dedicated UDP socket for video, separate from control
# ──────────────────────────────────────────────────────────────────────────────
class VideoReceiver:
    """
    Manages video reception from the drone using its OWN dedicated UDP socket.

    CORRECT ARCHITECTURE (matching wifi_uav_video_protocol.py in the reference
    project):

    The drone sends video frames back to whichever source IP:port sent the
    START_STREAM and REQUEST_A/REQUEST_B frame-request packets.  These do NOT
    need to come from the same socket as the control packets — the drone simply
    mirrors responses to the requester.

    Therefore:
      • FlightController owns its own socket for sending control packets.
      • VideoReceiver owns its own socket (bind to "", 0 so the OS picks a free
        port).  It sends START_STREAM + frame requests through that socket, and
        the drone replies with video fragments to the SAME port.
      • The two sockets are completely independent — no sharing, no locking.

    Frame assembly (from wifi_uav_video_protocol.py):
      payload[1]  == 0x01  → JPEG fragment
      payload[2]  == 0x38  → more fragments to come
      payload[2]  != 0x38  → LAST fragment
      payload[16:18]       → little-endian frame ID  (LE)
      payload[32:34]       → little-endian fragment index (LE)
      payload[56:]         → raw JPEG payload bytes
    """

    FRAME_TIMEOUT   = 0.10    # s — resend request if no reply
    MAX_RETRIES     = 3
    WARMUP_INTERVAL = 0.25    # s — how often to resend START_STREAM during warmup
    VIDEO_W         = 640
    VIDEO_H         = 360

    def __init__(self, drone_ip: str, drone_port: int, log_q: queue.Queue,
                 telemetry: "TelemetryParser",
                 flight_ctrl: "FlightController | None" = None):
        self.drone_ip    = drone_ip
        self.drone_port  = drone_port
        self.log_q       = log_q
        self.telemetry   = telemetry
        self._flight_ctrl = flight_ctrl   # will receive the socket injection

        self._frame_q   : queue.Queue = queue.Queue(maxsize=4)
        self._jpeg_hdr  = _build_jpeg_header(self.VIDEO_W, self.VIDEO_H)

        self._sock      : socket.socket | None = None
        self._send_lock = threading.Lock()

        self._running   = False
        self._rx_thread : threading.Thread | None = None
        self._wd_thread : threading.Thread | None = None
        self._wu_thread : threading.Thread | None = None  # warmup

        self._cur_fid     = 1
        self._fragments   : dict[int, bytes] = {}
        self._retry_cnt   = 0
        self._last_req_t  = 0.0
        self._first_frame = True

        self.frames_ok      = 0
        self.frames_dropped = 0

    # ── public API ────────────────────────────────────────────────────────
    def start(self):
        if self._running:
            return

        # Create the one shared socket. VideoReceiver owns it; FlightController
        # is injected with it so ALL outbound packets (control + frame-requests)
        # leave from the same source port. The drone sends video back to that port.
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 0))      # OS picks a free port
        sock.setblocking(False) # non-blocking; we use select() in rx loop
        self._sock = sock

        local_port = sock.getsockname()[1]
        self._log(f"VIDEO: socket bound to local port {local_port}")

        # Inject into FlightController BEFORE starting the receive loop
        # so control packets immediately leave from this port too.
        if self._flight_ctrl is not None:
            self._flight_ctrl.inject_socket(sock)
            self._log("VIDEO: socket injected into FlightController ✓")

        self._running    = True
        self._first_frame = True

        self._rx_thread = threading.Thread(target=self._rx_loop,
                                           daemon=True, name="VideoRx")
        self._wd_thread = threading.Thread(target=self._watchdog,
                                           daemon=True, name="VideoWD")
        self._wu_thread = threading.Thread(target=self._warmup_loop,
                                           daemon=True, name="VideoWarmup")

        self._rx_thread.start()
        self._wd_thread.start()

        # Blast START_STREAM several times — some drones ignore the first packet
        for _ in range(5):
            self._send_raw(START_STREAM)
            time.sleep(0.05)
        self._send_frame_req(0)

        self._wu_thread.start()
        self._log("VIDEO: receiver started — waiting for first frame…")

    def stop(self):
        self._running = False
        # Tell FlightController to drop the injected socket reference first
        # (it will create its own plain socket on next send if needed)
        if self._flight_ctrl is not None:
            self._flight_ctrl.inject_socket(None)
        sock = self._sock
        self._sock = None
        if sock:
            try: sock.close()
            except Exception: pass
        self._log("VIDEO: receiver stopped")

    def get_frame(self, timeout: float = 0.0):
        try:
            return self._frame_q.get(timeout=timeout)
        except queue.Empty:
            return None

    def is_running(self) -> bool:
        return self._running

    # ── internals ─────────────────────────────────────────────────────────
    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        try: self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full: pass

    def _send_raw(self, data: bytes):
        sock = self._sock
        if sock is None:
            return
        try:
            with self._send_lock:
                sock.sendto(data, (self.drone_ip, self.drone_port))
        except OSError:
            pass

    def _send_frame_req(self, frame_id: int):
        lo, hi = frame_id & 0xFF, (frame_id >> 8) & 0xFF
        ra = bytearray(_REQUEST_A); ra[12] = lo; ra[13] = hi
        rb = bytearray(_REQUEST_B)
        for base in (12, 88, 107):
            rb[base] = lo; rb[base + 1] = hi
        self._send_raw(bytes(ra))
        self._send_raw(bytes(rb))
        self._last_req_t = time.time()

    def _warmup_loop(self):
        """Keep resending START_STREAM until the first frame arrives."""
        while self._running and self._first_frame:
            time.sleep(self.WARMUP_INTERVAL)
            if self._first_frame and self._running:
                self._send_raw(START_STREAM)
                self._send_frame_req((self._cur_fid - 1) & 0xFFFF)

    def _rx_loop(self):
        import select as _select
        first_logged = False

        while self._running:
            sock = self._sock
            if sock is None:
                break
            try:
                ready, _, _ = _select.select([sock], [], [], 0.2)
            except Exception:
                break
            if not ready:
                continue

            try:
                payload, _ = sock.recvfrom(65536)
            except BlockingIOError:
                continue
            except OSError:
                break

            # Feed to telemetry (ignores video packets internally)
            self.telemetry.ingest(payload)

            # Must be a JPEG fragment packet
            if len(payload) < 56 or payload[1] != 0x01:
                continue

            if not first_logged:
                self._log("VIDEO: first packet received ✓ — assembling frames…")
                first_logged = True
            self._first_frame = False  # stop warmup loop

            self._retry_cnt = 0
            frame_id = int.from_bytes(payload[16:18], "little")

            if frame_id != self._cur_fid:
                self.frames_dropped += 1
                self._fragments.clear()
                self._cur_fid = frame_id

            frag_id = int.from_bytes(payload[32:34], "little")
            self._fragments.setdefault(frag_id, payload[56:])

            if payload[2] != 0x38:   # last fragment — assemble JPEG
                ordered = [self._fragments[i] for i in sorted(self._fragments)]
                jpeg = self._jpeg_hdr + b"".join(ordered) + _EOI
                self._fragments.clear()
                self._send_frame_req(frame_id)
                self._cur_fid = (frame_id + 1) & 0xFFFF
                self.frames_ok += 1
                try:
                    self._frame_q.put_nowait(jpeg)
                except queue.Full:
                    pass   # display too slow — drop, don't block

    def _watchdog(self):
        """Retry frame requests if the drone goes quiet."""
        while self._running:
            time.sleep(0.05)
            if not self._running:
                break
            if time.time() - self._last_req_t < self.FRAME_TIMEOUT:
                continue
            if self._retry_cnt < self.MAX_RETRIES:
                self._send_frame_req((self._cur_fid - 1) & 0xFFFF)
                self._retry_cnt += 1
            else:
                self._fragments.clear()
                self.frames_dropped += 1
                self._retry_cnt = 0
                self._cur_fid = (self._cur_fid + 1) & 0xFFFF
                self._send_frame_req((self._cur_fid - 1) & 0xFFFF)



DARK_BG   = "#0b0d13"
PANEL_BG  = "#12151f"
CARD_BG   = "#181d2a"
ACCENT    = "#00e5ff"
ACCENT2   = "#ff4081"
ACCENT3   = "#69ff47"
TEXT_MAIN = "#e0e6f0"
TEXT_DIM  = "#4a6070"
BTN_TAKE  = "#00c853"
BTN_LAND  = "#ff6d00"
BTN_STOP  = "#d50000"
BTN_HEAD  = "#7c4dff"
BTN_CAL   = "#0091ea"
IMU_COLOR = "#b388ff"

FONT_MONO  = ("Courier New", 10)
FONT_LABEL = ("Courier New", 9, "bold")
FONT_BTN   = ("Courier New", 10, "bold")
FONT_BIG   = ("Courier New", 14, "bold")
FONT_TITLE = ("Courier New", 18, "bold")
FONT_SMALL = ("Courier New", 8)


# ──────────────────────────────────────────────────────────────────────────────
# Attitude Indicator widget  (simple artificial horizon)
# ──────────────────────────────────────────────────────────────────────────────
class AttitudeIndicator(tk.Canvas):
    """A minimal SVG-style attitude indicator drawn on a Tk Canvas."""

    SIZE = 120

    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=self.SIZE, height=self.SIZE,
                         bg=CARD_BG, highlightthickness=1,
                         highlightbackground=TEXT_DIM, **kwargs)
        self._pitch = 0.0
        self._roll  = 0.0
        self._yaw   = 0.0
        self._draw()

    def update_attitude(self, pitch: float, roll: float, yaw: float):
        self._pitch = pitch
        self._roll  = roll
        self._yaw   = yaw
        self._draw()

    def _draw(self):
        self.delete("all")
        cx = cy = self.SIZE // 2
        r  = cx - 4

        # ── horizon ──
        # Horizon line shifts with pitch (pixels per degree) and rotates with roll
        pitch_px = max(-r, min(r, self._pitch * (r / 45.0)))
        roll_rad = math.radians(self._roll)

        # sky / ground split using polygon clipped to circle
        # Draw background circle
        self.create_oval(cx-r, cy-r, cx+r, cy+r, fill="#1a3a5c", outline="")

        # Ground half (rotated with roll)
        angle = roll_rad
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        # horizon line normal vector
        nx, ny = -sin_a, cos_a
        # horizon offset
        ox = nx * pitch_px
        oy = ny * pitch_px

        # Compute clipping polygon for ground
        pts = []
        steps = 36
        for i in range(steps + 1):
            th = math.pi * i / steps  # lower half
            px_c = cx + r * math.cos(th + math.pi)
            py_c = cy + r * math.sin(th + math.pi)
            pts.extend([px_c, py_c])
        # Project horizon line end-points
        dx = cos_a * r * 1.5
        dy = sin_a * r * 1.5
        h1x = cx + ox + dx; h1y = cy + oy + dy
        h2x = cx + ox - dx; h2y = cy + oy - dy
        ground_pts = [h1x, h1y] + pts + [h2x, h2y]
        try:
            self.create_polygon(ground_pts, fill="#5c3a1a", outline="", smooth=False)
        except Exception:
            pass

        # ── horizon line ──
        dx = cos_a * r; dy = sin_a * r
        self.create_line(cx+ox+dx, cy+oy+dy, cx+ox-dx, cy+oy-dy,
                         fill="white", width=2)

        # ── circle border ──
        self.create_oval(cx-r, cy-r, cx+r, cy+r,
                         outline=ACCENT, width=2)

        # ── aircraft symbol ──
        self.create_line(cx-24, cy, cx-8, cy, fill=ACCENT, width=2)
        self.create_line(cx+8,  cy, cx+24, cy, fill=ACCENT, width=2)
        self.create_oval(cx-4, cy-4, cx+4, cy+4, outline=ACCENT, width=2)

        # ── yaw arrow (top arc) ──
        yaw_norm = (self._yaw % 360) / 360.0
        yaw_angle_start = -90 + self._yaw
        self.create_arc(cx-r+8, cy-r+8, cx+r-8, cy+r-8,
                        start=-90, extent=self._yaw % 360,
                        outline=ACCENT2, width=1, style="arc")

        # ── degree labels ──
        self.create_text(cx, 6, text=f"Y {self._yaw:+.1f}°",
                         fill=ACCENT2, font=FONT_SMALL)
        self.create_text(cx, self.SIZE-6, text=f"P {self._pitch:+.1f}°",
                         fill=ACCENT3, font=FONT_SMALL)
        self.create_text(6, cy, text=f"R\n{self._roll:+.0f}°",
                         fill=IMU_COLOR, font=FONT_SMALL)


# ──────────────────────────────────────────────────────────────────────────────
# Throttle bar widget
# ──────────────────────────────────────────────────────────────────────────────
class ThrottleBar(tk.Canvas):
    WIDTH  = 24
    HEIGHT = 120

    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=self.WIDTH, height=self.HEIGHT,
                         bg=CARD_BG, highlightthickness=1,
                         highlightbackground=TEXT_DIM, **kwargs)
        self._value = 0.0   # -1 … +1
        self._draw()

    def set_value(self, v: float):
        self._value = max(-1.0, min(1.0, v))
        self._draw()

    def _draw(self):
        self.delete("all")
        w = self.WIDTH; h = self.HEIGHT
        mid = h // 2
        # Background
        self.create_rectangle(0, 0, w, h, fill=CARD_BG, outline="")
        # Fill bar
        bar_h = int(abs(self._value) * mid)
        if self._value >= 0:
            color = BTN_TAKE
            self.create_rectangle(2, mid - bar_h, w-2, mid, fill=color, outline="")
        else:
            color = ACCENT2
            self.create_rectangle(2, mid, w-2, mid + bar_h, fill=color, outline="")
        # Centre line
        self.create_line(0, mid, w, mid, fill=TEXT_DIM, width=1)
        # Border
        self.create_rectangle(1, 1, w-2, h-2, outline=TEXT_DIM, width=1)


# ──────────────────────────────────────────────────────────────────────────────
# VideoWindow  — floating Toplevel that renders the drone's camera feed
# ──────────────────────────────────────────────────────────────────────────────
class VideoWindow:
    """
    A separate Tk window that displays the live camera feed from the drone.
    Decodes JPEG bytes to a Tk PhotoImage using PIL (if available) or falls
    back to writing a temporary JPEG file decoded by tkinter's native PPM
    support via a subprocess — or just shows a "no cv2/PIL" warning banner.
    """

    WIN_W = 640
    WIN_H = 400   # 360 video + room for stats bar

    def __init__(self, parent: tk.Tk, video_rx: "VideoReceiver",
                 telemetry: "TelemetryParser", log_q: queue.Queue):
        self.video_rx  = video_rx
        self.telemetry = telemetry
        self.log_q     = log_q
        self._running  = False

        # Try importing PIL for JPEG→PhotoImage
        self._pil_ok = False
        try:
            from PIL import Image, ImageTk   # type: ignore
            self._Image    = Image
            self._ImageTk  = ImageTk
            self._pil_ok   = True
        except ImportError:
            pass

        self._win = tk.Toplevel(parent)
        self._win.title("K417 // Camera Feed")
        self._win.configure(bg=DARK_BG)
        self._win.geometry(f"{self.WIN_W}x{self.WIN_H}")
        self._win.resizable(True, True)
        self._win.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._running = True
        self._tick()

    # ── build ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Canvas for video frames
        self._canvas = tk.Canvas(self._win, width=self.WIN_W, height=360,
                                 bg="#000000", highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)
        self._img_ref = None   # keep reference so GC doesn't eat it

        # Stats bar at the bottom
        stats_bar = tk.Frame(self._win, bg=PANEL_BG, height=40)
        stats_bar.pack(fill="x")
        stats_bar.pack_propagate(False)

        self._fps_var  = tk.StringVar(value="FPS: —")
        self._alt_var  = tk.StringVar(value="Alt: N/A")
        self._bat_var  = tk.StringVar(value="Bat: N/A")
        self._stat_var = tk.StringVar(value="Frames: 0 ok / 0 drop")

        for var, color in [
            (self._fps_var,  ACCENT),
            (self._alt_var,  ACCENT3),
            (self._bat_var,  BTN_TAKE),
            (self._stat_var, TEXT_DIM),
        ]:
            tk.Label(stats_bar, textvariable=var, fg=color, bg=PANEL_BG,
                     font=FONT_LABEL, padx=12).pack(side="left")

        # Placeholder text on canvas
        self._placeholder_id = self._canvas.create_text(
            self.WIN_W // 2, 180,
            text="Waiting for video stream…\n\nMake sure the drone is connected\nand streaming has started.",
            fill=TEXT_DIM, font=FONT_LABEL, justify="center")

        # Dependency warning banner
        if not self._pil_ok and not CV2_AVAILABLE:
            self._canvas.create_text(
                self.WIN_W // 2, 290,
                text="⚠  Install Pillow (pip install Pillow) or opencv-python\n"
                     "   for live video decoding.",
                fill=ACCENT2, font=FONT_LABEL, justify="center")
        elif not self._pil_ok:
            self._canvas.create_text(
                self.WIN_W // 2, 300,
                text="Using OpenCV for decode. Install Pillow for better performance.",
                fill=TEXT_DIM, font=FONT_SMALL, justify="center")

        self._frame_count = 0
        self._fps_t0      = time.time()
        self._fps_frames  = 0

    # ── tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        if not self._running:
            return

        jpeg = self.video_rx.get_frame(timeout=0.0)
        if jpeg is not None:
            self._display_frame(jpeg)
            self._frame_count  += 1
            self._fps_frames   += 1

        # FPS counter (update every 30 frames)
        if self._fps_frames >= 30:
            elapsed = time.time() - self._fps_t0
            fps = self._fps_frames / elapsed if elapsed > 0 else 0
            self._fps_var.set(f"FPS: {fps:.1f}")
            self._fps_t0    = time.time()
            self._fps_frames = 0

        # Telemetry
        tel = self.telemetry.snapshot()
        if tel["altitude_cm"] >= 0:
            self._alt_var.set(f"Alt: {tel['altitude_cm']} cm")
        else:
            self._alt_var.set("Alt: N/A")
        if tel["battery_pct"] >= 0:
            self._bat_var.set(f"Bat: {tel['battery_pct']}%")
        else:
            self._bat_var.set("Bat: N/A")

        rx = self.video_rx
        self._stat_var.set(f"Frames: {rx.frames_ok} ok / {rx.frames_dropped} drop")

        self._win.after(33, self._tick)   # ~30 Hz GUI refresh

    def _display_frame(self, jpeg_bytes: bytes):
        """Decode JPEG and draw on canvas."""
        img_tk = None

        if self._pil_ok:
            try:
                import io
                pil_img = self._Image.open(io.BytesIO(jpeg_bytes))
                # Resize to canvas
                cw = self._canvas.winfo_width()  or self.WIN_W
                ch = self._canvas.winfo_height() or 360
                pil_img = pil_img.resize((cw, ch), self._Image.LANCZOS
                                         if hasattr(self._Image, "LANCZOS")
                                         else self._Image.ANTIALIAS)
                img_tk = self._ImageTk.PhotoImage(pil_img)
            except Exception as e:
                logger.debug("PIL decode error: %s", e)

        elif CV2_AVAILABLE and NUMPY_AVAILABLE:
            try:
                arr   = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    cw = self._canvas.winfo_width()  or self.WIN_W
                    ch = self._canvas.winfo_height() or 360
                    frame = cv2.resize(frame, (cw, ch))
                    # cv2 → PIL → PhotoImage
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    import io
                    from PIL import Image, ImageTk   # type: ignore
                    pil_img = Image.fromarray(frame_rgb)
                    img_tk  = ImageTk.PhotoImage(pil_img)
            except Exception as e:
                logger.debug("CV2 decode error: %s", e)

        if img_tk is not None:
            self._canvas.delete("all")
            self._img_ref = img_tk   # prevent GC
            self._canvas.create_image(0, 0, anchor="nw", image=img_tk)

    # ── close ─────────────────────────────────────────────────────────────
    def _on_close(self):
        self._running = False
        self.video_rx.stop()
        self._win.destroy()

    def destroy(self):
        self._on_close()


class K417GUI:
    def __init__(self, root: tk.Tk):
        self.root   = root
        self.state  = DroneState()
        self.log_q  : queue.Queue = queue.Queue(maxsize=300)
        self.ctrl   = FlightController(self.state, self.log_q)
        self.glove  = GloveController(self.state, self.log_q)
        self.serial : SerialReader | None = None

        # Video / telemetry
        self.telemetry  = TelemetryParser()
        self.video_rx   : VideoReceiver | None = None
        self._video_win : VideoWindow   | None = None

        self._keys: set[str] = set()
        self._build_ui()
        self._bind_keys()
        # NOTE: ctrl.start() is called inside _apply_connection() only —
        # never auto-started, so the socket doesn't exist until the user
        # clicks CONNECT (or START VIDEO, which injects the socket first).
        self._tick()

    # ──────────────────────────────────────── UI ─────────────────────────
    def _build_ui(self):
        r = self.root
        r.title("K417 // IMU Glove Controller")
        r.configure(bg=DARK_BG)
        r.resizable(True, True)
        r.geometry("1340x860")
        r.minsize(1200, 800)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame",       background=DARK_BG)
        style.configure("Panel.TFrame", background=PANEL_BG)

        # ── Title bar ──
        hdr = tk.Frame(r, bg=DARK_BG)
        hdr.pack(fill="x", padx=20, pady=(14, 4))
        tk.Label(hdr, text="K417", fg=ACCENT, bg=DARK_BG,
                 font=FONT_TITLE).pack(side="left")
        tk.Label(hdr, text="  //  IMU GLOVE CONTROLLER",
                 fg=TEXT_DIM, bg=DARK_BG, font=FONT_BIG).pack(side="left")

        self._status_label = tk.Label(hdr, text="● STOPPED",
                                      fg=ACCENT2, bg=DARK_BG, font=FONT_LABEL)
        self._status_label.pack(side="right")

        tk.Frame(r, height=1, bg=ACCENT).pack(fill="x", padx=20, pady=(0, 8))

        # ── Three-column layout ──
        cols = tk.Frame(r, bg=DARK_BG)
        cols.pack(fill="both", expand=True, padx=16)

        left   = tk.Frame(cols, bg=DARK_BG); left.pack(side="left",   fill="both")
        centre = tk.Frame(cols, bg=DARK_BG); centre.pack(side="left",  fill="both", padx=10)
        right  = tk.Frame(cols, bg=DARK_BG); right.pack(side="right",  fill="both")

        self._build_connection_panel(left)
        self._build_glove_panel(left)
        self._build_commands_panel(left)
        self._build_keyboard_legend(left)

        self._build_imu_panel(centre)
        self._build_sensitivity_panel(centre)
        self._build_video_panel(centre)

        self._build_sticks_panel(right)

        self._build_log_panel(r)

    def _panel(self, parent, title: str):
        outer = tk.Frame(parent, bg=DARK_BG)
        outer.pack(fill="x", pady=5)
        tk.Label(outer, text=f"  {title}  ",
                 fg=ACCENT, bg=DARK_BG,
                 font=("Courier New", 9, "bold")).pack(anchor="w")
        frame = tk.Frame(outer, bg=PANEL_BG, padx=10, pady=8)
        frame.pack(fill="x")
        return frame

    # ── Connection ──────────────────────────────────────────────────────
    def _build_connection_panel(self, parent):
        f = self._panel(parent, "DRONE CONNECTION")

        def row(label, default):
            r = tk.Frame(f, bg=PANEL_BG); r.pack(fill="x", pady=2)
            tk.Label(r, text=label, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, width=11, anchor="w").pack(side="left")
            var = tk.StringVar(value=default)
            tk.Entry(r, textvariable=var, width=18,
                     bg=CARD_BG, fg=TEXT_MAIN,
                     insertbackground=ACCENT, font=FONT_MONO,
                     relief="flat", bd=2).pack(side="left", padx=4)
            return var

        self._ip_var   = row("Drone IP",   DEFAULT_IP)
        self._port_var = row("Port (UDP)", str(DEFAULT_PORT))
        self._rate_var = row("Rate (Hz)",  "80")

        br = tk.Frame(f, bg=PANEL_BG); br.pack(fill="x", pady=(6, 0))
        tk.Button(br, text="CONNECT", bg="#0d47a1", fg=TEXT_MAIN,
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._apply_connection).pack(side="left", padx=2)
        tk.Button(br, text="DISCONNECT", bg="#37474f", fg=TEXT_MAIN,
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._disconnect).pack(side="left", padx=2)

    # ── Glove / Serial ──────────────────────────────────────────────────
    def _build_glove_panel(self, parent):
        f = self._panel(parent, "GLOVE  (Arduino Nano RP2040)")

        # Port row
        pr = tk.Frame(f, bg=PANEL_BG); pr.pack(fill="x", pady=2)
        tk.Label(pr, text="Serial port", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_LABEL, width=11, anchor="w").pack(side="left")
        self._serial_port_var = tk.StringVar(value="COM3")
        tk.Entry(pr, textvariable=self._serial_port_var, width=10,
                 bg=CARD_BG, fg=TEXT_MAIN, insertbackground=ACCENT,
                 font=FONT_MONO, relief="flat", bd=2).pack(side="left", padx=4)

        self._serial_status = tk.Label(pr, text="not connected",
                                       fg=ACCENT2, bg=PANEL_BG, font=FONT_SMALL)
        self._serial_status.pack(side="left", padx=6)

        # Baud row
        br2 = tk.Frame(f, bg=PANEL_BG); br2.pack(fill="x", pady=2)
        tk.Label(br2, text="Baud rate", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_LABEL, width=11, anchor="w").pack(side="left")
        self._baud_var = tk.StringVar(value="115200")
        tk.Entry(br2, textvariable=self._baud_var, width=10,
                 bg=CARD_BG, fg=TEXT_MAIN, insertbackground=ACCENT,
                 font=FONT_MONO, relief="flat", bd=2).pack(side="left", padx=4)

        # Buttons
        btn_r = tk.Frame(f, bg=PANEL_BG); btn_r.pack(fill="x", pady=(6, 2))
        tk.Button(btn_r, text="CONNECT GLOVE", bg="#1b5e20", fg=TEXT_MAIN,
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._connect_glove).pack(side="left", padx=2)
        tk.Button(btn_r, text="ZERO  [O]", bg=IMU_COLOR, fg=DARK_BG,
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._imu_zero).pack(side="left", padx=2)

        btn_r2 = tk.Frame(f, bg=PANEL_BG); btn_r2.pack(fill="x", pady=2)
        tk.Button(btn_r2, text="RE-CALIBRATE [F5]", bg="#4a148c", fg=TEXT_MAIN,
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._imu_recalibrate).pack(side="left", padx=2)
        self._glove_toggle_btn = tk.Button(btn_r2, text="IMU  ENABLED",
                  bg=ACCENT3, fg=DARK_BG,
                  font=FONT_BTN, relief="flat", cursor="hand2",
                  command=self._toggle_imu)
        self._glove_toggle_btn.pack(side="left", padx=2)

        # Calibration progress bar
        self._calib_frame = tk.Frame(f, bg=PANEL_BG); self._calib_frame.pack(fill="x", pady=4)
        tk.Label(self._calib_frame, text="Gyro bias cal:", fg=TEXT_DIM,
                 bg=PANEL_BG, font=FONT_SMALL).pack(side="left")
        self._calib_canvas = tk.Canvas(self._calib_frame, width=140, height=10,
                                       bg=CARD_BG, highlightthickness=0)
        self._calib_canvas.pack(side="left", padx=4)
        self._calib_label = tk.Label(self._calib_frame, text="0 / 150",
                                     fg=TEXT_DIM, bg=PANEL_BG, font=FONT_SMALL)
        self._calib_label.pack(side="left")

    # ── IMU live display ─────────────────────────────────────────────────
    def _build_imu_panel(self, parent):
        f = self._panel(parent, "IMU  ATTITUDE")

        top = tk.Frame(f, bg=PANEL_BG); top.pack(fill="x")

        # Attitude indicator
        self._ahi = AttitudeIndicator(top)
        self._ahi.pack(side="left", padx=(0, 12))

        # Throttle bar
        thr_f = tk.Frame(top, bg=PANEL_BG); thr_f.pack(side="left")
        tk.Label(thr_f, text="THR", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_SMALL).pack()
        self._thr_bar = ThrottleBar(thr_f)
        self._thr_bar.pack()

        # Numeric readouts
        nums = tk.Frame(top, bg=PANEL_BG); nums.pack(side="left", padx=8)
        self._imu_vars: dict[str, tk.StringVar] = {}
        for label, key, color in [
            ("YAW",   "yaw",   ACCENT2),
            ("PITCH", "pitch", ACCENT3),
            ("ROLL",  "roll",  IMU_COLOR),
            ("A2↑",   "a2",    BTN_TAKE),
            ("A3↓",   "a3",    ACCENT2),
        ]:
            row = tk.Frame(nums, bg=PANEL_BG); row.pack(fill="x", pady=1)
            tk.Label(row, text=f"{label:5s}", fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, width=6, anchor="w").pack(side="left")
            var = tk.StringVar(value="—")
            self._imu_vars[key] = var
            tk.Label(row, textvariable=var, fg=color, bg=PANEL_BG,
                     font=FONT_MONO, width=8, anchor="e").pack(side="left")

        # Calibration status
        self._calib_status = tk.Label(f, text="● Calibrating gyro bias…",
                                      fg=ACCENT2, bg=PANEL_BG, font=FONT_LABEL)
        self._calib_status.pack(pady=(6, 2))

    # ── Sensitivity controls ─────────────────────────────────────────────
    def _build_sensitivity_panel(self, parent):
        f = self._panel(parent, "IMU  SENSITIVITY")

        def param(label, from_, to, initial, setter, resolution=0.05):
            row = tk.Frame(f, bg=PANEL_BG); row.pack(fill="x", pady=3)
            tk.Label(row, text=label, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, width=14, anchor="w").pack(side="left")
            var  = tk.DoubleVar(value=initial)
            disp = tk.Label(row, textvariable=var, fg=ACCENT, bg=PANEL_BG,
                            font=FONT_MONO, width=5)
            disp.pack(side="right")

            def on_change(*_):
                v = round(var.get(), 3)
                setter(v)
                disp.config(text=f"{v:.2f}")

            tk.Scale(row, variable=var, from_=from_, to=to,
                     orient="horizontal", resolution=resolution, length=180,
                     bg=PANEL_BG, fg=TEXT_MAIN, troughcolor=CARD_BG,
                     highlightthickness=0, activebackground=ACCENT,
                     showvalue=False, command=on_change).pack(side="left", padx=4)

        param("Deadzone (°)",  0.0, 30.0, 8.0,
              lambda v: setattr(self.glove.mapper, "deadzone", v),    resolution=0.5)
        param("Sensitivity",   0.1,  2.0, 1.0,
              lambda v: setattr(self.glove.mapper, "sensitivity", v), resolution=0.05)
        param("Expo curve",    0.0,  1.0, 0.5,
              lambda v: setattr(self.glove.mapper, "expo", v),        resolution=0.05)
        param("Thr smoothing", 0.02, 0.5, 0.12,
              lambda v: setattr(self.glove.mapper, "_throttle_alpha", v), resolution=0.01)
        param("Flex scale",   20.0, 400.0, IMUAxisMapper.FLEX_NORM_SCALE,
              lambda v: setattr(self.glove.mapper, "flex_norm_scale", v), resolution=5.0)
        param("Mahony Kp",     1.0, 15.0, 5.0,
              lambda v: setattr(self.glove.ahrs, "kp", v),            resolution=0.5)

        # Deadzone indicator text
        tk.Label(f, text="Angles beyond deadzone → linear→expo response",
                 fg=TEXT_DIM, bg=PANEL_BG, font=FONT_SMALL,
                 wraplength=230, justify="left").pack(pady=(4, 0))

    # ── Video & Telemetry ────────────────────────────────────────────────
    def _build_video_panel(self, parent):
        f = self._panel(parent, "CAMERA  &  TELEMETRY")

        # Video start/stop button
        btn_row = tk.Frame(f, bg=PANEL_BG); btn_row.pack(fill="x", pady=(0, 6))
        self._video_btn = tk.Button(
            btn_row, text="▶  START VIDEO",
            bg="#005f73", fg=TEXT_MAIN,
            font=FONT_BTN, relief="flat", cursor="hand2",
            command=self._toggle_video)
        self._video_btn.pack(side="left", padx=2)

        # Dependency hint
        if not CV2_AVAILABLE:
            hint = "pip install Pillow   (or opencv-python)"
            tk.Label(btn_row, text=hint, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_SMALL).pack(side="left", padx=8)

        # Telemetry strip
        tele_row = tk.Frame(f, bg=PANEL_BG); tele_row.pack(fill="x", pady=2)
        for label, var_name, color in [
            ("ALTITUDE", "_tel_alt_var", ACCENT3),
            ("BATTERY",  "_tel_bat_var", BTN_TAKE),
        ]:
            tk.Label(tele_row, text=label, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, width=9, anchor="w").pack(side="left")
            var = tk.StringVar(value="N/A")
            setattr(self, var_name, var)
            tk.Label(tele_row, textvariable=var, fg=color, bg=PANEL_BG,
                     font=FONT_MONO, width=10, anchor="w").pack(side="left", padx=(0, 12))

        # Raw telemetry hex (last packet)
        raw_row = tk.Frame(f, bg=PANEL_BG); raw_row.pack(fill="x", pady=(2, 0))
        tk.Label(raw_row, text="Last telem hex:", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_SMALL).pack(side="left")
        self._tel_raw_var = tk.StringVar(value="—")
        tk.Label(raw_row, textvariable=self._tel_raw_var, fg=TEXT_DIM,
                 bg=PANEL_BG, font=(FONT_MONO[0], 7)).pack(side="left", padx=4)

    # ── Commands ─────────────────────────────────────────────────────────
    def _build_commands_panel(self, parent):
        f = self._panel(parent, "COMMANDS")
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)

        def btn(text, color, cmd, row, col):
            b = tk.Button(f, text=text, bg=color, fg="white",
                          activebackground=color, font=FONT_BTN,
                          relief="flat", cursor="hand2",
                          width=12, height=1, command=cmd)
            b.grid(row=row, column=col, padx=3, pady=3)
            return b

        btn("⬆  TAKEOFF", BTN_TAKE, self._cmd_takeoff,     0, 0)
        btn("⬇  LAND",    BTN_LAND, self._cmd_land,         0, 1)
        self._btn_stop = btn("✕  STOP",   BTN_STOP, self._cmd_stop, 1, 0)
        self._btn_head = btn("⧖  HEADLESS",BTN_HEAD,self._toggle_headless, 1, 1)
        btn("◎  CALIBRATE",BTN_CAL, self._cmd_calibrate,   2, 0)
        self._debug_btn = btn("⚙  DEBUG OFF","#37474f",self._toggle_debug, 2, 1)

    # ── Keyboard legend ──────────────────────────────────────────────────
    def _build_keyboard_legend(self, parent):
        f = self._panel(parent, "KEYBOARD  OVERRIDES")
        items = [
            ("T",     "Takeoff"),
            ("L",     "Land"),
            ("SPACE", "Emergency stop"),
            ("H",     "Headless"),
            ("C",     "Calibrate drone"),
            ("O",     "Zero IMU"),
            ("F5",    "Re-calibrate IMU"),
        ]
        for i, (key, desc) in enumerate(items):
            r = i % 4; c = (i // 4) * 2
            tk.Label(f, text=key, fg=ACCENT, bg=PANEL_BG,
                     font=FONT_MONO, width=6, anchor="e").grid(
                row=r, column=c, padx=(0, 4), pady=1, sticky="e")
            tk.Label(f, text=desc, fg=TEXT_DIM, bg=PANEL_BG,
                     font=FONT_LABEL, anchor="w").grid(
                row=r, column=c+1, padx=(0, 16), pady=1, sticky="w")

    # ── Live stick display ───────────────────────────────────────────────
    def _build_sticks_panel(self, parent):
        f = self._panel(parent, "LIVE  STICKS")
        self._stick_vars    : dict[str, tk.DoubleVar] = {}
        self._stick_val_lbl : dict[str, tk.Label]     = {}

        for i, (name, label) in enumerate([
            ("throttle", "THROTTLE"),
            ("yaw",      "YAW"),
            ("pitch",    "PITCH"),
            ("roll",     "ROLL"),
        ]):
            tk.Label(f, text=label, fg=ACCENT, bg=PANEL_BG,
                     font=FONT_LABEL, width=9, anchor="w").grid(
                row=i, column=0, padx=6, pady=5, sticky="w")

            var = tk.DoubleVar(value=STICK_MID)
            self._stick_vars[name] = var

            vl = tk.Label(f, text="128", fg=TEXT_MAIN, bg=PANEL_BG,
                          font=FONT_MONO, width=4)
            vl.grid(row=i, column=2, padx=6)
            self._stick_val_lbl[name] = vl

            tk.Scale(f, variable=var,
                     from_=STICK_MIN, to=STICK_MAX,
                     orient="horizontal", resolution=1, length=320,
                     bg=PANEL_BG, fg=TEXT_MAIN, troughcolor=CARD_BG,
                     highlightthickness=0, activebackground=ACCENT,
                     showvalue=False, state="disabled").grid(
                row=i, column=1, padx=6, pady=5)

        # last packet hex
        tk.Label(f, text="LAST PACKET (hex):", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_LABEL).grid(row=4, column=0, columnspan=3,
                                        sticky="w", padx=6, pady=(8, 2))
        self._hex_label = tk.Label(f, text="—", fg=ACCENT, bg=CARD_BG,
                                    font=("Courier New", 8),
                                    wraplength=320, justify="left", anchor="w",
                                    padx=4, pady=4)
        self._hex_label.grid(row=5, column=0, columnspan=3,
                              sticky="ew", padx=6, pady=(0, 6))

        # IMU angle bars (compact)
        bar_f = tk.Frame(f, bg=PANEL_BG); bar_f.grid(
            row=6, column=0, columnspan=3, sticky="ew", padx=6, pady=4)
        tk.Label(bar_f, text="IMU ANGLE BARS", fg=TEXT_DIM, bg=PANEL_BG,
                 font=FONT_LABEL).pack(anchor="w")
        bars_inner = tk.Frame(bar_f, bg=PANEL_BG)
        bars_inner.pack(fill="x")
        self._angle_bars: dict[str, tk.Canvas] = {}
        for axis, color in [("YAW", ACCENT2), ("PITCH", ACCENT3), ("ROLL", IMU_COLOR)]:
            r2 = tk.Frame(bars_inner, bg=PANEL_BG); r2.pack(fill="x", pady=1)
            tk.Label(r2, text=axis, fg=color, bg=PANEL_BG,
                     font=FONT_SMALL, width=6, anchor="w").pack(side="left")
            c = tk.Canvas(r2, width=260, height=10, bg=CARD_BG,
                          highlightthickness=0)
            c.pack(side="left", padx=2)
            self._angle_bars[axis] = c

    # ── Log ──────────────────────────────────────────────────────────────
    def _build_log_panel(self, parent):
        tk.Frame(parent, height=1, bg=TEXT_DIM).pack(fill="x", padx=20, pady=(6, 0))
        lf = tk.Frame(parent, bg=DARK_BG)
        lf.pack(fill="both", expand=True, padx=20, pady=(4, 10))
        tk.Label(lf, text="EVENT LOG", fg=TEXT_DIM, bg=DARK_BG,
                 font=FONT_LABEL).pack(anchor="w")
        self._log_text = scrolledtext.ScrolledText(
            lf, height=5, bg=CARD_BG, fg="#37ff8b",
            font=("Courier New", 8), relief="flat",
            state="disabled", wrap="none")
        self._log_text.pack(fill="both", expand=True)
        tk.Button(lf, text="Clear", bg=PANEL_BG, fg=TEXT_DIM,
                  font=FONT_LABEL, relief="flat", cursor="hand2",
                  command=self._clear_log).pack(side="right", pady=3)

    # ──────────────────────────────────────── key bindings ────────────────
    def _bind_keys(self):
        self.root.bind("<KeyPress>",   self._on_key_press)

    def _on_key_press(self, event):
        k = event.keysym
        if k == "t":           self._cmd_takeoff()
        elif k == "l":         self._cmd_land()
        elif k == "space":     self._cmd_stop()
        elif k == "h":         self._toggle_headless()
        elif k == "c":         self._cmd_calibrate()
        elif k.lower() == "o": self._imu_zero()
        elif k == "F5":        self._imu_recalibrate()

    # ──────────────────────────────────────── glove actions ──────────────
    def _connect_glove(self):
        if self.serial:
            self.serial.stop()
        port = self._serial_port_var.get().strip()
        try:
            baud = int(self._baud_var.get())
        except ValueError:
            baud = 115200
        self.serial = SerialReader(
            port=port, baud=baud,
            on_data=self.glove.on_sensor_data,
            on_status=self._on_serial_status,
            log_q=self.log_q,
        )
        self.serial.start()

    def _on_serial_status(self, msg: str):
        # Called from serial thread — schedule GUI update
        self.root.after(0, lambda: self._serial_status.config(
            text=msg,
            fg=ACCENT3 if "✓" in msg else ACCENT2
        ))
        self._log_event(f"SERIAL: {msg}")

    def _imu_zero(self):
        self.glove.capture_zero()
        self._log_event("IMU: orientation zeroed")

    def _imu_recalibrate(self):
        self.glove.reset_calibration()
        self._log_event("IMU: re-calibration started — hold glove still!")

    def _toggle_imu(self):
        self.glove.enabled = not self.glove.enabled
        if self.glove.enabled:
            self._glove_toggle_btn.config(text="IMU  ENABLED",
                                          bg=ACCENT3, fg=DARK_BG)
        else:
            self._glove_toggle_btn.config(text="IMU  PAUSED",
                                          bg=ACCENT2, fg="white")
        self._log_event(f"IMU control: {'ON' if self.glove.enabled else 'PAUSED'}")

    # ──────────────────────────────────────── drone commands ─────────────
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
        self._log_event("CMD: CALIBRATE DRONE IMU")

    def _toggle_headless(self):
        self.state.headless = not self.state.headless
        s = "ON" if self.state.headless else "OFF"
        self._btn_head.config(
            bg=ACCENT if self.state.headless else BTN_HEAD,
            fg=DARK_BG if self.state.headless else "white")
        self._log_event(f"HEADLESS: {s}")

    def _toggle_video(self):
        """Start or stop the video receiver and pop up the video window."""
        if self.video_rx is not None and self.video_rx.is_running():
            if self._video_win:
                try: self._video_win._running = False
                except Exception: pass
                self._video_win = None
            self.video_rx.stop()   # also un-injects socket from FlightController
            self.video_rx = None
            self._video_btn.config(text="▶  START VIDEO", bg="#005f73")
            self._log_event("VIDEO: stopped")
        else:
            ip   = self._ip_var.get().strip()
            port = int(self._port_var.get())
            rate = float(self._rate_var.get())

            # Stop control loop so it releases any old socket before we inject
            self.ctrl.stop()

            # Create VideoReceiver with reference to ctrl — it will inject the
            # shared socket into ctrl before control packets resume
            self.video_rx = VideoReceiver(ip, port, self.log_q,
                                          self.telemetry, self.ctrl)
            self.video_rx.start()   # creates socket AND injects into ctrl

            # Now (re)start control on the injected socket
            self.ctrl.drone_ip   = ip
            self.ctrl.drone_port = port
            self.ctrl.rate       = rate
            self.ctrl.start()

            self._video_win = VideoWindow(self.root, self.video_rx,
                                          self.telemetry, self.log_q)
            self._video_btn.config(text="■  STOP VIDEO", bg=BTN_STOP)
            self._status_label.config(text="● CONNECTED", fg=BTN_TAKE)
            self._log_event(f"VIDEO: started + control on shared socket → {ip}:{port}")

    def _toggle_debug(self):
        self.ctrl.debug = not self.ctrl.debug
        s = "ON" if self.ctrl.debug else "OFF"
        self._debug_btn.config(
            text=f"⚙  DEBUG {s}",
            bg=ACCENT if self.ctrl.debug else "#37474f",
            fg=DARK_BG if self.ctrl.debug else TEXT_MAIN)

    # ──────────────────────────────────────── connection ─────────────────
    def _apply_connection(self):
        try:
            ip   = self._ip_var.get().strip()
            port = int(self._port_var.get())
            rate = float(self._rate_var.get())
        except ValueError as e:
            self._log_event(f"ERROR: {e}")
            return
        self.ctrl.drone_ip   = ip
        self.ctrl.drone_port = port
        self.ctrl.rate       = rate
        if not self.ctrl._running:
            self.ctrl.start()
        self._status_label.config(text="● CONNECTED", fg=BTN_TAKE)
        self._log_event(f"CONNECTED  {ip}:{port}  @ {rate} Hz")

    def _disconnect(self):
        self.ctrl.stop()
        self._status_label.config(text="● STOPPED", fg=ACCENT2)
        self._log_event("DISCONNECTED")

    # ──────────────────────────────────────── log helpers ─────────────────
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

    # ──────────────────────────────────────── periodic tick ──────────────
    def _tick(self):
        # ── drain log queue ──
        msgs = []
        try:
            while True: msgs.append(self.log_q.get_nowait())
        except queue.Empty:
            pass
        if msgs:
            self._log_text.config(state="normal")
            for m in msgs:
                self._log_text.insert("end", m + "\n")
            self._log_text.see("end")
            self._log_text.config(state="disabled")

        # ── update stick displays ──
        snap = self.state.snapshot()
        for name in ("throttle", "yaw", "pitch", "roll"):
            self._stick_vars[name].set(snap[name])
            self._stick_val_lbl[name].config(text=str(int(snap[name])))

        # ── update packet hex ──
        cmd, headless = CMD_NONE, HEADLESS_ON if self.state.headless else HEADLESS_OFF
        preview = build_packet(
            int(snap["roll"]), int(snap["pitch"]),
            int(snap["throttle"]), int(snap["yaw"]),
            cmd, headless, 0, 0, 0)
        hex_rows = [" ".join(f"{b:02x}" for b in preview[i:i+16])
                    for i in range(0, min(len(preview), 48), 16)]
        self._hex_label.config(text="\n".join(hex_rows))

        # ── update IMU display ──
        g = self.glove
        self._imu_vars["yaw"].set(f"{g.yaw_deg:+7.1f}°")
        self._imu_vars["pitch"].set(f"{g.pitch_deg:+7.1f}°")
        self._imu_vars["roll"].set(f"{g.roll_deg:+7.1f}°")
        self._imu_vars["a2"].set(f"{g.a2_raw:7.0f}")
        self._imu_vars["a3"].set(f"{g.a3_raw:7.0f}")
        self._ahi.update_attitude(g.pitch_deg, g.roll_deg, g.yaw_deg)
        self._thr_bar.set_value(g.throttle_pct)

        # ── update angle bars ──
        for axis, val, rng in [
            ("YAW",   g.yaw_deg,   180.0),
            ("PITCH", g.pitch_deg,  90.0),
            ("ROLL",  g.roll_deg,   90.0),
        ]:
            c = self._angle_bars[axis]
            c.delete("all")
            w, h = 260, 10
            mid = w // 2
            norm = max(-1.0, min(1.0, val / rng))
            bar  = int(abs(norm) * (w // 2))
            color = ACCENT2 if norm < 0 else ACCENT3
            if norm >= 0:
                c.create_rectangle(mid, 1, mid+bar, h-1, fill=color, outline="")
            else:
                c.create_rectangle(mid-bar, 1, mid, h-1, fill=color, outline="")
            # deadzone markers
            dz = self.glove.mapper.deadzone
            dz_px = int((dz / rng) * (w // 2))
            c.create_line(mid-dz_px, 0, mid-dz_px, h, fill=TEXT_DIM, width=1)
            c.create_line(mid+dz_px, 0, mid+dz_px, h, fill=TEXT_DIM, width=1)
            c.create_line(mid, 0, mid, h, fill=TEXT_DIM, width=1)

        # ── calibration progress bar ──
        if g.calibrating:
            pct = min(1.0, g.calib_count / GloveController.CALIB_SAMPLES)
            self._calib_canvas.delete("all")
            self._calib_canvas.create_rectangle(
                0, 0, int(140*pct), 10, fill=ACCENT2, outline="")
            self._calib_label.config(
                text=f"{g.calib_count} / {GloveController.CALIB_SAMPLES}")
            self._calib_status.config(
                text="● Calibrating gyro + flex rest — hold STILL", fg=ACCENT2)
        else:
            self._calib_canvas.delete("all")
            self._calib_canvas.create_rectangle(0, 0, 140, 10, fill=ACCENT3, outline="")
            if g.flex_calibrated:
                m = g.mapper._flex_rest_mean
                self._calib_label.config(
                    text=f"DONE ✓  A2 rest={m[2]:.0f}  A3 rest={m[3]:.0f}")
            else:
                self._calib_label.config(text="DONE ✓")
            self._calib_status.config(
                text="● IMU ready — press O to zero orientation", fg=ACCENT3)

        # ── update telemetry display ──
        tel = self.telemetry.snapshot()
        if tel["altitude_cm"] >= 0:
            self._tel_alt_var.set(f"{tel['altitude_cm']} cm")
        else:
            self._tel_alt_var.set("N/A")
        if tel["battery_pct"] >= 0:
            self._tel_bat_var.set(f"{tel['battery_pct']}%")
        else:
            self._tel_bat_var.set("N/A")
        self._tel_raw_var.set(tel["raw"][:47])   # cap display length

        self.root.after(40, self._tick)

    # ──────────────────────────────────────── shutdown ────────────────────
    def on_close(self):
        if self.serial: self.serial.stop()
        self.ctrl.stop()
        if self.video_rx: self.video_rx.stop()
        if self._video_win:
            try: self._video_win._running = False
            except Exception: pass
        self.root.destroy()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main():
    if not SERIAL_AVAILABLE:
        print("WARNING: pyserial not found.  Install with:  pip install pyserial")
    root = tk.Tk()
    app  = K417GUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()