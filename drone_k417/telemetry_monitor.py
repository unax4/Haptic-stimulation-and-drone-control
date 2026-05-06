#!/usr/bin/env python3
"""
ground_control.py  –  K417 Passive Ground Control Station
══════════════════════════════════════════════════════════
Architecture
────────────
  Arduino Nano RP2040 ──USB Serial──► this PC
    • Arduino sends the SAME raw sensor CSV the original script used:
        timestamp, A3, A2, A1, A0, ax, ay, az, gx, gy, gz
    • This script runs the FULL Mahony AHRS + IMUAxisMapper locally,
      exactly as the original control_video_v6.py did
        • Keyboard commands are forwarded to Arduino via serial
            (single-char: T, L, X, H, C, O, F5, D).
    • Serial is read at full Arduino output rate — no artificial throttle.

    Modes:
        • Arduino mode: commands forwarded over serial; Arduino owns UDP control.
        • Video mode: Python owns UDP control + video socket for stable streaming.

UI: pixel-identical Tkinter layout to control_video_v6.py.
    AttitudeIndicator, ThrottleBar, sensitivity sliders, angle bars,
    calibration progress bar, stick displays, log panel — all identical.

Requirements:
    pip install pyserial Pillow
    pip install numpy opencv-python-headless
    pip install torch torchvision timm      # MiDaS distance estimator
    pip install ultralytics                 # optional YOLO person anchor
"""

from __future__ import annotations

import math
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue

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
    try:
        from distance_estimator_v2 import AsyncDistanceEstimator, YOLO_AVAILABLE as _DE_YOLO_AVAILABLE
    except ImportError:
        from etc.distance_estimator import AsyncDistanceEstimator, YOLO_AVAILABLE as _DE_YOLO_AVAILABLE
    DIST_EST_AVAILABLE = True
except ImportError:
    DIST_EST_AVAILABLE = False
    AsyncDistanceEstimator = None
    _DE_YOLO_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════════════
#  PROTOCOL CONSTANTS  (unchanged from control_video_v6.py)
# ══════════════════════════════════════════════════════════════════════════════

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
CMD_CALIBRATE = 0x04
CMD_STOP      = 0x05
CMD_CAM_DOWN  = 0x06

HEADLESS_OFF  = 0x02
HEADLESS_ON   = 0x03

START_STREAM = b"\xef\x00\x04\x00"

REQUEST_A = (
    b"\xef\x02\x58\x00\x02\x02"
    b"\x00\x01\x00\x00\x00\x00\x05\x00\x00\x00\x14\x00\x66\x14\x80\x80"
    b"\x80\x80\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x99"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x32\x4b\x14\x2d"
    b"\x00\x00"
)
REQUEST_B = (
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

_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"

_LUM_QT = [
    16,11,10,16,24, 40, 51, 61, 12,12,14,19,26, 58, 60, 55,
    14,13,16,24,40, 57, 69, 56, 14,17,22,29,51, 87, 80, 62,
    18,22,37,56,68,109,103, 77, 24,35,55,64,81,104,113, 92,
    49,64,78,87,103,121,120,101,72,92,95,98,112,100,103, 99,
]
_CHR_QT = [
    17,18,24,47,99,99,99,99, 18,21,26,66,99,99,99,99,
    24,26,56,99,99,99,99,99, 47,66,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99,
]

def _make_dqt(tid, table):
    payload = bytearray([(0 << 4) | tid]) + bytearray(table)
    seg = bytearray(b"\xff\xdb"); seg += (len(payload)+2).to_bytes(2,"big"); seg += payload
    return bytes(seg)

def _make_sof0(w, h):
    comps = bytes([1,0x11,0, 2,0x11,1, 3,0x11,1])
    return (b"\xff\xc0" + (8+9).to_bytes(2,"big") + b"\x08" +
            h.to_bytes(2,"big") + w.to_bytes(2,"big") + b"\x03" + comps)

def _make_sos():
    payload = bytearray([3, 1,0x00, 2,0x11, 3,0x11, 0,63,0])
    return b"\xff\xda" + (len(payload)+2).to_bytes(2,"big") + bytes(payload)

def build_jpeg_header(w=640, h=360):
    return _SOI + _make_dqt(0,_LUM_QT) + _make_dqt(1,_CHR_QT) + _make_sof0(w,h) + _make_sos()


def build_control_packet(roll, pitch, throttle, yaw, command, headless, c1, c2, c3):
    b_c1 = c1.to_bytes(2, "little")
    b_c2 = c2.to_bytes(2, "little")
    b_c3 = c3.to_bytes(2, "little")
    controls = [
        roll & 0xFF,
        pitch & 0xFF,
        throttle & 0xFF,
        yaw & 0xFF,
        command & 0xFF,
        headless & 0xFF,
    ]
    chk = 0
    for b in controls:
        chk ^= b
    pkt = bytearray()
    pkt += _HDR
    pkt += b_c1 + _C1_SUFFIX
    pkt += bytes(controls)
    pkt += _CTRL_PAD
    pkt.append(chk)
    pkt += _CKSUM_SFX
    pkt += b_c2 + _C2_SUFFIX
    pkt += b_c3 + _C3_SUFFIX
    return bytes(pkt)


# ══════════════════════════════════════════════════════════════════════════════
#  MAHONY AHRS  — copy-paste identical to control_video_v6.py
# ══════════════════════════════════════════════════════════════════════════════

class MahonyFilter:
    def __init__(self, kp=3.5, ki=0.03):
        self.kp=kp; self.ki=ki; self.q=[1.,0.,0.,0.]; self._eI=[0.,0.,0.]
        self.gyro_bias=[0.,0.,0.]; self.bias_samples=[]; self.calibrated=False
        self._q_offset=[1.,0.,0.,0.]

    def add_gyro_sample(self, gx, gy, gz, n=150):
        if self.calibrated: return True
        self.bias_samples.append([gx,gy,gz])
        if len(self.bias_samples)>=n:
            k=len(self.bias_samples)
            self.gyro_bias=[sum(s[i] for s in self.bias_samples)/k for i in range(3)]
            self.calibrated=True; self.capture_offset(); return True
        return False

    def capture_offset(self):
        w,x,y,z=self.q; self._q_offset=[w,-x,-y,-z]

    def update(self, ax, ay, az, gx, gy, gz, dt):
        if self.calibrated:
            gx-=self.gyro_bias[0]; gy-=self.gyro_bias[1]; gz-=self.gyro_bias[2]
        q=self.q
        na=math.sqrt(ax*ax+ay*ay+az*az)
        if na==0.: return
        ax/=na; ay/=na; az/=na
        vx=2.*(q[1]*q[3]-q[0]*q[2]); vy=2.*(q[0]*q[1]+q[2]*q[3])
        vz=q[0]**2-q[1]**2-q[2]**2+q[3]**2
        ex=ay*vz-az*vy; ey=az*vx-ax*vz; ez=ax*vy-ay*vx
        self._eI[0]+=ex*self.ki*dt; self._eI[1]+=ey*self.ki*dt; self._eI[2]+=ez*self.ki*dt
        gx+=self.kp*ex+self._eI[0]; gy+=self.kp*ey+self._eI[1]; gz+=self.kp*ez+self._eI[2]
        hw=.5*dt; pa,pb,pc=q[0],q[1],q[2]
        q[0]+=(-q[1]*gx-q[2]*gy-q[3]*gz)*hw; q[1]+=(pa*gx+q[2]*gz-q[3]*gy)*hw
        q[2]+=(pa*gy-pb*gz+q[3]*gx)*hw;       q[3]+=(pa*gz+pb*gy-pc*gx)*hw
        n=math.sqrt(sum(v*v for v in q)); self.q=[v/n for v in q]

    def get_euler_relative(self):
        qo=self._q_offset; qa=self.q
        w=qo[0]*qa[0]-qo[1]*qa[1]-qo[2]*qa[2]-qo[3]*qa[3]
        x=qo[0]*qa[1]+qo[1]*qa[0]+qo[2]*qa[3]-qo[3]*qa[2]
        y=qo[0]*qa[2]-qo[1]*qa[3]+qo[2]*qa[0]+qo[3]*qa[1]
        z=qo[0]*qa[3]+qo[1]*qa[2]-qo[2]*qa[1]+qo[3]*qa[0]
        roll=math.atan2(2.*(w*x+y*z),1.-2.*(x*x+y*y))
        sinp=max(-1.,min(1.,2.*(w*y-z*x))); pitch=math.asin(sinp)
        yaw=math.atan2(2.*(w*z+x*y),1.-2.*(y*y+z*z))
        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


# ══════════════════════════════════════════════════════════════════════════════
#  IMU AXIS MAPPER  — copy-paste identical to control_video_v6.py
#  This is the exact same safe behaviour as the original.
# ══════════════════════════════════════════════════════════════════════════════

class IMUAxisMapper:
    MAX_ANGLE=45.; FLEX_REST_SAMPLES=80; FLEX_THRESH_STD=3.; FLEX_NORM_SCALE=150.; THR_NET_DEADZONE=.12

    def __init__(self):
        self.pr_deadzone=8.; self.pr_sensitivity=1.; self.pr_expo=.5
        self.yaw_deadzone=8.; self.yaw_sensitivity=2.; self.yaw_expo=.5
        self.flex_norm_scale=self.FLEX_NORM_SCALE
        self.thr_deadzone=self.THR_NET_DEADZONE
        self._flex_rest_buf=[[] for _ in range(4)]
        self._flex_rest_mean=[512.]*4; self._flex_rest_std=[20.]*4
        self._flex_calibrated=False
        self._throttle_smooth=float(STICK_MID); self._throttle_alpha=.12

    @property
    def deadzone(self): return self.pr_deadzone

    def add_flex_rest_sample(self, a0, a1, a2, a3):
        if self._flex_calibrated: return True
        for i,v in enumerate([a0,a1,a2,a3]): self._flex_rest_buf[i].append(v)
        if len(self._flex_rest_buf[2])>=self.FLEX_REST_SAMPLES:
            for i in range(4):
                buf=self._flex_rest_buf[i]; n=len(buf); mean=sum(buf)/n
                std=math.sqrt(sum((x-mean)**2 for x in buf)/n)
                self._flex_rest_mean[i]=mean; self._flex_rest_std[i]=max(std,5.)
            self._flex_calibrated=True; return True
        return False

    def reset_flex_calibration(self):
        self._flex_rest_buf=[[] for _ in range(4)]; self._flex_calibrated=False

    def _flex_def(self, raw, idx):
        delta=raw-self._flex_rest_mean[idx]; thresh=self.FLEX_THRESH_STD*self._flex_rest_std[idx]
        if abs(delta)<thresh: return 0.
        signed=delta-math.copysign(thresh,delta)
        return max(-1.,min(1.,signed/self.flex_norm_scale))

    def _a2s(self, angle, deadzone, sensitivity, expo):
        sign=1. if angle>=0 else -1.; mag=abs(angle)
        if mag<deadzone: return float(STICK_MID)
        norm=min(1.,(mag-deadzone)/(self.MAX_ANGLE-deadzone))
        norm=min(1.,norm*sensitivity); e=expo
        curved=max(0.,min(1.,norm*(1.-e)+norm**3*e))
        return STICK_MID+sign*curved*(STICK_MAX-STICK_MID)

    def compute(self, yaw, pitch, roll, a0, a1):
        sy=self._a2s(yaw,   self.yaw_deadzone, self.yaw_sensitivity, self.yaw_expo)
        sp=self._a2s(pitch, self.pr_deadzone,  self.pr_sensitivity,  self.pr_expo)
        sr=self._a2s(roll,  self.pr_deadzone,  self.pr_sensitivity,  self.pr_expo)
        if not self._flex_calibrated:
            st=self._throttle_smooth          # stays at STICK_MID — safe, no runaway
        else:
            d0=self._flex_def(a0,0); d1=self._flex_def(a1,1)
            net=max(-1.,min(1.,d0-d1)); e_t=self.pr_expo*.6
            s=1. if net>=0 else -1.; m=abs(net)
            dz=max(0.,min(.95,self.thr_deadzone))
            mapped=0. if m<=dz else min(1.,(m-dz)/(1.-dz))
            ct=mapped*(1.-e_t)+mapped**3*e_t
            raw=max(float(STICK_MIN),min(float(STICK_MAX),STICK_MID+s*ct*(STICK_MAX-STICK_MID)))
            self._throttle_smooth+=(raw-self._throttle_smooth)*self._throttle_alpha
            st=self._throttle_smooth
        return {"throttle":st,"yaw":sy,"pitch":sp,"roll":sr}


# ══════════════════════════════════════════════════════════════════════════════
#  GLOVE STATE  — lightweight container (replaces DroneState)
# ══════════════════════════════════════════════════════════════════════════════

class GloveState:
    def __init__(self):
        self._lock=threading.Lock()
        self.throttle=self.yaw=self.pitch=self.roll=float(STICK_MID)
    def set_imu(self, v):
        with self._lock:
            for k in ("throttle","yaw","pitch","roll"):
                setattr(self, k, max(STICK_MIN, min(STICK_MAX, v[k])))
    def snapshot(self):
        with self._lock:
            return {k:getattr(self,k) for k in ("throttle","yaw","pitch","roll")}


class PCFlightController:
    """Python UDP flight-control sender used in Video Mode."""

    def __init__(self, state: GloveState, log_q: queue.Queue):
        self.state = state
        self.log_q = log_q
        self.drone_ip = DEFAULT_IP
        self.drone_port = DEFAULT_PORT
        # Match control_video_v6.py default control cadence for responsiveness.
        self.rate = 80.0
        self.headless = False

        self._running = False
        self._thread = None
        self._sock = None
        self._sock_lock = threading.Lock()
        self._injected = False
        # Same mitigation as control_video_v6.py when sharing socket with video.
        self._video_tx_redundancy = 2

        self._c1 = 0
        self._c2 = 1
        self._c3 = 2
        self._cmd = CMD_NONE
        self._cmd_lock = threading.Lock()

    def set_target(self, ip: str, port: int):
        self.drone_ip = ip
        self.drone_port = port

    def set_headless(self, on: bool):
        self.headless = bool(on)

    def trigger_command(self, cmd: int):
        with self._cmd_lock:
            if cmd == CMD_STOP or self._cmd == CMD_NONE:
                self._cmd = cmd

    def inject_socket(self, sock: socket.socket):
        with self._sock_lock:
            if self._sock and not self._injected:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = sock
            self._injected = True

    def release_socket(self):
        with self._sock_lock:
            if self._injected:
                self._sock = None
                self._injected = False

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="PC-Flight")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._close_sock()

    def _consume_cmd(self):
        with self._cmd_lock:
            cmd = self._cmd
            self._cmd = CMD_NONE
            return cmd

    def _next_counters(self):
        c1, c2, c3 = self._c1, self._c2, self._c3
        self._c1 = (self._c1 + 1) & 0xFFFF
        self._c2 = (self._c2 + 1) & 0xFFFF
        self._c3 = (self._c3 + 1) & 0xFFFF
        return c1, c2, c3

    def _ensure_sock(self):
        with self._sock_lock:
            if self._sock is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                self._sock = s
                self._injected = False

    def _close_sock(self):
        with self._sock_lock:
            if self._sock and not self._injected:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            self._injected = False

    def _send(self, pkt: bytes):
        with self._sock_lock:
            if self._sock is None:
                return
            try:
                self._sock.sendto(pkt, (self.drone_ip, self.drone_port))
            except OSError:
                pass

    def _send_control(self, pkt: bytes):
        self._send(pkt)
        if self._injected and self._video_tx_redundancy > 1:
            for _ in range(self._video_tx_redundancy - 1):
                self._send(pkt)

    def _loop(self):
        interval = 1.0 / self.rate
        next_tick = time.perf_counter()
        try:
            while self._running:
                self._ensure_sock()
                snap = self.state.snapshot()
                cmd = self._consume_cmd()
                headless = HEADLESS_ON if self.headless else HEADLESS_OFF
                c1, c2, c3 = self._next_counters()
                pkt = build_control_packet(
                    int(snap["roll"]), int(snap["pitch"]),
                    int(snap["throttle"]), int(snap["yaw"]),
                    cmd, headless, c1, c2, c3,
                )
                self._send_control(pkt)

                next_tick += interval
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()
        finally:
            self._close_sock()


# ══════════════════════════════════════════════════════════════════════════════
#  GLOVE CONTROLLER  — copy-paste identical to control_video_v6.py
# ══════════════════════════════════════════════════════════════════════════════

class GloveController:
    CALIB_SAMPLES=80

    def __init__(self, state: GloveState, log_q: queue.Queue):
        self.state=state; self.log_q=log_q
        self.ahrs=MahonyFilter(); self.mapper=IMUAxisMapper()
        self._last_t=time.time()
        self.calibrating=True; self.calib_count=0; self.enabled=True
        self.yaw_deg=self.pitch_deg=self.roll_deg=0.
        self.a0_raw=self.a1_raw=self.a2_raw=self.a3_raw=self.throttle_pct=0.
        self.nn_position=-1
        self.nn_action_text=""
        self.nn_action_ts=0.0
        self.flex_calibrated=False; self.flex_rest_mean=[0.]*4

    def reset_calibration(self):
        self.ahrs=MahonyFilter(); self.mapper.reset_flex_calibration()
        self.calibrating=True; self.calib_count=0; self.flex_calibrated=False
        self._log("IMU: re-calibrating gyro + flex rest baseline…")

    def capture_zero(self):
        self.ahrs.capture_offset(); self._log("IMU: orientation zeroed ✓")

    def on_sensor_data(self, a0, a1, a2, a3, ax_r, ay_r, az_r, gx_r, gy_r, gz_r):
        # Axis remap — identical to original
        ax=ay_r; ay=-ax_r; az=az_r; gx=gy_r; gy=-gx_r; gz=gz_r
        gr=[math.radians(v) for v in (gx,gy,gz)]
        now=time.time(); dt=min(now-self._last_t,.05); self._last_t=now
        self.a0_raw=a0; self.a1_raw=a1; self.a2_raw=a2; self.a3_raw=a3
        if self.calibrating:
            gd=self.ahrs.add_gyro_sample(*gr, self.CALIB_SAMPLES)
            fd=self.mapper.add_flex_rest_sample(a0,a1,a2,a3)
            self.calib_count+=1
            if gd and fd:
                self.calibrating=False; self.flex_calibrated=True
                self.flex_rest_mean=list(self.mapper._flex_rest_mean)
                m=self.mapper._flex_rest_mean
                self._log(f"IMU calibrated ✓  A0={m[0]:.0f}  A1={m[1]:.0f}  — press O to zero.")
            return
        self.ahrs.update(ax,ay,az,*gr,dt)
        yaw,pitch,roll=self.ahrs.get_euler_relative()
        self.yaw_deg=yaw; self.pitch_deg=pitch; self.roll_deg=roll
        sticks=self.mapper.compute(yaw,pitch,roll,a0,a1)
        self.throttle_pct=(sticks["throttle"]-STICK_MID)/(STICK_MAX-STICK_MID)
        if self.enabled: self.state.set_imu(sticks)

    def _log(self, msg):
        ts=time.strftime("%H:%M:%S")
        try: self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full: pass


# ══════════════════════════════════════════════════════════════════════════════
#  SERIAL READER
#  Reads at the Arduino's full output rate (no sleep injected).
#  The original script also read as fast as in_waiting permits — same here.
#  send_command() writes a newline-terminated string back to the Arduino.
# ══════════════════════════════════════════════════════════════════════════════

class SerialReader:
    def __init__(self, port, baud, on_data, on_status, log_q):
        self.port=port; self.baud=baud; self.on_data=on_data
        self.on_status=on_status; self.log_q=log_q
        self._running=False; self._thread=None
        self._ser=None; self._ser_lock=threading.Lock()
        self._awaiting_recalib=False

    def start(self):
        if not SERIAL_AVAILABLE: self.on_status("ERROR: pyserial not installed"); return
        self._running=True
        self._thread=threading.Thread(target=self._loop, daemon=True, name="SerialRd")
        self._thread.start()

    def stop(self): self._running=False

    def send_command(self, cmd: str):
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    c = cmd.strip().upper()
                    if c == "R" or c == "RECALIBRATE":
                        self._awaiting_recalib = True
                    self._ser.write((cmd.strip()+"\n").encode("utf-8"))
                except Exception as e:
                    ts=time.strftime("%H:%M:%S")
                    try: self.log_q.put_nowait(f"[{ts}] Serial write error: {e}")
                    except queue.Full: pass

    def _loop(self):
        self.on_status(f"Connecting {self.port}…")
        try:
            ser=serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2); self.on_status(f"✓ {self.port} @ {self.baud}")
        except Exception as e:
            self.on_status(f"✗ {e}"); self._running=False; return
        with self._ser_lock: self._ser=ser
        # Read at full speed — identical strategy to the original SerialReader
        while self._running:
            try:
                if ser.in_waiting>0:
                    line=ser.readline().decode("utf-8",errors="ignore").strip()
                    if line: self._parse(line)
            except Exception as e:
                self.on_status(f"Read error: {e}"); time.sleep(0.5)
        with self._ser_lock: self._ser=None
        ser.close()

    def _parse(self, line):
        gc = self.on_data.__self__

        # Arduino NN action/status lines: keep GUI as passive observer.
        if "[NN]" in line:
            if "action" in line.lower():
                msg = line.replace("[NN]", "").strip()
                gc.nn_action_text = msg
                gc.nn_action_ts = time.time()
                gc._log(f"Arduino NN: {msg}")
            return

        # Mirror Arduino serial-monitor calibration lifecycle messages.
        if "[CALIB] Re-calibrating" in line or "Keep glove STILL" in line:
            gc.calibrating = True
            gc.calib_count = 0
            gc.flex_calibrated = False
            return

        if "[CALIB] Done" in line or "Flight control ACTIVE" in line:
            gc.calibrating = False
            gc.flex_calibrated = True
            gc.calib_count = GloveController.CALIB_SAMPLES
            self._awaiting_recalib = False
            return

        # Arduino telemetry format (already processed by Arduino AHRS):
        #   "Y:-2.3 P:0.1 R:5.6  T:128 A0:510 A1:508 A2:512 A3:498"
        # Parse each named field and inject directly into GloveController,
        # bypassing the Python-side AHRS/mapper pipeline entirely.
        import re
        try:
            # Compatibility path: accept CSV lines from other firmware variants.
            if line.startswith("TELEM,"):
                parts = line[6:].split(",")
                if len(parts) >= 11:
                    yaw = float(parts[0])
                    pitch = float(parts[1])
                    roll = float(parts[2])
                    thr = float(parts[3])
                    a0 = float(parts[4])
                    a1 = float(parts[5])
                    a2 = float(parts[9])
                    a3 = float(parts[10])
                    pos = int(float(parts[11])) if len(parts) >= 12 else None
                elif len(parts) >= 8:
                    yaw = float(parts[0])
                    pitch = float(parts[1])
                    roll = float(parts[2])
                    thr = float(parts[3])
                    a0 = float(parts[4])
                    a1 = float(parts[5])
                    a2 = float(parts[6])
                    a3 = float(parts[7])
                    pos = int(float(parts[8])) if len(parts) >= 9 else None
                else:
                    return
            else:
                def _get(tag):
                    m = re.search(rf"{tag}:\s*([+-]?\d+(?:\.\d+)?)", line)
                    return float(m.group(1)) if m else None

                yaw   = _get("Y")
                pitch = _get("P")
                roll  = _get("R")
                thr   = _get("T")
                a0    = _get("A0")
                a1    = _get("A1")
                a2    = _get("A2")
                a3    = _get("A3")
                pos_v = _get("POS")
                pos   = int(pos_v) if pos_v is not None else None

                if None in (yaw, pitch, roll, thr, a0, a1):
                    return  # not a telemetry line — silently ignore (boot messages etc.)
                if a2 is None:
                    a2 = gc.a2_raw
                if a3 is None:
                    a3 = gc.a3_raw

            # Inject directly into the GloveController state fields.
            # on_data is GloveController.on_sensor_data (a bound method),
            # so .__self__ gives us the GloveController instance.
            gc.yaw_deg   = yaw
            gc.pitch_deg = pitch
            gc.roll_deg  = roll
            gc.a0_raw    = a0
            gc.a1_raw    = a1
            gc.a2_raw    = a2
            gc.a3_raw    = a3
            if pos is not None:
                gc.nn_position = pos

            # throttle_pct: T is a stick byte [40-220], normalise to [-1, 1]
            gc.throttle_pct = (thr - STICK_MID) / (STICK_MAX - STICK_MID)

            # Mark calibration done as soon as we get real data
            if gc.calibrating and not self._awaiting_recalib:
                gc.calibrating       = False
                gc.flex_calibrated   = True
                gc.calib_count       = GloveController.CALIB_SAMPLES
                gc._log("Arduino telemetry live ✓  (Arduino-side AHRS active)")

            # Push stick values into shared GloveState so the stick displays update
            if gc.enabled:
                gc.state.set_imu({
                    "throttle": thr,
                    "yaw":   gc.mapper._a2s(yaw,   gc.mapper.yaw_deadzone, gc.mapper.yaw_sensitivity, gc.mapper.yaw_expo),
                    "pitch": gc.mapper._a2s(pitch, gc.mapper.pr_deadzone,  gc.mapper.pr_sensitivity,  gc.mapper.pr_expo),
                    "roll":  gc.mapper._a2s(roll,  gc.mapper.pr_deadzone,  gc.mapper.pr_sensitivity,  gc.mapper.pr_expo),
                })

        except Exception:
            pass  # never crash the serial thread


# ══════════════════════════════════════════════════════════════════════════════
#  VIDEO ADAPTER  — passive receive, identical to K417VideoAdapter in original
#  Only outbound packets: START_STREAM + REQUEST_A/B (not control packets).
# ══════════════════════════════════════════════════════════════════════════════

class K417VideoAdapter:
    HEADER_LEN=56; FRAME_TIMEOUT=0.08; MAX_RETRIES=3; WATCHDOG_SLEEP=0.05

    def __init__(self, drone_ip=DEFAULT_IP, port=DEFAULT_PORT, jpeg_width=640, jpeg_height=360):
        self.drone_ip=drone_ip; self.port=port
        self._jpeg_header=build_jpeg_header(jpeg_width, jpeg_height)
        self._sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288)
        # Ephemeral local port is more robust across restarts on Windows.
        self._sock.bind(("", 0)); self._sock.setblocking(False)
        self._current_fid=1; self._fragments={}; self._last_req_ts=time.time()
        self._retry_cnt=0; self.frames_ok=0; self.frames_dropped=0
        self._frame_q=queue.Queue(maxsize=1); self._running=True; self._first_frame=True
        threading.Thread(target=self._warmup_loop,   daemon=True, name="Vid-Warmup").start()
        threading.Thread(target=self._watchdog_loop, daemon=True, name="Vid-Watchdog").start()
        threading.Thread(target=self._rx_loop,       daemon=True, name="Vid-RX").start()

        # Same handshake packets as control_video_v6.py, but send a short
        # startup burst to improve first-lock reliability on lossy links.
        for _ in range(3):
            self._send_start()
            self._send_frame_request((self._current_fid - 1) & 0xFFFF)
            time.sleep(0.03)

    def get_frame(self, timeout=0):
        try: return self._frame_q.get(timeout=timeout) if timeout>0 else self._frame_q.get_nowait()
        except queue.Empty: return None

    def stop(self):
        self._running=False
        try: self._sock.close()
        except Exception: pass

    def _send_start(self):
        try: self._sock.sendto(START_STREAM,(self.drone_ip,self.port))
        except (OSError,BlockingIOError): pass

    def _send_frame_request(self, frame_id):
        lo,hi=frame_id&0xFF,(frame_id>>8)&0xFF
        rq_a=bytearray(REQUEST_A); rq_a[12]=lo; rq_a[13]=hi
        rq_b=bytearray(REQUEST_B)
        for base in (12,88,107): rq_b[base]=lo; rq_b[base+1]=hi
        try:
            self._sock.sendto(bytes(rq_a),(self.drone_ip,self.port))
            self._sock.sendto(bytes(rq_b),(self.drone_ip,self.port))
        except (OSError,BlockingIOError): pass
        self._last_req_ts=time.time()

    def _handle_payload(self, payload):
        if len(payload)<self.HEADER_LEN or payload[1]!=0x01: return
        frame_id=int.from_bytes(payload[16:18],"little")
        frag_id=int.from_bytes(payload[32:34],"little")
        last_frag=payload[2]!=0x38
        if frame_id!=self._current_fid:
            self.frames_dropped+=1; self._fragments.clear(); self._current_fid=frame_id
        self._fragments.setdefault(frag_id, payload[self.HEADER_LEN:])
        self._retry_cnt=0
        if not last_frag: return
        ordered=[self._fragments[i] for i in sorted(self._fragments)]
        jpeg=self._jpeg_header+b"".join(ordered)+_EOI
        self.frames_ok+=1; self._first_frame=False
        try: self._frame_q.get_nowait()
        except queue.Empty: pass
        try: self._frame_q.put_nowait(jpeg)
        except queue.Full: pass
        self._fragments.clear(); self._send_frame_request(frame_id)
        self._current_fid=(frame_id+1)&0xFFFF; self._last_req_ts=time.time()

    def _rx_loop(self):
        import select
        while self._running:
            try:
                r,_,_=select.select([self._sock],[],[],0.01)
                if r:
                    payload,_=self._sock.recvfrom(65535); self._handle_payload(payload)
            except (OSError,ValueError):
                if self._running: time.sleep(0.01)
                break

    def _warmup_loop(self):
        while self._running and self._first_frame:
            self._send_start(); self._send_frame_request((self._current_fid-1)&0xFFFF); time.sleep(0.2)

    def _watchdog_loop(self):
        while self._running:
            time.sleep(self.WATCHDOG_SLEEP)
            if time.time()-self._last_req_ts<self.FRAME_TIMEOUT: continue
            if self._retry_cnt<self.MAX_RETRIES:
                self._send_frame_request((self._current_fid-1)&0xFFFF); self._retry_cnt+=1
            else:
                self.frames_dropped+=1; self._fragments.clear()
                self._retry_cnt=0; self._current_fid=(self._current_fid+1)&0xFFFF
                self._send_start()
                self._send_frame_request((self._current_fid-1)&0xFFFF)


# ══════════════════════════════════════════════════════════════════════════════
#  VIDEO DISPLAY  — identical to _run_video_display() in original.
#  Hotkeys post serial commands to Arduino instead of calling GUI drone methods.
# ══════════════════════════════════════════════════════════════════════════════

def _run_video_display(adapter: K417VideoAdapter, gui: "K417GCS"):
    def _post(method_name, *args):
        try:
            m=getattr(gui, method_name, None)
            if m: gui.root.after(0, m, *args)
        except Exception: pass

    KEY_UP=2490368; KEY_DOWN=2621440; KEY_LEFT=2424832; KEY_RIGHT=2555904
    KEY_PGUP=2162688; KEY_PGDN=2228224

    font=cv2.FONT_HERSHEY_SIMPLEX
    placeholder=np.zeros((360,640,3),np.uint8)
    text="Waiting for K417 video\u2026"
    (tw,th),_=cv2.getTextSize(text,font,0.6,2)
    cv2.putText(placeholder,text,((640-tw)//2,(360+th)//2),font,0.6,(0,100,255),2)

    window="K417 Live View"
    cv2.namedWindow(window,cv2.WINDOW_NORMAL); cv2.resizeWindow(window,640,360)

    last_img=placeholder; dist_on=False
    fps_t=time.time(); fps_count=0

    while adapter._running:
        dist_est=getattr(gui,"_dist_est",None)
        jpeg=adapter.get_frame(timeout=0)
        if jpeg is not None:
            arr=np.frombuffer(jpeg,dtype=np.uint8)
            decoded=cv2.imdecode(arr,cv2.IMREAD_COLOR)
            if decoded is not None:
                if dist_est is not None and dist_on: dist_est.submit(decoded)
                last_img=decoded; fps_count+=1

        display_img=last_img
        if dist_est is not None and dist_on:
            if not dist_est.ready:
                display_img=last_img.copy()
                cv2.putText(display_img,"DIST EST: loading models…",
                            (8,display_img.shape[0]-8),font,0.45,(0,200,255),1,cv2.LINE_AA)
            else:
                res=dist_est.result
                if res.overlay is not None: display_img=res.overlay

        if dist_est is not None:
            col=(0,220,80) if (dist_on and dist_est.ready) else (0,180,255) if dist_on else (80,80,80)
            lbl="DIST ON  [D=off]" if dist_on else "DIST OFF  [D=on]"
            cv2.putText(display_img,lbl,(8,20),font,0.45,col,1,cv2.LINE_AA)

        cv2.imshow(window,display_img)

        if fps_count and fps_count%60==0:
            fps_t=time.time(); fps_count=0

        key=cv2.waitKeyEx(1); key_ascii=key&0xFF if key!=-1 else -1

        if key_ascii in (ord("q"),ord("Q")):
            adapter.stop(); break
        elif key_ascii in (ord("d"),ord("D")) and dist_est is not None:
            dist_on=not dist_on
        if key==-1: continue

        if   key_ascii in (ord("t"),ord("T")):  _post("_cmd_takeoff")
        elif key_ascii in (ord("l"),ord("L")):  _post("_cmd_land")
        elif key_ascii==32:                      _post("_cmd_stop")
        elif key_ascii in (ord("h"),ord("H")):  _post("_toggle_headless")
        elif key_ascii in (ord("c"),ord("C")):  _post("_cmd_calibrate")
        elif key_ascii in (ord("o"),ord("O")):  _post("_imu_zero")
        elif key in (KEY_PGUP,):                _post("_cmd_land")
        elif key in (KEY_PGDN,):                _post("_cmd_cam_down")

    cv2.destroyWindow(window)


# ══════════════════════════════════════════════════════════════════════════════
#  GUI STYLES  — identical to control_video_v6.py
# ══════════════════════════════════════════════════════════════════════════════

DARK_BG="#0b0d13"; PANEL_BG="#12151f"; CARD_BG="#181d2a"
ACCENT="#00e5ff"; ACCENT2="#ff4081"; ACCENT3="#69ff47"
TEXT_MAIN="#e0e6f0"; TEXT_DIM="#4a6070"
BTN_TAKE="#00c853"; BTN_LAND="#ff6d00"; BTN_STOP="#d50000"
BTN_HEAD="#7c4dff"; BTN_CAL="#0091ea"; IMU_COLOR="#b388ff"
FONT_MONO=("Courier New",10); FONT_LABEL=("Courier New",9,"bold")
FONT_BTN=("Courier New",10,"bold"); FONT_BIG=("Courier New",14,"bold")
FONT_TITLE=("Courier New",18,"bold"); FONT_SMALL=("Courier New",8)


# ══════════════════════════════════════════════════════════════════════════════
#  WIDGETS  — pixel-identical to control_video_v6.py
# ══════════════════════════════════════════════════════════════════════════════

class AttitudeIndicator(tk.Canvas):
    SIZE=120
    def __init__(self,parent,**kw):
        super().__init__(parent,width=self.SIZE,height=self.SIZE,
                         bg=CARD_BG,highlightthickness=1,highlightbackground=TEXT_DIM,**kw)
        self._pitch=self._roll=self._yaw=0.; self._draw()
    def update_attitude(self,pitch,roll,yaw):
        self._pitch=pitch; self._roll=roll; self._yaw=yaw; self._draw()
    def _draw(self):
        self.delete("all"); cx=cy=self.SIZE//2; r=cx-4
        pp=max(-r,min(r,self._pitch*(r/45.))); rr=math.radians(self._roll)
        ca,sa=math.cos(rr),math.sin(rr); ox,oy=-sa*pp,ca*pp
        self.create_oval(cx-r,cy-r,cx+r,cy+r,fill="#1a3a5c",outline="")
        pts=[]
        for i in range(37):
            th=math.pi*i/36; pts.extend([cx+r*math.cos(th+math.pi),cy+r*math.sin(th+math.pi)])
        dx=ca*r*1.5; dy=sa*r*1.5
        h1x=cx+ox+dx; h1y=cy+oy+dy; h2x=cx+ox-dx; h2y=cy+oy-dy
        try: self.create_polygon([h1x,h1y]+pts+[h2x,h2y],fill="#5c3a1a",outline="")
        except Exception: pass
        self.create_line(h1x,h1y,h2x,h2y,fill="white",width=2)
        self.create_oval(cx-r,cy-r,cx+r,cy+r,outline=ACCENT,width=2)
        self.create_line(cx-24,cy,cx-8,cy,fill=ACCENT,width=2)
        self.create_line(cx+8,cy,cx+24,cy,fill=ACCENT,width=2)
        self.create_oval(cx-4,cy-4,cx+4,cy+4,outline=ACCENT,width=2)
        ye=self._yaw%360
        if ye>0.5: self.create_arc(cx-r+8,cy-r+8,cx+r-8,cy+r-8,start=-90,extent=ye,
                                    outline=ACCENT2,width=1,style="arc")
        self.create_text(cx,6,text=f"Y {self._yaw:+.1f}°",fill=ACCENT2,font=FONT_SMALL)
        self.create_text(cx,self.SIZE-6,text=f"P {self._pitch:+.1f}°",fill=ACCENT3,font=FONT_SMALL)
        self.create_text(6,cy,text=f"R\n{self._roll:+.0f}°",fill=IMU_COLOR,font=FONT_SMALL)

class ThrottleBar(tk.Canvas):
    WIDTH=24; HEIGHT=120
    def __init__(self,parent,**kw):
        super().__init__(parent,width=self.WIDTH,height=self.HEIGHT,
                         bg=CARD_BG,highlightthickness=1,highlightbackground=TEXT_DIM,**kw)
        self._value=0.; self._draw()
    def set_value(self,v): self._value=max(-1.,min(1.,v)); self._draw()
    def _draw(self):
        self.delete("all"); w=self.WIDTH; h=self.HEIGHT; mid=h//2
        bh=int(abs(self._value)*mid); color=BTN_TAKE if self._value>=0 else ACCENT2
        if self._value>=0: self.create_rectangle(2,mid-bh,w-2,mid,fill=color,outline="")
        else:              self.create_rectangle(2,mid,w-2,mid+bh,fill=color,outline="")
        self.create_line(0,mid,w,mid,fill=TEXT_DIM,width=1)
        self.create_rectangle(1,1,w-2,h-2,outline=TEXT_DIM,width=1)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN GUI  — same layout as K417GUI; "DRONE CONNECTION" replaced by a
#  combined Arduino serial + drone IP panel (video receive address only).
# ══════════════════════════════════════════════════════════════════════════════

class K417GCS:
    def __init__(self, root):
        self.root=root
        self.state=GloveState()
        self.log_q=queue.Queue(maxsize=300)
        self.glove=GloveController(self.state, self.log_q)
        self.pc_ctrl=PCFlightController(self.state, self.log_q)
        self.serial: SerialReader|None=None
        self.video_adapter: K417VideoAdapter|None=None
        self._video_thread: threading.Thread|None=None
        self._dist_est=None
        self._headless_on=False
        self._video_mode=False
        self._last_nn_position_seen=None
        self._last_nn_action_seen_ts=0.0
        self._nn_cmd_until=0.0
        self._ip_var=tk.StringVar(value=DEFAULT_IP)
        self._port_var=tk.StringVar(value=str(DEFAULT_PORT))
        self._build_ui(); self._bind_keys(); self._tick()

    # ── panel helper ─────────────────────────────────────────────────────────
    def _panel(self,parent,title):
        outer=tk.Frame(parent,bg=DARK_BG); outer.pack(fill="x",pady=5)
        tk.Label(outer,text=f"  {title}  ",fg=ACCENT,bg=DARK_BG,
                 font=("Courier New",9,"bold")).pack(anchor="w")
        f=tk.Frame(outer,bg=PANEL_BG,padx=10,pady=8); f.pack(fill="x"); return f

    # ── full UI build ─────────────────────────────────────────────────────────
    def _build_ui(self):
        r=self.root; r.title("K417 // GCS  (Arduino controller)")
        r.configure(bg=DARK_BG); r.resizable(True,True)
        r.geometry("1160x860"); r.minsize(1040,760)
        try:
            # Windows/macOS maximize (windowed fullscreen-like behavior).
            r.state("zoomed")
        except Exception:
            # Fallback for platforms where "zoomed" is unsupported.
            pass
        ttk.Style().theme_use("clam")
        hdr=tk.Frame(r,bg=DARK_BG); hdr.pack(fill="x",padx=20,pady=(14,4))
        tk.Label(hdr,text="K417",fg=ACCENT,bg=DARK_BG,font=FONT_TITLE).pack(side="left")
        tk.Label(hdr,text="  //  GROUND CONTROL STATION",fg=TEXT_DIM,bg=DARK_BG,font=FONT_BIG).pack(side="left")
        self._status_label=tk.Label(hdr,text="● STOPPED",fg=ACCENT2,bg=DARK_BG,font=FONT_LABEL)
        self._status_label.pack(side="right")
        tk.Frame(r,height=1,bg=ACCENT).pack(fill="x",padx=20,pady=(0,8))
        cols=tk.Frame(r,bg=DARK_BG); cols.pack(fill="both",expand=True,padx=16)
        left=tk.Frame(cols,bg=DARK_BG); left.pack(side="left",fill="both")
        centre=tk.Frame(cols,bg=DARK_BG); centre.pack(side="left",fill="both",padx=10,expand=True)
        right=tk.Frame(cols,bg=DARK_BG); right.pack(side="right",fill="both")
        self._build_glove(left); self._build_keys_legend(left); self._build_nn_cmd_panel(left)
        self._build_imu(centre); self._build_sensitivity(centre); self._build_video(centre)
        self._build_sticks(right); self._build_commands(right)
        self._build_log(r)

    def _build_glove(self,parent):
        f=self._panel(parent,"ARDUINO  (Nano RP2040 Connect)")
        # Serial port row
        pr=tk.Frame(f,bg=PANEL_BG); pr.pack(fill="x",pady=2)
        tk.Label(pr,text="Serial port",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=11,anchor="w").pack(side="left")
        self._serial_port_var=tk.StringVar(value="COM3")
        tk.Entry(pr,textvariable=self._serial_port_var,width=10,bg=CARD_BG,fg=TEXT_MAIN,
                 insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=4)
        self._serial_status=tk.Label(pr,text="not connected",fg=ACCENT2,bg=PANEL_BG,font=FONT_SMALL)
        self._serial_status.pack(side="left",padx=6)
        # Baud row
        br2=tk.Frame(f,bg=PANEL_BG); br2.pack(fill="x",pady=2)
        tk.Label(br2,text="Baud rate",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=11,anchor="w").pack(side="left")
        self._baud_var=tk.StringVar(value="115200")
        tk.Entry(br2,textvariable=self._baud_var,width=10,bg=CARD_BG,fg=TEXT_MAIN,
                 insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=4)
        # Drone IP row (video target + Video Mode UDP target)
        vr=tk.Frame(f,bg=PANEL_BG); vr.pack(fill="x",pady=2)
        tk.Label(vr,text="Drone IP",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=11,anchor="w").pack(side="left")
        tk.Entry(vr,textvariable=self._ip_var,width=16,bg=CARD_BG,fg=TEXT_MAIN,
                 insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=4)
        tk.Label(vr,text="Port",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL).pack(side="left",padx=(8,2))
        tk.Entry(vr,textvariable=self._port_var,width=6,bg=CARD_BG,fg=TEXT_MAIN,
                 insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=2)
        # Buttons
        btn_r=tk.Frame(f,bg=PANEL_BG); btn_r.pack(fill="x",pady=(6,2))
        tk.Button(btn_r,text="CONNECT GLOVE",bg="#1b5e20",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._connect_glove).pack(side="left",padx=2)
        tk.Button(btn_r,text="ZERO  [O]",bg=IMU_COLOR,fg=DARK_BG,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._imu_zero).pack(side="left",padx=2)
        btn_r2=tk.Frame(f,bg=PANEL_BG); btn_r2.pack(fill="x",pady=2)
        tk.Button(btn_r2,text="RE-CALIBRATE [F5]",bg="#4a148c",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._imu_recalib).pack(side="left",padx=2)
        self._glove_toggle_btn=tk.Button(btn_r2,text="IMU  ENABLED",bg=ACCENT3,fg=DARK_BG,
                  font=FONT_BTN,relief="flat",cursor="hand2",command=self._toggle_imu)
        self._glove_toggle_btn.pack(side="left",padx=2)

        mode_row=tk.Frame(f,bg=PANEL_BG); mode_row.pack(fill="x",pady=(4,2))
        self._mode_btn=tk.Button(mode_row,text="MODE: ARDUINO CTRL",bg="#455a64",fg=TEXT_MAIN,
              font=FONT_BTN,relief="flat",cursor="hand2",command=self._toggle_control_mode)
        self._mode_btn.pack(side="left",padx=2)
        self._mode_lbl=tk.Label(mode_row,text="Serial commands → Arduino UDP",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL)
        self._mode_lbl.pack(side="left",padx=8)
        # Calibration progress canvas
        cf=tk.Frame(f,bg=PANEL_BG); cf.pack(fill="x",pady=4)
        tk.Label(cf,text="Gyro bias cal:",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL).pack(side="left")
        self._calib_canvas=tk.Canvas(cf,width=140,height=10,bg=CARD_BG,highlightthickness=0)
        self._calib_canvas.pack(side="left",padx=4)
        self._calib_label=tk.Label(cf,text="0 / 150",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL)
        self._calib_label.pack(side="left")

    def _build_imu(self,parent):
        f=self._panel(parent,"IMU  ATTITUDE")
        top=tk.Frame(f,bg=PANEL_BG); top.pack(fill="x")
        self._ahi=AttitudeIndicator(top); self._ahi.pack(side="left",padx=(0,12))
        tf=tk.Frame(top,bg=PANEL_BG); tf.pack(side="left")
        tk.Label(tf,text="THR",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL).pack()
        self._thr_bar=ThrottleBar(tf); self._thr_bar.pack()
        nums=tk.Frame(top,bg=PANEL_BG); nums.pack(side="left",padx=8)
        self._imu_vars={}
        for label,key,color in [("YAW","yaw",ACCENT2),("PITCH","pitch",ACCENT3),
                                  ("ROLL","roll",IMU_COLOR),("A0","a0",BTN_TAKE),("A1","a1",ACCENT2),
                                  ("A2","a2",ACCENT3),("A3","a3",IMU_COLOR)]:
            rw=tk.Frame(nums,bg=PANEL_BG); rw.pack(fill="x",pady=1)
            tk.Label(rw,text=f"{label:5s}",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=6,anchor="w").pack(side="left")
            var=tk.StringVar(value="—"); self._imu_vars[key]=var
            tk.Label(rw,textvariable=var,fg=color,bg=PANEL_BG,font=FONT_MONO,width=8,anchor="e").pack(side="left")
        self._calib_status=tk.Label(f,text="● Calibrating gyro bias…",fg=ACCENT2,bg=PANEL_BG,font=FONT_LABEL)
        self._calib_status.pack(pady=(6,2))

    def _build_sensitivity(self,parent):
        f=self._panel(parent,"IMU  SENSITIVITY")
        def param(label,from_,to,initial,setter,res=0.05,color=ACCENT):
            rw=tk.Frame(f,bg=PANEL_BG); rw.pack(fill="x",pady=2)
            tk.Label(rw,text=label,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=16,anchor="w").pack(side="left")
            var=tk.DoubleVar(value=initial)
            disp=tk.Label(rw,text=f"{initial:.2f}",fg=color,bg=PANEL_BG,font=FONT_MONO,width=5)
            disp.pack(side="right")
            def on(*_,s=setter,d=disp,v=var):
                val=round(v.get(),3); s(val); d.config(text=f"{val:.2f}")
            tk.Scale(rw,variable=var,from_=from_,to=to,orient="horizontal",resolution=res,length=170,
                     bg=PANEL_BG,fg=TEXT_MAIN,troughcolor=CARD_BG,highlightthickness=0,
                     activebackground=color,showvalue=False,command=on).pack(side="left",padx=4)
        tk.Label(f,text="PITCH  &  ROLL",fg=ACCENT3,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(4,0))
        param("Deadzone (°)",0.,30.,8., lambda v:setattr(self.glove.mapper,"pr_deadzone",v),  res=0.5,color=ACCENT3)
        param("Sensitivity", 0.1,3.,1., lambda v:setattr(self.glove.mapper,"pr_sensitivity",v),       color=ACCENT3)
        param("Expo curve",  0.,1.,.5,  lambda v:setattr(self.glove.mapper,"pr_expo",v),               color=ACCENT3)
        tk.Label(f,text="YAW",fg=ACCENT2,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(6,0))
        param("Deadzone (°)",0.,30.,8., lambda v:setattr(self.glove.mapper,"yaw_deadzone",v), res=0.5,color=ACCENT2)
        param("Sensitivity", 0.1,3.,1.3, lambda v:setattr(self.glove.mapper,"yaw_sensitivity",v),      color=ACCENT2)
        param("Expo curve",  0.,1.,.5,  lambda v:setattr(self.glove.mapper,"yaw_expo",v),              color=ACCENT2)
        tk.Label(f,text="THROTTLE",fg=BTN_TAKE,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(6,0))
        param("Deadzone",    0.,1,IMUAxisMapper.THR_NET_DEADZONE,
              lambda v:setattr(self.glove.mapper,"thr_deadzone",v),res=0.01,color=BTN_TAKE)
        param("Smoothing",   .02,.5,.12,lambda v:setattr(self.glove.mapper,"_throttle_alpha",v),res=0.01,color=BTN_TAKE)
        param("Flex scale",  20.,400.,IMUAxisMapper.FLEX_NORM_SCALE,
              lambda v:setattr(self.glove.mapper,"flex_norm_scale",v),res=5.,color=BTN_TAKE)

    def _build_video(self,parent):
        f=self._panel(parent,"CAMERA  &  DISTANCE")
        br=tk.Frame(f,bg=PANEL_BG); br.pack(fill="x",pady=(0,6))
        self._video_btn=tk.Button(br,text="▶  START VIDEO",bg="#005f73",fg=TEXT_MAIN,
                                   font=FONT_BTN,relief="flat",cursor="hand2",command=self._toggle_video)
        self._video_btn.pack(side="left",padx=2)
        if DIST_EST_AVAILABLE:
            self._dist_btn=tk.Button(br,text="◎  DIST EST",bg="#1a237e",fg=TEXT_MAIN,
                                     font=FONT_BTN,relief="flat",cursor="hand2",
                                     command=self._toggle_dist_est)
            self._dist_btn.pack(side="left",padx=2)
        else:
            self._dist_btn=None
        if not CV2_AVAILABLE:
            tk.Label(br,text="pip install opencv-python numpy  (required for video)",
                     fg=ACCENT2,bg=PANEL_BG,font=FONT_SMALL).pack(side="left",padx=8)

    def _build_commands(self,parent):
        f=self._panel(parent,"COMMANDS"); f.columnconfigure(0,weight=1); f.columnconfigure(1,weight=1)
        def btn(text,color,cmd,row,col,span=1):
            b=tk.Button(f,text=text,bg=color,fg="white",activebackground=color,
                        font=FONT_BTN,relief="flat",cursor="hand2",width=12,height=1,command=cmd)
            b.grid(row=row,column=col,padx=3,pady=3,columnspan=span); return b
        btn("⬆  TAKEOFF",BTN_TAKE,self._cmd_takeoff,0,0)
        btn("⬇  LAND",BTN_LAND,self._cmd_land,0,1)
        self._btn_stop=btn("✕  STOP",BTN_STOP,self._cmd_stop,1,0)
        self._btn_head=btn("⧖  HEADLESS",BTN_HEAD,self._toggle_headless,1,1)
        btn("◎  CALIBRATE",BTN_CAL,self._cmd_calibrate,2,0)
        tk.Frame(f,height=1,bg=TEXT_DIM).grid(row=3,column=0,columnspan=2,sticky="ew",pady=(6,2))
        tk.Label(f,text="CAMERA  [PgDn]",fg=TEXT_DIM,bg=PANEL_BG,
                 font=FONT_LABEL).grid(row=4,column=0,columnspan=2,sticky="w",padx=4)
        btn("▼  CAM DOWN","#006064",self._cmd_cam_down,5,0,span=2)
        tk.Frame(f,height=1,bg=TEXT_DIM).grid(row=6,column=0,columnspan=2,sticky="ew",pady=(6,2))
        tk.Label(f,text="FLIPS  — must be airborne",fg=TEXT_DIM,bg=PANEL_BG,
                 font=FONT_LABEL).grid(row=7,column=0,columnspan=2,sticky="w",padx=4)
        btn("↑  FLIP FWD", "#4a148c",lambda:self._cmd_flip("forward"), 8,0)
        btn("↓  FLIP BACK","#4a148c",lambda:self._cmd_flip("backward"),8,1)
        btn("←  FLIP LEFT","#4a148c",lambda:self._cmd_flip("left"),    9,0)
        btn("→  FLIP RIGHT","#4a148c",lambda:self._cmd_flip("right"),  9,1)

    def _build_keys_legend(self,parent):
        f=self._panel(parent,"KEYBOARD  OVERRIDES")
        items=[("T","Takeoff"),("L","Land (cam-up sequence)"),("SPACE","Emergency stop"),
               ("H","Headless"),("C","Calibrate"),("O","Zero IMU"),
               ("F5","Re-calibrate"),("PgUp","Land"),("PgDn","Cam tilt down")]

        rows_per_col = (len(items) + 1) // 2
        for i,(key,desc) in enumerate(items):
            rr = i % rows_per_col
            cc = (i // rows_per_col) * 2
            tk.Label(f,text=key,fg=ACCENT,bg=PANEL_BG,font=FONT_MONO,width=7,anchor="e").grid(
                row=rr,column=cc,padx=(0,4),pady=1,sticky="e")
            tk.Label(f,text=desc,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,anchor="w").grid(
                row=rr,column=cc+1,padx=(0,10),pady=1,sticky="w")

    def _build_nn_cmd_panel(self,parent):
        f=self._panel(parent,"ARDUINO  NN  COMMAND")
        self._nn_pos_var=tk.StringVar(value="POS: -1")
        tk.Label(
            f,
            textvariable=self._nn_pos_var,
            fg=ACCENT,
            bg=PANEL_BG,
            font=("Courier New",13,"bold"),
            anchor="w",
            justify="left",
        ).pack(fill="x",pady=(0,4))
        self._nn_cmd_var=tk.StringVar(value="NN CMD: none")
        self._nn_cmd_card=tk.Frame(f,bg=CARD_BG,highlightthickness=2,highlightbackground=TEXT_DIM)
        self._nn_cmd_card.pack(fill="x",pady=(2,2))
        self._nn_cmd_label=tk.Label(
            self._nn_cmd_card,
            textvariable=self._nn_cmd_var,
            fg=TEXT_DIM,
            bg=CARD_BG,
            font=("Courier New",15,"bold"),
            justify="center",
            pady=10,
            wraplength=320,
        )
        self._nn_cmd_label.pack(fill="x")
        tk.Label(
            f,
            text="Display-only: command is sent by Arduino firmware",
            fg=TEXT_DIM,
            bg=PANEL_BG,
            font=FONT_SMALL,
        ).pack(anchor="w",pady=(4,0))

    def _build_sticks(self,parent):
        f=self._panel(parent,"LIVE  STICKS")
        self._stick_vars={}; self._stick_val_lbl={}
        for i,(name,label) in enumerate([("throttle","THROTTLE"),("yaw","YAW"),
                                          ("pitch","PITCH"),("roll","ROLL")]):
            tk.Label(f,text=label,fg=ACCENT,bg=PANEL_BG,font=FONT_LABEL,width=9,anchor="w").grid(
                row=i,column=0,padx=6,pady=5,sticky="w")
            var=tk.DoubleVar(value=STICK_MID); self._stick_vars[name]=var
            vl=tk.Label(f,text="128",fg=TEXT_MAIN,bg=PANEL_BG,font=FONT_MONO,width=4)
            vl.grid(row=i,column=2,padx=6); self._stick_val_lbl[name]=vl
            tk.Scale(f,variable=var,from_=STICK_MIN,to=STICK_MAX,orient="horizontal",
                     resolution=1,length=320,bg=PANEL_BG,fg=TEXT_MAIN,troughcolor=CARD_BG,
                     highlightthickness=0,activebackground=ACCENT,showvalue=False,
                     state="disabled").grid(row=i,column=1,padx=6,pady=5)
        bf=tk.Frame(f,bg=PANEL_BG); bf.grid(row=4,column=0,columnspan=3,sticky="ew",padx=6,pady=4)
        tk.Label(bf,text="IMU ANGLE BARS",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL).pack(anchor="w")
        bi=tk.Frame(bf,bg=PANEL_BG); bi.pack(fill="x")
        self._angle_bars={}
        for axis,color in [("YAW",ACCENT2),("PITCH",ACCENT3),("ROLL",IMU_COLOR)]:
            r2=tk.Frame(bi,bg=PANEL_BG); r2.pack(fill="x",pady=1)
            tk.Label(r2,text=axis,fg=color,bg=PANEL_BG,font=FONT_SMALL,width=6,anchor="w").pack(side="left")
            c=tk.Canvas(r2,width=260,height=10,bg=CARD_BG,highlightthickness=0)
            c.pack(side="left",padx=2); self._angle_bars[axis]=c

    def _build_log(self,parent):
        tk.Frame(parent,height=1,bg=TEXT_DIM).pack(fill="x",padx=20,pady=(6,0))
        lf=tk.Frame(parent,bg=DARK_BG); lf.pack(fill="both",expand=True,padx=20,pady=(4,10))
        tk.Label(lf,text="EVENT LOG",fg=TEXT_DIM,bg=DARK_BG,font=FONT_LABEL).pack(anchor="w")
        self._log_text=scrolledtext.ScrolledText(lf,height=5,bg=CARD_BG,fg="#37ff8b",
            font=("Courier New",8),relief="flat",state="disabled",wrap="none")
        self._log_text.pack(fill="both",expand=True)
        tk.Button(lf,text="Clear",bg=PANEL_BG,fg=TEXT_DIM,font=FONT_LABEL,relief="flat",
                  cursor="hand2",command=self._clear_log).pack(side="right",pady=3)

    # ── key bindings ─────────────────────────────────────────────────────────
    def _bind_keys(self):
        self.root.bind("<KeyPress>", self._on_key)

    def _on_key(self,event):
        k=event.keysym
        if   k=="t":         self._cmd_takeoff()
        elif k=="l":         self._cmd_land()
        elif k=="space":     self._cmd_stop()
        elif k=="h":         self._toggle_headless()
        elif k=="c":         self._cmd_calibrate()
        elif k.lower()=="o": self._imu_zero()
        elif k=="F5":        self._imu_recalib()
        elif k=="Prior":     self._cmd_land()
        elif k=="Next":      self._cmd_cam_down()

    # ── glove / serial ───────────────────────────────────────────────────────
    def _connect_glove(self):
        if self.serial: self.serial.stop()
        port=self._serial_port_var.get().strip()
        try:    baud=int(self._baud_var.get())
        except: baud=115200
        self.serial=SerialReader(port,baud,self.glove.on_sensor_data,
                                 self._on_serial_status,self.log_q)
        self.serial.start()

    def _target_ip_port(self):
        ip=self._ip_var.get().strip()
        try:    port=int(self._port_var.get())
        except: port=DEFAULT_PORT
        return ip, port

    def _toggle_control_mode(self):
        self._video_mode = not self._video_mode
        ip, port = self._target_ip_port()
        self.pc_ctrl.set_target(ip, port)
        self.pc_ctrl.set_headless(self._headless_on)
        if self._video_mode:
            if not self.serial:
                self._log_event("⚠  Serial not connected: Arduino UDP may still be active")
            self._send_serial("P")
            if self.video_adapter is not None:
                self.pc_ctrl.inject_socket(self.video_adapter._sock)
            self.pc_ctrl.start()
            self._mode_btn.config(text="MODE: VIDEO", bg="#1b5e20")
            self._mode_lbl.config(text="Python UDP control + shared video socket", fg=ACCENT3)
            self._status_label.config(text="● VIDEO MODE", fg=ACCENT3)
            self._log_event("MODE: VIDEO (Python UDP control)")
        else:
            self._send_serial("A")
            self.pc_ctrl.release_socket()
            self.pc_ctrl.stop()
            self._mode_btn.config(text="MODE: ARDUINO CTRL", bg="#455a64")
            self._mode_lbl.config(text="Serial commands → Arduino UDP", fg=TEXT_DIM)
            self._status_label.config(text="● STOPPED", fg=ACCENT2)
            self._log_event("MODE: ARDUINO CTRL")

    def _on_serial_status(self,msg):
        self.root.after(0,lambda:self._serial_status.config(
            text=msg, fg=ACCENT3 if "✓" in msg else ACCENT2))
        self._log_event(f"SERIAL: {msg}")

    def _imu_zero(self):
        # Live attitude comes from Arduino telemetry, so zero on Arduino.
        self._send_serial("O")
        self._log_event("IMU: zero requested (Arduino)")
    def _imu_recalib(self):
        # Recalibration must happen on Arduino side to match serial monitor behavior.
        self.glove.calibrating = True
        self.glove.calib_count = 0
        self.glove.flex_calibrated = False
        self._send_serial("R")
        self._log_event("IMU: re-calibrating (Arduino)")
    def _toggle_imu(self):
        self.glove.enabled=not self.glove.enabled
        self._glove_toggle_btn.config(
            text="IMU  ENABLED" if self.glove.enabled else "IMU  PAUSED",
            bg=ACCENT3 if self.glove.enabled else ACCENT2,
            fg=DARK_BG  if self.glove.enabled else "white")
        self._log_event(f"IMU: {'ON' if self.glove.enabled else 'PAUSED'}")

    # ── commands → serial → Arduino → drone ──────────────────────────────────
    def _send_serial(self, cmd: str):
        if self.serial:
            self.serial.send_command(cmd)
            self._log_event(f"→ Arduino: {cmd}")
        else:
            self._log_event("⚠  Serial not connected — command dropped")

    def _cmd_takeoff(self):
        if self._video_mode:
            self.pc_ctrl.trigger_command(CMD_TAKEOFF)
            self._log_event("CMD: TAKEOFF (Python UDP)")
        else:
            self._send_serial("T")
    def _cmd_stop(self):
        if self._video_mode:
            self.pc_ctrl.trigger_command(CMD_STOP)
            self._log_event("CMD: EMERGENCY STOP (Python UDP)")
        else:
            self._send_serial("X"); self._log_event("CMD: EMERGENCY STOP")
    def _cmd_calibrate(self):
        if self._video_mode:
            self.pc_ctrl.trigger_command(CMD_CALIBRATE)
            self._log_event("CMD: CALIBRATE (Python UDP)")
        else:
            self._send_serial("C")
    def _cmd_land(self):
        if self._video_mode:
            self.pc_ctrl.trigger_command(CMD_LAND)
            self._log_event("CMD: LAND (Python UDP)")
        else:
            self._send_serial("L"); self._log_event("CMD: LAND (cam-up sequence)")
    def _cmd_cam_down(self):
        if self._video_mode:
            self.pc_ctrl.trigger_command(CMD_CAM_DOWN)
            self._log_event("CMD: CAM TILT DOWN (Python UDP)")
        else:
            self._send_serial("D"); self._log_event("CMD: CAM TILT DOWN")
    def _cmd_flip(self, direction: str):
        if self._video_mode:
            self._log_event("FLIP: not available in VIDEO mode yet")
        else:
            self._send_serial(f"FLIP:{direction}"); self._log_event(f"CMD: FLIP {direction.upper()}")

    def _toggle_headless(self):
        self._headless_on=not self._headless_on
        if self._video_mode:
            self.pc_ctrl.set_headless(self._headless_on)
        else:
            self._send_serial("H")
        self._btn_head.config(bg=ACCENT if self._headless_on else BTN_HEAD,
                              fg=DARK_BG if self._headless_on else "white")
        self._log_event(f"HEADLESS: {'ON' if self._headless_on else 'OFF'}")

    # ── video ─────────────────────────────────────────────────────────────────
    def _toggle_video(self):
        if self.video_adapter is not None:
            self.video_adapter.stop(); self.video_adapter=None; self._video_thread=None
            self.pc_ctrl.release_socket()
            self._video_btn.config(text="▶  START VIDEO",bg="#005f73")
            if self._video_mode:
                self._status_label.config(text="● VIDEO MODE",fg=ACCENT3)
            else:
                self._status_label.config(text="● STOPPED",fg=ACCENT2)
            self._log_event("VIDEO: stopped")
        else:
            if not CV2_AVAILABLE:
                self._log_event("VIDEO: ERROR — pip install opencv-python numpy"); return
            ip=self._ip_var.get().strip()
            try:    port=int(self._port_var.get())
            except: port=DEFAULT_PORT

            if self._video_mode:
                # Re-assert Arduino UDP pause when starting stream.
                self._send_serial("P")
                self.pc_ctrl.set_target(ip, port)
                # Mirror control_video_v6.py clean handoff sequence.
                if self.pc_ctrl._running:
                    self.pc_ctrl.stop()
                self.pc_ctrl.release_socket()

            self.video_adapter=K417VideoAdapter(drone_ip=ip,port=port)

            if self._video_mode:
                self.pc_ctrl.inject_socket(self.video_adapter._sock)
                self.pc_ctrl.start()

            gui_ref=self
            def _dt(gui=gui_ref):
                _run_video_display(self.video_adapter, gui)
                self.root.after(0,self._on_video_closed)
            self._video_thread=threading.Thread(target=_dt,daemon=True,name="VideoDisplay")
            self._video_thread.start()
            self._video_btn.config(text="■  STOP VIDEO",bg=BTN_STOP)
            self._status_label.config(text="● VIDEO ACTIVE",fg=BTN_TAKE)
            self._log_event(f"VIDEO: started → {ip}:{port}")

    def _on_video_closed(self):
        self.pc_ctrl.release_socket()
        if self._video_mode and not self.pc_ctrl._running:
            self.pc_ctrl.start()
        self.video_adapter=None; self._video_thread=None
        self._video_btn.config(text="▶  START VIDEO",bg="#005f73")
        if self._video_mode:
            self._status_label.config(text="● VIDEO MODE",fg=ACCENT3)
        else:
            self._status_label.config(text="● STOPPED",fg=ACCENT2)
        self._log_event("VIDEO: window closed")

    def _toggle_dist_est(self):
        if not DIST_EST_AVAILABLE or self._dist_btn is None: return
        if self._dist_est is not None:
            self._dist_est.stop(); self._dist_est=None
            self._dist_btn.config(text="◎  DIST EST",bg="#1a237e")
            self._log_event("DIST EST: stopped")
        else:
            self._dist_est=AsyncDistanceEstimator(use_yolo=_DE_YOLO_AVAILABLE,draw_overlay=True)
            self._dist_est.start()
            self._dist_btn.config(text="■  DIST EST ON",bg=BTN_STOP)
            self._log_event("DIST EST: started (MiDaS loading in background)")
            self._log_event("Press  D  in the video window to toggle overlay")

    # ── log ───────────────────────────────────────────────────────────────────
    def _log_event(self,msg):
        ts=time.strftime("%H:%M:%S")
        try: self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full: pass

    def _set_nn_cmd_status(self, msg: str, color: str = ACCENT3, hold_s: float = 2.5):
        self._nn_cmd_var.set(msg)
        self._nn_cmd_label.config(fg=color)
        self._nn_cmd_card.config(highlightbackground=color)
        self._nn_cmd_until = time.time() + hold_s

    def _clear_log(self):
        self._log_text.config(state="normal"); self._log_text.delete("1.0","end")
        self._log_text.config(state="disabled")

    # ── tick — 40 ms = 25 Hz, identical to original _tick() ──────────────────
    def _tick(self):
        msgs=[]
        try:
            while True: msgs.append(self.log_q.get_nowait())
        except queue.Empty: pass
        if msgs:
            self._log_text.config(state="normal")
            for m in msgs: self._log_text.insert("end",m+"\n")
            self._log_text.see("end"); self._log_text.config(state="disabled")

        snap=self.state.snapshot()
        for name in ("throttle","yaw","pitch","roll"):
            self._stick_vars[name].set(snap[name])
            self._stick_val_lbl[name].config(text=str(int(snap[name])))

        g=self.glove
        self._imu_vars["yaw"].set(f"{g.yaw_deg:+7.1f}°")
        self._imu_vars["pitch"].set(f"{g.pitch_deg:+7.1f}°")
        self._imu_vars["roll"].set(f"{g.roll_deg:+7.1f}°")
        self._imu_vars["a0"].set(f"{g.a0_raw:7.0f}")
        self._imu_vars["a1"].set(f"{g.a1_raw:7.0f}")
        self._imu_vars["a2"].set(f"{g.a2_raw:7.0f}")
        self._imu_vars["a3"].set(f"{g.a3_raw:7.0f}")
        self._nn_pos_var.set(f"POS: {int(g.nn_position)}")
        self._ahi.update_attitude(g.pitch_deg, g.roll_deg, g.yaw_deg)
        self._thr_bar.set_value(g.throttle_pct)

        # Position display is passive: Arduino owns NN command execution.
        current_pos = int(g.nn_position)
        if current_pos != self._last_nn_position_seen:
            self._last_nn_position_seen = current_pos
            self._log_event(f"NN position: {current_pos}")

        if g.nn_action_ts > self._last_nn_action_seen_ts:
            self._last_nn_action_seen_ts = g.nn_action_ts
            txt = g.nn_action_text.strip() if g.nn_action_text else "NN action"
            color = ACCENT
            upper_txt = txt.upper()
            if "TAKEOFF" in upper_txt:
                color = BTN_TAKE
            elif "LAND" in upper_txt:
                color = BTN_LAND
            elif "STOP" in upper_txt:
                color = BTN_STOP
            elif "ZERO" in upper_txt:
                color = IMU_COLOR
            self._set_nn_cmd_status(f"ARDUINO NN CMD: {txt}", color)

        if self._nn_cmd_until and time.time() > self._nn_cmd_until:
            self._nn_cmd_var.set("NN CMD: none")
            self._nn_cmd_label.config(fg=TEXT_DIM)
            self._nn_cmd_card.config(highlightbackground=TEXT_DIM)
            self._nn_cmd_until = 0.0

        for axis,val,rng in [("YAW",g.yaw_deg,180.),("PITCH",g.pitch_deg,90.),("ROLL",g.roll_deg,90.)]:
            c=self._angle_bars[axis]; c.delete("all")
            w,h=260,10; mid=w//2
            norm=max(-1.,min(1.,val/rng)); bar=int(abs(norm)*(w//2))
            color=ACCENT2 if norm<0 else ACCENT3
            if norm>=0: c.create_rectangle(mid,1,mid+bar,h-1,fill=color,outline="")
            else:       c.create_rectangle(mid-bar,1,mid,h-1,fill=color,outline="")
            dz=(g.mapper.yaw_deadzone if axis=="YAW" else g.mapper.pr_deadzone)
            dz_px=int((dz/rng)*(w//2))
            c.create_line(mid-dz_px,0,mid-dz_px,h,fill=TEXT_DIM,width=1)
            c.create_line(mid+dz_px,0,mid+dz_px,h,fill=TEXT_DIM,width=1)
            c.create_line(mid,0,mid,h,fill=TEXT_DIM,width=1)

        if g.calibrating:
            pct=min(1.,g.calib_count/GloveController.CALIB_SAMPLES)
            self._calib_canvas.delete("all")
            self._calib_canvas.create_rectangle(0,0,int(140*pct),10,fill=ACCENT2,outline="")
            self._calib_label.config(text=f"{g.calib_count} / {GloveController.CALIB_SAMPLES}")
            self._calib_status.config(text="● Calibrating gyro + flex — hold STILL",fg=ACCENT2)
        else:
            self._calib_canvas.delete("all")
            self._calib_canvas.create_rectangle(0,0,140,10,fill=ACCENT3,outline="")
            if g.flex_calibrated:
                m=g.mapper._flex_rest_mean
                self._calib_label.config(text=f"DONE ✓  A0={m[0]:.0f}  A1={m[1]:.0f}")
            else:
                self._calib_label.config(text="DONE ✓")
            self._calib_status.config(text="● IMU ready — press O to zero",fg=ACCENT3)

        self.root.after(40, self._tick)   # 25 Hz — same as original

    def on_close(self):
        if self.serial:        self.serial.stop()
        if self.video_adapter: self.video_adapter.stop()
        self.pc_ctrl.stop()
        if self._dist_est:     self._dist_est.stop()
        self.root.destroy()


# ══════════════════════════════════════════════════════════════════════════════
def main():
    if not SERIAL_AVAILABLE: print("WARNING: pip install pyserial")
    if not PIL_AVAILABLE:    print("WARNING: pip install Pillow   ← needed for video display")
    root=tk.Tk()
    app=K417GCS(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__=="__main__":
    main()