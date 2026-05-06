#!/usr/bin/env python3
"""
k417_imu_controller.py  –  Karuisrc K417 WiFi Drone Controller
===============================================================
Glove-based IMU controller for the Karuisrc K417 drone.

IMU (Arduino Nano RP2040):
  - Yaw / Pitch / Roll via Mahony AHRS filter
  - Throttle via A2 (up) and A3 (down) finger flex sensors

Serial data format:  timestamp, A3, A2, A1, A0, ax, ay, az, gx, gy, gz

──────────────────────────────────────────────────────────────
VIDEO SOCKET ARCHITECTURE  (ported from backend.zip)
──────────────────────────────────────────────────────────────
The WifiUavVideoProtocolAdapter in the original project:
  1. Creates its OWN UDP socket (bind("", 0), settimeout(1.0)).
  2. Passes that socket to the RC adapter so control packets also
     leave from that port — the drone sees one unified endpoint.
  3. Runs a dedicated BLOCKING recv thread (recvfrom(4096)).
  4. Warmup loop resends START_STREAM every 0.2 s.
  5. Watchdog resends frame request on 80 ms timeout.

When video is NOT active, FlightController creates its own plain
send-only socket.  When video IS active, VideoProtocol creates
the shared socket and hands it to FlightController via
inject_shared_socket().

──────────────────────────────────────────────────────────────
Requirements:  pip install pyserial Pillow
               (opencv-python as fallback decoder)
──────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import io
import math
import os
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import logging

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from PIL import Image, ImageTk, ImageDraw
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
    # Support both the canonical name and the versioned/typo filename
    try:
        from distance_estimator_v2 import AsyncDistanceEstimator, YOLO_AVAILABLE as _DE_YOLO_AVAILABLE
    except ImportError:
        print("distance_estimator module not found; distance estimation disabled.")
        _DE_YOLO_AVAILABLE = False
    DIST_EST_AVAILABLE = True
except ImportError:
    DIST_EST_AVAILABLE = False
    AsyncDistanceEstimator = None
    _DE_YOLO_AVAILABLE = False

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

# ── Video protocol ─────────────────────────────────────────────────────────
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

# JPEG header generation (ported from wifi_uav_jpeg.py)
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"

_LUM_QT = [
    16,11,10,16,24, 40, 51, 61,
    12,12,14,19,26, 58, 60, 55,
    14,13,16,24,40, 57, 69, 56,
    14,17,22,29,51, 87, 80, 62,
    18,22,37,56,68,109,103, 77,
    24,35,55,64,81,104,113, 92,
    49,64,78,87,103,121,120,101,
    72,92,95,98,112,100,103, 99,
]
_CHR_QT = [
    17,18,24,47,99,99,99,99,
    18,21,26,66,99,99,99,99,
    24,26,56,99,99,99,99,99,
    47,66,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
    99,99,99,99,99,99,99,99,
]

def _make_dqt(tid: int, table: list) -> bytes:
    payload = bytearray([(0 << 4) | tid]) + bytearray(table)
    seg     = bytearray(b"\xff\xdb")
    seg    += (len(payload) + 2).to_bytes(2, "big")
    seg    += payload
    return bytes(seg)

def _make_sof0(w: int, h: int) -> bytes:
    # Y(1,1,0) Cb(1,1,1) Cr(1,1,1) — 4:4:4
    comps  = bytes([1, 0x11, 0,  2, 0x11, 1,  3, 0x11, 1])
    length = (8 + 9).to_bytes(2, "big")
    return (b"\xff\xc0" + length + b"\x08" +
            h.to_bytes(2,"big") + w.to_bytes(2,"big") + b"\x03" + comps)

def _make_sos() -> bytes:
    # Y dc0/ac0, Cb dc1/ac1, Cr dc1/ac1
    payload = bytearray([3,
                          1, 0x00,
                          2, 0x11,
                          3, 0x11,
                          0, 63, 0])
    length  = (len(payload) + 2).to_bytes(2, "big")
    return b"\xff\xda" + length + bytes(payload)

def build_jpeg_header(w: int = 640, h: int = 360) -> bytes:
    return (_SOI
            + _make_dqt(0, _LUM_QT)
            + _make_dqt(1, _CHR_QT)
            + _make_sof0(w, h)
            + _make_sos())


def build_packet(roll, pitch, throttle, yaw, command, headless, c1, c2, c3):
    b_c1 = c1.to_bytes(2,"little"); b_c2 = c2.to_bytes(2,"little"); b_c3 = c3.to_bytes(2,"little")
    controls = [roll&0xFF, pitch&0xFF, throttle&0xFF, yaw&0xFF, command&0xFF, headless&0xFF]
    chk = 0
    for b in controls: chk ^= b
    pkt = bytearray()
    pkt += _HDR; pkt += b_c1 + _C1_SUFFIX; pkt += bytes(controls); pkt += _CTRL_PAD
    pkt.append(chk); pkt += _CKSUM_SFX; pkt += b_c2 + _C2_SUFFIX; pkt += b_c3 + _C3_SUFFIX
    return bytes(pkt)


# ──────────────────────────────────────────────────────────────────────────────
# Mahony AHRS
# ──────────────────────────────────────────────────────────────────────────────
class MahonyFilter:
    def __init__(self, kp=5.0, ki=0.02):
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


# ──────────────────────────────────────────────────────────────────────────────
# IMU Axis Mapper
# ──────────────────────────────────────────────────────────────────────────────
class IMUAxisMapper:
    MAX_ANGLE=45.; FLEX_REST_SAMPLES=80; FLEX_THRESH_STD=3.; FLEX_NORM_SCALE=150.

    def __init__(self):
        # Pitch + Roll controls
        self.pr_deadzone=8.; self.pr_sensitivity=1.; self.pr_expo=.5
        # Yaw controls (separate — glove movement feels very different)
        self.yaw_deadzone=8.; self.yaw_sensitivity=1.; self.yaw_expo=.5
        self.flex_norm_scale=self.FLEX_NORM_SCALE
        self._flex_rest_buf=[[] for _ in range(4)]
        self._flex_rest_mean=[512.]*4; self._flex_rest_std=[20.]*4
        self._flex_calibrated=False
        self._throttle_smooth=float(STICK_MID); self._throttle_alpha=.12

    # ── backwards-compat shim (deadzone bar in _tick uses mapper.deadzone) ──
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

    def compute(self, yaw, pitch, roll, a2, a3):
        sy=self._a2s(yaw,   self.yaw_deadzone, self.yaw_sensitivity, self.yaw_expo)
        sp=self._a2s(pitch, self.pr_deadzone,  self.pr_sensitivity,  self.pr_expo)
        sr=self._a2s(roll,  self.pr_deadzone,  self.pr_sensitivity,  self.pr_expo)
        if not self._flex_calibrated:
            st=self._throttle_smooth
        else:
            d2=self._flex_def(a2,2); d3=self._flex_def(a3,3)
            net=max(-1.,min(1.,d2-d3)); e_t=self.pr_expo*.6
            s=1. if net>=0 else -1.; m=abs(net)
            ct=m*(1.-e_t)+m**3*e_t
            raw=max(float(STICK_MIN),min(float(STICK_MAX),STICK_MID+s*ct*(STICK_MAX-STICK_MID)))
            self._throttle_smooth+=(raw-self._throttle_smooth)*self._throttle_alpha
            st=self._throttle_smooth
        return {"throttle":st,"yaw":sy,"pitch":sp,"roll":sr}


# ──────────────────────────────────────────────────────────────────────────────
# DroneState
# ──────────────────────────────────────────────────────────────────────────────
class DroneState:
    def __init__(self):
        self._lock=threading.Lock()
        self.throttle=self.yaw=self.pitch=self.roll=float(STICK_MID)
        self.takeoff_flag=self.land_flag=self.stop_flag=self.calibrate_flag=False
        self.headless=False; self._c1=0; self._c2=1; self._c3=2

    def set_imu(self, v):
        with self._lock:
            for k in ("throttle","yaw","pitch","roll"):
                setattr(self,k,max(STICK_MIN,min(STICK_MAX,v[k])))

    def next_counters(self):
        with self._lock:
            c1,c2,c3=self._c1,self._c2,self._c3
            self._c1=(self._c1+1)&0xFFFF; self._c2=(self._c2+1)&0xFFFF; self._c3=(self._c3+1)&0xFFFF
        return c1,c2,c3

    def consume_flags(self):
        with self._lock:
            if   self.takeoff_flag:   cmd,self.takeoff_flag   =CMD_TAKEOFF,  False
            elif self.stop_flag:      cmd,self.stop_flag       =CMD_STOP,     False
            elif self.land_flag:      cmd,self.land_flag       =CMD_LAND,     False
            elif self.calibrate_flag: cmd,self.calibrate_flag  =CMD_CALIBRATE,False
            else:                     cmd=CMD_NONE
            hless=HEADLESS_ON if self.headless else HEADLESS_OFF
        return cmd,hless

    def snapshot(self):
        with self._lock:
            return {k:getattr(self,k) for k in ("throttle","yaw","pitch","roll")}


# ──────────────────────────────────────────────────────────────────────────────
# FlightController  — sends control packets; can share a socket with video
# ──────────────────────────────────────────────────────────────────────────────
class FlightController:
    """
    Sends UDP control packets.

    Uses whatever socket it is given via inject_socket().  When video is
    active the K417VideoAdapter creates the socket first (matching the
    original backend architecture) and injects it here so control packets
    leave from the same port the drone already knows to stream video to.
    When video is not active a plain send-only socket is created on start().
    """

    def __init__(self, state: DroneState, log_q: queue.Queue):
        self.state=state; self.log_q=log_q
        self.drone_ip=DEFAULT_IP; self.drone_port=DEFAULT_PORT; self.rate=40.
        self._running=False; self._thread=None
        self._sock: socket.socket|None=None
        self._sock_lock=threading.Lock()
        self._injected=False   # True when socket was handed in by VideoAdapter
        self.debug=False

    def inject_socket(self, sock: socket.socket):
        """Called by K417VideoAdapter before start() to share its socket."""
        with self._sock_lock:
            # Close any plain socket we may own
            if self._sock and not self._injected:
                try: self._sock.close()
                except Exception: pass
            self._sock=sock
            self._injected=True


    def release_socket(self):
        """Called when video stops; revert to creating own socket on next start."""
        with self._sock_lock:
            self._sock=None
            self._injected=False

    def _ensure_sock(self):
        with self._sock_lock:
            if self._sock is None:
                s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                self._sock=s
                self._injected=False

    def _send(self, pkt):
        with self._sock_lock:
            if self._sock is None: return
            try: self._sock.sendto(pkt,(self.drone_ip,self.drone_port))
            except OSError: pass

    def _loop(self):
        self._ensure_sock()
        interval=1./self.rate; pkt_num=0
        while self._running:
            t0=time.time()
            cmd,headless=self.state.consume_flags()
            c1,c2,c3=self.state.next_counters()
            snap=self.state.snapshot()
            pkt=build_packet(int(snap["roll"]),int(snap["pitch"]),
                             int(snap["throttle"]),int(snap["yaw"]),
                             cmd,headless,c1,c2,c3)
            self._send(pkt); pkt_num+=1
            if self.debug and pkt_num%80==0:
                msg=(f"#{pkt_num:06d} T:{int(snap['throttle'])} "
                     f"Y:{int(snap['yaw'])} P:{int(snap['pitch'])} R:{int(snap['roll'])}")
                try: self.log_q.put_nowait(msg)
                except queue.Full: pass
            # Small sleep to give receive thread CPU time and prevent socket buffer overflow
            elapsed = time.time() - t0
            sleep_time = max(0.001, interval - elapsed)  # minimum 1ms sleep
            time.sleep(sleep_time)

    def start(self):
        if self._running: return
        self._running=True
        self._thread=threading.Thread(target=self._loop,daemon=True,name="FlightCtrl")
        self._thread.start()

    def stop(self):
        self._running=False
        if self._thread: self._thread.join(timeout=2.)
        with self._sock_lock:
            # Only close socket if we own it (not injected by VideoAdapter —
            # VideoAdapter.stop() is responsible for closing the shared socket)
            if self._sock and not self._injected:
                try: self._sock.close()
                except Exception: pass
            # Always clear reference; release_socket() or next start() will set it
            self._sock=None
            self._injected=False

    def reconnect(self,ip,port,rate):
        was=self._running
        if was: self.stop()
        self.drone_ip=ip; self.drone_port=port; self.rate=rate
        if was: self.start()


# ──────────────────────────────────────────────────────────────────────────────
# K417VideoAdapter  — exact port of pruebas.py (the working video implementation)
# ──────────────────────────────────────────────────────────────────────────────
import ctypes
import sys

class K417VideoAdapter:
    """
    Handles the WiFi-UAV/K417 proprietary UDP video protocol.
    Identical logic to pruebas.py — always creates its own socket first so
    the drone locks on to this port for video.  FlightController is then
    injected with the same socket via inject_socket().
    """

    HEADER_LEN     = 56
    FRAME_TIMEOUT  = 0.08
    MAX_RETRIES    = 3
    WATCHDOG_SLEEP = 0.05

    def __init__(self, drone_ip=DEFAULT_IP, port=DEFAULT_PORT,
                 jpeg_width=640, jpeg_height=360):
        self.drone_ip = drone_ip
        self.port     = port
        self._jpeg_header = build_jpeg_header(jpeg_width, jpeg_height)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288)  # 512 KB recv buffer
        self._sock.bind(("", 0))
        self._sock.setblocking(False)  # Non-blocking mode to prevent send/recv contention

        self._current_fid = 1
        self._fragments: dict = {}
        self._last_req_ts = time.time()
        self._retry_cnt   = 0
        self.frames_ok      = 0
        self.frames_dropped = 0
        self._frame_q: queue.Queue = queue.Queue(maxsize=1)
        self._running     = True
        self._first_frame = True

        threading.Thread(target=self._warmup_loop,   daemon=True, name="K417-Warmup").start()
        threading.Thread(target=self._watchdog_loop, daemon=True, name="K417-Watchdog").start()
        threading.Thread(target=self._rx_loop,       daemon=True, name="K417-RX").start()

        self._send_start()
        self._send_frame_request(0)

    def get_frame(self, timeout=0):
        try:    return self._frame_q.get(timeout=timeout) if timeout > 0 else self._frame_q.get_nowait()
        except queue.Empty: return None

    def stop(self):
        self._running = False
        try: self._sock.close()
        except Exception: pass

    def _send_start(self):
        try:
            self._sock.sendto(START_STREAM, (self.drone_ip, self.port))
        except (OSError, BlockingIOError):
            pass  # Non-blocking socket or network unavailable

    def _send_frame_request(self, frame_id):
        lo, hi = frame_id & 0xFF, (frame_id >> 8) & 0xFF
        rq_a = bytearray(REQUEST_A); rq_a[12] = lo; rq_a[13] = hi
        rq_b = bytearray(REQUEST_B)
        for base in (12, 88, 107): rq_b[base] = lo; rq_b[base+1] = hi
        try:
            self._sock.sendto(bytes(rq_a), (self.drone_ip, self.port))
            self._sock.sendto(bytes(rq_b), (self.drone_ip, self.port))
        except (OSError, BlockingIOError):
            pass  # Non-blocking socket or network unavailable
        self._last_req_ts = time.time()

    def _handle_payload(self, payload):
        if len(payload) < self.HEADER_LEN or payload[1] != 0x01:
            return
        frame_id  = int.from_bytes(payload[16:18], "little")
        frag_id   = int.from_bytes(payload[32:34], "little")
        last_frag = payload[2] != 0x38
        if frame_id != self._current_fid:
            self.frames_dropped += 1
            self._fragments.clear()
            self._current_fid = frame_id
        self._fragments.setdefault(frag_id, payload[self.HEADER_LEN:])
        self._retry_cnt = 0
        if not last_frag:
            return
        ordered = [self._fragments[i] for i in sorted(self._fragments)]
        jpeg = self._jpeg_header + b"".join(ordered) + _EOI
        self.frames_ok += 1
        if self._first_frame:
            self._first_frame = False
        # Drain stale frame so display always gets the latest
        try:    self._frame_q.get_nowait()
        except queue.Empty: pass
        try:    self._frame_q.put_nowait(jpeg)
        except queue.Full:  pass
        self._fragments.clear()
        self._send_frame_request(frame_id)
        self._current_fid = (frame_id + 1) & 0xFFFF
        self._last_req_ts = time.time()

    def _rx_loop(self):
        import select
        while self._running:
            try:
                # Use select with short timeout to avoid busy-waiting
                readable, _, _ = select.select([self._sock], [], [], 0.01)
                if readable:
                    payload, _ = self._sock.recvfrom(65535)
                    self._handle_payload(payload)
            except (OSError, ValueError):
                # ValueError: fileno() invalid if socket closed
                # OSError: socket operation failed
                if self._running:
                    time.sleep(0.01)
                break

    def _warmup_loop(self):
        while self._running and self._first_frame:
            self._send_start()
            self._send_frame_request((self._current_fid - 1) & 0xFFFF)
            time.sleep(0.2)

    def _watchdog_loop(self):
        while self._running:
            time.sleep(self.WATCHDOG_SLEEP)
            if time.time() - self._last_req_ts < self.FRAME_TIMEOUT: continue
            if self._retry_cnt < self.MAX_RETRIES:
                self._send_frame_request((self._current_fid - 1) & 0xFFFF)
                self._retry_cnt += 1
            else:
                self.frames_dropped += 1
                self._fragments.clear()
                self._retry_cnt   = 0
                self._current_fid = (self._current_fid + 1) & 0xFFFF
                self._send_frame_request((self._current_fid - 1) & 0xFFFF)


# ──────────────────────────────────────────────────────────────────────────────
# OpenCV display loop — identical to pruebas.py's run_display()
# ──────────────────────────────────────────────────────────────────────────────

def _run_video_display(adapter: K417VideoAdapter, dist_est_or_gui=None):
    """
    Runs in its own daemon thread.  Opens a native OpenCV window.
    Press  Q  to quit,  D  to toggle distance estimation overlay.

    dist_est_or_gui may be:
      - an AsyncDistanceEstimator (legacy / direct)
      - a K417GUI instance  → dist_est is read live from gui._dist_est so
        toggling the DIST EST button after video has started works correctly.
      - None
    """
    def _get_dist_est():
        if dist_est_or_gui is None:
            return None
        if isinstance(dist_est_or_gui, AsyncDistanceEstimator):
            return dist_est_or_gui
        # K417GUI reference — always return the current estimator
        return getattr(dist_est_or_gui, "_dist_est", None)
    window = "K417 Live View"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 640, 360)

    placeholder = np.zeros((360, 640, 3), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = "Waiting for K417 video\u2026"
    (tw, th), _ = cv2.getTextSize(text, font, 0.6, 2)
    cv2.putText(placeholder, text,
                ((640 - tw) // 2, (360 + th) // 2),
                font, 0.6, (0, 100, 255), 2)

    last_img  = placeholder
    dist_on   = False     # toggled with D key
    fps_t     = time.time()
    fps_count = 0

    while adapter._running:
        # Re-read the estimator every frame so GUI button changes take effect
        dist_est = _get_dist_est()

        jpeg = adapter.get_frame(timeout=0)

        if jpeg is not None:
            arr     = np.frombuffer(jpeg, dtype=np.uint8)
            decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if decoded is not None:
                if dist_est is not None and dist_on:
                    dist_est.submit(decoded)   # non-blocking hand-off to bg thread
                last_img  = decoded
                fps_count += 1

        # Choose what to display
        display_img = last_img
        if dist_est is not None and dist_on:
            if not dist_est.ready:
                # Models still loading — show raw frame with badge
                display_img = last_img.copy() if last_img is not placeholder else placeholder.copy()
                cv2.putText(display_img, "DIST EST: loading models…",
                            (8, display_img.shape[0] - 8),
                            font, 0.45, (0, 200, 255), 1, cv2.LINE_AA)
            else:
                res = dist_est.result
                if res.overlay is not None:
                    display_img = res.overlay

        # D-key status badge (top-left, always visible when dist_est available)
        if dist_est is not None:
            col = (0, 220, 80) if (dist_on and dist_est.ready) else \
                  (0, 180, 255) if dist_on else (80, 80, 80)
            lbl = "DIST ON  [D=off]" if dist_on else "DIST OFF  [D=on]"
            cv2.putText(display_img, lbl, (8, 20), font, 0.45, col, 1, cv2.LINE_AA)

        cv2.imshow(window, display_img)

        if fps_count and fps_count % 60 == 0:
            elapsed = max(time.time() - fps_t, 1e-6)
            fps_t, fps_count = time.time(), 0

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            adapter.stop()
            break
        elif key == ord("d") and dist_est is not None:
            dist_on = not dist_on

    cv2.destroyWindow(window)


# ──────────────────────────────────────────────────────────────────────────────
# TelemetryParser
# ──────────────────────────────────────────────────────────────────────────────
class TelemetryParser:
    def __init__(self):
        self._lock=threading.Lock()
        self.battery_pct=-1; self.altitude_cm=-1; self.raw_last=b""

    def ingest(self, payload):
        if len(payload)<8 or payload[1]==0x01: return False
        with self._lock:
            self.raw_last=payload[:16]
            if len(payload)>4:
                b=payload[4]
                if 0<=b<=100: self.battery_pct=b
            if len(payload)>6:
                self.altitude_cm=payload[6]
        return True

    def snapshot(self):
        with self._lock:
            return {"battery_pct":self.battery_pct,"altitude_cm":self.altitude_cm,
                    "raw":self.raw_last.hex(" ") if self.raw_last else "—"}


# ──────────────────────────────────────────────────────────────────────────────
# SerialReader
# ──────────────────────────────────────────────────────────────────────────────
class SerialReader:
    def __init__(self,port,baud,on_data,on_status,log_q):
        self.port=port;self.baud=baud;self.on_data=on_data
        self.on_status=on_status;self.log_q=log_q
        self._running=False;self._thread=None

    def start(self):
        if not SERIAL_AVAILABLE: self.on_status("ERROR: pyserial not installed"); return
        self._running=True
        self._thread=threading.Thread(target=self._loop,daemon=True,name="SerialRd")
        self._thread.start()

    def stop(self): self._running=False

    def _loop(self):
        self.on_status(f"Connecting {self.port}…")
        try:
            ser=serial.Serial(self.port,self.baud,timeout=0.1)
            time.sleep(2); self.on_status(f"✓ {self.port} @ {self.baud}")
        except Exception as e:
            self.on_status(f"✗ {e}"); self._running=False; return
        while self._running:
            try:
                if ser.in_waiting>0:
                    line=ser.readline().decode("utf-8",errors="ignore").strip()
                    if line and "," in line: self._parse(line)
            except Exception as e:
                self.on_status(f"Read error: {e}"); time.sleep(0.5)
        ser.close()

    def _parse(self, line):
        parts=line.split(",")
        if len(parts)<11: return
        try:
            vals=[float(v) for v in parts[:11]]
            # ts, A3, A2, A1, A0, ax, ay, az, gx, gy, gz
            a3=vals[3];a2=vals[4];a1=vals[1];a0=vals[2]
            ax,ay,az,gx,gy,gz=vals[5:11]
            self.on_data(a0,a1,a2,a3,ax,ay,az,gx,gy,gz)
        except (ValueError,IndexError): pass


# ──────────────────────────────────────────────────────────────────────────────
# GloveController
# ──────────────────────────────────────────────────────────────────────────────
class GloveController:
    CALIB_SAMPLES=150

    def __init__(self,state,log_q):
        self.state=state;self.log_q=log_q
        self.ahrs=MahonyFilter();self.mapper=IMUAxisMapper()
        self._last_t=time.time()
        self.calibrating=True;self.calib_count=0;self.enabled=True
        self.yaw_deg=self.pitch_deg=self.roll_deg=0.
        self.a2_raw=self.a3_raw=self.throttle_pct=0.
        self.flex_calibrated=False;self.flex_rest_mean=[0.]*4

    def reset_calibration(self):
        self.ahrs=MahonyFilter();self.mapper.reset_flex_calibration()
        self.calibrating=True;self.calib_count=0;self.flex_calibrated=False
        self._log("IMU: re-calibrating gyro + flex rest baseline…")

    def capture_zero(self):
        self.ahrs.capture_offset(); self._log("IMU: orientation zeroed ✓")

    def on_sensor_data(self,a0,a1,a2,a3,ax_r,ay_r,az_r,gx_r,gy_r,gz_r):
        ax=ay_r;ay=-ax_r;az=az_r;gx=gy_r;gy=-gx_r;gz=gz_r
        gr=[math.radians(v) for v in (gx,gy,gz)]
        now=time.time();dt=min(now-self._last_t,.05);self._last_t=now
        self.a2_raw=a2;self.a3_raw=a3
        if self.calibrating:
            gd=self.ahrs.add_gyro_sample(*gr,self.CALIB_SAMPLES)
            fd=self.mapper.add_flex_rest_sample(a0,a1,a2,a3)
            self.calib_count+=1
            if gd and fd:
                self.calibrating=False;self.flex_calibrated=True
                self.flex_rest_mean=list(self.mapper._flex_rest_mean)
                m=self.mapper._flex_rest_mean
                self._log(f"IMU calibrated ✓  A2={m[2]:.0f}  A3={m[3]:.0f}  — press O to zero.")
            return
        self.ahrs.update(ax,ay,az,*gr,dt)
        yaw,pitch,roll=self.ahrs.get_euler_relative()
        self.yaw_deg=yaw;self.pitch_deg=pitch;self.roll_deg=roll
        sticks=self.mapper.compute(yaw,pitch,roll,a2,a3)
        self.throttle_pct=(sticks["throttle"]-STICK_MID)/(STICK_MAX-STICK_MID)
        if self.enabled: self.state.set_imu(sticks)

    def _log(self,msg):
        ts=time.strftime("%H:%M:%S")
        try: self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full: pass


# ──────────────────────────────────────────────────────────────────────────────
# GUI colours & fonts
# ──────────────────────────────────────────────────────────────────────────────
DARK_BG="#0b0d13"; PANEL_BG="#12151f"; CARD_BG="#181d2a"
ACCENT="#00e5ff"; ACCENT2="#ff4081"; ACCENT3="#69ff47"
TEXT_MAIN="#e0e6f0"; TEXT_DIM="#4a6070"
BTN_TAKE="#00c853"; BTN_LAND="#ff6d00"; BTN_STOP="#d50000"
BTN_HEAD="#7c4dff"; BTN_CAL="#0091ea"; IMU_COLOR="#b388ff"

FONT_MONO =("Courier New",10); FONT_LABEL=("Courier New",9,"bold")
FONT_BTN  =("Courier New",10,"bold"); FONT_BIG=("Courier New",14,"bold")
FONT_TITLE=("Courier New",18,"bold"); FONT_SMALL=("Courier New",8)


# ──────────────────────────────────────────────────────────────────────────────
# Attitude Indicator
# ──────────────────────────────────────────────────────────────────────────────
class AttitudeIndicator(tk.Canvas):
    SIZE=120
    def __init__(self,parent,**kw):
        super().__init__(parent,width=self.SIZE,height=self.SIZE,
                         bg=CARD_BG,highlightthickness=1,highlightbackground=TEXT_DIM,**kw)
        self._pitch=self._roll=self._yaw=0.;self._draw()
    def update_attitude(self,pitch,roll,yaw):
        self._pitch=pitch;self._roll=roll;self._yaw=yaw;self._draw()
    def _draw(self):
        self.delete("all");cx=cy=self.SIZE//2;r=cx-4
        pp=max(-r,min(r,self._pitch*(r/45.)));rr=math.radians(self._roll)
        ca,sa=math.cos(rr),math.sin(rr);ox,oy=-sa*pp,ca*pp
        self.create_oval(cx-r,cy-r,cx+r,cy+r,fill="#1a3a5c",outline="")
        pts=[]
        for i in range(37):
            th=math.pi*i/36;pts.extend([cx+r*math.cos(th+math.pi),cy+r*math.sin(th+math.pi)])
        dx=ca*r*1.5;dy=sa*r*1.5
        h1x=cx+ox+dx;h1y=cy+oy+dy;h2x=cx+ox-dx;h2y=cy+oy-dy
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
    WIDTH=24;HEIGHT=120
    def __init__(self,parent,**kw):
        super().__init__(parent,width=self.WIDTH,height=self.HEIGHT,
                         bg=CARD_BG,highlightthickness=1,highlightbackground=TEXT_DIM,**kw)
        self._value=0.;self._draw()
    def set_value(self,v): self._value=max(-1.,min(1.,v));self._draw()
    def _draw(self):
        self.delete("all");w=self.WIDTH;h=self.HEIGHT;mid=h//2
        bh=int(abs(self._value)*mid);color=BTN_TAKE if self._value>=0 else ACCENT2
        if self._value>=0: self.create_rectangle(2,mid-bh,w-2,mid,fill=color,outline="")
        else:              self.create_rectangle(2,mid,w-2,mid+bh,fill=color,outline="")
        self.create_line(0,mid,w,mid,fill=TEXT_DIM,width=1)
        self.create_rectangle(1,1,w-2,h-2,outline=TEXT_DIM,width=1)





# ──────────────────────────────────────────────────────────────────────────────
# Main GUI
# ──────────────────────────────────────────────────────────────────────────────
class K417GUI:
    def __init__(self,root):
        self.root=root
        self.state=DroneState()
        self.log_q:queue.Queue=queue.Queue(maxsize=300)
        self.ctrl=FlightController(self.state,self.log_q)
        self.glove=GloveController(self.state,self.log_q)
        self.serial:SerialReader|None=None
        self.telemetry=TelemetryParser()
        self.video_adapter:K417VideoAdapter|None=None
        self._video_thread:threading.Thread|None=None
        self._dist_est=None
        self._build_ui();self._bind_keys();self._tick()

    def _build_ui(self):
        r=self.root; r.title("K417 // IMU Glove Controller")
        r.configure(bg=DARK_BG);r.resizable(True,True)
        r.geometry("1340x860");r.minsize(1200,800)
        ttk.Style().theme_use("clam")
        hdr=tk.Frame(r,bg=DARK_BG);hdr.pack(fill="x",padx=20,pady=(14,4))
        tk.Label(hdr,text="K417",fg=ACCENT,bg=DARK_BG,font=FONT_TITLE).pack(side="left")
        tk.Label(hdr,text="  //  IMU GLOVE CONTROLLER",fg=TEXT_DIM,bg=DARK_BG,font=FONT_BIG).pack(side="left")
        self._status_label=tk.Label(hdr,text="● STOPPED",fg=ACCENT2,bg=DARK_BG,font=FONT_LABEL)
        self._status_label.pack(side="right")
        tk.Frame(r,height=1,bg=ACCENT).pack(fill="x",padx=20,pady=(0,8))
        cols=tk.Frame(r,bg=DARK_BG);cols.pack(fill="both",expand=True,padx=16)
        left=tk.Frame(cols,bg=DARK_BG);left.pack(side="left",fill="both")
        centre=tk.Frame(cols,bg=DARK_BG);centre.pack(side="left",fill="both",padx=10)
        right=tk.Frame(cols,bg=DARK_BG);right.pack(side="right",fill="both")
        self._build_connection(left);self._build_glove(left)
        self._build_commands(left);self._build_keys_legend(left)
        self._build_imu(centre);self._build_sensitivity(centre);self._build_video(centre)
        self._build_sticks(right);self._build_log(r)

    def _panel(self,parent,title):
        outer=tk.Frame(parent,bg=DARK_BG);outer.pack(fill="x",pady=5)
        tk.Label(outer,text=f"  {title}  ",fg=ACCENT,bg=DARK_BG,
                 font=("Courier New",9,"bold")).pack(anchor="w")
        f=tk.Frame(outer,bg=PANEL_BG,padx=10,pady=8);f.pack(fill="x");return f

    def _build_connection(self,parent):
        f=self._panel(parent,"DRONE CONNECTION")
        def row(label,default):
            rw=tk.Frame(f,bg=PANEL_BG);rw.pack(fill="x",pady=2)
            tk.Label(rw,text=label,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=11,anchor="w").pack(side="left")
            var=tk.StringVar(value=default)
            tk.Entry(rw,textvariable=var,width=18,bg=CARD_BG,fg=TEXT_MAIN,
                     insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=4)
            return var
        self._ip_var=row("Drone IP",DEFAULT_IP)
        self._port_var=row("Port (UDP)",str(DEFAULT_PORT))
        self._rate_var=row("Rate (Hz)","40")
        br=tk.Frame(f,bg=PANEL_BG);br.pack(fill="x",pady=(6,0))
        tk.Button(br,text="CONNECT",bg="#0d47a1",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._apply_connection).pack(side="left",padx=2)
        tk.Button(br,text="DISCONNECT",bg="#37474f",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._disconnect).pack(side="left",padx=2)

    def _build_glove(self,parent):
        f=self._panel(parent,"GLOVE  (Arduino Nano RP2040)")
        pr=tk.Frame(f,bg=PANEL_BG);pr.pack(fill="x",pady=2)
        tk.Label(pr,text="Serial port",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=11,anchor="w").pack(side="left")
        self._serial_port_var=tk.StringVar(value="COM3")
        tk.Entry(pr,textvariable=self._serial_port_var,width=10,bg=CARD_BG,fg=TEXT_MAIN,
                 insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=4)
        self._serial_status=tk.Label(pr,text="not connected",fg=ACCENT2,bg=PANEL_BG,font=FONT_SMALL)
        self._serial_status.pack(side="left",padx=6)
        br2=tk.Frame(f,bg=PANEL_BG);br2.pack(fill="x",pady=2)
        tk.Label(br2,text="Baud rate",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=11,anchor="w").pack(side="left")
        self._baud_var=tk.StringVar(value="115200")
        tk.Entry(br2,textvariable=self._baud_var,width=10,bg=CARD_BG,fg=TEXT_MAIN,
                 insertbackground=ACCENT,font=FONT_MONO,relief="flat",bd=2).pack(side="left",padx=4)
        btn_r=tk.Frame(f,bg=PANEL_BG);btn_r.pack(fill="x",pady=(6,2))
        tk.Button(btn_r,text="CONNECT GLOVE",bg="#1b5e20",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._connect_glove).pack(side="left",padx=2)
        tk.Button(btn_r,text="ZERO  [O]",bg=IMU_COLOR,fg=DARK_BG,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._imu_zero).pack(side="left",padx=2)
        btn_r2=tk.Frame(f,bg=PANEL_BG);btn_r2.pack(fill="x",pady=2)
        tk.Button(btn_r2,text="RE-CALIBRATE [F5]",bg="#4a148c",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._imu_recalib).pack(side="left",padx=2)
        self._glove_toggle_btn=tk.Button(btn_r2,text="IMU  ENABLED",bg=ACCENT3,fg=DARK_BG,
                  font=FONT_BTN,relief="flat",cursor="hand2",command=self._toggle_imu)
        self._glove_toggle_btn.pack(side="left",padx=2)
        cf=tk.Frame(f,bg=PANEL_BG);cf.pack(fill="x",pady=4)
        tk.Label(cf,text="Gyro bias cal:",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL).pack(side="left")
        self._calib_canvas=tk.Canvas(cf,width=140,height=10,bg=CARD_BG,highlightthickness=0)
        self._calib_canvas.pack(side="left",padx=4)
        self._calib_label=tk.Label(cf,text="0 / 150",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL)
        self._calib_label.pack(side="left")

    def _build_imu(self,parent):
        f=self._panel(parent,"IMU  ATTITUDE")
        top=tk.Frame(f,bg=PANEL_BG);top.pack(fill="x")
        self._ahi=AttitudeIndicator(top);self._ahi.pack(side="left",padx=(0,12))
        tf=tk.Frame(top,bg=PANEL_BG);tf.pack(side="left")
        tk.Label(tf,text="THR",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL).pack()
        self._thr_bar=ThrottleBar(tf);self._thr_bar.pack()
        nums=tk.Frame(top,bg=PANEL_BG);nums.pack(side="left",padx=8)
        self._imu_vars:dict[str,tk.StringVar]={}
        for label,key,color in [("YAW","yaw",ACCENT2),("PITCH","pitch",ACCENT3),
                                  ("ROLL","roll",IMU_COLOR),("A2↑","a2",BTN_TAKE),("A3↓","a3",ACCENT2)]:
            rw=tk.Frame(nums,bg=PANEL_BG);rw.pack(fill="x",pady=1)
            tk.Label(rw,text=f"{label:5s}",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=6,anchor="w").pack(side="left")
            var=tk.StringVar(value="—");self._imu_vars[key]=var
            tk.Label(rw,textvariable=var,fg=color,bg=PANEL_BG,font=FONT_MONO,width=8,anchor="e").pack(side="left")
        self._calib_status=tk.Label(f,text="● Calibrating gyro bias…",fg=ACCENT2,bg=PANEL_BG,font=FONT_LABEL)
        self._calib_status.pack(pady=(6,2))

    def _build_sensitivity(self,parent):
        f=self._panel(parent,"IMU  SENSITIVITY")
        def param(label,from_,to,initial,setter,res=0.05,color=ACCENT):
            rw=tk.Frame(f,bg=PANEL_BG);rw.pack(fill="x",pady=2)
            tk.Label(rw,text=label,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=16,anchor="w").pack(side="left")
            var=tk.DoubleVar(value=initial)
            disp=tk.Label(rw,text=f"{initial:.2f}",fg=color,bg=PANEL_BG,font=FONT_MONO,width=5)
            disp.pack(side="right")
            def on(*_,s=setter,d=disp,v=var): val=round(v.get(),3);s(val);d.config(text=f"{val:.2f}")
            tk.Scale(rw,variable=var,from_=from_,to=to,orient="horizontal",resolution=res,length=170,
                     bg=PANEL_BG,fg=TEXT_MAIN,troughcolor=CARD_BG,highlightthickness=0,
                     activebackground=color,showvalue=False,command=on).pack(side="left",padx=4)

        # ── Pitch + Roll ──────────────────────────────────────────────────
        tk.Label(f,text="PITCH  &  ROLL",fg=ACCENT3,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(4,0))
        param("Deadzone (°)", 0.,30., 8., lambda v:setattr(self.glove.mapper,"pr_deadzone",v),  res=0.5, color=ACCENT3)
        param("Sensitivity",  0.1,3., 1., lambda v:setattr(self.glove.mapper,"pr_sensitivity",v),       color=ACCENT3)
        param("Expo curve",   0.,1.,  .5, lambda v:setattr(self.glove.mapper,"pr_expo",v),               color=ACCENT3)

        # ── Yaw ───────────────────────────────────────────────────────────
        tk.Label(f,text="YAW",fg=ACCENT2,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(6,0))
        param("Deadzone (°)", 0.,30., 8., lambda v:setattr(self.glove.mapper,"yaw_deadzone",v), res=0.5, color=ACCENT2)
        param("Sensitivity",  0.1,3., 1., lambda v:setattr(self.glove.mapper,"yaw_sensitivity",v),       color=ACCENT2)
        param("Expo curve",   0.,1.,  .5, lambda v:setattr(self.glove.mapper,"yaw_expo",v),               color=ACCENT2)

        # ── Throttle ──────────────────────────────────────────────────────
        tk.Label(f,text="THROTTLE",fg=BTN_TAKE,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(6,0))
        param("Smoothing",    .02,.5, .12,lambda v:setattr(self.glove.mapper,"_throttle_alpha",v),res=0.01,color=BTN_TAKE)
        param("Flex scale",   20.,400.,IMUAxisMapper.FLEX_NORM_SCALE,
              lambda v:setattr(self.glove.mapper,"flex_norm_scale",v),res=5.,color=BTN_TAKE)

    def _build_video(self,parent):
        f=self._panel(parent,"CAMERA  &  TELEMETRY")
        br=tk.Frame(f,bg=PANEL_BG);br.pack(fill="x",pady=(0,6))
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
            tk.Label(br,text="pip install opencv-python numpy  (required for video)",fg=ACCENT2,bg=PANEL_BG,font=FONT_SMALL).pack(side="left",padx=8)
        tr=tk.Frame(f,bg=PANEL_BG);tr.pack(fill="x",pady=2)
        for label,attr,color in [("ALTITUDE","_tel_alt_var",ACCENT3),("BATTERY","_tel_bat_var",BTN_TAKE)]:
            tk.Label(tr,text=label,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,width=9,anchor="w").pack(side="left")
            var=tk.StringVar(value="N/A");setattr(self,attr,var)
            tk.Label(tr,textvariable=var,fg=color,bg=PANEL_BG,font=FONT_MONO,width=10,anchor="w").pack(side="left",padx=(0,12))
        rr=tk.Frame(f,bg=PANEL_BG);rr.pack(fill="x",pady=(2,0))
        tk.Label(rr,text="Last telem hex:",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_SMALL).pack(side="left")
        self._tel_raw_var=tk.StringVar(value="—")
        tk.Label(rr,textvariable=self._tel_raw_var,fg=TEXT_DIM,bg=PANEL_BG,
                 font=(FONT_MONO[0],7)).pack(side="left",padx=4)

    def _build_commands(self,parent):
        f=self._panel(parent,"COMMANDS");f.columnconfigure(0,weight=1);f.columnconfigure(1,weight=1)
        def btn(text,color,cmd,row,col):
            b=tk.Button(f,text=text,bg=color,fg="white",activebackground=color,
                        font=FONT_BTN,relief="flat",cursor="hand2",width=12,height=1,command=cmd)
            b.grid(row=row,column=col,padx=3,pady=3);return b
        btn("⬆  TAKEOFF",BTN_TAKE,self._cmd_takeoff,0,0)
        btn("⬇  LAND",BTN_LAND,self._cmd_land,0,1)
        self._btn_stop=btn("✕  STOP",BTN_STOP,self._cmd_stop,1,0)
        self._btn_head=btn("⧖  HEADLESS",BTN_HEAD,self._toggle_headless,1,1)
        btn("◎  CALIBRATE",BTN_CAL,self._cmd_calibrate,2,0)
        self._debug_btn=btn("⚙  DEBUG OFF","#37474f",self._toggle_debug,2,1)

    def _build_keys_legend(self,parent):
        f=self._panel(parent,"KEYBOARD  OVERRIDES")
        items=[("T","Takeoff"),("L","Land"),("SPACE","Emergency stop"),
               ("H","Headless"),("C","Calibrate"),("O","Zero IMU"),
               ("F5","Re-calibrate"),("S","Snapshot")]
        for i,(key,desc) in enumerate(items):
            rr,cc=i%4,(i//4)*2
            tk.Label(f,text=key,fg=ACCENT,bg=PANEL_BG,font=FONT_MONO,width=6,anchor="e").grid(
                row=rr,column=cc,padx=(0,4),pady=1,sticky="e")
            tk.Label(f,text=desc,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,anchor="w").grid(
                row=rr,column=cc+1,padx=(0,16),pady=1,sticky="w")

    def _build_sticks(self,parent):
        f=self._panel(parent,"LIVE  STICKS")
        self._stick_vars:dict[str,tk.DoubleVar]={}; self._stick_val_lbl:dict[str,tk.Label]={}
        for i,(name,label) in enumerate([("throttle","THROTTLE"),("yaw","YAW"),
                                          ("pitch","PITCH"),("roll","ROLL")]):
            tk.Label(f,text=label,fg=ACCENT,bg=PANEL_BG,font=FONT_LABEL,width=9,anchor="w").grid(
                row=i,column=0,padx=6,pady=5,sticky="w")
            var=tk.DoubleVar(value=STICK_MID);self._stick_vars[name]=var
            vl=tk.Label(f,text="128",fg=TEXT_MAIN,bg=PANEL_BG,font=FONT_MONO,width=4)
            vl.grid(row=i,column=2,padx=6);self._stick_val_lbl[name]=vl
            tk.Scale(f,variable=var,from_=STICK_MIN,to=STICK_MAX,orient="horizontal",
                     resolution=1,length=320,bg=PANEL_BG,fg=TEXT_MAIN,troughcolor=CARD_BG,
                     highlightthickness=0,activebackground=ACCENT,showvalue=False,state="disabled").grid(
                row=i,column=1,padx=6,pady=5)
        bf=tk.Frame(f,bg=PANEL_BG);bf.grid(row=4,column=0,columnspan=3,sticky="ew",padx=6,pady=4)
        tk.Label(bf,text="IMU ANGLE BARS",fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL).pack(anchor="w")
        bi=tk.Frame(bf,bg=PANEL_BG);bi.pack(fill="x")
        self._angle_bars:dict[str,tk.Canvas]={}
        for axis,color in [("YAW",ACCENT2),("PITCH",ACCENT3),("ROLL",IMU_COLOR)]:
            r2=tk.Frame(bi,bg=PANEL_BG);r2.pack(fill="x",pady=1)
            tk.Label(r2,text=axis,fg=color,bg=PANEL_BG,font=FONT_SMALL,width=6,anchor="w").pack(side="left")
            c=tk.Canvas(r2,width=260,height=10,bg=CARD_BG,highlightthickness=0)
            c.pack(side="left",padx=2);self._angle_bars[axis]=c

    def _build_log(self,parent):
        tk.Frame(parent,height=1,bg=TEXT_DIM).pack(fill="x",padx=20,pady=(6,0))
        lf=tk.Frame(parent,bg=DARK_BG);lf.pack(fill="both",expand=True,padx=20,pady=(4,10))
        tk.Label(lf,text="EVENT LOG",fg=TEXT_DIM,bg=DARK_BG,font=FONT_LABEL).pack(anchor="w")
        self._log_text=scrolledtext.ScrolledText(lf,height=5,bg=CARD_BG,fg="#37ff8b",
            font=("Courier New",8),relief="flat",state="disabled",wrap="none")
        self._log_text.pack(fill="both",expand=True)
        tk.Button(lf,text="Clear",bg=PANEL_BG,fg=TEXT_DIM,font=FONT_LABEL,relief="flat",
                  cursor="hand2",command=self._clear_log).pack(side="right",pady=3)

    # ── key bindings ──────────────────────────────────────────────────────
    def _bind_keys(self): self.root.bind("<KeyPress>",self._on_key)
    def _on_key(self,event):
        k=event.keysym
        if   k=="t":        self._cmd_takeoff()
        elif k=="l":        self._cmd_land()
        elif k=="space":    self._cmd_stop()
        elif k=="h":        self._toggle_headless()
        elif k=="c":        self._cmd_calibrate()
        elif k.lower()=="o":self._imu_zero()
        elif k=="F5":       self._imu_recalib()
        elif k.lower()=="s": pass  # snapshot handled inside OpenCV window (press S there)

    # ── glove ─────────────────────────────────────────────────────────────
    def _connect_glove(self):
        if self.serial: self.serial.stop()
        port=self._serial_port_var.get().strip()
        try:    baud=int(self._baud_var.get())
        except: baud=115200
        self.serial=SerialReader(port,baud,self.glove.on_sensor_data,
                                 self._on_serial_status,self.log_q)
        self.serial.start()
    def _on_serial_status(self,msg):
        self.root.after(0,lambda:self._serial_status.config(
            text=msg,fg=ACCENT3 if "✓" in msg else ACCENT2))
        self._log_event(f"SERIAL: {msg}")
    def _imu_zero(self):    self.glove.capture_zero();   self._log_event("IMU: zeroed")
    def _imu_recalib(self): self.glove.reset_calibration(); self._log_event("IMU: re-calibrating")
    def _toggle_imu(self):
        self.glove.enabled=not self.glove.enabled
        self._glove_toggle_btn.config(
            text="IMU  ENABLED" if self.glove.enabled else "IMU  PAUSED",
            bg=ACCENT3 if self.glove.enabled else ACCENT2,
            fg=DARK_BG if self.glove.enabled else "white")
        self._log_event(f"IMU: {'ON' if self.glove.enabled else 'PAUSED'}")

    # ── drone commands ────────────────────────────────────────────────────
    def _cmd_takeoff(self):   self.state.takeoff_flag  =True;self._log_event("CMD: TAKEOFF")
    def _cmd_land(self):      self.state.land_flag      =True;self._log_event("CMD: LAND")
    def _cmd_stop(self):      self.state.stop_flag      =True;self._log_event("CMD: EMERGENCY STOP")
    def _cmd_calibrate(self): self.state.calibrate_flag =True;self._log_event("CMD: CALIBRATE DRONE IMU")
    def _toggle_headless(self):
        self.state.headless=not self.state.headless;s="ON" if self.state.headless else "OFF"
        self._btn_head.config(bg=ACCENT if self.state.headless else BTN_HEAD,
                              fg=DARK_BG if self.state.headless else "white")
        self._log_event(f"HEADLESS: {s}")
    def _toggle_debug(self):
        self.ctrl.debug=not self.ctrl.debug;s="ON" if self.ctrl.debug else "OFF"
        self._debug_btn.config(text=f"⚙  DEBUG {s}",
                               bg=ACCENT if self.ctrl.debug else "#37474f",
                               fg=DARK_BG if self.ctrl.debug else TEXT_MAIN)

    # ── video ─────────────────────────────────────────────────────────────
    def _toggle_video(self):
        if self.video_adapter is not None:
            # ── STOP ──────────────────────────────────────────────────────
            # Stop adapter (this closes the shared socket), release it from
            # FlightController, then restart FlightController with its own socket
            was_ctrl = self.ctrl._running
            self.ctrl.stop()           # stops loop, does NOT close injected sock
            self.ctrl.release_socket() # forget the injected socket reference
            self.video_adapter.stop()  # now actually close the socket
            self.video_adapter = None
            self._video_thread = None
            if was_ctrl:
                self.ctrl.drone_ip = self._ip_var.get().strip()
                try:    self.ctrl.drone_port = int(self._port_var.get())
                except: self.ctrl.drone_port = DEFAULT_PORT
                self.ctrl.start()      # restarts with its own fresh plain socket
            self._video_btn.config(text="▶  START VIDEO", bg="#005f73")
            self._log_event("VIDEO: stopped")
        else:
            if not CV2_AVAILABLE:
                self._log_event("VIDEO: ERROR — pip install opencv-python numpy")
                return

            ip = self._ip_var.get().strip()
            try:    port = int(self._port_var.get())
            except: port = DEFAULT_PORT
            self.ctrl.drone_ip = ip
            self.ctrl.drone_port = port

            # ── START ─────────────────────────────────────────────────────
            # Critical ordering (matches original backend ZIP architecture):
            # The drone responds to video only on the port it first receives
            # START_STREAM from.  So:
            #   1. Stop FlightController — closes its socket so the drone
            #      stops hearing from the old port immediately.
            #   2. Adapter creates a FRESH socket — this is the first port
            #      the drone will now hear from.
            #   3. Inject that socket into FlightController — control packets
            #      now also leave from this port.
            #   4. Restart FlightController — drone sees one unified endpoint.
            was_ctrl = self.ctrl._running
            if was_ctrl:
                self.ctrl.stop()           # closes old socket cleanly
                self.ctrl.release_socket()
                self._log_event("VIDEO: cycling ctrl socket for clean handoff")

            self.video_adapter = K417VideoAdapter(drone_ip=ip, port=port)


            self.ctrl.inject_socket(self.video_adapter._sock)
            self.ctrl.start()   # always start — drone needs control packets too

            # Pass dist_est via the GUI instance so the running video thread
            # always sees the current estimator even if toggled after video start.
            gui_ref = self
            def _display_thread(gui=gui_ref):
                _run_video_display(self.video_adapter, gui)
                self.root.after(0, self._on_video_closed)

            self._video_thread = threading.Thread(
                target=_display_thread, daemon=True, name="VideoDisplay")
            self._video_thread.start()

            self._video_btn.config(text="■  STOP VIDEO", bg=BTN_STOP)
            self._status_label.config(text="● CONNECTED + VIDEO", fg=BTN_TAKE)
            self._log_event(f"VIDEO: started → {ip}:{port}")

    def _on_video_closed(self):
        """Called from the main thread when the OpenCV window is closed."""
        # Adapter socket is already closed — just restore FlightController
        self.ctrl.release_socket()
        self.video_adapter = None
        self._video_thread = None
        self.ctrl.drone_ip = self._ip_var.get().strip()
        try:    self.ctrl.drone_port = int(self._port_var.get())
        except: self.ctrl.drone_port = DEFAULT_PORT
        self.ctrl.start()
        self._video_btn.config(text="▶  START VIDEO", bg="#005f73")
        self._status_label.config(text="● CONNECTED", fg=BTN_TAKE)
        self._log_event("VIDEO: window closed")

    # ── distance estimator ───────────────────────────────────────────────
    def _toggle_dist_est(self):
        if not DIST_EST_AVAILABLE or self._dist_btn is None: return
        if self._dist_est is not None:
            self._dist_est.stop()
            self._dist_est = None
            self._dist_btn.config(text="◎  DIST EST", bg="#1a237e")
            self._log_event("DIST EST: stopped")
        else:
            self._dist_est = AsyncDistanceEstimator(
                use_yolo=True, draw_overlay=True)

            self._dist_est.start()
            self._dist_btn.config(text="■  DIST EST ON", bg=BTN_STOP)
            self._log_event("DIST EST: started (MiDaS loading in background)")
            self._log_event("Press  D  in the video window to toggle overlay")

    # ── connection ────────────────────────────────────────────────────────
    def _apply_connection(self):
        try:
            ip=self._ip_var.get().strip();port=int(self._port_var.get());rate=float(self._rate_var.get())
        except ValueError as e: self._log_event(f"ERROR: {e}"); return
        self.ctrl.drone_ip=ip;self.ctrl.drone_port=port;self.ctrl.rate=rate
        if not self.ctrl._running: self.ctrl.start()
        self._status_label.config(text="● CONNECTED",fg=BTN_TAKE)
        self._log_event(f"CONNECTED  {ip}:{port}  @ {rate} Hz")

    def _disconnect(self):
        if self.video_adapter: self.video_adapter.stop(); self.video_adapter=None
        self.ctrl.stop()
        self._status_label.config(text="● STOPPED",fg=ACCENT2)
        self._log_event("DISCONNECTED")

    # ── log ───────────────────────────────────────────────────────────────
    def _log_event(self,msg):
        ts=time.strftime("%H:%M:%S")
        try: self.log_q.put_nowait(f"[{ts}] {msg}")
        except queue.Full: pass
    def _clear_log(self):
        self._log_text.config(state="normal");self._log_text.delete("1.0","end")
        self._log_text.config(state="disabled")

    # ── tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        msgs=[]
        try:
            while True: msgs.append(self.log_q.get_nowait())
        except queue.Empty: pass
        if msgs:
            self._log_text.config(state="normal")
            for m in msgs: self._log_text.insert("end",m+"\n")
            self._log_text.see("end");self._log_text.config(state="disabled")

        snap=self.state.snapshot()
        for name in ("throttle","yaw","pitch","roll"):
            self._stick_vars[name].set(snap[name])
            self._stick_val_lbl[name].config(text=str(int(snap[name])))

        cmd,headless=CMD_NONE,HEADLESS_ON if self.state.headless else HEADLESS_OFF

        g=self.glove
        self._imu_vars["yaw"].set(f"{g.yaw_deg:+7.1f}°")
        self._imu_vars["pitch"].set(f"{g.pitch_deg:+7.1f}°")
        self._imu_vars["roll"].set(f"{g.roll_deg:+7.1f}°")
        self._imu_vars["a2"].set(f"{g.a2_raw:7.0f}")
        self._imu_vars["a3"].set(f"{g.a3_raw:7.0f}")
        self._ahi.update_attitude(g.pitch_deg,g.roll_deg,g.yaw_deg)
        self._thr_bar.set_value(g.throttle_pct)

        for axis,val,rng in [("YAW",g.yaw_deg,180.),("PITCH",g.pitch_deg,90.),("ROLL",g.roll_deg,90.)]:
            c=self._angle_bars[axis];c.delete("all")
            w,h=260,10;mid=w//2;norm=max(-1.,min(1.,val/rng));bar=int(abs(norm)*(w//2))
            color=ACCENT2 if norm<0 else ACCENT3
            if norm>=0: c.create_rectangle(mid,1,mid+bar,h-1,fill=color,outline="")
            else:       c.create_rectangle(mid-bar,1,mid,h-1,fill=color,outline="")
            dz=(self.glove.mapper.yaw_deadzone if axis=="YAW" else self.glove.mapper.pr_deadzone)
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
                self._calib_label.config(text=f"DONE ✓  A2={m[2]:.0f}  A3={m[3]:.0f}")
            else:
                self._calib_label.config(text="DONE ✓")
            self._calib_status.config(text="● IMU ready — press O to zero",fg=ACCENT3)

        tel=self.telemetry.snapshot()
        self._tel_alt_var.set(f"{tel['altitude_cm']} cm" if tel['altitude_cm']>=0 else "N/A")
        self._tel_bat_var.set(f"{tel['battery_pct']}%"   if tel['battery_pct']>=0  else "N/A")
        self._tel_raw_var.set(tel["raw"][:47])

        self.root.after(40,self._tick)

    def on_close(self):
        if self.serial:        self.serial.stop()
        if self.video_adapter: self.video_adapter.stop()
        if self._dist_est:     self._dist_est.stop()
        self.ctrl.stop(); self.root.destroy()


# ──────────────────────────────────────────────────────────────────────────────
def main():
    if not SERIAL_AVAILABLE: print("WARNING: pip install pyserial")
    if not PIL_AVAILABLE:    print("WARNING: pip install Pillow   ← needed for video display")
    root=tk.Tk()
    app=K417GUI(root)
    root.protocol("WM_DELETE_WINDOW",app.on_close)
    root.mainloop()

if __name__=="__main__":
    main()