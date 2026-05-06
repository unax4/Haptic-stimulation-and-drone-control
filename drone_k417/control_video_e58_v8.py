#!/usr/bin/env python3
"""
e58_wifi_cam_imu_controller.py  –  E58 WIFI CAM Drone Controller
=================================================================
Glove-based IMU controller for E58 WIFI CAM protocol drones.

IMU (Arduino Nano RP2040):
  - Yaw / Pitch / Roll via Mahony AHRS filter
  - Throttle via A2 (up) and A3 (down) finger flex sensors

Serial data format:  timestamp, A3, A2, A1, A0, ax, ay, az, gx, gy, gz

──────────────────────────────────────────────────────────────
VIDEO SOCKET ARCHITECTURE  (WIFI CAM)
──────────────────────────────────────────────────────────────
The E58 WIFI CAM flow:
  1. Connect/disconnect on UDP 8080 with 42 76 / 42 77.
  2. Start control/video poke on UDP 8090 with AA 80 80 00 80 00 80 55.
  3. Continuous RC loop on UDP 8090 with CAM8 packets:
      66 roll pitch throttle yaw cmd chk 99
  4. Video appears as JPEG data in UDP payload stream and is reconstructed
      by SOI/EOI marker carving.

──────────────────────────────────────────────────────────────
Requirements:  pip install pyserial opencv-python numpy
──────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import math
import os
import re
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import logging
from pathlib import Path

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import numpy as np
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import joblib
    JOBLIB_AVAILABLE = True
except ImportError:
    JOBLIB_AVAILABLE = False

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
# Protocol constants (WIFI CAM E58)
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_IP = "192.168.4.153"
DEFAULT_SESSION_PORT = 8080
DEFAULT_CONTROL_PORT = 8090
DEFAULT_PORT = DEFAULT_CONTROL_PORT

STICK_MIN = 40
STICK_MID = 128
STICK_MAX = 220

CONNECT = bytes.fromhex("42 76")
DISCONNECT = bytes.fromhex("42 77")
START_CONTROL = bytes.fromhex("AA 80 80 00 80 00 80 55")

CMD_NONE      = 0x00
CMD_TAKEOFF   = 0x01
CMD_LAND      = 0x02
CMD_STOP      = 0x04
CMD_CALIBRATE = 0x80
CMD_CAM_UP    = 0x00
CMD_CAM_DOWN  = 0x00
HEADLESS_OFF  = 0x00
HEADLESS_ON   = 0x10

# Flip direction values for the roll/pitch stick when in flip mode
# These follow the same STICK_MAX / STICK_MIN pattern used by the A17
FLIP_FORWARD  = "forward"   # pitch → STICK_MAX
FLIP_BACKWARD = "backward"  # pitch → STICK_MIN
FLIP_LEFT     = "left"      # roll  → STICK_MIN
FLIP_RIGHT    = "right"     # roll  → STICK_MAX

# SOMERSAULT flag — XOR'd into the headless byte to trigger a flip
_SOMERSAULT_FLAG = 0x08

# ── Video protocol ─────────────────────────────────────────────────────────
# WIFI CAM video packets are received directly from the drone after CONNECT
# and START_CONTROL traffic; no frame request packets needed.
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"


def build_packet(roll, pitch, throttle, yaw, command, headless, c1, c2, c3,
                 somersault_flag=False):
    """
    Build a WIFI CAM CAM8 control packet.
    Packet format: 66 roll pitch throttle yaw cmd chk 99
    cmd byte carries command bits (e.g. 0x10 headless event).
    chk = roll XOR pitch XOR throttle XOR yaw XOR cmd
    """
    roll_i = int(max(0, min(255, roll)))
    pitch_i = int(max(0, min(255, pitch)))
    throttle_i = int(max(0, min(255, throttle)))
    yaw_i = int(max(0, min(255, yaw)))
    cmd_i = (int(command) | int(headless)) & 0xFF
    if somersault_flag:
        cmd_i |= _SOMERSAULT_FLAG
    chk = roll_i ^ pitch_i ^ throttle_i ^ yaw_i ^ cmd_i
    return bytes((0x66, roll_i, pitch_i, throttle_i, yaw_i, cmd_i, chk & 0xFF, 0x99))


# ──────────────────────────────────────────────────────────────────────────────
# Mahony AHRS
# ──────────────────────────────────────────────────────────────────────────────
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


# ──────────────────────────────────────────────────────────────────────────────
# IMU Axis Mapper
# ──────────────────────────────────────────────────────────────────────────────
class IMUAxisMapper:
    MAX_ANGLE=45.; FLEX_REST_SAMPLES=80; FLEX_THRESH_STD=2.; FLEX_NORM_SCALE=90.
    THR_NET_DEADZONE=.12; THR_EXPO=.10; THR_NEUTRAL_SNAP_STICK=2.0

    def __init__(self):
        # Pitch + Roll controls
        self.pr_deadzone=8.; self.pr_sensitivity=1.; self.pr_expo=.5
        # Yaw controls (separate — glove movement feels very different)
        self.yaw_deadzone=8.; self.yaw_sensitivity=1.; self.yaw_expo=.5
        self.flex_norm_scale=self.FLEX_NORM_SCALE
        self.thr_deadzone=self.THR_NET_DEADZONE
        self.thr_expo=self.THR_EXPO
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
        # Invert yaw direction so glove yaw matches drone yaw response.
        sy=self._a2s(-yaw,  self.yaw_deadzone, self.yaw_sensitivity, self.yaw_expo)
        sp=self._a2s(pitch, self.pr_deadzone,  self.pr_sensitivity,  self.pr_expo)
        sr=self._a2s(roll,  self.pr_deadzone,  self.pr_sensitivity,  self.pr_expo)
        if not self._flex_calibrated:
            st=self._throttle_smooth
        else:
            d2=self._flex_def(a2,2); d3=self._flex_def(a3,3)
            net=max(-1.,min(1.,d2-d3))
            s=1. if net>=0 else -1.; m=abs(net)
            dz=max(0.,min(.95,self.thr_deadzone))
            mapped=0. if m<=dz else min(1.,(m-dz)/(1.-dz))
            e_t=max(0.,min(1.,self.thr_expo))
            ct=mapped*(1.-e_t)+mapped**3*e_t
            raw=max(float(STICK_MIN),min(float(STICK_MAX),STICK_MID+s*ct*(STICK_MAX-STICK_MID)))
            self._throttle_smooth+=(raw-self._throttle_smooth)*self._throttle_alpha
            if mapped==0. and abs(self._throttle_smooth-float(STICK_MID))<=self.THR_NEUTRAL_SNAP_STICK:
                self._throttle_smooth=float(STICK_MID)
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
        self.headless=False
        self.headless_flag=False
        self._c1=0; self._c2=1; self._c3=2

        # ── new feature state ──────────────────────────────────────────
        # Camera tilt
        self.cam_up_flag   = False  # send CMD_CAM_UP once
        self.cam_down_flag = False  # send CMD_CAM_DOWN once

        # Flip — set flip_dir to one of the FLIP_* constants, then
        # flip_active=True causes the flight loop to burst the somersault
        # packets for ~600 ms and restore sticks afterwards.
        self.flip_active = False
        self.flip_dir: str | None = None  # FLIP_FORWARD/BACKWARD/LEFT/RIGHT

        # Smart landing — when True the FlightController runs its own
        # gradual descent coroutine instead of sending CMD_LAND.
        self.smart_land_active = False

    def set_imu(self, v):
        with self._lock:
            # Do not override sticks during an active flip
            if not self.flip_active:
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
            elif self.headless_flag:  cmd,self.headless_flag   =CMD_NONE,     False
            elif self.calibrate_flag: cmd,self.calibrate_flag  =CMD_CALIBRATE,False
            elif self.cam_up_flag:    cmd,self.cam_up_flag     =CMD_CAM_UP,   False
            elif self.cam_down_flag:  cmd,self.cam_down_flag   =CMD_CAM_DOWN, False
            else:                     cmd=CMD_NONE
        return cmd,(HEADLESS_ON if self.headless else HEADLESS_OFF)

    def snapshot(self):
        with self._lock:
            return {k:getattr(self,k) for k in ("throttle","yaw","pitch","roll")}


# ──────────────────────────────────────────────────────────────────────────────
# FlightController  — sends control packets; can share a socket with video
# ──────────────────────────────────────────────────────────────────────────────
class FlightController:
    """
    Sends UDP control packets at a fixed rate.

    Default mode: own private send-only socket.
    Video mode: shared socket injected by WifiCamVideoAdapter so control and
    video traffic use the same source port.
    """

    def __init__(self, state: DroneState, log_q: queue.Queue):
        self.state=state; self.log_q=log_q
        self.drone_ip=DEFAULT_IP
        self.drone_session_port=DEFAULT_SESSION_PORT
        self.drone_control_port=DEFAULT_CONTROL_PORT
        self.drone_port=DEFAULT_CONTROL_PORT
        self.rate=30.
        self._running=False; self._thread=None
        self._sock: socket.socket|None=None
        self._session_sock: socket.socket|None=None
        self._sock_lock=threading.Lock()
        self._injected=False
        self.debug=False
        self._telemetry=None  # injected by GUI after TelemetryParser is created
        # Software headless mode state: estimate drone yaw from commanded yaw.
        self._yaw_heading_deg = 0.0
        self._headless_ref_deg = 0.0
        self._headless_prev = False
        self._last_loop_t = None
        self._max_yaw_rate_dps = 160.0

    @staticmethod
    def _wrap_deg(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    def _update_heading_estimate(self, yaw_stick: int, dt: float):
        yaw_norm = (float(yaw_stick) - float(STICK_MID)) / float(STICK_MAX - STICK_MID)
        yaw_norm = max(-1.0, min(1.0, yaw_norm))
        self._yaw_heading_deg = self._wrap_deg(
            self._yaw_heading_deg + yaw_norm * self._max_yaw_rate_dps * dt
        )

    def _apply_headless_transform(self, roll_i: int, pitch_i: int) -> tuple[int, int]:
        # Rotate pilot-frame stick intent into drone body frame.
        rel_deg = self._wrap_deg(self._yaw_heading_deg - self._headless_ref_deg)
        th = math.radians(rel_deg)
        x = float(roll_i - STICK_MID)
        y = float(pitch_i - STICK_MID)
        xr = x * math.cos(th) - y * math.sin(th)
        yr = x * math.sin(th) + y * math.cos(th)
        out_roll = int(max(STICK_MIN, min(STICK_MAX, round(STICK_MID + xr))))
        out_pitch = int(max(STICK_MIN, min(STICK_MAX, round(STICK_MID + yr))))
        return out_roll, out_pitch

    def inject_socket(self, sock: socket.socket):
        """Called by WifiCamVideoAdapter to share the video socket for control."""
        with self._sock_lock:
            if self._sock and not self._injected:
                try: self._sock.close()
                except Exception: pass
            self._sock = sock
            self._injected = True

    def release_socket(self):
        """Drop shared socket (video stopped) so FlightController uses own socket."""
        with self._sock_lock:
            if self._sock and self._injected:
                # don't close adapter-owned socket here; video adapter owns it
                self._sock = None
                self._injected = False

    def _ensure_sock(self):
        with self._sock_lock:
            if self._sock is None:
                s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                s.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,65536)
                self._sock=s
                self._injected=False
            if self._session_sock is None:
                self._session_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._session_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def _close_sock(self):
        with self._sock_lock:
            if self._sock and not self._injected:
                try: self._sock.close()
                except Exception: pass
            self._sock=None
            self._injected=False
            if self._session_sock:
                try: self._session_sock.close()
                except Exception: pass
                self._session_sock=None

    def _send(self, pkt):
        with self._sock_lock:
            if self._sock is None: return
            try: self._sock.sendto(pkt,(self.drone_ip,self.drone_control_port))
            except OSError: pass

    def send_connect(self):
        self._ensure_sock()
        with self._sock_lock:
            if self._session_sock is None:
                return
            try:
                self._session_sock.sendto(CONNECT, (self.drone_ip, self.drone_session_port))
            except OSError:
                return
        try: self.log_q.put_nowait("WIFI CAM: CONNECT sent")
        except queue.Full: pass

    def send_disconnect(self):
        self._ensure_sock()
        with self._sock_lock:
            if self._session_sock is None:
                return
            try:
                self._session_sock.sendto(DISCONNECT, (self.drone_ip, self.drone_session_port))
            except OSError:
                return
        try: self.log_q.put_nowait("WIFI CAM: DISCONNECT sent")
        except queue.Full: pass

    def send_start_control(self, burst=6):
        self._ensure_sock()
        burst = max(1, int(burst))
        for _ in range(burst):
            self._send(START_CONTROL)
            time.sleep(0.03)
        try: self.log_q.put_nowait(f"WIFI CAM: START burst x{burst}")
        except queue.Full: pass

    def _loop(self):
        interval=1./self.rate; pkt_num=0
        try:
            while self._running:
                self._ensure_sock()
                t0=time.time()
                if self._last_loop_t is None:
                    dt = interval
                else:
                    dt = max(0.001, min(0.1, t0 - self._last_loop_t))
                self._last_loop_t = t0

                # ── flip burst ──────────────────────────────────────────────
                if self.state.flip_active:
                    self._do_flip()
                    continue

                # ── normal packet ────────────────────────────────────────────────
                cmd,headless=self.state.consume_flags()
                c1,c2,c3=self.state.next_counters()
                snap=self.state.snapshot()

                # Keep a lightweight heading estimate from commanded yaw.
                self._update_heading_estimate(int(snap["yaw"]), dt)

                headless_active = bool(self.state.headless)
                if headless_active and not self._headless_prev:
                    self._headless_ref_deg = self._yaw_heading_deg
                    try: self.log_q.put_nowait("HEADLESS: ON (pilot frame locked)")
                    except queue.Full: pass
                elif (not headless_active) and self._headless_prev:
                    try: self.log_q.put_nowait("HEADLESS: OFF")
                    except queue.Full: pass
                self._headless_prev = headless_active

                # App-sniffed calibration packet: cmd=0x80 with all analog
                # channels set to stick-mid (0x80).
                if cmd == CMD_CALIBRATE:
                    roll = pitch = throttle = yaw = STICK_MID
                else:
                    roll = int(snap["roll"])
                    pitch = int(snap["pitch"])
                    throttle = int(snap["throttle"])
                    yaw = int(snap["yaw"])

                if headless_active and cmd != CMD_CALIBRATE and not self.state.flip_active:
                    roll, pitch = self._apply_headless_transform(roll, pitch)

                pkt=build_packet(roll,pitch,throttle,yaw,cmd,headless,c1,c2,c3)
                self._send(pkt); pkt_num+=1
                if self.debug and pkt_num%80==0:
                    msg=(f"#{pkt_num:06d} T:{int(snap['throttle'])} "
                         f"Y:{int(snap['yaw'])} P:{int(snap['pitch'])} R:{int(snap['roll'])}")
                    try: self.log_q.put_nowait(msg)
                    except queue.Full: pass
                elapsed = time.time() - t0
                sleep_time = max(0.001, interval - elapsed)
                time.sleep(sleep_time)
        finally:
            self._close_sock()

    # ── camera tilt helpers ────────────────────────────────────────────────
    def _send_camera_tilt(self, direction: str):
        """
        Send a burst of camera-tilt packets.
        'up'   → CMD_CAM_UP  (0x05)
        'down' → CMD_CAM_DOWN (0x06)
        Mirroring how the A17 sends these as separate, repeated commands.
        """
        cmd = CMD_CAM_UP if direction == "up" else CMD_CAM_DOWN
        snap = self.state.snapshot()
        _,headless = self.state.consume_flags()
        for _ in range(8):   # ~8 packets at 40 Hz ≈ 200 ms
            c1,c2,c3 = self.state.next_counters()
            pkt = build_packet(int(snap["roll"]),int(snap["pitch"]),
                               int(snap["throttle"]),int(snap["yaw"]),
                               cmd,headless,c1,c2,c3)
            self._send(pkt)
            time.sleep(0.025)
        msg = f"CAM: tilt {direction}"
        try: self.log_q.put_nowait(msg)
        except queue.Full: pass

    # ── flip helper ───────────────────────────────────────────────────────
    def _do_flip(self):
        """
        Execute a flip burst:
        1. Determine direction sticks from state.flip_dir.
        2. Send ~20 packets with the SOMERSAULT flag set + direction stick.
        3. Restore sticks and clear flip_active.

        Timing derived from the A17 source: the controller fires the
        somersault command while holding a directional input, then releases.
        ~20 packets at 40 Hz ≈ 500 ms is enough for one full rotation.
        """
        direction = self.state.flip_dir
        # Direction stick values
        if direction == FLIP_FORWARD:
            flip_pitch = STICK_MAX; flip_roll = STICK_MID
        elif direction == FLIP_BACKWARD:
            flip_pitch = STICK_MIN; flip_roll = STICK_MID
        elif direction == FLIP_RIGHT:
            flip_pitch = STICK_MID; flip_roll = STICK_MAX
        elif direction == FLIP_LEFT:
            flip_pitch = STICK_MID; flip_roll = STICK_MIN
        else:  # default to forward
            flip_pitch = STICK_MAX; flip_roll = STICK_MID

        snap = self.state.snapshot()
        _,headless = self.state.consume_flags()

        # Burst packets with somersault flag
        for _ in range(20):
            c1,c2,c3 = self.state.next_counters()
            pkt = build_packet(flip_roll, flip_pitch,
                               int(snap["throttle"]), int(snap["yaw"]),
                               CMD_NONE, headless, c1, c2, c3,
                               somersault_flag=True)
            self._send(pkt)
            time.sleep(1./self.rate)

        # Send 10 neutral packets to let the drone settle
        for _ in range(10):
            c1,c2,c3 = self.state.next_counters()
            pkt = build_packet(STICK_MID, STICK_MID,
                               int(snap["throttle"]), int(snap["yaw"]),
                               CMD_NONE, headless, c1, c2, c3)
            self._send(pkt)
            time.sleep(1./self.rate)

        self.state.flip_active = False
        self.state.flip_dir    = None
        msg = f"FLIP: {direction} — done"
        try: self.log_q.put_nowait(msg)
        except queue.Full: pass

    def start(self):
        if self._running: return
        self._running=True
        self._thread=threading.Thread(target=self._loop,daemon=True,name="FlightCtrl")
        self._thread.start()

    def stop(self):
        self._running=False
        if self._thread:
            self._thread.join(timeout=2.)
            self._thread=None
        # _close_sock() is called by _loop's finally block; call here too
        # in case start() was never called.
        self._close_sock()

    def reconnect(self,ip,port,rate):
        was=self._running
        if was: self.stop()
        self.drone_ip=ip
        self.drone_control_port=port
        self.drone_port=port
        self.rate=rate
        if was: self.start()


# ──────────────────────────────────────────────────────────────────────────────
# WifiCamVideoAdapter
# ──────────────────────────────────────────────────────────────────────────────
class WifiCamVideoAdapter:
    """Receive WIFI CAM UDP payloads and reconstruct JPEG frames."""

    def __init__(self, drone_ip=DEFAULT_IP, port=DEFAULT_CONTROL_PORT,
                 session_port=DEFAULT_SESSION_PORT, log_cb=None):
        self.drone_ip = drone_ip
        self.port = port
        self.session_port = session_port
        self._log_cb = log_cb

        # Mirror known-good behavior: independent session/control sockets with
        # auto-picked local source ports, and receive from both.
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self._sock.bind(("", 0))
        self._sock.setblocking(False)

        self._session_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._session_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._session_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        self._session_sock.bind(("", 0))
        self._session_sock.setblocking(False)

        self.local_control_port = self._sock.getsockname()[1]
        self.local_session_port = self._session_sock.getsockname()[1]

        self._frame_q: queue.Queue = queue.Queue(maxsize=2)
        self._running = True
        self._video_frag = bytearray()
        self.frames_ok = 0
        self.rx_datagrams = 0
        self.rx_from_drone = 0
        self._started_at = time.time()
        self._last_diag_ts = 0.0
        self._diag_stage = 0

        self._log(
            f"VIDEO: RX sockets ready (local ctrl:{self.local_control_port} "
            f"local session:{self.local_session_port})"
        )
        threading.Thread(target=self._rx_loop, daemon=True, name="WifiCam-RX").start()

    def send_connect(self):
        try:
            self._session_sock.sendto(CONNECT, (self.drone_ip, self.session_port))
            self._log(
                f"VIDEO: CONNECT sent to {self.drone_ip}:{self.session_port} "
                f"(local session {self.local_session_port})"
            )
        except OSError as e:
            self._log(f"VIDEO: CONNECT failed ({e})")

    def send_disconnect(self):
        try:
            self._session_sock.sendto(DISCONNECT, (self.drone_ip, self.session_port))
            self._log(f"VIDEO: DISCONNECT sent to {self.drone_ip}:{self.session_port}")
        except OSError as e:
            self._log(f"VIDEO: DISCONNECT failed ({e})")

    def send_start_burst(self, burst=6):
        burst = max(1, int(burst))
        sent = 0
        for _ in range(burst):
            try:
                self._sock.sendto(START_CONTROL, (self.drone_ip, self.port))
                sent += 1
            except OSError:
                pass
            time.sleep(0.03)
        self._log(f"VIDEO: START burst x{sent}/{burst} sent to {self.drone_ip}:{self.port}")

    def get_frame(self, timeout=0):
        try:
            return self._frame_q.get(timeout=timeout) if timeout > 0 else self._frame_q.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            self._session_sock.close()
        except Exception:
            pass

    def _log(self, msg):
        if self._log_cb is None:
            return
        try:
            self._log_cb(msg)
        except Exception:
            pass

    def _emit_jpeg(self, jpeg):
        self.frames_ok += 1
        try:
            self._frame_q.get_nowait()
        except queue.Empty:
            pass
        try:
            self._frame_q.put_nowait(jpeg)
        except queue.Full:
            pass

    def _extract_jpegs(self, payload):
        # Fast path: one or more complete JPEGs in this datagram.
        found = []
        pos = 0
        while True:
            soi = payload.find(_SOI, pos)
            if soi < 0:
                break
            eoi = payload.find(_EOI, soi + 2)
            if eoi < 0:
                break
            jpg = payload[soi:eoi + 2]
            if len(jpg) >= 300:
                found.append(jpg)
            pos = eoi + 2
        if found:
            self._video_frag.clear()
            for jpg in found:
                self._emit_jpeg(jpg)
            return

        # Fragment path: accumulate from SOI to EOI across datagrams.
        soi = payload.find(_SOI)
        if soi >= 0:
            self._video_frag = bytearray(payload[soi:])
        elif self._video_frag:
            self._video_frag.extend(payload)

        if len(self._video_frag) > 2 * 1024 * 1024:
            self._video_frag.clear()
            return

        if self._video_frag:
            eoi = self._video_frag.find(_EOI)
            if eoi >= 0:
                jpg = bytes(self._video_frag[:eoi + 2])
                self._video_frag = bytearray(self._video_frag[eoi + 2:])
                if len(jpg) >= 300:
                    self._emit_jpeg(jpg)

    def _maybe_report_diagnostics(self):
        now = time.time()
        if now - self._last_diag_ts < 1.5:
            return
        self._last_diag_ts = now

        elapsed = now - self._started_at

        if self.frames_ok > 0 and self._diag_stage < 10:
            self._diag_stage = 10
            self._log(
                f"VIDEO: stream OK ({self.frames_ok} frames, "
                f"{self.rx_from_drone} drone datagrams)"
            )
            return

        if elapsed < 2.0:
            return

        if self.rx_datagrams == 0 and self._diag_stage < 1:
            self._diag_stage = 1
            self._log(
                "VIDEO DIAG: no UDP datagrams received. "
                "Likely CONNECT/START sequence missing, wrong port, or host firewall blocks UDP."
            )
            return

        if self.rx_datagrams > 0 and self.rx_from_drone == 0 and self._diag_stage < 2:
            self._diag_stage = 2
            self._log(
                f"VIDEO DIAG: UDP datagrams seen but not from drone IP {self.drone_ip}. "
                "Check selected drone IP / active network adapter."
            )
            return

        if self.rx_from_drone >= 20 and self.frames_ok == 0 and self._diag_stage < 3:
            self._diag_stage = 3
            self._log(
                "VIDEO DIAG: drone packets arriving but no valid JPEG frames detected. "
                "Protocol payload may differ or stream is not yet enabled on drone."
            )
            return

    def _rx_loop(self):
        import select

        while self._running:
            try:
                readable, _, _ = select.select([self._sock, self._session_sock], [], [], 0.02)
                if not readable:
                    self._maybe_report_diagnostics()
                    continue
                for sock in readable:
                    while True:
                        try:
                            payload, addr = sock.recvfrom(65535)
                        except BlockingIOError:
                            break
                        except OSError:
                            break

                        if not payload:
                            continue

                        self.rx_datagrams += 1
                        if addr[0] != self.drone_ip:
                            continue

                        self.rx_from_drone += 1
                        self._extract_jpegs(payload)

                self._maybe_report_diagnostics()
            except (OSError, ValueError):
                if self._running:
                    time.sleep(0.01)
                break


# ──────────────────────────────────────────────────────────────────────────────
# OpenCV display loop — identical to pruebas.py's run_display()
# ──────────────────────────────────────────────────────────────────────────────

def _run_video_display(adapter: WifiCamVideoAdapter, dist_est_or_gui=None):
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
    window = "E58 WIFI CAM Live View"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    def _get_screen_size():
        if hasattr(dist_est_or_gui, "root"):
            try:
                return int(dist_est_or_gui.root.winfo_screenwidth()), int(dist_est_or_gui.root.winfo_screenheight())
            except Exception:
                pass
        try:
            probe = tk.Tk()
            probe.withdraw()
            size = (int(probe.winfo_screenwidth()), int(probe.winfo_screenheight()))
            probe.destroy()
            return size
        except Exception:
            return 1280, 720

    screen_w, screen_h = _get_screen_size()
    cv2.resizeWindow(window, screen_w, screen_h)
    try:
        cv2.moveWindow(window, 0, 0)
    except Exception:
        pass

    def _auto_orient_frame(frame):
        if frame is None:
            return None
        height, width = frame.shape[:2]
        if height > width:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        return frame

    # If we were passed the GUI object, forward video-window hotkeys to Tk.
    gui = dist_est_or_gui if hasattr(dist_est_or_gui, "root") else None

    def _post_gui_call(method_name: str, *args):
        if gui is None:
            return
        try:
            method = getattr(gui, method_name, None)
            if method is not None:
                gui.root.after(0, method, *args)
        except Exception:
            # GUI may be closing while the video thread is still unwinding.
            pass

    # OpenCV doesn't provide reliable key-release events, so for flip combos
    # we arm on "F" and accept one arrow key shortly afterwards.
    flip_armed_until = 0.0

    # Common waitKeyEx codes on Windows.
    KEY_UP = 2490368
    KEY_DOWN = 2621440
    KEY_LEFT = 2424832
    KEY_RIGHT = 2555904
    KEY_PGUP = 2162688
    KEY_PGDN = 2228224

    placeholder = np.zeros((360, 640, 3), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = "Waiting for E58 WIFI CAM video..."
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
                decoded = _auto_orient_frame(decoded)
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

        key = cv2.waitKeyEx(1)
        key_ascii = key & 0xFF if key != -1 else -1

        if key_ascii in (ord("q"), ord("Q")):
            adapter.stop()
            break
        elif key_ascii in (ord("d"), ord("D")) and dist_est is not None:
            dist_on = not dist_on

        # Mirror GUI keyboard controls while the OpenCV window has focus.
        if key == -1:
            continue

        if key_ascii in (ord("t"), ord("T")):
            _post_gui_call("_cmd_takeoff")
        elif key_ascii in (ord("l"), ord("L")):
            _post_gui_call("_cmd_land")
        elif key_ascii == 32:
            _post_gui_call("_cmd_stop")
        elif key_ascii in (ord("h"), ord("H")):
            _post_gui_call("_toggle_headless")
        elif key_ascii in (ord("c"), ord("C")):
            _post_gui_call("_cmd_calibrate")
        elif key_ascii in (ord("o"), ord("O")):
            _post_gui_call("_imu_zero")
        elif key in (KEY_PGUP,):
            _post_gui_call("_cmd_cam_up")
        elif key in (KEY_PGDN,):
            _post_gui_call("_cmd_cam_down")
        elif key_ascii in (ord("f"), ord("F")):
            flip_armed_until = time.time() + 0.8
        elif key in (KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT):
            if time.time() <= flip_armed_until:
                if key == KEY_UP:
                    _post_gui_call("_cmd_flip", FLIP_FORWARD)
                elif key == KEY_DOWN:
                    _post_gui_call("_cmd_flip", FLIP_BACKWARD)
                elif key == KEY_LEFT:
                    _post_gui_call("_cmd_flip", FLIP_LEFT)
                elif key == KEY_RIGHT:
                    _post_gui_call("_cmd_flip", FLIP_RIGHT)
                flip_armed_until = 0.0

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
    _A0_A1_RE = re.compile(r"A0\s*:\s*(-?\d+(?:\.\d+)?)\s+A1\s*:\s*(-?\d+(?:\.\d+)?)", re.IGNORECASE)

    def __init__(self,port,baud,on_data,on_status,log_q,on_nn=None):
        self.port=port;self.baud=baud;self.on_data=on_data
        self.on_status=on_status;self.log_q=log_q
        self.on_nn=on_nn
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
                    if line: self._parse(line)
            except Exception as e:
                self.on_status(f"Read error: {e}"); time.sleep(0.5)
        ser.close()

    def _parse(self, line):
        # Same A1/A0 extraction behavior as neural/live_glove_position_viewer.py
        m=self._A0_A1_RE.search(line)
        if m and self.on_nn is not None:
            try:
                a0=float(m.group(1)); a1=float(m.group(2))
                self.on_nn(a1,a0)
            except ValueError:
                pass

        parts=line.split(",")
        if "," in line and len(parts)>=5 and self.on_nn is not None:
            try:
                # Legacy CSV: timestamp,A3,A2,A1,A0,...
                self.on_nn(float(parts[3].strip()), float(parts[4].strip()))
            except ValueError:
                pass

        if len(parts)<11: return
        try:
            vals=[float(v) for v in parts[:11]]
            # ts, A3, A2, A1, A0, ax, ay, az, gx, gy, gz
            a3=vals[1];a2=vals[2];a1=vals[3];a0=vals[4]
            ax,ay,az,gx,gy,gz=vals[5:11]
            self.on_data(a0,a1,a2,a3,ax,ay,az,gx,gy,gz)
        except (ValueError,IndexError): pass


# ──────────────────────────────────────────────────────────────────────────────
# GloveController
# ──────────────────────────────────────────────────────────────────────────────
class GloveController:
    CALIB_SAMPLES=150
    # Auto recenter when hand posture changed (reposition) and then held still.
    AUTO_RECENTER_ENABLED = True
    AUTO_RECENTER_COOLDOWN_S = 2.0
    AUTO_RECENTER_STILL_HOLD_S = 0.45
    AUTO_RECENTER_STILL_GYRO_DPS = 10.0
    AUTO_RECENTER_MOVE_GYRO_DPS = 35.0
    AUTO_RECENTER_MIN_MOVE_DEG = 18.0
    AUTO_RECENTER_THR_NEUTRAL_STICK = 8.0

    def __init__(self,state,log_q):
        self.state=state;self.log_q=log_q
        self.ahrs=MahonyFilter();self.mapper=IMUAxisMapper()
        self._last_t=time.time()
        self.calibrating=True;self.calib_count=0;self.enabled=True
        self.yaw_deg=self.pitch_deg=self.roll_deg=0.
        self.a0_raw=self.a1_raw=self.a2_raw=self.a3_raw=self.throttle_pct=0.
        self.nn_pred_class=-1
        self.nn_margin=0.
        self.flex_calibrated=False;self.flex_rest_mean=[0.]*4
        self._nn_model=None
        self._nn_enabled=False
        self.auto_recenter_enabled = self.AUTO_RECENTER_ENABLED
        self._auto_move_deg = 0.0
        self._auto_still_s = 0.0
        self._auto_recenter_cooldown_until = 0.0
        self._load_nn_model()

    def _load_nn_model(self):
        if not JOBLIB_AVAILABLE:
            self._log("NN: joblib not installed (pip install joblib)")
            return
        try:
            model_path = Path(__file__).resolve().parent / "neural" / "glove_fcnn_model.joblib"
            if not model_path.exists():
                self._log("NN: model not found (neural/glove_fcnn_model.joblib)")
                return
            saved = joblib.load(model_path)
            self._nn_model = saved["model"] if isinstance(saved, dict) and "model" in saved else saved
            self._nn_enabled = True
            self._log("NN: loaded glove_fcnn_model.joblib")
        except Exception as e:
            self._nn_enabled = False
            self._nn_model = None
            self._log(f"NN: load failed ({e})")

    def _predict_position(self, a1, a0):
        if not self._nn_enabled or self._nn_model is None:
            self.nn_pred_class = -1
            self.nn_margin = 0.
            return
        try:
            x = np.array([[float(a1), float(a0)]], dtype=float)
            # Match live_glove_position_viewer.py: direct class prediction.
            self.nn_pred_class = int(self._nn_model.predict(x)[0])
            self.nn_margin = 1.0
        except Exception:
            self.nn_pred_class = -1
            self.nn_margin = 0.

    def on_nn_sample(self, a1, a0):
        self.a1_raw=a1; self.a0_raw=a0
        self._predict_position(a1,a0)

    def reset_calibration(self):
        self.ahrs=MahonyFilter();self.mapper.reset_flex_calibration()
        self.calibrating=True;self.calib_count=0;self.flex_calibrated=False
        self._log("IMU: re-calibrating gyro + flex rest baseline…")

    def capture_zero(self):
        self.ahrs.capture_offset(); self._log("IMU: orientation zeroed ✓")

    def _update_auto_recenter(self, dt, gx_dps, gy_dps, gz_dps, sticks):
        if not self.auto_recenter_enabled:
            return
        if self.calibrating or not self.enabled:
            self._auto_move_deg = 0.0
            self._auto_still_s = 0.0
            return

        now = time.time()
        if now < self._auto_recenter_cooldown_until:
            return

        gyro_norm = math.sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps)
        thr_neutral = abs(float(sticks["throttle"]) - float(STICK_MID)) <= self.AUTO_RECENTER_THR_NEUTRAL_STICK

        # Only learn reposition events while throttle intent is neutral.
        if not thr_neutral:
            self._auto_move_deg = max(0.0, self._auto_move_deg - 2.5 * dt)
            self._auto_still_s = 0.0
            return

        if gyro_norm >= self.AUTO_RECENTER_MOVE_GYRO_DPS:
            self._auto_move_deg += gyro_norm * dt
            self._auto_still_s = 0.0
        elif gyro_norm <= self.AUTO_RECENTER_STILL_GYRO_DPS:
            self._auto_still_s += dt
        else:
            self._auto_still_s = max(0.0, self._auto_still_s - 0.5 * dt)

        # Trigger one-shot zero only after a real move+settle sequence.
        if (self._auto_move_deg >= self.AUTO_RECENTER_MIN_MOVE_DEG
                and self._auto_still_s >= self.AUTO_RECENTER_STILL_HOLD_S):
            self.ahrs.capture_offset()
            self._auto_move_deg = 0.0
            self._auto_still_s = 0.0
            self._auto_recenter_cooldown_until = now + self.AUTO_RECENTER_COOLDOWN_S
            self._log("IMU: auto-zero after reposition ✓")

    def on_sensor_data(self,a0,a1,a2,a3,ax_r,ay_r,az_r,gx_r,gy_r,gz_r):
        ax=ay_r;ay=-ax_r;az=az_r;gx=gy_r;gy=-gx_r;gz=gz_r
        gr=[math.radians(v) for v in (gx,gy,gz)]
        now=time.time();dt=min(now-self._last_t,.05);self._last_t=now
        self.a0_raw=a0;self.a1_raw=a1;self.a2_raw=a2;self.a3_raw=a3
        self._predict_position(a1, a0)
        if self.calibrating:
            gd=self.ahrs.add_gyro_sample(*gr,self.CALIB_SAMPLES)
            fd=self.mapper.add_flex_rest_sample(a0,a1,a2,a3)
            self.calib_count+=1
            if gd and fd:
                self.calibrating=False;self.flex_calibrated=True
                self.flex_rest_mean=list(self.mapper._flex_rest_mean)
                m=self.mapper._flex_rest_mean
                self._log(f"IMU calibrated ✓  A0={m[0]:.0f}  A1={m[1]:.0f}  A2={m[2]:.0f}  A3={m[3]:.0f}  — press O to zero.")
            return
        self.ahrs.update(ax,ay,az,*gr,dt)
        yaw,pitch,roll=self.ahrs.get_euler_relative()
        self.yaw_deg=yaw;self.pitch_deg=pitch;self.roll_deg=roll
        sticks=self.mapper.compute(yaw,pitch,roll,a2,a3)
        self._update_auto_recenter(dt, gx, gy, gz, sticks)
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
        self.video_adapter:WifiCamVideoAdapter|None=None
        self._video_thread:threading.Thread|None=None
        self._dist_est=None
        self._flip_key_held=False  # True while F key is held for flip+direction combos
        self._nn_pred_last=-1
        self._nn_last_class=-1
        self._nn_class_start_ts=0.0
        self._nn_stable_pos=-1
        self._nn_last_action_class=-1
        self._nn_last_action_ts=0.0
        self._nn_cmd_until=0.0
        self._nn_hold_s=0.35
        self._nn_cooldown_s=0.9
        self._nn_min_margin=0.05
        self._nn_min_margin_land=0.04
        self._build_ui();self._bind_keys();self._tick()
        # Wire telemetry into FlightController so smart-land can read altitude
        self.ctrl._telemetry=self.telemetry

    def _build_ui(self):
        r=self.root; r.title("K417 // IMU Glove Controller")
        r.configure(bg=DARK_BG);r.resizable(True,True)
        r.geometry("1160x860");r.minsize(1040,760)
        ttk.Style().theme_use("clam")
        hdr=tk.Frame(r,bg=DARK_BG);hdr.pack(fill="x",padx=20,pady=(14,4))
        tk.Label(hdr,text="K417",fg=ACCENT,bg=DARK_BG,font=FONT_TITLE).pack(side="left")
        tk.Label(hdr,text="  //  IMU GLOVE CONTROLLER",fg=TEXT_DIM,bg=DARK_BG,font=FONT_BIG).pack(side="left")
        self._status_label=tk.Label(hdr,text="● STOPPED",fg=ACCENT2,bg=DARK_BG,font=FONT_LABEL)
        self._status_label.pack(side="right")
        tk.Frame(r,height=1,bg=ACCENT).pack(fill="x",padx=20,pady=(0,8))
        cols=tk.Frame(r,bg=DARK_BG);cols.pack(fill="both",expand=True,padx=16)
        left=tk.Frame(cols,bg=DARK_BG);left.pack(side="left",fill="both")
        centre=tk.Frame(cols,bg=DARK_BG);centre.pack(side="left",fill="both",padx=10,expand=True)
        right=tk.Frame(cols,bg=DARK_BG);right.pack(side="right",fill="both")
        self._build_connection(left);self._build_glove(left);self._build_keys_legend(left);self._build_nn_cmd_panel(left)
        self._build_imu(centre);self._build_sensitivity(centre);self._build_video(centre)
        self._build_sticks(right);self._build_commands(right)
        self._build_log(r)

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
        self._port_var=row("Control UDP",str(DEFAULT_CONTROL_PORT))
        self._rate_var=row("Rate (Hz)","40")
        br=tk.Frame(f,bg=PANEL_BG);br.pack(fill="x",pady=(6,0))
        tk.Button(br,text="CONNECT",bg="#0d47a1",fg=TEXT_MAIN,font=FONT_BTN,
                  relief="flat",cursor="hand2",command=self._apply_connection).pack(side="left",padx=2)
        tk.Button(br,text="START",bg="#1b5e20",fg=TEXT_MAIN,font=FONT_BTN,
              relief="flat",cursor="hand2",command=self._start_control).pack(side="left",padx=2)
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
                                  ("ROLL","roll",IMU_COLOR),("A0","a0",BTN_TAKE),
                                  ("A1","a1",ACCENT2),("A2↑","a2",ACCENT3),("A3↓","a3",IMU_COLOR)]:
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
        param("Deadzone (°)", 0.,30., 10., lambda v:setattr(self.glove.mapper,"yaw_deadzone",v), res=0.5, color=ACCENT2)
        param("Sensitivity",  0.1,3., 1., lambda v:setattr(self.glove.mapper,"yaw_sensitivity",v),       color=ACCENT2)
        param("Expo curve",   0.,1.,  .5, lambda v:setattr(self.glove.mapper,"yaw_expo",v),               color=ACCENT2)

        # ── Throttle ──────────────────────────────────────────────────────
        tk.Label(f,text="THROTTLE",fg=BTN_TAKE,bg=PANEL_BG,font=FONT_SMALL).pack(anchor="w",pady=(6,0))
        param("Deadzone",     0.,1., IMUAxisMapper.THR_NET_DEADZONE,
              lambda v:setattr(self.glove.mapper,"thr_deadzone",v),res=0.01,color=BTN_TAKE)
        param("Expo",         0.,1., IMUAxisMapper.THR_EXPO,
              lambda v:setattr(self.glove.mapper,"thr_expo",v),res=0.01,color=BTN_TAKE)
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
        def btn(text,color,cmd,row,col,span=1):
            b=tk.Button(f,text=text,bg=color,fg="white",activebackground=color,
                        font=FONT_BTN,relief="flat",cursor="hand2",width=12,height=1,command=cmd)
            b.grid(row=row,column=col,padx=3,pady=3,columnspan=span);return b
        btn("⬆  TAKEOFF",BTN_TAKE,self._cmd_takeoff,0,0)
        btn("⬇  LAND",BTN_LAND,self._cmd_land,0,1)
        self._btn_stop=btn("✕  STOP",BTN_STOP,self._cmd_stop,1,0)
        self._btn_head=btn("⧖  HEADLESS",BTN_HEAD,self._toggle_headless,1,1)
        btn("◎  CALIBRATE",BTN_CAL,self._cmd_calibrate,2,0)
        self._debug_btn=btn("⚙  DEBUG OFF","#37474f",self._toggle_debug,2,1)

        # ── camera tilt ───────────────────────────────────────────────
        tk.Frame(f,height=1,bg=TEXT_DIM).grid(row=3,column=0,columnspan=2,sticky="ew",pady=(6,2))
        tk.Label(f,text="CAMERA TILT  [PgUp / PgDn]",fg=TEXT_DIM,bg=PANEL_BG,
                 font=FONT_LABEL).grid(row=4,column=0,columnspan=2,sticky="w",padx=4)
        btn("▲  CAM UP","#006064",self._cmd_cam_up,5,0)
        btn("▼  CAM DOWN","#006064",self._cmd_cam_down,5,1)

        # ── flips ─────────────────────────────────────────────────────
        tk.Frame(f,height=1,bg=TEXT_DIM).grid(row=6,column=0,columnspan=2,sticky="ew",pady=(6,2))
        tk.Label(f,text="FLIPS  [F+arrow key]  — must be airborne",fg=TEXT_DIM,bg=PANEL_BG,
                 font=FONT_LABEL).grid(row=7,column=0,columnspan=2,sticky="w",padx=4)
        btn("↑  FLIP FWD", "#4a148c",lambda:self._cmd_flip(FLIP_FORWARD), 8,0)
        btn("↓  FLIP BACK","#4a148c",lambda:self._cmd_flip(FLIP_BACKWARD),8,1)
        btn("←  FLIP LEFT","#4a148c",lambda:self._cmd_flip(FLIP_LEFT),    9,0)
        btn("→  FLIP RIGHT","#4a148c",lambda:self._cmd_flip(FLIP_RIGHT),  9,1)

    def _build_keys_legend(self,parent):
        f=self._panel(parent,"KEYBOARD  OVERRIDES")
        items=[("T","Takeoff"),("L","Land (smart)"),("SPACE","Emergency stop"),
               ("H","Headless"),("C","Calibrate"),("O","Zero IMU"),
               ("F5","Re-calibrate"),("S","Snapshot"),
               ("PgUp","Cam tilt up"),("PgDn","Cam tilt down"),
               ("F+↑","Flip forward"),("F+↓","Flip back"),
               ("F+←","Flip left"),("F+→","Flip right")]
        for i,(key,desc) in enumerate(items):
            rr,cc=i%7,(i//7)*2
            tk.Label(f,text=key,fg=ACCENT,bg=PANEL_BG,font=FONT_MONO,width=6,anchor="e").grid(
                row=rr,column=cc,padx=(0,4),pady=1,sticky="e")
            tk.Label(f,text=desc,fg=TEXT_DIM,bg=PANEL_BG,font=FONT_LABEL,anchor="w").grid(
                row=rr,column=cc+1,padx=(0,16),pady=1,sticky="w")

    def _build_nn_cmd_panel(self,parent):
        f=self._panel(parent,"GLOVE  POSITION  NN")
        self._nn_pos_var=tk.StringVar(value="POS: -1")
        tk.Label(f,textvariable=self._nn_pos_var,fg=ACCENT,bg=PANEL_BG,
                 font=("Courier New",13,"bold"),anchor="w").pack(fill="x",pady=(0,4))
        self._nn_cmd_var=tk.StringVar(value="NN CMD: none")
        self._nn_cmd_card=tk.Frame(f,bg=CARD_BG,highlightthickness=2,highlightbackground=TEXT_DIM)
        self._nn_cmd_card.pack(fill="x",pady=(2,2))
        self._nn_cmd_label=tk.Label(self._nn_cmd_card,textvariable=self._nn_cmd_var,
                                    fg=TEXT_DIM,bg=CARD_BG,font=("Courier New",13,"bold"),
                                    justify="center",pady=8,wraplength=320)
        self._nn_cmd_label.pack(fill="x")
        state_txt = "model ready" if self.glove._nn_enabled else "model unavailable"
        state_col = ACCENT3 if self.glove._nn_enabled else ACCENT2
        tk.Label(f,text=f"Source: joblib ({state_txt})",fg=state_col,bg=PANEL_BG,
                 font=FONT_SMALL).pack(anchor="w",pady=(4,0))

    def _set_nn_cmd_status(self,msg,color=ACCENT3,hold_s=2.5):
        self._nn_cmd_var.set(msg)
        self._nn_cmd_label.config(fg=color)
        self._nn_cmd_card.config(highlightbackground=color)
        self._nn_cmd_until=time.time()+hold_s

    def _trigger_nn_action(self, pred: int):
        if pred == 2:
            self._cmd_stop()
            self._set_nn_cmd_status("NN CMD: STOP", BTN_STOP)
        elif pred == 3:
            self._cmd_takeoff()
            self._set_nn_cmd_status("NN CMD: TAKEOFF", BTN_TAKE)
        elif pred == 4:
            self._imu_zero()
            self._set_nn_cmd_status("NN CMD: ZERO", IMU_COLOR)
        elif pred == 7:
            self._imu_recalib()
            self._set_nn_cmd_status("NN CMD: RE-CALIBRATE", ACCENT2)
        elif pred == 1:
            self._cmd_land()
            self._set_nn_cmd_status("NN CMD: LAND", BTN_LAND)
        else:
            return
        self._log_event(f"NN action: class {pred}")

    def _update_nn_logic(self):
        g=self.glove
        pred=int(getattr(g,"nn_pred_class",-1))
        margin=float(getattr(g,"nn_margin",0.0))

        if pred != self._nn_pred_last:
            self._nn_pred_last = pred
            self._log_event(f"NN pred: {pred} (margin={margin:.3f})")

        # Position display follows live_glove_position_viewer.py behavior.
        self._nn_stable_pos = pred
        self._nn_pos_var.set(f"POS: {self._nn_stable_pos}")

        if pred != self._nn_last_class:
            self._nn_last_class = pred
            self._nn_class_start_ts = time.time()

        if pred < 0:
            return

        now=time.time()
        held=(now-self._nn_class_start_ts)>=self._nn_hold_s

        if pred == 0:
            if held:
                self._nn_last_action_class = -1
            return

        req_margin = self._nn_min_margin_land if pred == 1 else self._nn_min_margin
        if not held or margin < req_margin:
            return

        if pred == self._nn_last_action_class:
            return
        if (now - self._nn_last_action_ts) < self._nn_cooldown_s:
            return

        self._trigger_nn_action(pred)
        self._nn_last_action_class = pred
        self._nn_last_action_ts = now

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
    def _bind_keys(self):
        self.root.bind("<KeyPress>",  self._on_key)
        self.root.bind("<KeyRelease>",self._on_key_release)

    def _on_key_release(self,event):
        if event.keysym.lower()=="f":
            self._flip_key_held=False

    def _on_key(self,event):
        k=event.keysym
        if   k=="t":        self._cmd_takeoff()
        elif k=="l":        self._cmd_land()
        elif k=="space":    self._cmd_stop()
        elif k=="h":        self._toggle_headless()
        elif k=="c":        self._cmd_calibrate()
        elif k.lower()=="o":self._imu_zero()
        elif k=="F5":       self._imu_recalib()
        elif k.lower()=="s": pass  # snapshot handled inside OpenCV window
        # ── camera tilt ───────────────────────────────────────────────
        elif k=="Prior":    self._cmd_cam_up()    # PageUp
        elif k=="Next":     self._cmd_cam_down()  # PageDown
        # ── flip mode: hold F then press an arrow key ──────────────────
        elif k.lower()=="f":
            self._flip_key_held=True
        elif k=="Up"    and self._flip_key_held: self._cmd_flip(FLIP_FORWARD)
        elif k=="Down"  and self._flip_key_held: self._cmd_flip(FLIP_BACKWARD)
        elif k=="Left"  and self._flip_key_held: self._cmd_flip(FLIP_LEFT)
        elif k=="Right" and self._flip_key_held: self._cmd_flip(FLIP_RIGHT)

    # ── glove ─────────────────────────────────────────────────────────────
    def _connect_glove(self):
        if self.serial: self.serial.stop()
        port=self._serial_port_var.get().strip()
        try:    baud=int(self._baud_var.get())
        except: baud=115200
        self.serial=SerialReader(port,baud,self.glove.on_sensor_data,
                                 self._on_serial_status,self.log_q,
                                 on_nn=self.glove.on_nn_sample)
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
    def _cmd_stop(self):      self.state.stop_flag      =True;self._log_event("CMD: EMERGENCY STOP")
    def _cmd_calibrate(self): self.state.calibrate_flag =True;self._log_event("CMD: CALIBRATE DRONE IMU")

    def _cmd_land(self):
        self.state.land_flag = True
        self._log_event("CMD: LAND")

    # ── camera tilt ───────────────────────────────────────────────────────
    def _cmd_cam_up(self):
        self.state.cam_up_flag=True
        self._log_event("CMD: CAM TILT UP")

    def _cmd_cam_down(self):
        self.state.cam_down_flag=True
        self._log_event("CMD: CAM TILT DOWN")

    # ── flips ─────────────────────────────────────────────────────────────
    def _cmd_flip(self, direction: str):
        if self.state.flip_active:
            self._log_event("FLIP: already in progress — ignored")
            return
        self.state.flip_dir   =direction
        self.state.flip_active=True
        self._log_event(f"CMD: FLIP {direction.upper()}")

    def _toggle_headless(self):
        self.state.headless = not self.state.headless
        # Send one explicit protocol event (0x10) like the mobile app.
        self.state.headless_flag = True
        s = "ON" if self.state.headless else "OFF"
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
            self.video_adapter.send_disconnect()
            self.video_adapter.stop()
            self.video_adapter = None
            self._video_thread = None
            self.ctrl.release_socket()
            if not self.ctrl._running:
                self.ctrl.start()
            self._video_btn.config(text="▶  START VIDEO", bg="#005f73")
            self._status_label.config(text="● WIFI CAM CONNECTED", fg=BTN_TAKE)
            self._log_event("VIDEO: stopped")
        else:
            if not CV2_AVAILABLE:
                self._log_event("VIDEO: ERROR — pip install opencv-python numpy")
                return

            ip = self._ip_var.get().strip()
            try:    port = int(self._port_var.get())
            except: port = DEFAULT_PORT

            # ── START ─────────────────────────────────────────────────────
            # Use shared control socket so RC packets and video share endpoint.
            self.ctrl.drone_ip   = ip
            self.ctrl.drone_control_port = port
            self.ctrl.drone_port = port

            was_ctrl = self.ctrl._running
            if was_ctrl:
                self.ctrl.stop()
                self.ctrl.release_socket()
                self._log_event("VIDEO: cycling ctrl socket for clean handoff")

            self.video_adapter = WifiCamVideoAdapter(
                drone_ip=ip,
                port=port,
                session_port=self.ctrl.drone_session_port,
                log_cb=self._log_event,
            )
            self.ctrl.inject_socket(self.video_adapter._sock)

            # Match known-good app sequence using adapter-owned sockets.
            self.video_adapter.send_connect()
            self.video_adapter.send_start_burst(burst=6)
            self.ctrl.start()

            gui_ref = self
            def _display_thread(gui=gui_ref):
                _run_video_display(self.video_adapter, gui)
                self.root.after(0, self._on_video_closed)

            self._video_thread = threading.Thread(
                target=_display_thread, daemon=True, name="VideoDisplay")
            self._video_thread.start()

            self._video_btn.config(text="■  STOP VIDEO", bg=BTN_STOP)
            self._status_label.config(text="● WIFI CAM + VIDEO", fg=BTN_TAKE)
            self._log_event(f"VIDEO: started → {ip}:{port}")

    def _on_video_closed(self):
        """Called from the main thread when the OpenCV window is closed."""
        self.video_adapter = None
        self._video_thread = None
        self.ctrl.release_socket()
        if not self.ctrl._running:
            self.ctrl.start()
        self._video_btn.config(text="▶  START VIDEO", bg="#005f73")
        self._status_label.config(text="● WIFI CAM CONNECTED", fg=BTN_TAKE)
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
        self.ctrl.drone_ip = ip
        self.ctrl.drone_control_port = port
        self.ctrl.drone_port = port
        self.ctrl.rate = rate
        self.ctrl.send_connect()
        self._status_label.config(text="● WIFI CAM CONNECTED",fg=BTN_TAKE)
        self._log_event(f"CONNECTED  {ip}:8080 / {port}  @ {rate} Hz")

    def _start_control(self):
        try:
            ip=self._ip_var.get().strip();port=int(self._port_var.get());rate=float(self._rate_var.get())
        except ValueError as e:
            self._log_event(f"ERROR: {e}")
            return
        self.ctrl.drone_ip = ip
        self.ctrl.drone_control_port = port
        self.ctrl.drone_port = port
        self.ctrl.rate = rate
        self.ctrl.send_connect()
        self.ctrl.send_start_control(burst=6)
        if not self.ctrl._running:
            self.ctrl.start()
        self._status_label.config(text="● WIFI CAM STARTED", fg=BTN_TAKE)
        self._log_event("CONNECT + START sent, control loop running")

    def _disconnect(self):
        if self.video_adapter:
            self.video_adapter.send_disconnect()
            self.video_adapter.stop(); self.video_adapter=None
        else:
            self.ctrl.send_disconnect()
        self.ctrl.release_socket()
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

        g=self.glove
        self._imu_vars["yaw"].set(f"{g.yaw_deg:+7.1f}°")
        self._imu_vars["pitch"].set(f"{g.pitch_deg:+7.1f}°")
        self._imu_vars["roll"].set(f"{g.roll_deg:+7.1f}°")
        self._imu_vars["a0"].set(f"{g.a0_raw:7.0f}")
        self._imu_vars["a1"].set(f"{g.a1_raw:7.0f}")
        self._imu_vars["a2"].set(f"{g.a2_raw:7.0f}")
        self._imu_vars["a3"].set(f"{g.a3_raw:7.0f}")
        self._ahi.update_attitude(g.pitch_deg,g.roll_deg,g.yaw_deg)
        self._thr_bar.set_value(g.throttle_pct)
        self._update_nn_logic()

        if self._nn_cmd_until and time.time() > self._nn_cmd_until:
            self._nn_cmd_var.set("NN CMD: none")
            self._nn_cmd_label.config(fg=TEXT_DIM)
            self._nn_cmd_card.config(highlightbackground=TEXT_DIM)
            self._nn_cmd_until = 0.0

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
                self._calib_label.config(text=f"DONE ✓  A0={m[0]:.0f}  A1={m[1]:.0f}  A2={m[2]:.0f}  A3={m[3]:.0f}")
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
    root=tk.Tk()
    app=K417GUI(root)
    root.protocol("WM_DELETE_WINDOW",app.on_close)
    root.mainloop()

if __name__=="__main__":
    main()