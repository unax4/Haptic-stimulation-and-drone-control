#!/usr/bin/env python3
"""
Standalone K417/WiFi-UAV video receiver with optional distance estimation.

Usage examples:
  python receive_video_distance.py
  python receive_video_distance.py --drone-ip 192.168.169.1 --port 8800
  python receive_video_distance.py --distance-estimator
  python receive_video_distance.py --distance-estimator --use-yolo
    python receive_video_distance.py --protocol rtsp --rtsp-url rtsp://192.168.1.1:7070/webcam
"""

from __future__ import annotations

import argparse
import ipaddress
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

try:
    from distance_estimator_v2 import AsyncDistanceEstimator
    DIST_EST_AVAILABLE = True
except Exception:
    AsyncDistanceEstimator = None
    DIST_EST_AVAILABLE = False

DEFAULT_IP = "192.168.169.1"
DEFAULT_PORT = 8800

S2X_DEFAULT_IP = "172.16.10.1"
S2X_DEFAULT_CONTROL_PORT = 8080
S2X_DEFAULT_VIDEO_PORT = 8888

RTSP_DEFAULT_URLS = [
    "rtsp://192.168.1.1:7070/webcam",
    "rtsp://172.16.10.1:7070/webcam",
]

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

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"

LUM_QT = [
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
]
CHR_QT = [
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
]


def _make_dqt(tid: int, table: list[int]) -> bytes:
    payload = bytearray([(0 << 4) | tid]) + bytearray(table)
    segment = bytearray(b"\xff\xdb")
    segment += (len(payload) + 2).to_bytes(2, "big")
    segment += payload
    return bytes(segment)


def _make_sof0(width: int, height: int) -> bytes:
    comps = bytes([1, 0x11, 0, 2, 0x11, 1, 3, 0x11, 1])
    length = (8 + 9).to_bytes(2, "big")
    return (
        b"\xff\xc0"
        + length
        + b"\x08"
        + height.to_bytes(2, "big")
        + width.to_bytes(2, "big")
        + b"\x03"
        + comps
    )


def _make_sos() -> bytes:
    payload = bytearray([3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0])
    length = (len(payload) + 2).to_bytes(2, "big")
    return b"\xff\xda" + length + bytes(payload)


def build_jpeg_header(width: int = 640, height: int = 360) -> bytes:
    return SOI + _make_dqt(0, LUM_QT) + _make_dqt(1, CHR_QT) + _make_sof0(width, height) + _make_sos()


@dataclass
class ReceiverStats:
    frames_ok: int = 0
    frames_dropped: int = 0


class K417VideoReceiver:
    HEADER_LEN = 56
    FRAME_TIMEOUT = 0.08
    MAX_RETRIES = 3
    WATCHDOG_SLEEP = 0.05

    def __init__(self, drone_ip: str, port: int, jpeg_width: int = 640, jpeg_height: int = 360):
        self.drone_ip = drone_ip
        self.port = port
        self.jpeg_header = build_jpeg_header(jpeg_width, jpeg_height)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288)
        self.sock.bind(("", 0))
        self.sock.setblocking(False)

        self.running = True
        self.first_frame = True
        self.current_fid = 1
        self.fragments: dict[int, bytes] = {}
        self.last_req_ts = 0.0
        self.retry_cnt = 0

        self.stats = ReceiverStats()
        self.frame_q: queue.Queue[bytes] = queue.Queue(maxsize=1)

    def start(self) -> None:
        self._send_start()
        self._send_frame_request(0)
        threading.Thread(target=self._warmup_loop, daemon=True, name="K417-Warmup").start()
        threading.Thread(target=self._watchdog_loop, daemon=True, name="K417-Watchdog").start()
        threading.Thread(target=self._rx_loop, daemon=True, name="K417-RX").start()

    def stop(self) -> None:
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass

    def get_frame(self, timeout: float = 0.0) -> Optional[bytes]:
        try:
            if timeout > 0:
                return self.frame_q.get(timeout=timeout)
            return self.frame_q.get_nowait()
        except queue.Empty:
            return None

    def _send_start(self) -> None:
        try:
            self.sock.sendto(START_STREAM, (self.drone_ip, self.port))
        except Exception:
            pass

    def _send_frame_request(self, frame_id: int) -> None:
        lo = frame_id & 0xFF
        hi = (frame_id >> 8) & 0xFF

        req_a = bytearray(REQUEST_A)
        req_a[12] = lo
        req_a[13] = hi

        req_b = bytearray(REQUEST_B)
        for base in (12, 88, 107):
            req_b[base] = lo
            req_b[base + 1] = hi

        try:
            self.sock.sendto(bytes(req_a), (self.drone_ip, self.port))
            self.sock.sendto(bytes(req_b), (self.drone_ip, self.port))
        except Exception:
            pass

        self.last_req_ts = time.time()

    def _handle_payload(self, payload: bytes) -> None:
        if len(payload) < self.HEADER_LEN or payload[1] != 0x01:
            return

        frame_id = int.from_bytes(payload[16:18], "little")
        frag_id = int.from_bytes(payload[32:34], "little")
        last_frag = payload[2] != 0x38

        if frame_id != self.current_fid:
            self.stats.frames_dropped += 1
            self.fragments.clear()
            self.current_fid = frame_id

        self.fragments.setdefault(frag_id, payload[self.HEADER_LEN:])
        self.retry_cnt = 0

        if not last_frag:
            return

        ordered = [self.fragments[idx] for idx in sorted(self.fragments)]
        jpeg = self.jpeg_header + b"".join(ordered) + EOI
        self.stats.frames_ok += 1

        if self.first_frame:
            self.first_frame = False

        try:
            self.frame_q.get_nowait()
        except queue.Empty:
            pass

        try:
            self.frame_q.put_nowait(jpeg)
        except queue.Full:
            pass

        self.fragments.clear()
        self._send_frame_request(frame_id)
        self.current_fid = (frame_id + 1) & 0xFFFF
        self.last_req_ts = time.time()

    def _rx_loop(self) -> None:
        import select

        while self.running:
            try:
                readable, _, _ = select.select([self.sock], [], [], 0.01)
                if not readable:
                    continue
                payload, _ = self.sock.recvfrom(65535)
                self._handle_payload(payload)
            except (OSError, ValueError):
                if self.running:
                    time.sleep(0.01)
                break

    def _warmup_loop(self) -> None:
        while self.running and self.first_frame:
            self._send_start()
            self._send_frame_request((self.current_fid - 1) & 0xFFFF)
            time.sleep(0.2)

    def _watchdog_loop(self) -> None:
        while self.running:
            time.sleep(self.WATCHDOG_SLEEP)
            if time.time() - self.last_req_ts < self.FRAME_TIMEOUT:
                continue

            if self.retry_cnt < self.MAX_RETRIES:
                self._send_frame_request((self.current_fid - 1) & 0xFFFF)
                self.retry_cnt += 1
                continue

            self.stats.frames_dropped += 1
            self.fragments.clear()
            self.retry_cnt = 0
            self.current_fid = (self.current_fid + 1) & 0xFFFF
            self._send_frame_request((self.current_fid - 1) & 0xFFFF)


class S2xVideoReceiver:
    SYNC_BYTES = b"\x40\x40"
    EOS_MARKER = b"\x23\x23"
    HEADER_LEN = 8

    def __init__(self, drone_ip: str, control_port: int, video_port: int):
        self.drone_ip = drone_ip
        self.control_port = control_port
        self.video_port = video_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.video_port))
        self.sock.settimeout(1.0)

        self.running = True
        self.frame_q: queue.Queue[bytes] = queue.Queue(maxsize=1)
        self.stats = ReceiverStats()

        self._cur_fid: Optional[int] = None
        self._frags: dict[int, bytes] = {}

    def start(self) -> None:
        self._send_start_command()
        threading.Thread(target=self._keepalive_loop, daemon=True, name="S2x-Keepalive").start()
        threading.Thread(target=self._rx_loop, daemon=True, name="S2x-RX").start()

    def stop(self) -> None:
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass

    def get_frame(self, timeout: float = 0.0) -> Optional[bytes]:
        try:
            if timeout > 0:
                return self.frame_q.get(timeout=timeout)
            return self.frame_q.get_nowait()
        except queue.Empty:
            return None

    def _discover_local_ip(self) -> str:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((self.drone_ip, 1))
            return probe.getsockname()[0]
        finally:
            probe.close()

    def _send_start_command(self) -> None:
        try:
            local_ip = self._discover_local_ip()
            payload = b"\x08" + ipaddress.IPv4Address(local_ip).packed
            ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                ctrl_sock.sendto(payload, (self.drone_ip, self.control_port))
            finally:
                ctrl_sock.close()
        except Exception:
            pass

    def _keepalive_loop(self) -> None:
        while self.running:
            self._send_start_command()
            time.sleep(2.0)

    def _handle_payload(self, payload: bytes) -> None:
        if len(payload) <= self.HEADER_LEN or payload[:2] != self.SYNC_BYTES:
            return

        fid = payload[2]
        sid_raw = payload[5]
        body = payload[self.HEADER_LEN :]

        if body.endswith(self.EOS_MARKER):
            body = body[: -len(self.EOS_MARKER)]

        if self._cur_fid is None:
            self._cur_fid = fid
        elif fid != self._cur_fid:
            self._finalize_current_frame()
            self._cur_fid = fid
            self._frags.clear()

        self._frags.setdefault(sid_raw, body)

    def _finalize_current_frame(self) -> None:
        if not self._frags:
            return

        keys = sorted(self._frags)
        complete = len(keys) == (keys[-1] - keys[0] + 1)
        if not complete:
            self.stats.frames_dropped += 1
            return

        data = b"".join(self._frags[k] for k in keys)
        start = data.find(SOI)
        end = data.rfind(EOI)
        if start < 0 or end < 0 or end <= start:
            self.stats.frames_dropped += 1
            return

        jpeg = data[start : end + len(EOI)]
        self.stats.frames_ok += 1

        try:
            self.frame_q.get_nowait()
        except queue.Empty:
            pass

        try:
            self.frame_q.put_nowait(jpeg)
        except queue.Full:
            pass

    def _rx_loop(self) -> None:
        while self.running:
            try:
                payload, _ = self.sock.recvfrom(4096)
                self._handle_payload(payload)
            except socket.timeout:
                continue
            except OSError:
                if self.running:
                    time.sleep(0.01)
                break


class RtspVideoReceiver:
    def __init__(self, rtsp_url: str):
        self.rtsp_url = rtsp_url
        self.cap: Optional[cv2.VideoCapture] = None
        self.running = True
        self.frame_q: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self.stats = ReceiverStats()

    def start(self) -> None:
        self.cap = cv2.VideoCapture(self.rtsp_url)
        threading.Thread(target=self._rx_loop, daemon=True, name="RTSP-RX").start()

    def stop(self) -> None:
        self.running = False
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

    def get_frame(self, timeout: float = 0.0) -> Optional[np.ndarray]:
        try:
            if timeout > 0:
                return self.frame_q.get(timeout=timeout)
            return self.frame_q.get_nowait()
        except queue.Empty:
            return None

    def _reopen(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = cv2.VideoCapture(self.rtsp_url)

    def _rx_loop(self) -> None:
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self._reopen()
                time.sleep(0.5)
                continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.stats.frames_dropped += 1
                self._reopen()
                time.sleep(0.2)
                continue

            self.stats.frames_ok += 1
            try:
                self.frame_q.get_nowait()
            except queue.Empty:
                pass

            try:
                self.frame_q.put_nowait(frame)
            except queue.Full:
                pass


def probe_rtsp_url(candidates: list[str], timeout_sec: float = 2.0) -> Optional[str]:
    for url in candidates:
        cap = cv2.VideoCapture(url)
        t0 = time.time()
        ok = False
        frame = None
        while time.time() - t0 < timeout_sec:
            ok, frame = cap.read()
            if ok and frame is not None:
                break
            time.sleep(0.05)
        cap.release()
        if ok and frame is not None:
            return url
    return None


def choose_protocol(args: argparse.Namespace) -> str:
    if args.protocol != "auto":
        return args.protocol

    if args.rtsp_url:
        return "rtsp"
    if args.drone_ip.startswith("192.168.169."):
        return "k417"
    if args.drone_ip.startswith("172.16.10."):
        return "s2x"
    if args.drone_ip.startswith("192.168.1."):
        return "rtsp"
    return "rtsp"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Standalone drone video receiver (K417 + E58/S2x + RTSP) with optional distance estimator")
    p.add_argument("--protocol", choices=["auto", "k417", "s2x", "rtsp"], default="auto", help="Video protocol mode")
    p.add_argument("--drone-ip", default=DEFAULT_IP, help="Drone IP on WiFi network")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help="K417 UDP control/video port (default 8800)")
    p.add_argument("--control-port", type=int, default=S2X_DEFAULT_CONTROL_PORT, help="S2x/E58 control port (default 8080)")
    p.add_argument("--video-port", type=int, default=S2X_DEFAULT_VIDEO_PORT, help="S2x/E58 video listen port (default 8888)")
    p.add_argument("--rtsp-url", default="", help="RTSP URL for WiFi Cam mode")
    p.add_argument("--rtsp-probe", action="store_true", help="Probe known RTSP URLs if explicit URL fails or is not provided")
    p.add_argument("--width", type=int, default=640, help="Expected video width")
    p.add_argument("--height", type=int, default=360, help="Expected video height")
    p.add_argument("--distance-estimator", action="store_true", help="Enable distance_estimator_v2 overlay")
    p.add_argument("--use-yolo", action="store_true", help="Distance estimator: enable YOLO person anchor")
    p.add_argument("--yolo-every-n", type=int, default=2, help="Distance estimator: run YOLO every N frames")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    selected_protocol = choose_protocol(args)

    if selected_protocol == "s2x" and args.drone_ip == DEFAULT_IP:
        # Make auto/user-default behavior E58-friendly if protocol switched to s2x.
        args.drone_ip = S2X_DEFAULT_IP

    if args.distance_estimator and not DIST_EST_AVAILABLE:
        print("[video] distance estimator requested but distance_estimator_v2.py is not importable")
        print("[video] continuing without estimator")
        args.distance_estimator = False

    rtsp_url = args.rtsp_url.strip()
    if selected_protocol == "rtsp":
        if not rtsp_url:
            probe_list = RTSP_DEFAULT_URLS.copy()
            if args.drone_ip:
                probe_list.insert(0, f"rtsp://{args.drone_ip}:7070/webcam")
            rtsp_url = probe_rtsp_url(probe_list, timeout_sec=2.0)
            if rtsp_url is None:
                print("[video] no RTSP endpoint responded")
                print("[video] tried: " + ", ".join(probe_list))
                print("[video] try --rtsp-url explicitly if your app uses a different endpoint")
                return 2
        elif args.rtsp_probe:
            chosen = probe_rtsp_url([rtsp_url] + RTSP_DEFAULT_URLS, timeout_sec=2.0)
            if chosen is not None:
                rtsp_url = chosen

    if selected_protocol == "k417":
        receiver = K417VideoReceiver(args.drone_ip, args.port, args.width, args.height)
        endpoint = f"{args.drone_ip}:{args.port}"
    elif selected_protocol == "s2x":
        receiver = S2xVideoReceiver(args.drone_ip, args.control_port, args.video_port)
        endpoint = f"ctrl {args.drone_ip}:{args.control_port} | video *:{args.video_port}"
    else:
        receiver = RtspVideoReceiver(rtsp_url)
        endpoint = rtsp_url

    receiver.start()

    dist_est = None
    dist_enabled = args.distance_estimator
    if dist_enabled:
        dist_est = AsyncDistanceEstimator(
            use_yolo=args.use_yolo,
            yolo_every_n=max(1, args.yolo_every_n),
            draw_overlay=True,
            depth_map_out=False,
        )
        dist_est.start()
        print("[video] distance estimator enabled (press D to toggle)")

    print(f"[video] protocol: {selected_protocol}")
    print(f"[video] endpoint: {endpoint}")
    print("[video] keys: Q quit, D toggle distance overlay")
    if selected_protocol == "s2x":
        print("[video] E58 tip: for SSID like wifi_8k_..., use --protocol s2x --drone-ip 172.16.10.1")
    if selected_protocol == "rtsp":
        print("[video] WiFi Cam tip: common endpoint is rtsp://192.168.1.1:7070/webcam")

    window = f"Drone Video Receiver ({selected_protocol})"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    fps_t0 = time.time()
    fps_frames = 0
    fps_value = 0.0

    try:
        while True:
            if selected_protocol == "rtsp":
                frame = receiver.get_frame(timeout=0.5)
                if frame is None:
                    frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                    cv2.putText(frame, "Waiting for RTSP video...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 140, 255), 2)
            else:
                jpeg = receiver.get_frame(timeout=0.5)
                if jpeg is None:
                    frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                    cv2.putText(frame, "Waiting for video...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 140, 255), 2)
                else:
                    arr = np.frombuffer(jpeg, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is None:
                        frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                        cv2.putText(frame, "Decode failed", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            if dist_est is not None and dist_enabled and frame is not None:
                dist_est.submit(frame)
                latest = dist_est.result
                if latest.overlay is not None:
                    frame = latest.overlay
                if latest.distance_m > 0:
                    label = f"Dist: {latest.distance_m:.2f} m  Conf: {latest.confidence:.0%}  {latest.method}"
                    cv2.putText(frame, label, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            fps_frames += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                fps_value = fps_frames / max(now - fps_t0, 1e-6)
                fps_frames = 0
                fps_t0 = now

            cv2.putText(
                frame,
                f"FPS:{fps_value:4.1f} OK:{receiver.stats.frames_ok} DROP:{receiver.stats.frames_dropped}",
                (10, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("d") and dist_est is not None:
                dist_enabled = not dist_enabled
                print(f"[video] distance overlay {'ON' if dist_enabled else 'OFF'}")

    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        if dist_est is not None:
            dist_est.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
