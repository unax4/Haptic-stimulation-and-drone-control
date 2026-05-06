#!/usr/bin/env python3
"""
WIFI CAM drone controller (observed protocol family: UDP 8080/8090).

This script emulates the app flow captured from Wireshark:
1) Session connect on 8080 with payload 42 76
2) Optional pre-control stream poke on 8090 with payload AA 80 80 00 80 00 80 55
3) Continuous RC control loop on 8090 with 8-byte CAM frame:
   66 b1 b2 b3 b4 cmd chk 99
   chk = b1 XOR b2 XOR b3 XOR b4 XOR cmd
4) Session disconnect on 8080 with payload 42 77

Notes:
- Uses separate UDP sockets for session and control, like the app behavior.
- Axis mapping is based on your captures and typical toy-drone layout:
  b1=roll, b2=pitch, b3=throttle, b4=yaw
- Command byte (cmd) currently supports observed one-shot values:
  0x01 takeoff, 0x02 land, 0x04 stop, 0x10 headless toggle
"""

from __future__ import annotations

import argparse
import select
import socket
import threading
import time

import cv2
import numpy as np

CONNECT = bytes.fromhex("42 76")
DISCONNECT = bytes.fromhex("42 77")
PRESTREAM = bytes.fromhex("AA 80 80 00 80 00 80 55")


class WifiCamController:
    def __init__(
        self,
        drone_ip: str,
        session_port: int = 8080,
        control_port: int = 8090,
        rate_hz: float = 30.0,
        local_session_port: int = 0,
        local_control_port: int = 0,
        enable_video: bool = True,
        video_window: str = "WIFI CAM Video",
    ) -> None:
        self.drone_ip = drone_ip
        self.session_port = session_port
        self.control_port = control_port
        self.interval = 1.0 / max(1e-6, rate_hz)
        self.enable_video = enable_video
        self.video_window = video_window

        # Separate sockets to mimic app behavior with distinct source ports.
        self.session_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.session_sock.bind(("", max(0, local_session_port)))
        self.control_sock.bind(("", max(0, local_control_port)))
        self.session_sock.setblocking(False)
        self.control_sock.setblocking(False)

        self.roll = 0x80
        self.pitch = 0x80
        self.throttle = 0x80
        self.yaw = 0x80
        self.cmd = 0x00

        self._running = False
        self._thread: threading.Thread | None = None
        self._video_running = False
        self._video_thread: threading.Thread | None = None
        self._video_frag = bytearray()
        self._video_frames = 0
        self._video_last_log = 0.0

    @staticmethod
    def _clamp(v: int) -> int:
        return max(0, min(255, int(v)))

    def set_axis(self, axis: str, value: int) -> None:
        value = self._clamp(value)
        if axis == "roll":
            self.roll = value
        elif axis == "pitch":
            self.pitch = value
        elif axis == "throttle":
            self.throttle = value
        elif axis == "yaw":
            self.yaw = value
        else:
            raise ValueError(f"Unknown axis: {axis}")

    def set_neutral(self) -> None:
        self.roll = 0x80
        self.pitch = 0x80
        self.throttle = 0x80
        self.yaw = 0x80

    def oneshot_cmd(self, cmd: int) -> None:
        self.cmd = cmd & 0xFF

    def connect(self) -> None:
        self.session_sock.sendto(CONNECT, (self.drone_ip, self.session_port))
        print(
            f"[wifi-cam] CONNECT sent to {self.drone_ip}:{self.session_port} "
            f"(local session port {self.session_sock.getsockname()[1]})"
        )
        if self.enable_video:
            self.start_video_receiver()

    def disconnect(self) -> None:
        self.session_sock.sendto(DISCONNECT, (self.drone_ip, self.session_port))
        print(f"[wifi-cam] DISCONNECT sent to {self.drone_ip}:{self.session_port}")

    def send_prestream(self, count: int = 6, gap_s: float = 0.03) -> None:
        for _ in range(max(1, count)):
            self.control_sock.sendto(PRESTREAM, (self.drone_ip, self.control_port))
            time.sleep(max(0.0, gap_s))
        print(
            f"[wifi-cam] PRESTREAM x{max(1, count)} sent to {self.drone_ip}:{self.control_port}"
        )

    def _build_cam8(self) -> bytes:
        b1 = self.roll & 0xFF
        b2 = self.pitch & 0xFF
        b3 = self.throttle & 0xFF
        b4 = self.yaw & 0xFF
        cmd = self.cmd & 0xFF
        chk = b1 ^ b2 ^ b3 ^ b4 ^ cmd
        return bytes((0x66, b1, b2, b3, b4, cmd, chk & 0xFF, 0x99))

    def _extract_jpegs_from_payload(self, payload: bytes) -> list[bytes]:
        out: list[bytes] = []
        # Fast path: one or more complete JPEGs inside this single datagram.
        pos = 0
        while True:
            soi = payload.find(b"\xFF\xD8", pos)
            if soi < 0:
                break
            eoi = payload.find(b"\xFF\xD9", soi + 2)
            if eoi < 0:
                break
            jpg = payload[soi : eoi + 2]
            if len(jpg) >= 300:
                out.append(jpg)
            pos = eoi + 2

        # If complete JPEGs were found in this packet, prefer them and reset fragment state.
        if out:
            self._video_frag.clear()
            return out

        # Fragment path: accumulate from SOI to EOI across packets.
        soi = payload.find(b"\xFF\xD8")
        if soi >= 0:
            self._video_frag = bytearray(payload[soi:])
        elif self._video_frag:
            self._video_frag.extend(payload)

        # Keep fragment buffer bounded.
        if len(self._video_frag) > 2 * 1024 * 1024:
            self._video_frag.clear()
            return out

        if self._video_frag:
            eoi = self._video_frag.find(b"\xFF\xD9")
            if eoi >= 0:
                jpg = bytes(self._video_frag[: eoi + 2])
                self._video_frag = bytearray(self._video_frag[eoi + 2 :])
                if len(jpg) >= 300:
                    out.append(jpg)

        return out

    def _video_loop(self) -> None:
        print(
            f"[wifi-cam] video receiver started "
            f"(local control port {self.control_sock.getsockname()[1]})"
        )

        sockets = [self.control_sock, self.session_sock]

        while self._video_running:
            try:
                readable, _, _ = select.select(sockets, [], [], 0.02)
            except OSError:
                continue

            if not readable:
                cv2.waitKey(1)
                continue

            for sock in readable:
                # Drain all pending datagrams from each readable socket.
                while True:
                    try:
                        payload, addr = sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError:
                        break

                    if addr[0] != self.drone_ip:
                        continue

                    for jpg in self._extract_jpegs_from_payload(payload):
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if frame is None:
                            continue

                        self._video_frames += 1
                        cv2.imshow(self.video_window, frame)

            cv2.waitKey(1)
            now = time.time()
            if now - self._video_last_log > 1.0:
                print(f"[wifi-cam] video frames decoded: {self._video_frames}")
                self._video_last_log = now

        try:
            cv2.destroyWindow(self.video_window)
        except cv2.error:
            pass
        print("[wifi-cam] video receiver stopped")

    def start_video_receiver(self) -> None:
        if not self.enable_video or self._video_running:
            return
        self._video_running = True
        self._video_thread = threading.Thread(target=self._video_loop, name="WifiCamVideo", daemon=True)
        self._video_thread.start()

    def stop_video_receiver(self) -> None:
        self._video_running = False
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=1.0)

    def _loop(self) -> None:
        while self._running:
            pkt = self._build_cam8()
            self.control_sock.sendto(pkt, (self.drone_ip, self.control_port))

            # Observed commands behave as one-shot actions in app control flow.
            if self.cmd in (0x01, 0x02, 0x04, 0x10):
                self.cmd = 0x00

            time.sleep(self.interval)

    def start_control_stream(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="WifiCamControl", daemon=True)
        self._thread.start()
        print(f"[wifi-cam] control stream started @ {1.0/self.interval:.1f} Hz")
        if self.enable_video:
            self.start_video_receiver()

    def stop_control_stream(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        print("[wifi-cam] control stream stopped")

    def close(self) -> None:
        self.stop_control_stream()
        self.stop_video_receiver()
        try:
            self.session_sock.close()
        except OSError:
            pass
        try:
            self.control_sock.close()
        except OSError:
            pass

    def status(self) -> str:
        pkt = self._build_cam8()
        return (
            f"roll={self.roll} pitch={self.pitch} throttle={self.throttle} yaw={self.yaw} "
            f"cmd=0x{self.cmd:02X} packet={pkt.hex(' ')}"
        )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="WIFI CAM protocol controller")
    p.add_argument("--drone-ip", type=str, default="192.168.4.153", help="Drone IP")
    p.add_argument("--session-port", type=int, default=8080, help="Session UDP port")
    p.add_argument("--control-port", type=int, default=8090, help="Control UDP port")
    p.add_argument("--rate", type=float, default=30.0, help="Control stream rate (Hz)")
    p.add_argument(
        "--prestream-count",
        type=int,
        default=6,
        help="How many prestream packets to send before starting CAM8 stream",
    )
    p.add_argument(
        "--no-disconnect",
        action="store_true",
        help="Do not send disconnect packet when exiting",
    )
    p.add_argument(
        "--no-video",
        action="store_true",
        help="Disable live video receiver",
    )
    p.add_argument(
        "--video-window",
        type=str,
        default="WIFI CAM Video",
        help="OpenCV window title for video",
    )
    p.add_argument(
        "--local-session-port",
        type=int,
        default=0,
        help="Local UDP port to bind session socket (0 = auto)",
    )
    p.add_argument(
        "--local-control-port",
        type=int,
        default=0,
        help="Local UDP port to bind control/video socket (0 = auto)",
    )
    return p


def print_help() -> None:
    print("Commands:")
    print("  help")
    print("  connect")
    print("  start                 # send prestream burst + start CAM8 loop")
    print("  stopstream            # stop CAM8 loop")
    print("  startvideo | stopvideo")
    print("  disconnect")
    print("  neutral")
    print("  set <roll|pitch|throttle|yaw> <0..255>")
    print("  takeoff | land | stop | headless")
    print("  status")
    print("  quit")


def main() -> None:
    args = build_parser().parse_args()

    ctl = WifiCamController(
        drone_ip=args.drone_ip,
        session_port=args.session_port,
        control_port=args.control_port,
        rate_hz=args.rate,
        local_session_port=args.local_session_port,
        local_control_port=args.local_control_port,
        enable_video=not args.no_video,
        video_window=args.video_window,
    )

    print("[wifi-cam] ready")
    print(f"[wifi-cam] target {args.drone_ip} session:{args.session_port} control:{args.control_port}")
    print_help()

    try:
        while True:
            raw = input("wifi-cam> ").strip()
            if not raw:
                continue
            parts = raw.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit", "q"):
                break
            if cmd == "help":
                print_help()
            elif cmd == "connect":
                ctl.connect()
            elif cmd == "disconnect":
                ctl.disconnect()
            elif cmd == "start":
                ctl.connect()
                ctl.send_prestream(count=args.prestream_count)
                ctl.start_control_stream()
            elif cmd == "stopstream":
                ctl.stop_control_stream()
            elif cmd == "startvideo":
                ctl.start_video_receiver()
            elif cmd == "stopvideo":
                ctl.stop_video_receiver()
            elif cmd == "neutral":
                ctl.set_neutral()
                print(ctl.status())
            elif cmd == "set" and len(parts) == 3:
                axis = parts[1].lower()
                value = int(parts[2])
                ctl.set_axis(axis, value)
                print(ctl.status())
            elif cmd == "takeoff":
                ctl.oneshot_cmd(0x01)
                print(ctl.status())
            elif cmd == "land":
                ctl.oneshot_cmd(0x02)
                print(ctl.status())
            elif cmd == "stop":
                ctl.oneshot_cmd(0x04)
                print(ctl.status())
            elif cmd == "headless":
                ctl.oneshot_cmd(0x10)
                print(ctl.status())
            elif cmd == "status":
                print(ctl.status())
            else:
                print("Unknown command. Type: help")
    except KeyboardInterrupt:
        pass
    finally:
        if not args.no_disconnect:
            try:
                ctl.disconnect()
            except OSError:
                pass
        ctl.close()


if __name__ == "__main__":
    main()
