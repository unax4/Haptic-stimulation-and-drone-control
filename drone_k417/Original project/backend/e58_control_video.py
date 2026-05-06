#!/usr/bin/env python3
import argparse
import ipaddress
import queue
import signal
import socket
import sys
import threading
import time

from models.wifi_uav_rc import WifiUavRcModel
from protocols.wifi_uav_rc_protocol_adapter import WifiUavRcProtocolAdapter
from protocols.wifi_uav_video_protocol import WifiUavVideoProtocolAdapter
from services.flight_controller import FlightController
from services.video_receiver import VideoReceiverService
from utils.wifi_uav_packets import REQUEST_A, REQUEST_B, START_STREAM
from views.cli_rc import CLIView
from views.opencv_video_view import OpenCVVideoView


DEFAULT_E58_IP_CANDIDATES = [
    "192.168.4.1",
    "192.168.169.1",
    "192.168.0.1",
    "192.168.10.1",
    "192.168.1.1",
    ]


def _build_probe_requests(frame_id: int = 0) -> tuple[bytes, bytes]:
    lo, hi = frame_id & 0xFF, (frame_id >> 8) & 0xFF

    req_a = bytearray(REQUEST_A)
    req_a[12], req_a[13] = lo, hi

    req_b = bytearray(REQUEST_B)
    for base in (12, 88, 107):
        req_b[base], req_b[base + 1] = lo, hi

    return bytes(req_a), bytes(req_b)


def auto_detect_drone_ip(
    candidates: list[str],
    control_port: int,
    timeout_per_ip: float,
) -> str | None:
    req_a, req_b = _build_probe_requests(0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", 0))
        sock.settimeout(0.05)

        for ip in candidates:
            print(f"[e58] probing {ip}:{control_port}...")
            deadline = time.time() + timeout_per_ip

            # Repeat a few times in case first packet is dropped on Wi-Fi.
            for _ in range(3):
                sock.sendto(START_STREAM, (ip, control_port))
                sock.sendto(req_a, (ip, control_port))
                sock.sendto(req_b, (ip, control_port))

            while time.time() < deadline:
                try:
                    payload, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    # Windows can raise WinError 10054 on UDP when a probed host
                    # replies with ICMP port-unreachable. This is expected while
                    # scanning and should not abort detection.
                    continue

                # Wifi-UAV video packets are typically from the drone IP and
                # contain at least the 56-byte transport header.
                if addr[0] == ip and len(payload) >= 56 and payload[1] == 0x01:
                    print(f"[e58] detected drone at {ip}")
                    return ip

        return None
    finally:
        sock.close()


def _is_video_like_packet(payload: bytes) -> bool:
    return len(payload) >= 56 and payload[1] == 0x01


def _local_ipv4_addresses() -> list[str]:
    addrs = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addrs.add(ip)
    except socket.gaierror:
        pass
    return sorted(addrs)


def _local_private_subnets_24() -> list[str]:
    subnets = set()
    for ip in _local_ipv4_addresses():
        try:
            ip_obj = ipaddress.IPv4Address(ip)
        except ipaddress.AddressValueError:
            continue

        if not ip_obj.is_private:
            continue

        subnet = ipaddress.IPv4Network(f"{ip}/24", strict=False)
        subnets.add(str(subnet))

    return sorted(subnets)


def scan_local_subnets_for_drone_ip(control_port: int, timeout: float = 1.5) -> str | None:
    """
    Fallback discovery: probe every host in local private /24 subnets and
    return the first source IP that emits WiFi-UAV style video packets.
    """
    req_a, req_b = _build_probe_requests(0)
    subnets = _local_private_subnets_24()

    if not subnets:
        return None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", 0))
        sock.settimeout(0.05)

        for subnet_text in subnets:
            subnet = ipaddress.IPv4Network(subnet_text)
            print(f"[e58] subnet scan on {subnet_text} (udp:{control_port})...")

            for host_ip in subnet.hosts():
                ip_text = str(host_ip)
                sock.sendto(START_STREAM, (ip_text, control_port))
                sock.sendto(req_a, (ip_text, control_port))
                sock.sendto(req_b, (ip_text, control_port))

            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    payload, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    # Ignore ICMP port-unreachable noise from non-drone hosts.
                    continue

                if ipaddress.IPv4Address(addr[0]) in subnet and _is_video_like_packet(payload):
                    print(f"[e58] subnet scan detected drone at {addr[0]}")
                    return addr[0]

        return None
    finally:
        sock.close()


def preflight_video_probe(drone_ip: str, control_port: int, timeout: float = 1.5) -> int:
    """
    Send a short WiFi-UAV handshake and count incoming video-like UDP packets.
    This helps confirm if packets are reaching the host before curses starts.
    """
    req_a, req_b = _build_probe_requests(0)
    pkt_count = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", 0))
        sock.settimeout(0.05)

        deadline = time.time() + timeout
        while time.time() < deadline:
            sock.sendto(START_STREAM, (drone_ip, control_port))
            sock.sendto(req_a, (drone_ip, control_port))
            sock.sendto(req_b, (drone_ip, control_port))

            poll_deadline = time.time() + 0.15
            while time.time() < poll_deadline:
                try:
                    payload, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    # Ignore ICMP port-unreachable noise from non-drone hosts.
                    continue

                if addr[0] == drone_ip and _is_video_like_packet(payload):
                    pkt_count += 1

        return pkt_count
    finally:
        sock.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control and receive video from E58 drone (WiFi UAV protocol)."
    )
    parser.add_argument(
        "--drone-ip",
        type=str,
        default=None,
        help="E58 drone IP address (manual override)",
    )
    parser.add_argument(
        "--ip-candidates",
        type=str,
        default=",".join(DEFAULT_E58_IP_CANDIDATES),
        help="Comma-separated candidate IPs for auto-detection",
    )
    parser.add_argument(
        "--probe-timeout",
        type=float,
        default=1.0,
        help="Seconds to probe each candidate IP during auto-detect",
    )
    parser.add_argument(
        "--no-auto-detect-ip",
        action="store_true",
        help="Disable IP auto-detection and use --drone-ip only",
    )
    parser.add_argument(
        "--no-subnet-scan",
        action="store_true",
        help="Disable fallback scan across local private /24 subnets",
    )
    parser.add_argument(
        "--subnet-scan-timeout",
        type=float,
        default=1.5,
        help="Seconds to listen for replies after each subnet probe burst",
    )
    parser.add_argument(
        "--preflight-timeout",
        type=float,
        default=1.5,
        help="Seconds to wait for inbound video packets before UI startup",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip inbound packet preflight check",
    )
    parser.add_argument(
        "--debug-video",
        action="store_true",
        help="Enable verbose WiFi-UAV video protocol logs",
    )
    parser.add_argument(
        "--control-port",
        type=int,
        default=8800,
        help="UDP control port (default: 8800)",
    )
    parser.add_argument(
        "--video-port",
        type=int,
        default=8800,
        help="UDP video port (default: 8800)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=80.0,
        help="Control packet rate in Hz (default: 80.0)",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="normal",
        choices=["normal", "precise", "aggressive"],
        help="Stick sensitivity preset",
    )
    parser.add_argument(
        "--dump-frames",
        action="store_true",
        help="Save decoded frames to disk",
    )
    parser.add_argument(
        "--dump-packets",
        action="store_true",
        help="Save raw video packets to disk",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    if args.drone_ip:
        drone_ip = args.drone_ip
        print(f"[e58] using manual drone IP: {drone_ip}")
    elif args.no_auto_detect_ip:
        drone_ip = DEFAULT_E58_IP_CANDIDATES[0]
        print(f"[e58] auto-detect disabled, using fallback IP: {drone_ip}")
    else:
        candidates = [c.strip() for c in args.ip_candidates.split(",") if c.strip()]
        detected = auto_detect_drone_ip(
            candidates=candidates,
            control_port=args.control_port,
            timeout_per_ip=args.probe_timeout,
        )
        if detected is None:
            if not args.no_subnet_scan:
                detected = scan_local_subnets_for_drone_ip(
                    control_port=args.control_port,
                    timeout=args.subnet_scan_timeout,
                )

            if detected is None:
                drone_ip = DEFAULT_E58_IP_CANDIDATES[0]
                print(
                    f"[e58] auto-detect failed, falling back to {drone_ip}. "
                    "Use --drone-ip to force a specific address."
                )
            else:
                drone_ip = detected
        else:
            drone_ip = detected

    if not args.skip_preflight:
        print(f"[e58] preflight: checking inbound video packets from {drone_ip}:{args.control_port}...")
        pkt_count = preflight_video_probe(
            drone_ip=drone_ip,
            control_port=args.control_port,
            timeout=args.preflight_timeout,
        )
        if pkt_count > 0:
            print(f"[e58] preflight OK: received {pkt_count} video-like packets")
        else:
            print(
                "[e58] preflight WARNING: no inbound video packets detected. "
                "If video stays black, try --drone-ip 192.168.4.1/192.168.169.1/192.168.0.1, "
                "and disable VPN/firewall. Some E58 variants do not use WiFi-UAV packets and require "
                "the alternate Eachine-E58 protocol path."
            )

    drone_model = WifiUavRcModel(profile=args.profile)
    rc_adapter = WifiUavRcProtocolAdapter(
        drone_ip=drone_ip,
        control_port=args.control_port,
    )
    controller = FlightController(drone_model, rc_adapter, update_rate=args.rate)

    frame_queue = queue.Queue(maxsize=100)
    video_receiver = VideoReceiverService(
        protocol_adapter_class=WifiUavVideoProtocolAdapter,
        protocol_adapter_args={
            "drone_ip": drone_ip,
            "control_port": args.control_port,
            "video_port": args.video_port,
            "debug": args.debug_video,
        },
        frame_queue=frame_queue,
        dump_frames=args.dump_frames,
        dump_packets=args.dump_packets,
        rc_adapter=rc_adapter,
    )
    video_view = OpenCVVideoView(frame_queue, window_name="E58 Video")
    video_thread = threading.Thread(target=video_view.run, name="OpenCVVideoThread")

    def shutdown() -> None:
        video_receiver.stop()
        video_view.stop()
        if video_thread.is_alive():
            video_thread.join(timeout=1.0)
        controller.stop()

    def signal_handler(_sig, _frame) -> None:
        print("\n[e58] shutdown requested")
        shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("[e58] Starting video receiver...")
    video_receiver.start()

    print("[e58] Starting OpenCV video window (press q in window to close)...")
    video_thread.start()

    print("[e58] Starting flight controller...")
    controller.start()

    print("[e58] Controls: W/S throttle, A/D yaw, arrows pitch/roll, T takeoff, L land, Q quit")
    try:
        CLIView(controller).run()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
