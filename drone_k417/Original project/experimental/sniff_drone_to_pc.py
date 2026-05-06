#!/usr/bin/env python3
"""
Sniff inbound drone->PC UDP packets during WIFI CAM connect/video startup.

Use this while running the app (or wifi_cam_controller) and pressing connect.
The script captures packets sent by the drone and saves metadata/payload hex.

Requirements:
- pip install scapy
- Windows: run terminal as Administrator with Npcap installed

Example:
  python experimental/sniff_drone_to_pc.py \
    --iface "Wi-Fi" \
    --drone-ip 192.168.4.153 \
    --duration 20
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from collections import Counter, defaultdict

try:
    from scapy.all import IP, UDP, get_if_list, sniff  # type: ignore
except Exception:
    print("Failed to import scapy. Install with: pip install scapy", file=sys.stderr)
    raise


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Capture inbound drone->PC UDP packets")
    p.add_argument("--iface", type=str, default=None, help="Capture interface name")
    p.add_argument("--list-ifaces", action="store_true", help="List interfaces and exit")
    p.add_argument("--drone-ip", type=str, required=False, help="Drone/source IP")
    p.add_argument("--pc-ip", type=str, default=None, help="Optional local PC/destination IP filter")
    p.add_argument(
        "--src-ports",
        type=str,
        default=None,
        help="Optional comma-separated source UDP ports to include (e.g. 7070,8800)",
    )
    p.add_argument(
        "--exclude-src-ports",
        type=str,
        default="53",
        help="Comma-separated source UDP ports to exclude (default: 53 to drop DNS)",
    )
    p.add_argument(
        "--dst-ports",
        type=str,
        default=None,
        help="Optional comma-separated destination UDP ports to include",
    )
    p.add_argument(
        "--min-udp-len",
        type=int,
        default=1,
        help="Discard packets smaller than this UDP payload length",
    )
    p.add_argument("--duration", type=float, default=30.0, help="Capture duration in seconds")
    p.add_argument("--no-bpf", action="store_true", help="Use broad 'ip and udp' BPF and filter in Python")
    p.add_argument("--debug-preview", type=int, default=30, help="Print first N UDP packets seen")
    p.add_argument(
        "--out-events",
        type=str,
        default="captures/drone_to_pc_events.jsonl",
        help="JSONL output path",
    )
    p.add_argument(
        "--out-summary",
        type=str,
        default="captures/drone_to_pc_summary.json",
        help="Summary output path",
    )
    return p.parse_args()


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def as_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def looks_like_rtp(payload: bytes) -> bool:
    if len(payload) < 12:
        return False
    version = (payload[0] >> 6) & 0x03
    return version == 2


def looks_like_wifi_uav_video(payload: bytes) -> bool:
    return len(payload) >= 56 and payload[1] == 0x01


def has_jpeg_markers(payload: bytes) -> bool:
    return (b"\xFF\xD8" in payload) or (b"\xFF\xD9" in payload)


def main() -> None:
    args = parse_args()

    if args.list_ifaces:
        print("Available interfaces:")
        for name in get_if_list():
            print(f"- {name}")
        return

    if not args.iface:
        print("--iface is required unless using --list-ifaces", file=sys.stderr)
        sys.exit(2)
    if not args.drone_ip:
        print("--drone-ip is required", file=sys.stderr)
        sys.exit(2)

    def _parse_ports(text: str | None) -> set[int]:
        if not text:
            return set()
        vals: set[int] = set()
        for part in text.split(","):
            part = part.strip()
            if not part:
                continue
            vals.add(int(part))
        return vals

    include_src_ports = _parse_ports(args.src_ports)
    exclude_src_ports = _parse_ports(args.exclude_src_ports)
    include_dst_ports = _parse_ports(args.dst_ports)

    ensure_parent(args.out_events)
    ensure_parent(args.out_summary)

    strict_bpf = f"ip and udp and src host {args.drone_ip}"
    bpf = "ip and udp" if args.no_bpf else strict_bpf

    print("Capture config:")
    print(f"  iface      : {args.iface}")
    print(f"  drone_ip   : {args.drone_ip}")
    print(f"  pc_ip      : {args.pc_ip}")
    print(f"  src include: {sorted(include_src_ports) if include_src_ports else '(any)'}")
    print(f"  src exclude: {sorted(exclude_src_ports) if exclude_src_ports else '(none)'}")
    print(f"  dst include: {sorted(include_dst_ports) if include_dst_ports else '(any)'}")
    print(f"  min_udp_len: {args.min_udp_len}")
    print(f"  duration_s : {args.duration}")
    print(f"  bpf        : {bpf}")
    print(f"  out events : {args.out_events}")
    print(f"  out summary: {args.out_summary}")

    running = True

    def _stop(_sig, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    start_ts = time.time()
    udp_seen = 0
    matched = 0
    printed = 0

    port_counts = Counter()
    len_counts = Counter()
    payload_counts = Counter()
    pattern_counts = Counter()

    with open(args.out_events, "w", encoding="utf-8") as fp:

        def on_packet(pkt) -> None:
            nonlocal udp_seen, matched, printed
            if not pkt.haslayer(IP) or not pkt.haslayer(UDP):
                return
            ip = pkt[IP]
            udp = pkt[UDP]
            payload = bytes(udp.payload)
            udp_seen += 1

            if printed < args.debug_preview:
                printed += 1
                print(
                    f"[preview {printed}/{args.debug_preview}] "
                    f"{ip.src}:{udp.sport} -> {ip.dst}:{udp.dport} len={len(payload)}"
                )

            if ip.src != args.drone_ip:
                return
            if args.pc_ip and ip.dst != args.pc_ip:
                return
            if include_src_ports and int(udp.sport) not in include_src_ports:
                return
            if exclude_src_ports and int(udp.sport) in exclude_src_ports:
                return
            if include_dst_ports and int(udp.dport) not in include_dst_ports:
                return
            if len(payload) < args.min_udp_len:
                return

            matched += 1
            now = time.time()
            payload_hex = as_hex(payload)

            is_rtp = looks_like_rtp(payload)
            is_wifi_uav = looks_like_wifi_uav_video(payload)
            has_jpeg = has_jpeg_markers(payload)

            if is_rtp:
                pattern_counts["rtp_like"] += 1
            if is_wifi_uav:
                pattern_counts["wifi_uav_like"] += 1
            if has_jpeg:
                pattern_counts["jpeg_markers"] += 1

            event = {
                "ts": now,
                "src_ip": ip.src,
                "src_port": int(udp.sport),
                "dst_ip": ip.dst,
                "dst_port": int(udp.dport),
                "udp_len": len(payload),
                "payload_hex": payload_hex,
                "rtp_like": is_rtp,
                "wifi_uav_like": is_wifi_uav,
                "jpeg_markers": has_jpeg,
            }
            fp.write(json.dumps(event, ensure_ascii=True) + "\n")

            port_counts[(int(udp.sport), int(udp.dport))] += 1
            len_counts[len(payload)] += 1
            payload_counts[payload_hex] += 1

            tag = "RAW"
            if is_wifi_uav:
                tag = "WIFI-UAV?"
            elif is_rtp:
                tag = "RTP?"
            elif has_jpeg:
                tag = "JPEG?"

            print(
                f"[{matched:06d}] {tag} {ip.src}:{udp.sport} -> {ip.dst}:{udp.dport} "
                f"len={len(payload)} payload={payload_hex[:96]}"
            )

        while running and (time.time() - start_ts) < args.duration:
            sniff(iface=args.iface, filter=bpf, prn=on_packet, store=False, timeout=1)

    summary = {
        "drone_ip": args.drone_ip,
        "pc_ip": args.pc_ip,
        "duration_s": args.duration,
        "udp_seen_pre_filter": udp_seen,
        "matched_packets": matched,
        "patterns": dict(pattern_counts),
        "top_ports": [
            {"src_port": s, "dst_port": d, "count": c}
            for (s, d), c in port_counts.most_common(20)
        ],
        "top_lengths": [{"udp_len": l, "count": c} for l, c in len_counts.most_common(20)],
        "top_payloads": [
            {"payload_hex": p, "count": c} for p, c in payload_counts.most_common(20)
        ],
    }

    with open(args.out_summary, "w", encoding="utf-8") as fp:
        json.dump(summary, fp, indent=2, ensure_ascii=True)

    print("\nCapture finished")
    print(f"UDP seen (pre-filter): {udp_seen}")
    print(f"Matched drone->pc packets: {matched}")
    print(f"Summary file: {args.out_summary}")
    if matched == 0:
        print("Hint: try --no-bpf --exclude-src-ports 53 --min-udp-len 100 and capture while video is on.")


if __name__ == "__main__":
    main()
