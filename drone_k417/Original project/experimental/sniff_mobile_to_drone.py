#!/usr/bin/env python3
"""
Capture mobile->drone UDP traffic while the phone is connected through the PC hotspot.

This script is designed for the workflow you described:
1) Phone connected to Windows hotspot
2) Open drone app and press controls
3) Capture packets on PC and share payloads for reverse engineering

Requirements:
- Python 3.10+
- scapy: pip install scapy
- Windows: Npcap installed (WinPcap API compatible mode) and run terminal as Administrator

Examples:
  python experimental/sniff_mobile_to_drone.py --list-ifaces

  python experimental/sniff_mobile_to_drone.py \
    --iface "Wi-Fi" --phone-ip 192.168.137.195 --drone-ip 192.168.4.153 --ports 8080,8090
& c:\python311\python.exe "c:/Users/Unax/Desktop/Legacy Code/drone_k417/Original project/experimental/sniff_mobile_to_drone.py" --iface "Wi-Fi" --no-bpf --debug-udp-preview 50 --ports 8080,8090
Output files (default):
- captures/mobile_to_drone_events.jsonl
- captures/mobile_to_drone_unique.json
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Tuple

try:
    from scapy.all import IP, UDP, get_if_list, sniff  # type: ignore
except Exception as exc:  # pragma: no cover
    print("Failed to import scapy. Install with: pip install scapy", file=sys.stderr)
    raise


@dataclass
class UniquePacketStats:
    count: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sniff mobile->drone UDP packets")
    parser.add_argument("--iface", type=str, default=None, help="Capture interface name")
    parser.add_argument("--list-ifaces", action="store_true", help="List interfaces and exit")
    parser.add_argument("--phone-ip", type=str, required=False, help="Phone/source IP (optional)")
    parser.add_argument("--drone-ip", type=str, required=False, help="Drone/destination IP (optional)")
    parser.add_argument(
        "--ports",
        type=str,
        default="8080,8090",
        help="Comma-separated UDP destination ports to include (default: 8080,8090)",
    )
    parser.add_argument(
        "--out-events",
        type=str,
        default="captures/mobile_to_drone_events.jsonl",
        help="Path to JSONL output for packet events",
    )
    parser.add_argument(
        "--out-unique",
        type=str,
        default="captures/mobile_to_drone_unique.json",
        help="Path to JSON output for unique payload summary",
    )
    parser.add_argument(
        "--no-bpf",
        action="store_true",
        help="Disable strict libpcap filter and capture broad UDP, then filter in Python",
    )
    parser.add_argument(
        "--debug-udp-preview",
        type=int,
        default=0,
        help="Print first N UDP packets seen before filtering to discover correct IP/ports",
    )
    return parser.parse_args()


def xor5_ok(payload: bytes) -> bool:
    """Check candidate 8-byte WIFI CAM control frame checksum.

    Expected shape: [0x66][b1][b2][b3][b4][b5][xor(b1..b5)][0x99]
    """
    if len(payload) != 8:
        return False
    if payload[0] != 0x66 or payload[7] != 0x99:
        return False
    chk = payload[1] ^ payload[2] ^ payload[3] ^ payload[4] ^ payload[5]
    return chk == payload[6]


def as_hex(payload: bytes) -> str:
    return " ".join(f"{b:02X}" for b in payload)


def ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


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

    try:
        ports = [int(p.strip()) for p in args.ports.split(",") if p.strip()]
    except ValueError:
        print("Invalid --ports value. Example: 8080,8090", file=sys.stderr)
        sys.exit(2)

    if not ports:
        print("No ports specified", file=sys.stderr)
        sys.exit(2)

    ensure_parent_dir(args.out_events)
    ensure_parent_dir(args.out_unique)

    unique: Dict[Tuple[int, str], UniquePacketStats] = defaultdict(UniquePacketStats)
    total = 0

    bpf_ports = " or ".join(f"udp dst port {p}" for p in ports)
    strict_bpf = (
        f"ip and udp and src host {args.phone_ip} and dst host {args.drone_ip} and ({bpf_ports})"
        if args.phone_ip and args.drone_ip
        else f"ip and udp and ({bpf_ports})"
    )
    bpf = "ip and udp" if args.no_bpf else strict_bpf

    print("Capture config:")
    print(f"  iface      : {args.iface}")
    print(f"  phone_ip   : {args.phone_ip}")
    print(f"  drone_ip   : {args.drone_ip}")
    print(f"  ports      : {ports}")
    print(f"  bpf mode   : {'broad (ip and udp)' if args.no_bpf else 'strict'}")
    print(f"  bpf filter : {bpf}")
    print(f"  events out : {args.out_events}")
    print(f"  unique out : {args.out_unique}")
    print("Press Ctrl+C to stop and write summary.")

    running = True

    def _stop(_sig, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    udp_seen = 0
    udp_preview_printed = 0

    with open(args.out_events, "w", encoding="utf-8") as event_fp:

        def on_packet(pkt) -> None:
            nonlocal total, udp_seen, udp_preview_printed
            if not pkt.haslayer(IP) or not pkt.haslayer(UDP):
                return

            ip = pkt[IP]
            udp = pkt[UDP]
            payload = bytes(udp.payload)
            udp_seen += 1

            if args.debug_udp_preview > 0 and udp_preview_printed < args.debug_udp_preview:
                udp_preview_printed += 1
                print(
                    f"[udp-preview {udp_preview_printed}/{args.debug_udp_preview}] "
                    f"{ip.src}:{udp.sport} -> {ip.dst}:{udp.dport} len={len(payload)}"
                )

            # Python-side filtering (works even when BPF cannot be trusted on some adapters)
            if int(udp.dport) not in ports:
                return
            if args.phone_ip and ip.src != args.phone_ip:
                return
            if args.drone_ip and ip.dst != args.drone_ip:
                return

            total += 1
            now = time.time()
            payload_hex = as_hex(payload)
            key = (int(udp.dport), payload_hex)
            stats = unique[key]
            if stats.count == 0:
                stats.first_ts = now
            stats.count += 1
            stats.last_ts = now

            is_cam8 = xor5_ok(payload)
            event = {
                "ts": now,
                "src_ip": ip.src,
                "src_port": int(udp.sport),
                "dst_ip": ip.dst,
                "dst_port": int(udp.dport),
                "udp_len": len(payload),
                "payload_hex": payload_hex,
                "candidate_cam8": is_cam8,
            }
            event_fp.write(json.dumps(event, ensure_ascii=True) + "\n")
            event_fp.flush()

            label = "CAM8" if is_cam8 else "RAW"
            print(
                f"[{total:06d}] {label} "
                f"{ip.src}:{udp.sport} -> {ip.dst}:{udp.dport} "
                f"len={len(payload)} payload={payload_hex}"
            )

            if is_cam8:
                print(
                    "         fields: "
                    f"b1=0x{payload[1]:02X} b2=0x{payload[2]:02X} "
                    f"b3=0x{payload[3]:02X} b4=0x{payload[4]:02X} "
                    f"cmd=0x{payload[5]:02X} chk=0x{payload[6]:02X}"
                )

        while running:
            sniff(
                iface=args.iface,
                filter=bpf,
                prn=on_packet,
                store=False,
                timeout=1,
            )

    unique_out = []
    for (dst_port, payload_hex), stats in sorted(
        unique.items(),
        key=lambda kv: kv[1].count,
        reverse=True,
    ):
        unique_out.append(
            {
                "dst_port": dst_port,
                "payload_hex": payload_hex,
                "count": stats.count,
                "first_ts": stats.first_ts,
                "last_ts": stats.last_ts,
            }
        )

    with open(args.out_unique, "w", encoding="utf-8") as fp:
        json.dump(
            {
                "total_packets": total,
                "phone_ip": args.phone_ip,
                "drone_ip": args.drone_ip,
                "ports": ports,
                "unique_packets": unique_out,
            },
            fp,
            indent=2,
            ensure_ascii=True,
        )

    print("\nCapture stopped.")
    print(f"UDP packets seen (pre-filter): {udp_seen}")
    print(f"Total packets: {total}")
    print(f"Wrote events : {args.out_events}")
    print(f"Wrote unique : {args.out_unique}")

    if unique_out:
        print("Top unique payloads:")
        for row in unique_out[:10]:
            print(
                f"- dport={row['dst_port']} count={row['count']} "
                f"payload={row['payload_hex']}"
            )


if __name__ == "__main__":
    main()
