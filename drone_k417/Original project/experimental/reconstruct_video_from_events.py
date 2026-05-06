#!/usr/bin/env python3
"""
Attempt video reconstruction from captured drone->PC UDP events JSONL.

Input format: lines containing at least:
- ts
- src_port
- dst_port
- udp_len
- payload_hex

This script tries, in order:
1) JPEG carving from concatenated payload stream (SOI/EOI markers)
2) RTP/H264 extraction to .h264
3) WiFi-UAV-like JPEG frame reassembly (56-byte custom header)

It writes outputs into --out-dir and prints what succeeded.
"""

from __future__ import annotations

import argparse
import json
import os
from collections import defaultdict
from typing import Dict, List, Tuple


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Reconstruct video from UDP event captures")
    p.add_argument("--events", type=str, required=True, help="Path to drone_to_pc_events.jsonl")
    p.add_argument("--out-dir", type=str, default="captures/reconstructed", help="Output directory")
    p.add_argument("--min-udp-len", type=int, default=20, help="Ignore tiny UDP payloads")
    return p.parse_args()


def hex_to_bytes(s: str) -> bytes:
    s = s.replace(" ", "").strip()
    return bytes.fromhex(s) if s else b""


def load_payloads(path: str, min_udp_len: int) -> List[bytes]:
    rows: List[Tuple[float, bytes]] = []
    with open(path, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if int(obj.get("udp_len", 0)) < min_udp_len:
                continue
            payload = hex_to_bytes(obj.get("payload_hex", ""))
            if payload:
                rows.append((float(obj.get("ts", 0.0)), payload))
    rows.sort(key=lambda x: x[0])
    return [p for _, p in rows]


def carve_jpeg(payloads: List[bytes], out_dir: str) -> int:
    stream = b"".join(payloads)
    frames = []
    i = 0
    while True:
        soi = stream.find(b"\xFF\xD8", i)
        if soi < 0:
            break
        eoi = stream.find(b"\xFF\xD9", soi + 2)
        if eoi < 0:
            break
        frame = stream[soi : eoi + 2]
        if len(frame) > 300:
            frames.append(frame)
        i = eoi + 2

    if not frames:
        return 0

    jpg_dir = os.path.join(out_dir, "jpeg_carved")
    os.makedirs(jpg_dir, exist_ok=True)
    for idx, frame in enumerate(frames, start=1):
        with open(os.path.join(jpg_dir, f"frame_{idx:05d}.jpg"), "wb") as fp:
            fp.write(frame)
    return len(frames)


def _is_rtp(payload: bytes) -> bool:
    return len(payload) >= 12 and ((payload[0] >> 6) & 0x03) == 2


def extract_rtp_h264(payloads: List[bytes], out_dir: str) -> int:
    h264_path = os.path.join(out_dir, "stream_rtp.h264")
    written_nals = 0
    fu_buffers: Dict[int, bytearray] = {}

    with open(h264_path, "wb") as out:
        for p in payloads:
            if not _is_rtp(p):
                continue
            cc = p[0] & 0x0F
            x = (p[0] >> 4) & 0x01
            header_len = 12 + 4 * cc
            if len(p) <= header_len:
                continue
            if x == 1:
                # Skip extension header if present.
                if len(p) < header_len + 4:
                    continue
                ext_len_words = int.from_bytes(p[header_len + 2 : header_len + 4], "big")
                header_len += 4 + ext_len_words * 4
                if len(p) <= header_len:
                    continue

            payload = p[header_len:]
            if not payload:
                continue

            nal_type = payload[0] & 0x1F

            # Single NAL unit packet.
            if 1 <= nal_type <= 23:
                out.write(b"\x00\x00\x00\x01" + payload)
                written_nals += 1
                continue

            # FU-A fragmented NAL.
            if nal_type == 28 and len(payload) >= 2:
                fu_indicator = payload[0]
                fu_header = payload[1]
                start = (fu_header >> 7) & 1
                end = (fu_header >> 6) & 1
                orig_nal_type = fu_header & 0x1F
                nri = fu_indicator & 0x60
                fbit = fu_indicator & 0x80
                rebuilt_nal_hdr = bytes([fbit | nri | orig_nal_type])

                key = 0  # single track
                if start:
                    fu_buffers[key] = bytearray(rebuilt_nal_hdr + payload[2:])
                elif key in fu_buffers:
                    fu_buffers[key].extend(payload[2:])

                if end and key in fu_buffers:
                    out.write(b"\x00\x00\x00\x01" + bytes(fu_buffers[key]))
                    written_nals += 1
                    del fu_buffers[key]

    if written_nals == 0:
        try:
            os.remove(h264_path)
        except OSError:
            pass
    return written_nals


def reassemble_wifi_uav(payloads: List[bytes], out_dir: str) -> int:
    # Heuristic for WiFi-UAV-like packets:
    # payload[1] == 0x01, frame id at bytes 16-17, frag id at bytes 32-33, jpeg bytes start at 56.
    frames = defaultdict(dict)
    last_flags = {}

    for p in payloads:
        if len(p) < 56 or p[1] != 0x01:
            continue
        fid = int.from_bytes(p[16:18], "little")
        frag = int.from_bytes(p[32:34], "little")
        frames[fid][frag] = p[56:]
        last_flags[(fid, frag)] = p[2]

    out_count = 0
    out_path = os.path.join(out_dir, "wifi_uav_frames")
    os.makedirs(out_path, exist_ok=True)

    for fid in sorted(frames):
        frags = frames[fid]
        if not frags:
            continue
        blob = b"".join(frags[k] for k in sorted(frags))
        if not blob:
            continue

        # If no JPEG markers exist, skip writing since this family may require external headers.
        if b"\xFF\xD8" not in blob and b"\xFF\xD9" not in blob:
            continue

        # Ensure minimal JPEG boundaries.
        soi = blob.find(b"\xFF\xD8")
        eoi = blob.rfind(b"\xFF\xD9")
        if soi >= 0 and eoi > soi:
            jpg = blob[soi : eoi + 2]
            if len(jpg) > 300:
                out_count += 1
                with open(os.path.join(out_path, f"frame_{fid:05d}.jpg"), "wb") as fp:
                    fp.write(jpg)

    if out_count == 0:
        try:
            os.rmdir(out_path)
        except OSError:
            pass
    return out_count


def main() -> None:
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    payloads = load_payloads(args.events, args.min_udp_len)
    print(f"Loaded {len(payloads)} payloads with len >= {args.min_udp_len}")
    if not payloads:
        print("No payloads to process.")
        return

    jpg_count = carve_jpeg(payloads, args.out_dir)
    print(f"JPEG carve frames: {jpg_count}")

    nal_count = extract_rtp_h264(payloads, args.out_dir)
    print(f"RTP/H264 NAL units: {nal_count}")

    wu_count = reassemble_wifi_uav(payloads, args.out_dir)
    print(f"WiFi-UAV-like frames: {wu_count}")

    print("Done. Check output directory:")
    print(args.out_dir)


if __name__ == "__main__":
    main()
