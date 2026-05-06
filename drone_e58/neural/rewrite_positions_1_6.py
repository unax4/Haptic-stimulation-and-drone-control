#!/usr/bin/env python3
"""Rewrite selected class samples in-place by re-capturing from live serial telemetry.

Default behavior:
- Opens dataset_glove_2in_9pos.txt
- Finds rows whose label is 1 or 6
- For each matching row, asks user to perform that position and press Enter
- Captures a fresh A1/A0 sample from serial and replaces that row
- Keeps non-target rows unchanged
- Saves a backup before overwriting
"""

from __future__ import annotations

import argparse
import re
import threading
import time
from collections.abc import Iterable
from pathlib import Path

import serial


DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200
DEFAULT_DATASET = Path(__file__).with_name("dataset_glove_2in_9pos.txt")
DEFAULT_TARGET_LABELS = (1, 6)

PATTERN = re.compile(
    r"A0\s*:\s*(-?\d+(?:\.\d+)?)\s+A1\s*:\s*(-?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def parse_a1_a0(line: str) -> tuple[float, float] | None:
    """Return (a1, a0) from telemetry line, else None."""
    m = PATTERN.search(line)
    if m:
        return float(m.group(2)), float(m.group(1))

    # Legacy CSV fallback: timestamp,A3,A2,A1,A0,...
    if "," in line:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 5:
            try:
                return float(parts[3]), float(parts[4])
            except ValueError:
                return None

    return None


def serial_reader_loop(
    ser: serial.Serial,
    stop_event: threading.Event,
    latest_holder: dict,
    latest_lock: threading.Lock,
) -> None:
    """Continuously parse serial lines and store latest valid A1/A0 sample."""
    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="ignore").strip()
        parsed = parse_a1_a0(line)
        if parsed is None:
            continue

        with latest_lock:
            latest_holder["sample"] = parsed
            latest_holder["ts"] = time.time()


def parse_dataset_row(line: str) -> tuple[int, int, int] | None:
    parts = line.strip().split()
    if len(parts) != 3:
        return None

    try:
        a1 = int(round(float(parts[0])))
        a0 = int(round(float(parts[1])))
        label = int(round(float(parts[2])))
    except ValueError:
        return None

    return a1, a0, label


def wait_for_fresh_sample(
    latest_holder: dict,
    latest_lock: threading.Lock,
    after_ts: float,
    timeout_s: float,
) -> tuple[int, int]:
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        with latest_lock:
            sample = latest_holder.get("sample")
            ts = latest_holder.get("ts", 0.0)

        if sample is not None and ts > after_ts:
            a1, a0 = sample
            return int(round(a1)), int(round(a0))

        time.sleep(0.01)

    raise TimeoutError("Timed out waiting for a fresh serial sample")


def find_target_rows(lines: Iterable[str], target_labels: set[int]) -> list[tuple[int, int, int, int]]:
    """Return list of (row_index, old_a1, old_a0, label) to rewrite."""
    targets: list[tuple[int, int, int, int]] = []

    for i, line in enumerate(lines):
        parsed = parse_dataset_row(line)
        if parsed is None:
            continue
        old_a1, old_a0, label = parsed
        if label in target_labels:
            targets.append((i, old_a1, old_a0, label))

    return targets


def main() -> None:
    parser = argparse.ArgumentParser(description="Rewrite labels 1/6 dataset rows by recapturing serial samples")
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET, help="Dataset txt path")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud")
    parser.add_argument("--labels", default="1,6", help="Comma-separated labels to rewrite")
    parser.add_argument("--timeout", type=float, default=8.0, help="Timeout per capture (seconds)")
    args = parser.parse_args()

    if not args.dataset.exists():
        raise FileNotFoundError(f"Dataset not found: {args.dataset}")

    target_labels = {
        int(x.strip())
        for x in args.labels.split(",")
        if x.strip() != ""
    }

    lines = args.dataset.read_text(encoding="utf-8", errors="ignore").splitlines()
    targets = find_target_rows(lines, target_labels)

    if not targets:
        print(f"No rows found for labels: {sorted(target_labels)}")
        return

    print(f"Dataset: {args.dataset.resolve()}")
    print(f"Rows to rewrite: {len(targets)} (labels={sorted(target_labels)})")
    print(f"Connecting serial: {args.port} @ {args.baud}")

    ser = serial.Serial(args.port, args.baud, timeout=0.25)
    time.sleep(1.5)
    ser.reset_input_buffer()

    latest_holder = {"sample": None, "ts": 0.0}
    latest_lock = threading.Lock()
    stop_event = threading.Event()
    reader = threading.Thread(
        target=serial_reader_loop,
        args=(ser, stop_event, latest_holder, latest_lock),
        daemon=True,
    )
    reader.start()

    rewritten = 0
    try:
        for n, (row_idx, old_a1, old_a0, label) in enumerate(targets, start=1):
            print("=" * 60)
            print(f"[{n}/{len(targets)}] Row {row_idx + 1}: label={label} old=({old_a1}, {old_a0})")
            print(f"Do position {label} now.")

            with latest_lock:
                last_ts = float(latest_holder.get("ts", 0.0))

            input("Press Enter to capture a fresh sample... ")

            try:
                new_a1, new_a0 = wait_for_fresh_sample(
                    latest_holder,
                    latest_lock,
                    after_ts=last_ts,
                    timeout_s=args.timeout,
                )
            except TimeoutError:
                print("No fresh sample arrived in time. Keep old row and continue.")
                continue

            lines[row_idx] = f"{new_a1} {new_a0} {label}"
            rewritten += 1
            print(f"Saved row {row_idx + 1} -> {lines[row_idx]}")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        stop_event.set()
        reader.join(timeout=0.6)
        ser.close()

    backup = args.dataset.with_suffix(args.dataset.suffix + ".bak")
    backup.write_text("\n".join(args.dataset.read_text(encoding="utf-8", errors="ignore").splitlines()) + "\n", encoding="utf-8")
    args.dataset.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("\nDone.")
    print(f"Rows rewritten: {rewritten}/{len(targets)}")
    print(f"Backup saved to: {backup}")
    print(f"Updated dataset: {args.dataset}")


if __name__ == "__main__":
    main()
