#!/usr/bin/env python3
"""Simple 2-input glove dataset builder.

Runs with no arguments:
- Connects to COM3 @ 115200
- Randomly asks for a position (0..8)
- Waits for Enter
- Reads one valid telemetry sample and appends one line to a .txt file:
    A1 A0 position_num
"""

from __future__ import annotations

import random
import re
import threading
import time
from collections import Counter
from pathlib import Path
from typing import Optional, Tuple

import serial


PORT = "COM3"
BAUD = 115200
OUTPUT_FILE = Path("drone_k417/neural/dataset_glove_2in_9pos_new.txt")
NUM_POSITIONS = 9

PATTERN = re.compile(
    r"A0\s*:\s*(-?\d+(?:\.\d+)?)\s+A1\s*:\s*(-?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def parse_a1_a0(line: str) -> Optional[Tuple[float, float]]:
    """Return (a1, a0) from telemetry line, else None."""
    m = PATTERN.search(line)
    if m:
        return float(m.group(2)), float(m.group(1))

    # Legacy fallback: timestamp,A3,A2,A1,A0,...
    if "," in line:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 5:
            try:
                return float(parts[3]), float(parts[4])
            except ValueError:
                return None

    return None


def read_one_sample(ser: serial.Serial):
    """Block until one valid A1/A0 sample is received."""
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        parsed = parse_a1_a0(line)
        if parsed is not None:
            return parsed


def serial_reader_loop(
    ser: serial.Serial,
    stop_event: threading.Event,
    latest_holder: dict,
    latest_lock: threading.Lock,
) -> None:
    """Continuously update latest A1/A0 sample from serial."""
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


def show_dataset_class_histogram(dataset_path: Path, num_positions: int) -> None:
    """Display a bar chart with sample counts per class from dataset file."""
    counts = Counter({i: 0 for i in range(num_positions)})
    valid_rows = 0

    if not dataset_path.exists():
        print(f"Dataset file not found: {dataset_path}")
        return

    with dataset_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            parts = line.split()
            if len(parts) != 3:
                continue

            try:
                label = int(float(parts[2]))
            except ValueError:
                continue

            if 0 <= label < num_positions:
                counts[label] += 1
                valid_rows += 1

    print("\nSamples per class:")
    for label in range(num_positions):
        print(f"  class {label}: {counts[label]}")
    print(f"Total valid rows in dataset: {valid_rows}")

    try:
        import matplotlib.pyplot as plt

        labels = list(range(num_positions))
        values = [counts[i] for i in labels]

        plt.figure("Glove dataset class counts", figsize=(8, 4))
        bars = plt.bar(labels, values)
        plt.title("Samples per class")
        plt.xlabel("Class label")
        plt.ylabel("Samples")
        plt.xticks(labels)

        for bar, value in zip(bars, values):
            plt.text(
                bar.get_x() + bar.get_width() / 2,
                value,
                str(value),
                ha="center",
                va="bottom",
                fontsize=9,
            )

        plt.tight_layout()
        plt.show()
    except Exception as exc:
        print(f"Could not display bar chart: {exc}")
        print("Install matplotlib if needed: pip install matplotlib")


def get_dataset_class_counts(dataset_path: Path, num_positions: int) -> Counter:
    """Read dataset and return class counts for labels 0..num_positions-1."""
    counts = Counter({i: 0 for i in range(num_positions)})

    if not dataset_path.exists():
        return counts

    with dataset_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            parts = line.split()
            if len(parts) != 3:
                continue

            try:
                label = int(float(parts[2]))
            except ValueError:
                continue

            if 0 <= label < num_positions:
                counts[label] += 1

    return counts


def choose_balanced_position(dataset_path: Path, num_positions: int) -> int:
    """Sample position with higher probability for classes with fewer samples."""
    counts = get_dataset_class_counts(dataset_path, num_positions)
    max_count = max(counts.values()) if counts else 0

    # Inverse-to-frequency weighting: fewer samples => larger weight.
    weights = [(max_count - counts[i] + 1) for i in range(num_positions)]
    return random.choices(range(num_positions), weights=weights, k=1)[0]


def main() -> None:
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)

    print(f"Connecting to {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.25)
    time.sleep(1.5)
    ser.reset_input_buffer()

    latest_holder = {"sample": None}
    latest_lock = threading.Lock()
    stop_event = threading.Event()
    reader = threading.Thread(
        target=serial_reader_loop,
        args=(ser, stop_event, latest_holder, latest_lock),
        daemon=True,
    )
    reader.start()

    print(f"Saving dataset lines to: {OUTPUT_FILE.resolve()}")
    print("Format: A1 A0 position_num")
    print("Press Ctrl+C to stop.\n")

    sample_count = 0

    try:
        with OUTPUT_FILE.open("a", encoding="utf-8") as f:
            while True:
                pos = choose_balanced_position(OUTPUT_FILE, NUM_POSITIONS)
                print("=" * 50)
                print(f"Do position: {pos}")
                input("Press Enter to capture current sample... ")

                with latest_lock:
                    sample = latest_holder["sample"]

                if sample is None:
                    print("No serial sample received yet. Try again.")
                    continue

                a1, a0 = sample
                a1_i = int(round(a1))
                a0_i = int(round(a0))
                line = f"{a1_i} {a0_i} {pos}\n"
                f.write(line)
                f.flush()

                sample_count += 1
                print(f"Saved #{sample_count}: {line.strip()}")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_event.set()
        reader.join(timeout=0.5)
        ser.close()
        print(f"Total saved samples: {sample_count}")
        show_dataset_class_histogram(OUTPUT_FILE, NUM_POSITIONS)


if __name__ == "__main__":
    main()
