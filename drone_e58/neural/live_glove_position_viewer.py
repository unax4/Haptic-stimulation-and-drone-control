#!/usr/bin/env python3
"""Realtime glove position viewer using a trained FCNN model.

Reads serial telemetry from Arduino, extracts A1/A0 values,
predicts position with the saved model, and shows it in a small GUI window.
"""

from __future__ import annotations

import argparse
import re
import threading
import time
import tkinter as tk
from pathlib import Path
from typing import Optional, Tuple

import joblib
import numpy as np
import serial


LABELED_PATTERN = re.compile(
    r"A0\s*:\s*(-?\d+(?:\.\d+)?)\s+A1\s*:\s*(-?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def parse_a1_a0(line: str) -> Optional[Tuple[float, float]]:
    """Parse telemetry line and return (a1, a0), or None if invalid."""
    m = LABELED_PATTERN.search(line)
    if m:
        a0 = float(m.group(1))
        a1 = float(m.group(2))
        return a1, a0

    # Legacy CSV fallback: timestamp,A3,A2,A1,A0,...
    if "," in line:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 5:
            try:
                return float(parts[3]), float(parts[4])
            except ValueError:
                return None

    return None


def serial_predict_loop(
    ser: serial.Serial,
    model,
    shared_state: dict,
    state_lock: threading.Lock,
    stop_event: threading.Event,
) -> None:
    """Read serial lines, predict position, and update shared state."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            sample = parse_a1_a0(line)
            if sample is None:
                continue

            a1, a0 = sample
            x = np.array([[a1, a0]], dtype=float)
            pred = int(model.predict(x)[0])

            with state_lock:
                shared_state["a1"] = int(round(a1))
                shared_state["a0"] = int(round(a0))
                shared_state["pred"] = pred
                shared_state["updated_at"] = time.time()
                shared_state["raw_line"] = line
        except Exception as exc:  # Keep UI alive even if one read fails.
            with state_lock:
                shared_state["error"] = str(exc)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    default_model = script_dir / "glove_fcnn_model.joblib"

    parser = argparse.ArgumentParser(description="Live glove position viewer")
    parser.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--model", type=Path, default=default_model, help="Path to saved model joblib")
    parser.add_argument("--refresh-ms", type=int, default=100, help="GUI refresh interval")
    args = parser.parse_args()

    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    saved = joblib.load(args.model)
    model = saved["model"] if isinstance(saved, dict) and "model" in saved else saved

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.25)
    time.sleep(1.5)
    ser.reset_input_buffer()

    shared_state = {
        "a1": None,
        "a0": None,
        "pred": None,
        "updated_at": 0.0,
        "raw_line": "",
        "error": None,
    }
    state_lock = threading.Lock()
    stop_event = threading.Event()

    worker = threading.Thread(
        target=serial_predict_loop,
        args=(ser, model, shared_state, state_lock, stop_event),
        daemon=True,
    )
    worker.start()

    root = tk.Tk()
    root.title("Glove Position Predictor")
    root.geometry("440x240")
    root.minsize(380, 200)

    title_lbl = tk.Label(root, text="Current Predicted Position", font=("Segoe UI", 16, "bold"))
    title_lbl.pack(pady=(18, 8))

    pred_var = tk.StringVar(value="--")
    pred_lbl = tk.Label(root, textvariable=pred_var, font=("Segoe UI", 52, "bold"), fg="#0b6e4f")
    pred_lbl.pack(pady=(0, 8))

    input_var = tk.StringVar(value="A1: --   A0: --")
    input_lbl = tk.Label(root, textvariable=input_var, font=("Consolas", 13))
    input_lbl.pack(pady=(2, 4))

    status_var = tk.StringVar(value="Waiting for serial data...")
    status_lbl = tk.Label(root, textvariable=status_var, font=("Segoe UI", 10), fg="#444444")
    status_lbl.pack(pady=(2, 4))

    def refresh_ui() -> None:
        with state_lock:
            a1 = shared_state["a1"]
            a0 = shared_state["a0"]
            pred = shared_state["pred"]
            updated_at = shared_state["updated_at"]
            err = shared_state["error"]

        if pred is not None:
            pred_var.set(str(pred))
            input_var.set(f"A1: {a1}   A0: {a0}")
            age = time.time() - updated_at
            status_var.set(f"Live | last update: {age:.2f}s ago")
        else:
            pred_var.set("--")
            input_var.set("A1: --   A0: --")
            status_var.set("Waiting for valid A1/A0 telemetry...")

        if err:
            status_var.set(f"Read error: {err}")

        root.after(max(20, args.refresh_ms), refresh_ui)

    def on_close() -> None:
        stop_event.set()
        worker.join(timeout=0.6)
        ser.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    refresh_ui()
    root.mainloop()


if __name__ == "__main__":
    main()
