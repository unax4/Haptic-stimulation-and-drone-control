#!/usr/bin/env python3
"""
Live monitor for Arduino haptic debug lines.

Parses lines like:
  [HDBG] TRAIN axis=YAW pos=M4 pot=22 active=[YRT]
  [HDBG] TRAIN idle (all controls neutral)

And plots current pot value per channel in real time.
"""

from __future__ import annotations

import argparse
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial\n"
        "Install with: pip install pyserial matplotlib"
    ) from exc


AXES = ("YAW", "PITCH", "ROLL", "THROTTLE")
AXIS_TO_SHORT = {"YAW": "Y", "PITCH": "P", "ROLL": "R", "THROTTLE": "T"}
SHORT_TO_AXIS = {v: k for k, v in AXIS_TO_SHORT.items()}
POS_TO_AXIS = {"M4": "YAW", "M8": "PITCH", "M12": "ROLL", "M20": "THROTTLE"}

TRAIN_RE = re.compile(
    r"^\[HDBG\]\s+TRAIN\s+axis=(YAW|PITCH|ROLL|THROTTLE)\s+pos=(M\d+)\s+pot=(\d+)\s+active=\[([YPRT]*)\]"
)
IDLE_RE = re.compile(r"^\[HDBG\]\s+TRAIN\s+idle\b")
BURST_RE = re.compile(
    r"^\[HDBG\]\s+BURST\s+pos=(M\d+)\s+pot=(\d+)\s+count=(\d+)\s+on=(\d+)ms\s+off=(\d+)ms\s+lock=(\d+)ms"
)


@dataclass
class AxisState:
    pot: int = 0
    pos: str = "M?"
    active: bool = False
    last_seen: float = 0.0


@dataclass
class BurstState:
    pos: str = "M?"
    pot: int = 0
    count: int = 0
    on_ms: int = 0
    off_ms: int = 0
    lock_ms: int = 0
    last_seen: float = 0.0


@dataclass
class MonitorState:
    axis: Dict[str, AxisState] = field(default_factory=lambda: {a: AxisState() for a in AXES})
    last_line: str = ""
    last_update: float = 0.0
    burst: Optional[BurstState] = None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot live haptic channel stimulation from Arduino serial debug.")
    p.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--stale-ms", type=int, default=120, help="Mark axis inactive if no update within this time")
    p.add_argument("--refresh-ms", type=int, default=50, help="Plot refresh interval")
    return p.parse_args()


def serial_reader(port: str, baud: int, out_q: queue.Queue[str], stop_evt: threading.Event) -> None:
    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
        time.sleep(1.5)  # Give board time after opening port
        ser.write(b"HDBGON\n")
        while not stop_evt.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            out_q.put(line)


def apply_line(state: MonitorState, line: str) -> None:
    now = time.time()
    state.last_line = line

    m_burst = BURST_RE.match(line)
    if m_burst:
        pos, pot_text, count_text, on_text, off_text, lock_text = m_burst.groups()
        state.burst = BurstState(
            pos=pos,
            pot=int(pot_text),
            count=int(count_text),
            on_ms=int(on_text),
            off_ms=int(off_text),
            lock_ms=int(lock_text),
            last_seen=now,
        )
        state.last_update = now
        return

    if IDLE_RE.match(line):
        for axis in AXES:
            st = state.axis[axis]
            st.active = False
            st.pot = 0
        state.last_update = now
        return

    m = TRAIN_RE.match(line)
    if not m:
        return

    axis_name, pos, pot_text, active_text = m.groups()
    pot = int(pot_text)
    active_set = {SHORT_TO_AXIS[c] for c in active_text if c in SHORT_TO_AXIS}

    # Update the chosen mux axis with current routing/pot.
    chosen = state.axis[axis_name]
    chosen.pot = pot
    chosen.pos = pos
    chosen.last_seen = now

    # Update active flags from the transmitted active bitmap.
    for axis in AXES:
        st = state.axis[axis]
        st.active = axis in active_set
        if not st.active:
            st.pot = 0

    state.last_update = now


def main() -> None:
    args = parse_args()
    q: queue.Queue[str] = queue.Queue()
    stop_evt = threading.Event()
    state = MonitorState()

    thread = threading.Thread(
        target=serial_reader, args=(args.port, args.baud, q, stop_evt), daemon=True
    )
    thread.start()

    fig, ax = plt.subplots(figsize=(9, 4.5))
    bars = ax.bar(AXES, [0, 0, 0, 0], color=["#3b82f6"] * 4)
    labels = [ax.text(i, 3, "", ha="center", va="bottom", fontsize=10) for i in range(len(AXES))]

    ax.set_ylim(0, 255)
    ax.set_ylabel("Pot Value (0-255)")
    ax.set_title("Live Haptic Channel Stimulation")
    ax.grid(axis="y", alpha=0.25)
    status_text = fig.text(
        0.01, 0.985, "Waiting for [HDBG] TRAIN lines...",
        fontsize=10, va="top", ha="left"
    )
    burst_text = fig.text(
        0.99, 0.985, "",
        fontsize=10, va="top", ha="right", color="#b45309"
    )

    stale_s = args.stale_ms / 1000.0

    def redraw(_frame: int):
        while True:
            try:
                line = q.get_nowait()
            except queue.Empty:
                break
            apply_line(state, line)

        now = time.time()
        burst_axis: Optional[str] = None
        burst_pot = 0
        burst_recent = False
        burst_age_ms = -1
        if state.burst is not None:
            burst_age_ms = int((now - state.burst.last_seen) * 1000)
            active_window_ms = max(250, state.burst.count * (state.burst.on_ms + state.burst.off_ms))
            burst_recent = burst_age_ms <= active_window_ms
            burst_axis = POS_TO_AXIS.get(state.burst.pos)
            burst_pot = state.burst.pot

        for i, axis in enumerate(AXES):
            st = state.axis[axis]
            stale = (now - st.last_seen) > stale_s
            visible_active = st.active and not stale
            val = st.pot if visible_active else 0
            if burst_recent and burst_axis == axis:
                val = max(val, burst_pot)
            bars[i].set_height(val)
            if burst_recent and burst_axis == axis:
                bars[i].set_color("#f59e0b")
                labels[i].set_text(f"{st.pos} | P={val} | BURST")
            else:
                bars[i].set_color("#16a34a" if visible_active else "#94a3b8")
                labels[i].set_text(f"{st.pos} | P={val}")
            labels[i].set_y(min(252, val + 4))

        age_ms = int((now - state.last_update) * 1000) if state.last_update else -1
        status_text.set_text(
            f"Last update: {age_ms} ms ago | Last line: {state.last_line[:120]}"
        )
        if state.burst is not None:
            burst_text.set_text(
                f"Last BURST: {state.burst.pos} P={state.burst.pot} x{state.burst.count} ({burst_age_ms} ms ago)"
            )
        else:
            burst_text.set_text("")
        return [*bars, *labels, status_text, burst_text]

    ani = FuncAnimation(fig, redraw, interval=args.refresh_ms, blit=False)

    try:
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
        plt.show()
    finally:
        stop_evt.set()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
