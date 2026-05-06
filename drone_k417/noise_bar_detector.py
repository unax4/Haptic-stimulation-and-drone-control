#!/usr/bin/env python3
"""
E58 live viewer with raw noise profiling and profile-driven live cleaning.
"""

from __future__ import annotations

import json
import select
import socket
import threading
import time
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np

CONNECT = bytes.fromhex("42 76")
DISCONNECT = bytes.fromhex("42 77")
PRESTREAM = bytes.fromhex("AA 80 80 00 80 00 80 55")

DEFAULT_IP = "192.168.4.153"
DEFAULT_SESSION_PORT = 8080
DEFAULT_CONTROL_PORT = 8090


def _build_cam8(roll: int, pitch: int, throttle: int, yaw: int, cmd: int = 0) -> bytes:
    b1 = int(max(0, min(255, roll)))
    b2 = int(max(0, min(255, pitch)))
    b3 = int(max(0, min(255, throttle)))
    b4 = int(max(0, min(255, yaw)))
    c = int(max(0, min(255, cmd)))
    chk = b1 ^ b2 ^ b3 ^ b4 ^ c
    return bytes((0x66, b1, b2, b3, b4, c, chk & 0xFF, 0x99))


class WifiCamStream:
    def __init__(
        self,
        drone_ip: str,
        session_port: int = DEFAULT_SESSION_PORT,
        control_port: int = DEFAULT_CONTROL_PORT,
        rate_hz: float = 30.0,
        prestream_count: int = 6,
    ) -> None:
        self.drone_ip = drone_ip
        self.session_port = int(session_port)
        self.control_port = int(control_port)
        self.prestream_count = max(1, int(prestream_count))
        self.interval = 1.0 / max(1e-6, float(rate_hz))

        self.session_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.session_sock.bind(("", 0))
        self.control_sock.bind(("", 0))
        self.session_sock.setblocking(False)
        self.control_sock.setblocking(False)

        self._running = False
        self._ctrl_thread: threading.Thread | None = None
        self._latest_frame: np.ndarray | None = None
        self._frag = bytearray()

    def start(self) -> None:
        self.session_sock.sendto(CONNECT, (self.drone_ip, self.session_port))
        time.sleep(0.02)
        for _ in range(self.prestream_count):
            self.control_sock.sendto(PRESTREAM, (self.drone_ip, self.control_port))
            time.sleep(0.03)

        self._running = True
        self._ctrl_thread = threading.Thread(target=self._ctrl_loop, daemon=True, name="NoiseCtrl")
        self._ctrl_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._ctrl_thread and self._ctrl_thread.is_alive():
            self._ctrl_thread.join(timeout=1.0)

        try:
            self.session_sock.sendto(DISCONNECT, (self.drone_ip, self.session_port))
        except OSError:
            pass

        for s in (self.session_sock, self.control_sock):
            try:
                s.close()
            except OSError:
                pass

    def _ctrl_loop(self) -> None:
        neutral = _build_cam8(0x80, 0x80, 0x80, 0x80, 0x00)
        while self._running:
            try:
                self.control_sock.sendto(neutral, (self.drone_ip, self.control_port))
            except OSError:
                pass
            time.sleep(self.interval)

    def _extract_jpegs(self, payload: bytes) -> list[bytes]:
        out: list[bytes] = []

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

        if out:
            self._frag.clear()
            return out

        soi = payload.find(b"\xFF\xD8")
        if soi >= 0:
            self._frag = bytearray(payload[soi:])
        elif self._frag:
            self._frag.extend(payload)

        if len(self._frag) > 2 * 1024 * 1024:
            self._frag.clear()
            return out

        if self._frag:
            eoi = self._frag.find(b"\xFF\xD9")
            if eoi >= 0:
                jpg = bytes(self._frag[: eoi + 2])
                self._frag = bytearray(self._frag[eoi + 2 :])
                if len(jpg) >= 300:
                    out.append(jpg)

        return out

    def poll(self, timeout_s: float = 0.02) -> np.ndarray | None:
        sockets = [self.control_sock, self.session_sock]
        try:
            readable, _, _ = select.select(sockets, [], [], max(0.0, timeout_s))
        except OSError:
            return self._latest_frame

        for sock in readable:
            while True:
                try:
                    payload, addr = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                except OSError:
                    break

                if addr[0] != self.drone_ip:
                    continue

                for jpg in self._extract_jpegs(payload):
                    arr = np.frombuffer(jpg, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        self._latest_frame = frame

        return self._latest_frame.copy() if self._latest_frame is not None else None


def _detect_bad_rows(frame: np.ndarray, z_thresh: float, row_expand: int) -> np.ndarray:
    """Detect row spikes from high-frequency row energy in luma/chroma means."""
    ycc = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    y = ycc[:, :, 0].astype(np.float32)
    cr = ycc[:, :, 1].astype(np.float32)
    cb = ycc[:, :, 2].astype(np.float32)

    row_y = np.mean(y, axis=1)
    row_cr = np.mean(cr, axis=1)
    row_cb = np.mean(cb, axis=1)

    dy = np.abs(np.diff(row_y, prepend=row_y[0]))
    dcr = np.abs(np.diff(row_cr, prepend=row_cr[0]))
    dcb = np.abs(np.diff(row_cb, prepend=row_cb[0]))

    score = dy + (0.9 * dcr) + (0.9 * dcb)
    score = cv2.GaussianBlur(score.reshape(-1, 1), (1, 7), 0).reshape(-1)

    med = float(np.median(score))
    mad = float(np.median(np.abs(score - med)))
    mad = max(1e-6, mad)
    z = 0.6745 * (score - med) / mad
    mask = z > float(max(0.5, z_thresh))

    ex = int(max(0, row_expand))
    if ex > 0:
        k = np.ones((2 * ex + 1,), dtype=np.uint8)
        mask = np.convolve(mask.astype(np.uint8), k, mode="same") > 0

    return mask


def _selective_row_inpaint(frame: np.ndarray, bad_rows: np.ndarray) -> np.ndarray:
    """Replace only bad rows by averaging nearest good neighbors above/below."""
    out = frame.copy().astype(np.float32)
    src = frame.astype(np.float32)
    h = src.shape[0]

    bad_idx = np.where(bad_rows)[0]
    if bad_idx.size == 0:
        return frame

    for r in bad_idx:
        up = max(0, r - 1)
        down = min(h - 1, r + 1)

        while up > 0 and bad_rows[up]:
            up -= 1
        while down < h - 1 and bad_rows[down]:
            down += 1

        if up == down:
            out[r] = src[r]
        else:
            out[r] = 0.5 * (src[up] + src[down])

    return np.clip(out, 0, 255).astype(np.uint8)


class RowTemporalStabilizer:
    """Temporal blend only on detected bad rows to reduce flickering streaks."""

    def __init__(self):
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        self._prev = None

    def apply(self, frame: np.ndarray, bad_rows: np.ndarray, alpha: float) -> np.ndarray:
        a = float(np.clip(alpha, 0.10, 0.90))
        f = frame.astype(np.float32)
        if self._prev is None or self._prev.shape != frame.shape:
            self._prev = f.copy()
            return frame

        out = f.copy()
        rows = np.where(bad_rows)[0]
        if rows.size > 0:
            out[rows] = ((1.0 - a) * out[rows]) + (a * self._prev[rows])

        self._prev = out.copy()
        return np.clip(out, 0, 255).astype(np.uint8)


def _overlay_bad_rows(frame: np.ndarray, bad_rows: np.ndarray) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]
    rows = np.where(bad_rows)[0]
    if rows.size == 0:
        return out

    tint = out.copy()
    for r in rows:
        cv2.line(tint, (0, int(r)), (w - 1, int(r)), (0, 0, 255), 1)
    cv2.addWeighted(tint, 0.16, out, 0.84, 0.0, dst=out)
    return out


class LearnedNoiseCleaner:
    """Live cleaner tuned from captured black-frame noise profiles."""

    def __init__(self, profile_dir: Path):
        self.profile_dir = profile_dir
        self.profile_count = 0
        self.learned_rows: np.ndarray | None = None
        self.learned_cols: np.ndarray | None = None
        self.learned_period_px = 0.0
        self._prev_clean: np.ndarray | None = None
        self.reload_profiles()

    @staticmethod
    def _robust_threshold(v: np.ndarray, floor: float) -> float:
        med = float(np.median(v))
        mad = float(np.median(np.abs(v - med)))
        return max(floor, med + (4.5 * max(1e-6, mad)))

    @staticmethod
    def _nearest_good_index(mask: np.ndarray, idx: int) -> tuple[int, int]:
        n = mask.shape[0]
        l = idx
        r = idx
        while l > 0 and mask[l]:
            l -= 1
        while r < n - 1 and mask[r]:
            r += 1
        return l, r

    def reset_state(self) -> None:
        self._prev_clean = None

    def reload_profiles(self) -> None:
        self.profile_count = 0
        self.learned_rows = None
        self.learned_cols = None
        self.learned_period_px = 0.0

        if not self.profile_dir.exists():
            return

        files = sorted(self.profile_dir.glob("noise_profile_*.json"), key=lambda p: p.stat().st_mtime)
        if not files:
            return

        use_files = files[-2:] if len(files) >= 2 else files[-1:]
        rows = []
        cols = []
        periods = []
        for p in use_files:
            try:
                data = json.loads(p.read_text(encoding="utf-8"))
            except Exception:
                continue
            rr = data.get("row_non_black_ratio")
            cc = data.get("col_non_black_ratio")
            if isinstance(rr, list) and isinstance(cc, list) and rr and cc:
                rows.append(np.array(rr, dtype=np.float32))
                cols.append(np.array(cc, dtype=np.float32))
                periods.append(float(data.get("dominant_row_period_px", 0.0) or 0.0))

        if not rows or not cols:
            return

        row_mean = np.mean(np.stack(rows, axis=0), axis=0)
        col_mean = np.mean(np.stack(cols, axis=0), axis=0)

        thr_row = self._robust_threshold(row_mean, floor=0.008)
        thr_col = self._robust_threshold(col_mean, floor=0.010)

        row_mask = row_mean > thr_row
        col_mask = col_mean > thr_col

        # Small dilation to cover full stripe width.
        if row_mask.any():
            row_mask = np.convolve(row_mask.astype(np.uint8), np.ones((5,), dtype=np.uint8), mode="same") > 0
        if col_mask.any():
            col_mask = np.convolve(col_mask.astype(np.uint8), np.ones((3,), dtype=np.uint8), mode="same") > 0

        self.profile_count = len(use_files)
        self.learned_rows = row_mask
        self.learned_cols = col_mask
        if periods:
            self.learned_period_px = float(np.mean(periods))

    def _resize_mask(self, mask: np.ndarray | None, length: int) -> np.ndarray:
        if mask is None or mask.size == 0:
            return np.zeros((length,), dtype=bool)
        if mask.shape[0] == length:
            return mask.astype(bool)
        x_old = np.linspace(0.0, 1.0, mask.shape[0], dtype=np.float32)
        x_new = np.linspace(0.0, 1.0, length, dtype=np.float32)
        y_new = np.interp(x_new, x_old, mask.astype(np.float32))
        return y_new > 0.5

    def apply(self, frame: np.ndarray, enabled: bool = True) -> tuple[np.ndarray, dict]:
        h, w = frame.shape[:2]
        if not enabled:
            return frame, {"rows": 0, "cols": 0, "profiles": self.profile_count}

        dynamic_rows = _detect_bad_rows(frame, z_thresh=2.7, row_expand=1)
        learned_rows = self._resize_mask(self.learned_rows, h)
        learned_cols = self._resize_mask(self.learned_cols, w)

        bad_rows = dynamic_rows | learned_rows
        out = _selective_row_inpaint(frame, bad_rows)

        # Suppress persistent noisy columns (often seen at right edge).
        if np.any(learned_cols):
            out_f = out.astype(np.float32)
            cols = np.where(learned_cols)[0]
            for c in cols:
                l, r = self._nearest_good_index(learned_cols, int(c))
                if l == r:
                    continue
                out_f[:, c, :] = 0.5 * (out_f[:, l, :] + out_f[:, r, :])
            out = np.clip(out_f, 0, 255).astype(np.uint8)

        # Chroma-only temporal stabilization on noisy rows/cols.
        ycc = cv2.cvtColor(out, cv2.COLOR_BGR2YCrCb).astype(np.float32)
        if self._prev_clean is not None and self._prev_clean.shape == out.shape:
            prev_ycc = cv2.cvtColor(self._prev_clean, cv2.COLOR_BGR2YCrCb).astype(np.float32)
            alpha = 0.42
            rows = np.where(bad_rows)[0]
            if rows.size > 0:
                ycc[rows, :, 1] = ((1.0 - alpha) * ycc[rows, :, 1]) + (alpha * prev_ycc[rows, :, 1])
                ycc[rows, :, 2] = ((1.0 - alpha) * ycc[rows, :, 2]) + (alpha * prev_ycc[rows, :, 2])
            cols = np.where(learned_cols)[0]
            if cols.size > 0:
                ycc[:, cols, 1] = ((1.0 - alpha) * ycc[:, cols, 1]) + (alpha * prev_ycc[:, cols, 1])
                ycc[:, cols, 2] = ((1.0 - alpha) * ycc[:, cols, 2]) + (alpha * prev_ycc[:, cols, 2])

        out = cv2.cvtColor(np.clip(ycc, 0, 255).astype(np.uint8), cv2.COLOR_YCrCb2BGR)
        self._prev_clean = out.copy()

        return out, {
            "rows": int(np.count_nonzero(bad_rows)),
            "cols": int(np.count_nonzero(learned_cols)),
            "profiles": self.profile_count,
        }


class LiveFilterApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("E58 Noise Profiler + Cleaner")
        self.root.geometry("700x300")

        self.stream: WifiCamStream | None = None
        self._running = False
        self._thread: threading.Thread | None = None

        profile_dir = Path(__file__).resolve().parent / "captures" / "noise_profiles"
        self.cleaner = LearnedNoiseCleaner(profile_dir=profile_dir)
        self.clean_enabled = True

        # Black-frame noise profiling state.
        self.profile_active = False
        self.profile_frames: list[np.ndarray] = []
        self.profile_target_frames = 160
        self.profile_black_thresh = 12

        self._build_ui()

    def _build_ui(self) -> None:
        top = tk.Frame(self.root)
        top.pack(fill="x", padx=10, pady=(12, 6))

        tk.Label(top, text="Drone IP:").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        tk.Entry(top, textvariable=self.ip_var, width=18).pack(side="left", padx=6)

        self.play_btn = tk.Button(top, text="PLAY", bg="#1b5e20", fg="white", command=self.start)
        self.play_btn.pack(side="left", padx=4)

        self.stop_btn = tk.Button(top, text="STOP", bg="#9e9e9e", fg="white", state="disabled", command=self.stop)
        self.stop_btn.pack(side="left", padx=4)

        self.profile_btn = tk.Button(top, text="CAPTURE NOISE PROFILE", bg="#0d47a1", fg="white",
                         state="disabled", command=self.start_noise_profile_capture)
        self.profile_btn.pack(side="left", padx=6)

        mode_frame = tk.LabelFrame(self.root, text="Cleaner")
        mode_frame.pack(fill="x", padx=10, pady=6)

        self.clean_var = tk.BooleanVar(value=True)
        tk.Checkbutton(mode_frame, text="Enable profile-driven cleaner",
                  variable=self.clean_var, command=self._on_clean_toggle).pack(anchor="w", padx=8, pady=2)
        tk.Button(mode_frame, text="Reload Profiles", command=self._reload_profiles).pack(anchor="w", padx=8, pady=2)

        ctl = tk.Frame(self.root)
        ctl.pack(fill="x", padx=10, pady=6)

        tk.Label(ctl, text="Active profiles").grid(row=0, column=0, sticky="w")
        self.profile_count_var = tk.StringVar(value=str(self.cleaner.profile_count))
        tk.Label(ctl, textvariable=self.profile_count_var).grid(row=0, column=1, sticky="w")

        tk.Label(ctl, text="Noise black threshold").grid(row=0, column=2, sticky="w", padx=(24, 0))
        self.black_thr_scale = tk.Scale(ctl, from_=1, to=40, resolution=1,
                        orient="horizontal", length=180,
                        command=self._on_black_threshold)
        self.black_thr_scale.set(self.profile_black_thresh)
        self.black_thr_scale.grid(row=0, column=3, sticky="w")

        tk.Label(ctl, text="Noise capture frames").grid(row=1, column=2, sticky="w", padx=(24, 0))
        self.noise_frames_scale = tk.Scale(ctl, from_=40, to=400, resolution=10,
                           orient="horizontal", length=180,
                           command=self._on_noise_frames)
        self.noise_frames_scale.set(self.profile_target_frames)
        self.noise_frames_scale.grid(row=1, column=3, sticky="w")

        self.status_var = tk.StringVar(value="Idle")
        tk.Label(self.root, textvariable=self.status_var, fg="#1565c0").pack(anchor="w", padx=12, pady=(2, 8))

    def _on_clean_toggle(self) -> None:
        self.clean_enabled = bool(self.clean_var.get())

    def _reload_profiles(self) -> None:
        self.cleaner.reload_profiles()
        self.profile_count_var.set(str(self.cleaner.profile_count))
        self.status_var.set(f"Reloaded profiles: {self.cleaner.profile_count}")

    def _on_black_threshold(self, _value: str) -> None:
        self.profile_black_thresh = int(self.black_thr_scale.get())

    def _on_noise_frames(self, _value: str) -> None:
        self.profile_target_frames = int(self.noise_frames_scale.get())

    def start_noise_profile_capture(self) -> None:
        if not self._running or self.stream is None:
            self.status_var.set("Start stream first, then capture noise profile")
            return
        if self.profile_active:
            return
        self.profile_active = True
        self.profile_frames = []
        self.status_var.set(
            f"Noise capture started: cover lens fully | target={self.profile_target_frames} frames"
        )

    def _build_noise_profile(self, frames: list[np.ndarray], black_thresh: int) -> dict:
        stack = np.stack(frames, axis=0).astype(np.float32)
        gray = np.mean(stack, axis=3)
        non_black = gray > float(black_thresh)

        mean_img = np.mean(stack, axis=0)
        std_img = np.std(stack, axis=0)
        hot_map = np.mean(non_black.astype(np.float32), axis=0)

        row_ratio = np.mean(non_black.astype(np.float32), axis=(0, 2))
        col_ratio = np.mean(non_black.astype(np.float32), axis=(0, 1))

        centered = row_ratio - np.mean(row_ratio)
        spec = np.abs(np.fft.rfft(centered))
        freqs = np.fft.rfftfreq(centered.shape[0], d=1.0)
        dominant_period_px = 0.0
        dominant_strength = 0.0
        if spec.shape[0] > 4:
            idx = int(np.argmax(spec[2:])) + 2
            f = float(freqs[idx])
            dominant_strength = float(spec[idx] / (np.mean(spec[2:]) + 1e-6))
            if f > 1e-6:
                dominant_period_px = 1.0 / f

        profile = {
            "frames": int(stack.shape[0]),
            "frame_h": int(stack.shape[1]),
            "frame_w": int(stack.shape[2]),
            "black_threshold": int(black_thresh),
            "overall_non_black_ratio": float(np.mean(non_black.astype(np.float32))),
            "mean_bgr": [float(v) for v in np.mean(stack, axis=(0, 1, 2))],
            "std_bgr": [float(v) for v in np.std(stack, axis=(0, 1, 2))],
            "p95_gray": float(np.percentile(gray, 95.0)),
            "p99_gray": float(np.percentile(gray, 99.0)),
            "dominant_row_period_px": float(dominant_period_px),
            "dominant_row_strength": float(dominant_strength),
            "row_non_black_ratio": [float(v) for v in row_ratio],
            "col_non_black_ratio": [float(v) for v in col_ratio],
        }

        return {
            "profile": profile,
            "mean_img": np.clip(mean_img, 0, 255).astype(np.uint8),
            "std_img": np.clip(std_img * 4.0, 0, 255).astype(np.uint8),
            "hot_map": np.clip(hot_map * 255.0, 0, 255).astype(np.uint8),
        }

    def _save_noise_profile(self, result: dict) -> Path:
        out_dir = Path(__file__).resolve().parent / "captures" / "noise_profiles"
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")

        json_path = out_dir / f"noise_profile_{ts}.json"
        mean_path = out_dir / f"noise_mean_{ts}.png"
        std_path = out_dir / f"noise_std_x4_{ts}.png"
        hot_path = out_dir / f"noise_hotmap_{ts}.png"

        heat = cv2.applyColorMap(result["hot_map"], cv2.COLORMAP_JET)

        json_path.write_text(json.dumps(result["profile"], indent=2), encoding="utf-8")
        cv2.imwrite(str(mean_path), result["mean_img"])
        cv2.imwrite(str(std_path), result["std_img"])
        cv2.imwrite(str(hot_path), heat)

        return json_path

    def start(self) -> None:
        if self._running:
            return

        ip = self.ip_var.get().strip() or DEFAULT_IP
        self.stream = WifiCamStream(drone_ip=ip)
        try:
            self.stream.start()
        except OSError as e:
            self.status_var.set(f"Connect failed: {e}")
            self.stream = None
            return

        self.cleaner.reset_state()
        self.profile_active = False
        self.profile_frames = []
        self._running = True
        self.play_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal", bg="#b71c1c")
        self.profile_btn.configure(state="normal")
        self.status_var.set(
            f"Streaming {ip} | cleaner={'ON' if self.clean_enabled else 'OFF'} "
            f"profiles={self.cleaner.profile_count}"
        )

        self._thread = threading.Thread(target=self._loop, daemon=True, name="LiveFilter")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.2)
        self._thread = None

        if self.stream is not None:
            self.stream.stop()
        self.stream = None

        try:
            cv2.destroyWindow("E58 Live Video")
        except cv2.error:
            pass

        self.play_btn.configure(state="normal")
        self.stop_btn.configure(state="disabled", bg="#9e9e9e")
        self.profile_btn.configure(state="disabled")
        self.status_var.set("Stopped")

    def _apply_filter(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        out, stats = self.cleaner.apply(frame, enabled=self.clean_enabled)
        return out, stats

    def _loop(self) -> None:
        fps = 0.0
        t_prev = time.time()

        while self._running and self.stream is not None:
            frame = self.stream.poll(timeout_s=0.02)
            if frame is None:
                blank = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for E58 video...", (185, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2, cv2.LINE_AA)
                cv2.imshow("E58 Live Video", blank)
                cv2.waitKey(1)
                continue

            out, clean_stats = self._apply_filter(frame)

            if self.profile_active:
                self.profile_frames.append(frame.copy())
                got = len(self.profile_frames)
                need = self.profile_target_frames
                self.status_var.set(
                    f"Capturing noise profile (cover lens): {got}/{need} frames"
                )
                if got >= need:
                    self.profile_active = False
                    result = self._build_noise_profile(self.profile_frames, self.profile_black_thresh)
                    self.profile_frames = []
                    json_path = self._save_noise_profile(result)
                    self.cleaner.reload_profiles()
                    self.profile_count_var.set(str(self.cleaner.profile_count))
                    self.status_var.set(
                        f"Noise profile saved: {json_path.name} | non_black="
                        f"{result['profile']['overall_non_black_ratio']:.4f}"
                    )

            now = time.time()
            dt = max(1e-6, now - t_prev)
            t_prev = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            label = (
                f"mode={'clean' if self.clean_enabled else 'raw'} fps={fps:4.1f} "
                f"rows={clean_stats['rows']} cols={clean_stats['cols']} p={clean_stats['profiles']}"
            )

            cv2.putText(out, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (15, 15, 15), 2, cv2.LINE_AA)
            cv2.putText(out, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (230, 255, 230), 1, cv2.LINE_AA)

            cv2.imshow("E58 Live Video", out)
            key = cv2.waitKey(1)
            if key == ord("q"):
                self._running = False
                break

            if not self.profile_active:
                self.status_var.set(
                    f"Streaming | cleaner={'ON' if self.clean_enabled else 'OFF'} "
                    f"rows={clean_stats['rows']} cols={clean_stats['cols']} profiles={clean_stats['profiles']}"
                )

        self.root.after(0, self.stop)

    def on_close(self) -> None:
        self.stop()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = LiveFilterApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
