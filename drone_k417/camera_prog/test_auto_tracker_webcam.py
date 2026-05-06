#!/usr/bin/env python3
"""
test_auto_tracker_webcam.py  –  Real-time Auto-Tracker Testing with Webcam
==========================================================================
Test the PersonAutoTracker in real-time using your webcam.

Features:
  • Live video feed with person detection overlay
  • Real-time display of motor stick values (pitch, roll, yaw, throttle)
  • Position error visualization (crosshairs)
  • Distance estimation and control
  • Live console output of all control signals
  • Interactive distance adjustment (+ / - keys)

Usage:
  python test_auto_tracker_webcam.py

Controls:
  q         - Quit
  d         - Toggle distance estimator display
  + or ]    - Increase target distance (+0.5m)
  - or [    - Decrease target distance (-0.5m)
  r         - Reset auto-tracker
  space     - Print detailed status to console

Requirements:
  pip install opencv-python numpy torch ultralytics pillow
  (or install what you have available)
"""

from __future__ import annotations

import cv2
import numpy as np
import time
import sys
from typing import Optional

# ── Import our tracking system ────────────────────────────────────────────────
try:
    from drone_k417.camera_prog.auto_tracker import PersonAutoTracker, TrackerStatus
    TRACKER_AVAILABLE = True
except ImportError:
    print("ERROR: auto_tracker.py not found!")
    TRACKER_AVAILABLE = False
    sys.exit(1)

try:
    from distance_estimator_v2 import AsyncDistanceEstimator, YOLO_AVAILABLE
    DIST_EST_AVAILABLE = True
except ImportError:
    print("WARNING: distance_estimator_v2.py not found!")
    DIST_EST_AVAILABLE = False


# ──────────────────────────────────────────────────────────────────────────────
# Test Application
# ──────────────────────────────────────────────────────────────────────────────

class AutoTrackerTester:
    """Interactive webcam-based auto-tracker testing application."""

    def __init__(self, camera_index: int = 0, show_distance_overlay: bool = True):
        """
        Args:
            camera_index: Webcam index (0 = default)
            show_distance_overlay: Whether to show distance estimator output
        """
        self.camera_index = camera_index
        self.show_distance_overlay = show_distance_overlay

        # OpenCV setup
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")

        # Get camera properties
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_size = (frame_width, frame_height)
        print(f"[Tester] Camera opened: {frame_width}×{frame_height}")

        # Auto-tracker
        self.tracker = PersonAutoTracker(frame_size=self.frame_size)
        self.tracker.enable(True)  # Always enabled for testing

        # Distance estimator (optional)
        self.dist_est = None
        if DIST_EST_AVAILABLE:
            self.dist_est = AsyncDistanceEstimator(use_yolo=YOLO_AVAILABLE, draw_overlay=True)
            self.dist_est.start()
            print("[Tester] Distance estimator starting (loading models...)")
        else:
            print("[Tester] Distance estimator not available (YOLO detection disabled)")

        # Testing state
        self.target_distance = 2.0
        self.show_dist_est = show_distance_overlay
        self.paused = False
        self.frame_count = 0
        self.fps_time = time.time()
        self.fps = 0

    def run(self):
        """Main test loop."""
        print("\n" + "=" * 80)
        print("AUTO-TRACKER WEBCAM TEST - Controls:")
        print("=" * 80)
        print("  q         - Quit")
        print("  d         - Toggle distance estimator overlay")
        print("  + or ]    - Increase target distance (+0.5m)")
        print("  - or [    - Decrease target distance (-0.5m)")
        print("  r         - Reset tracker")
        print("  space     - Print detailed status")
        print("=" * 80 + "\n")

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("[ERROR] Failed to read frame from camera")
                    break

                # Flip frame for selfie view
                frame = cv2.flip(frame, 1)

                # Update distance estimator
                if self.dist_est is not None:
                    self.dist_est.submit(frame)

                # Update auto-tracker
                if self.dist_est is not None:
                    result = self.dist_est.result
                    self.tracker.update(result, enable=True)

                # Get tracker output
                status = self.tracker.get_status()
                commands = self.tracker.get_current_commands()

                # Draw visualization
                display_frame = self._draw_visualization(frame, status, commands)

                # Display FPS
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    elapsed = time.time() - self.fps_time
                    self.fps = 30 / elapsed
                    self.fps_time = time.time()

                cv2.putText(display_frame, f"FPS: {self.fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Show frame
                cv2.imshow("Auto-Tracker Test", display_frame)

                # Print live status to console
                self._print_status(status, commands)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('d'):
                    self.show_dist_est = not self.show_dist_est
                    print(f"[Control] Distance estimator overlay: {self.show_dist_est}")
                elif key == ord('+') or key == ord(']'):
                    self.target_distance = min(10.0, self.target_distance + 0.5)
                    self.tracker.set_distance_control(target_distance=self.target_distance)
                    print(f"[Control] Target distance: {self.target_distance:.1f}m")
                elif key == ord('-') or key == ord('['):
                    self.target_distance = max(0.5, self.target_distance - 0.5)
                    self.tracker.set_distance_control(target_distance=self.target_distance)
                    print(f"[Control] Target distance: {self.target_distance:.1f}m")
                elif key == ord('r'):
                    self.tracker.enable(False)
                    self.tracker = PersonAutoTracker(frame_size=self.frame_size)
                    self.tracker.enable(True)
                    print("[Control] Tracker reset")
                elif key == 32:  # Space
                    self._print_detailed_status(status, commands)

        finally:
            self.cleanup()

    def _draw_visualization(self, frame: np.ndarray, status: TrackerStatus, commands) -> np.ndarray:
        """Draw tracking visualization on frame."""
        out = frame.copy()
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        # Frame center indicator (red dot)
        cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)
        cv2.line(out, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
        cv2.line(out, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)

        # Draw distance estimator overlay if available
        if self.show_dist_est and self.dist_est is not None and self.dist_est.ready:
            result = self.dist_est.result
            if result.overlay is not None:
                # Blend distance estimator overlay
                out = cv2.addWeighted(out, 0.6, result.overlay, 0.4, 0)

        # Draw tracking info if available
        if status.is_tracking and status.person_bbox != (0, 0, 0, 0):
            x1, y1, x2, y2 = status.person_bbox
            # Draw person bbox (green)
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw person center
            pcx, pcy = status.person_center_px
            cv2.circle(out, (pcx, pcy), 6, (0, 255, 0), -1)

            # Draw error vector (line from frame center to person center)
            cv2.arrowedLine(out, (cx, cy), (pcx, pcy), (0, 255, 255), 2, tipLength=0.2)

            # Error text
            error_px = status.error_px
            error_text = f"Error: ({error_px[0]:+.0f}, {error_px[1]:+.0f}) px"
            cv2.putText(out, error_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (0, 255, 255), 1, cv2.LINE_AA)

        # Tracker status
        mode_colors = {
            "idle": (200, 200, 200),
            "acquiring": (0, 165, 255),
            "tracking": (0, 255, 0),
            "lost": (0, 0, 255),
        }
        mode_color = mode_colors.get(status.mode, (200, 200, 200))
        mode_text = f"Mode: {status.mode.upper()}"
        cv2.putText(out, mode_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                   0.6, mode_color, 2)

        # Distance info
        if status.distance_m > 0:
            dist_text = f"Distance: {status.distance_m:.2f}m (conf: {status.distance_confidence:.2f})"
            cv2.putText(out, dist_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (255, 200, 0), 1, cv2.LINE_AA)

        # Motor stick values (if tracking)
        if commands is not None:
            self._draw_stick_panel(out, commands, status.distance_m)

        # Target distance
        target_text = f"Target distance: {self.target_distance:.1f}m (press +/- to adjust)"
        cv2.putText(out, target_text, (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX,
                   0.5, (200, 200, 200), 1, cv2.LINE_AA)

        # Instructions
        instr_text = "Press D: toggle dist, SPACE: details, Q: quit"
        cv2.putText(out, instr_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                   0.4, (150, 150, 150), 1, cv2.LINE_AA)

        return out

    def _draw_stick_panel(self, frame: np.ndarray, commands, distance_m: float):
        """Draw motor stick command visualization."""
        h, w = frame.shape[:2]
        panel_x = w - 250
        panel_y = 50
        panel_w = 240
        panel_h = 200

        # Background panel
        cv2.rectangle(frame, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h),
                     (40, 40, 40), -1)
        cv2.rectangle(frame, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h),
                     (200, 200, 200), 2)

        # Title
        cv2.putText(frame, "STICK VALUES", (panel_x + 10, panel_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 200), 2)

        # Stick layout:
        # THROTTLE (left)   |   PITCH (right)
        # YAW (left)        |   ROLL (right)

        stick_size = 40
        stick_x1 = panel_x + 30
        stick_y1 = panel_y + 60
        stick_x2 = panel_x + 150
        stick_y2 = panel_y + 60

        # Throttle (left)
        self._draw_stick(frame, "THR", commands.throttle, stick_x1, stick_y1, stick_size)

        # Pitch (right)
        self._draw_stick(frame, "PITCH", commands.pitch, stick_x2, stick_y1, stick_size)

        # Yaw (left, below)
        self._draw_stick(frame, "YAW", commands.yaw, stick_x1, stick_y2 + 60, stick_size)

        # Roll (right, below)
        self._draw_stick(frame, "ROLL", commands.roll, stick_x2, stick_y2 + 60, stick_size)

        # Confidence score
        conf_text = f"Conf: {commands.confidence:.2f}"
        cv2.putText(frame, conf_text, (panel_x + 10, panel_y + panel_h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)

    def _draw_stick(self, frame: np.ndarray, label: str, value: int, x: int, y: int, size: int):
        """Draw a single stick indicator."""
        # Background circle
        cv2.circle(frame, (x, y), size, (100, 100, 100), -1)
        cv2.circle(frame, (x, y), size, (200, 200, 200), 1)

        # Normalize value to [-1, 1]
        # Stick range: 40-220, center: 128
        normalized = (value - 128) / (220 - 128)
        normalized = max(-1, min(1, normalized))

        # Indicator position
        ind_x = int(x + normalized * size * 0.7)
        ind_y = y

        # Draw indicator
        cv2.circle(frame, (ind_x, ind_y), 6, (0, 255, 0), -1)
        cv2.line(frame, (x, y), (ind_x, ind_y), (0, 200, 0), 2)

        # Label and value
        cv2.putText(frame, label, (x - 20, y - size - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(frame, f"{value}", (x - 15, y + size + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 255), 1)

    def _print_status(self, status: TrackerStatus, commands):
        """Print status to console (updates every frame)."""
        # Print at top of screen every 30 frames to avoid spam
        if self.frame_count % 30 == 0:
            print("\n" + "=" * 100)
            print(f"Frame: {self.frame_count} | Mode: {status.mode:12} | Distance: {status.distance_m:6.2f}m")
            if commands:
                print(f"Motor Commands → Pitch: {commands.pitch:3d} | Roll: {commands.roll:3d} | "
                      f"Yaw: {commands.yaw:3d} | Throttle: {commands.throttle:3d}")
            if status.error_px:
                print(f"Error (pixels) → Y: {status.error_px[0]:+7.1f} | X: {status.error_px[1]:+7.1f}")

    def _print_detailed_status(self, status: TrackerStatus, commands):
        """Print detailed status when user presses space."""
        print("\n" + "=" * 100)
        print("DETAILED TRACKER STATUS")
        print("=" * 100)
        print(f"  Mode: {status.mode}")
        print(f"  Tracking: {status.is_tracking}")
        print(f"  Person Detected: {status.person_detected}")
        print(f"  Person Bbox: {status.person_bbox}")
        print(f"  Person Center (px): {status.person_center_px}")
        print(f"  Position Error (px): Y={status.error_px[0]:+.1f}, X={status.error_px[1]:+.1f}")
        print(f"  Distance: {status.distance_m:.2f}m (confidence: {status.distance_confidence:.2f})")
        print()
        if commands:
            print("MOTOR STICK COMMANDS (sent to drone):")
            print(f"  Pitch:    {commands.pitch:3d}  (40-220, center=128)")
            print(f"  Roll:     {commands.roll:3d}  {'←' if commands.roll < 128 else '→' if commands.roll > 128 else '●'}")
            print(f"  Yaw:      {commands.yaw:3d}  (rotation)")
            print(f"  Throttle: {commands.throttle:3d}  {'↓' if commands.throttle < 128 else '↑' if commands.throttle > 128 else '●'}")
            print(f"  Confidence: {commands.confidence:.2f}")
        print()
        if status.pid_outputs:
            print("PID OUTPUTS (before mapping to stick values):")
            for axis, output in status.pid_outputs.items():
                print(f"  {axis.upper():6}: {output:+8.2f}")
        print()
        print(f"Target Distance: {self.target_distance:.1f}m")
        print("=" * 100 + "\n")

    def cleanup(self):
        """Clean up resources."""
        if self.dist_est is not None:
            self.dist_est.stop()
        self.cap.release()
        cv2.destroyAllWindows()
        print("[Tester] Cleanup complete")


# ──────────────────────────────────────────────────────────────────────────────
# Main Entry Point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    """Run the auto-tracker tester."""
    try:
        tester = AutoTrackerTester(camera_index=0, show_distance_overlay=True)
        tester.run()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
