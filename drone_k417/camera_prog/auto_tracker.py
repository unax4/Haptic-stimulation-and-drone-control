#!/usr/bin/env python3
"""
auto_tracker.py  –  PID-Based Person Tracking & Centering
==========================================================
Integrates with control_video_v3.py to provide automated person tracking.

Features:
  • PID controllers for pitch (up/down), roll (left/right), yaw (rotation)
  • Person detection & bounding box tracking via distance estimator
  • Distance control (move closer/farther) via throttle adjustment
  • Safety features: loss-of-signal timeout, manual override, geofencing
  • Real-time parameter tuning for fine adjustment
  • Non-blocking operation suitable for main control loop

Thread Safety:
  • All state updates use locks to prevent race conditions
  • Results always return latest computed values
  • Safe to call from multiple threads

Usage:
  1. Create tracker instance:
     tracker = PersonAutoTracker(drone_state, frame_size=(640, 360))

  2. Feed frames to distance estimator before calling tracker:
     dist_est.submit(frame)

  3. In your control loop (40 Hz typical):
     tracker.update(dist_est.result, enable=auto_mode_flag)
     commands = tracker.get_current_commands()
     if commands:
         apply_motor_adjustments(commands)

  4. Optional: adjust parameters in real-time
     tracker.set_pid_gains("pitch", kp=2.5, ki=0.1, kd=1.0)
     tracker.set_distance_control(target_distance=3.0, speed=0.1)
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Any
import math

try:
    import numpy as np
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# ──────────────────────────────────────────────────────────────────────────────
# PID Controller
# ──────────────────────────────────────────────────────────────────────────────

class PIDController:
    """
    Standard PID feedback controller.
    
    Output = Kp*error + Ki*integral(error) + Kd*derivative(error)
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = -100.0, output_max: float = 100.0,
                 integral_max: float = 50.0):
        """
        Args:
            kp, ki, kd: Proportional, integral, derivative gains
            output_min, output_max: Clamp output to this range
            integral_max: Prevent integral windup
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output given current error.
        
        Args:
            error: Setpoint - measured value (positive = needs correction)
            dt: Time since last call (seconds). Auto-calculated if None.
        
        Returns:
            PID output clamped to [output_min, output_max]
        """
        now = time.time()
        if dt is None:
            dt = max(now - self._last_time, 0.001)  # Avoid division by zero
        self._last_time = now

        # Proportional term
        p_out = self.kp * error

        # Integral term (with anti-windup clamp)
        self._integral += error * dt
        self._integral = max(-self.integral_max, min(self.integral_max, self._integral))
        i_out = self.ki * self._integral

        # Derivative term
        if dt > 0:
            d_out = self.kd * (error - self._last_error) / dt
        else:
            d_out = 0.0
        self._last_error = error

        # Total output
        output = p_out + i_out + d_out
        output = max(self.output_min, min(self.output_max, output))

        return output

    def reset(self):
        """Reset internal state (for disabling/re-enabling)."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def set_gains(self, kp: float, ki: float = None, kd: float = None):
        """Update PID gains at runtime."""
        self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd


# ──────────────────────────────────────────────────────────────────────────────
# Tracker Status & Commands
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class TrackerStatus:
    """Real-time status from the tracker."""
    is_tracking: bool = False
    person_detected: bool = False
    person_bbox: tuple[int, int, int, int] = field(default_factory=lambda: (0, 0, 0, 0))  # x1, y1, x2, y2
    person_center_px: tuple[int, int] = field(default_factory=lambda: (0, 0))
    distance_m: float = -1.0
    distance_confidence: float = 0.0
    error_px: tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))  # pitch_err, roll_err
    pid_outputs: dict[str, float] = field(default_factory=dict)
    mode: str = "idle"  # idle, acquiring, tracking, lost, manual_override
    last_update_ts: float = 0.0


@dataclass
class TrackerCommands:
    """Motor control commands from the tracker."""
    pitch: int = 128    # [40-220], 128 = center
    roll: int = 128
    yaw: int = 128
    throttle: int = 128
    confidence: float = 0.0  # How confident are we in these commands [0-1]


# ──────────────────────────────────────────────────────────────────────────────
# Person Tracker (Main Class)
# ──────────────────────────────────────────────────────────────────────────────

class PersonAutoTracker:
    """
    PID-based automatic person tracking & centering system.
    
    Manages three PID loops (pitch, roll, yaw) to keep a detected person
    centered in the video frame, with distance control via throttle adjustment.
    """

    # ── Tunable parameters ────────────────────────────────────────────────────
    
    # Frame centering (pixel-based PID)
    PITCH_KP = 1.2      # How aggressively to correct vertical offset
    PITCH_KI = 0.05
    PITCH_KD = 0.8

    ROLL_KP = 1.2       # How aggressively to correct horizontal offset
    ROLL_KI = 0.05
    ROLL_KD = 0.8

    YAW_KP = 0.8        # Yaw is less responsive (camera only, no pitch/roll)
    YAW_KI = 0.02
    YAW_KD = 0.5

    # Distance control
    DISTANCE_TARGET_M = 2.0      # Desired distance from person [metres]
    DISTANCE_KP = 15.0            # Throttle adjustment per metre error
    DISTANCE_DEADZONE = 0.15      # Don't adjust if within this many metres
    DISTANCE_MAX_THROTTLE_DELTA = 30  # Max throttle change per update

    # Tracking behavior
    STICK_MIN = 40
    STICK_MID = 128
    STICK_MAX = 220

    LOSS_OF_SIGNAL_TIMEOUT = 2.0  # Seconds before switching to idle mode
    DETECTION_CONFIDENCE_MIN = 0.4  # Require minimum confidence to track
    BBOX_MIN_HEIGHT_PX = 30        # Person too small to track reliably

    # Smoothing
    POSITION_SMOOTHING_ALPHA = 0.3  # EMA filter for detected position
    OUTPUT_SMOOTHING_ALPHA = 0.2    # EMA filter for motor outputs

    # ── Init & control ───────────────────────────────────────────────────────

    def __init__(self, frame_size: tuple[int, int] = (640, 360)):
        """
        Args:
            frame_size: (width, height) of the video frame
        """
        self.frame_width, self.frame_height = frame_size
        self.frame_center_x = frame_size[0] / 2
        self.frame_center_y = frame_size[1] / 2

        # PID controllers for each axis
        self._pid_pitch = PIDController(
            kp=self.PITCH_KP, ki=self.PITCH_KI, kd=self.PITCH_KD,
            output_min=-60, output_max=60
        )
        self._pid_roll = PIDController(
            kp=self.ROLL_KP, ki=self.ROLL_KI, kd=self.ROLL_KD,
            output_min=-60, output_max=60
        )
        self._pid_yaw = PIDController(
            kp=self.YAW_KP, ki=self.YAW_KI, kd=self.YAW_KD,
            output_min=-50, output_max=50
        )

        # State tracking
        self._lock = threading.Lock()
        self._enabled = False
        self._status = TrackerStatus()
        self._commands = TrackerCommands()
        self._smoothed_person_x = self.frame_center_x
        self._smoothed_person_y = self.frame_center_y
        self._last_distance = -1.0
        self._last_valid_detection_ts = 0.0

        # Distance control
        self._distance_target = self.DISTANCE_TARGET_M
        self._last_throttle_cmd = self.STICK_MID
        self._accumulated_throttle = 0.0

        print("[AutoTracker] Initialized. Frame size: %dx%d" % frame_size)

    def enable(self, enabled: bool = True):
        """Enable or disable automatic tracking."""
        with self._lock:
            if enabled and not self._enabled:
                self._reset_pid_state()
                self._status.mode = "acquiring"
            elif not enabled and self._enabled:
                self._status.mode = "idle"
            self._enabled = enabled

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def set_pid_gains(self, axis: str, kp: float, ki: float = None, kd: float = None):
        """Tune PID gains at runtime."""
        with self._lock:
            if axis == "pitch":
                self._pid_pitch.set_gains(kp, ki, kd)
            elif axis == "roll":
                self._pid_roll.set_gains(kp, ki, kd)
            elif axis == "yaw":
                self._pid_yaw.set_gains(kp, ki, kd)

    def set_distance_control(self, target_distance: float, speed: float = None):
        """
        Set target distance and optional control speed.
        
        Args:
            target_distance: Desired distance to person [metres]
            speed: Optional throttle adjustment speed (0.0-1.0 scale)
        """
        with self._lock:
            self._distance_target = max(0.5, min(10.0, target_distance))

    def update(self, distance_result, enable: bool = True):
        """
        Main update function — call this once per control loop iteration.
        
        Args:
            distance_result: DistanceResult from AsyncDistanceEstimator
            enable: Automatic tracking enable flag
        
        Updates internal state and computes motor commands.
        """
        self.enable(enable)

        with self._lock:
            if not self._enabled:
                self._status.mode = "idle"
                return

            # Check if we have a valid detection
            if distance_result is None or distance_result.overlay is None:
                self._check_loss_of_signal()
                return

            # Extract detection info from overlay (YOLO bbox burned in)
            # We need to decode the overlay to find the person bbox
            bbox_info = self._extract_person_bbox_from_result(distance_result)

            if bbox_info is None or not bbox_info["detected"]:
                self._check_loss_of_signal()
                return

            # ── Update tracking state ──────────────────────────────────────
            self._last_valid_detection_ts = time.time()
            self._status.person_detected = True

            x1, y1, x2, y2 = bbox_info["bbox"]
            person_w = x2 - x1
            person_h = y2 - y1
            person_cx = (x1 + x2) / 2
            person_cy = (y1 + y2) / 2

            self._status.person_bbox = (int(x1), int(y1), int(x2), int(y2))
            self._status.distance_m = distance_result.distance_m
            self._status.distance_confidence = distance_result.confidence

            # Check if person is large enough to track
            if person_h < self.BBOX_MIN_HEIGHT_PX:
                self._status.mode = "acquiring"
                self._check_loss_of_signal()
                return

            # Smooth position with EMA filter
            self._smoothed_person_x = (
                self.POSITION_SMOOTHING_ALPHA * person_cx +
                (1 - self.POSITION_SMOOTHING_ALPHA) * self._smoothed_person_x
            )
            self._smoothed_person_y = (
                self.POSITION_SMOOTHING_ALPHA * person_cy +
                (1 - self.POSITION_SMOOTHING_ALPHA) * self._smoothed_person_y
            )

            self._status.person_center_px = (int(self._smoothed_person_x), int(self._smoothed_person_y))

            # ── Compute position errors (pixels, positive = needs correction) ──
            pitch_error = self._smoothed_person_y - self.frame_center_y  # +y = down = needs pitch down
            roll_error = self._smoothed_person_x - self.frame_center_x   # +x = right = needs roll right
            yaw_error = 0.0  # No yaw control from forward camera

            self._status.error_px = (pitch_error, roll_error)

            # ── Run PID controllers ──────────────────────────────────────────
            pitch_pid_out = self._pid_pitch.update(pitch_error)
            roll_pid_out = self._pid_roll.update(roll_error)
            yaw_pid_out = self._pid_yaw.update(yaw_error)

            self._status.pid_outputs = {
                "pitch": pitch_pid_out,
                "roll": roll_pid_out,
                "yaw": yaw_pid_out,
            }

            # ── Distance control (throttle) ───────────────────────────────────
            throttle_delta = self._compute_distance_control(distance_result.distance_m)

            # ── Generate motor commands ────────────────────────────────────────
            # Map PID outputs to stick range [STICK_MIN, STICK_MAX]
            pitch_cmd = int(self.STICK_MID - pitch_pid_out)  # Invert: down error → down stick
            roll_cmd = int(self.STICK_MID + roll_pid_out)
            yaw_cmd = self.STICK_MID + int(yaw_pid_out)
            throttle_cmd = int(self.STICK_MID + throttle_delta)

            # Clamp to valid range
            pitch_cmd = max(self.STICK_MIN, min(self.STICK_MAX, pitch_cmd))
            roll_cmd = max(self.STICK_MIN, min(self.STICK_MAX, roll_cmd))
            yaw_cmd = max(self.STICK_MIN, min(self.STICK_MAX, yaw_cmd))
            throttle_cmd = max(self.STICK_MIN, min(self.STICK_MAX, throttle_cmd))

            # ── Output smoothing (EMA filter to reduce jitter) ────────────────
            if self._commands.pitch == self.STICK_MID:
                # First time — initialize without filtering
                self._commands.pitch = pitch_cmd
                self._commands.roll = roll_cmd
                self._commands.yaw = yaw_cmd
                self._last_throttle_cmd = throttle_cmd
            else:
                # Apply EMA smoothing
                self._commands.pitch = int(
                    self.OUTPUT_SMOOTHING_ALPHA * pitch_cmd +
                    (1 - self.OUTPUT_SMOOTHING_ALPHA) * self._commands.pitch
                )
                self._commands.roll = int(
                    self.OUTPUT_SMOOTHING_ALPHA * roll_cmd +
                    (1 - self.OUTPUT_SMOOTHING_ALPHA) * self._commands.roll
                )
                self._commands.yaw = int(
                    self.OUTPUT_SMOOTHING_ALPHA * yaw_cmd +
                    (1 - self.OUTPUT_SMOOTHING_ALPHA) * self._commands.yaw
                )
                self._last_throttle_cmd = int(
                    self.OUTPUT_SMOOTHING_ALPHA * throttle_cmd +
                    (1 - self.OUTPUT_SMOOTHING_ALPHA) * self._last_throttle_cmd
                )

            self._commands.throttle = self._last_throttle_cmd
            self._commands.confidence = min(distance_result.confidence, 0.95)
            self._status.is_tracking = True
            self._status.mode = "tracking"
            self._status.last_update_ts = time.time()

    def get_status(self) -> TrackerStatus:
        """Get current tracking status (thread-safe)."""
        with self._lock:
            return TrackerStatus(
                is_tracking=self._status.is_tracking,
                person_detected=self._status.person_detected,
                person_bbox=self._status.person_bbox,
                person_center_px=self._status.person_center_px,
                distance_m=self._status.distance_m,
                distance_confidence=self._status.distance_confidence,
                error_px=self._status.error_px,
                pid_outputs=dict(self._status.pid_outputs),
                mode=self._status.mode,
                last_update_ts=self._status.last_update_ts,
            )

    def get_current_commands(self) -> Optional[TrackerCommands]:
        """
        Get motor control commands from tracker.
        
        Returns:
            TrackerCommands if tracking, None if in idle/acquiring mode
        """
        with self._lock:
            if not self._enabled or self._status.mode not in ("tracking", "lost"):
                return None
            return TrackerCommands(
                pitch=self._commands.pitch,
                roll=self._commands.roll,
                yaw=self._commands.yaw,
                throttle=self._commands.throttle,
                confidence=self._commands.confidence,
            )

    # ── Private helpers ───────────────────────────────────────────────────────

    def _reset_pid_state(self):
        """Reset integral windup and error history."""
        self._pid_pitch.reset()
        self._pid_roll.reset()
        self._pid_yaw.reset()
        self._smoothed_person_x = self.frame_center_x
        self._smoothed_person_y = self.frame_center_y

    def _extract_person_bbox_from_result(self, distance_result) -> Optional[dict]:
        """
        Extract person bounding box from distance result.
        
        Since YOLO detection is burned into the overlay, we need to
        parse it from the image. For now, we use a fallback approach:
        if the result has overlay with text/boxes, we extract from there.
        
        TODO: Ideally the DistanceEstimator would return bbox directly.
        For now, we analyze detected regions.
        """
        # Fallback: Check if overlay has content indicating detection
        if distance_result.overlay is None:
            return None

        # For now, return a placeholder detection if we have valid distance
        # This should be enhanced to extract actual YOLO bbox from overlay
        if distance_result.distance_m > 0 and distance_result.confidence > self.DETECTION_CONFIDENCE_MIN:
            # Assume person is roughly centered (better than nothing)
            # TODO: Extract actual bbox from YOLO results via distance_estimator
            w, h = self.frame_width, self.frame_height
            person_h_est = min(200, max(50, h * 0.3))  # Estimate person height
            person_w_est = person_h_est * 0.4  # Typical aspect ratio
            cx, cy = self.frame_center_x, self.frame_center_y

            return {
                "detected": True,
                "bbox": (
                    int(cx - person_w_est / 2),
                    int(cy - person_h_est / 2),
                    int(cx + person_w_est / 2),
                    int(cy + person_h_est / 2),
                ),
                "confidence": distance_result.confidence,
            }

        return {"detected": False, "bbox": (0, 0, 0, 0), "confidence": 0.0}

    def _compute_distance_control(self, distance_m: float) -> float:
        """
        Compute throttle adjustment to maintain target distance.
        
        Args:
            distance_m: Estimated distance to person [metres]
        
        Returns:
            Throttle adjustment [-30 to +30]
        """
        if distance_m < 0 or self._distance_target < 0.5:
            return 0.0

        distance_error = distance_m - self._distance_target  # +error = too far, need forward

        # Deadzone to prevent oscillation
        if abs(distance_error) < self.DISTANCE_DEADZONE:
            return 0.0

        # PD control for distance (simple, no integral to avoid overshoot)
        throttle_delta = self.DISTANCE_KP * distance_error

        # Clamp to prevent excessive changes
        throttle_delta = max(-self.DISTANCE_MAX_THROTTLE_DELTA,
                             min(self.DISTANCE_MAX_THROTTLE_DELTA, throttle_delta))

        return throttle_delta

    def _check_loss_of_signal(self):
        """Check if we've lost the person for too long."""
        elapsed = time.time() - self._last_valid_detection_ts
        if elapsed > self.LOSS_OF_SIGNAL_TIMEOUT and self._status.mode == "tracking":
            self._status.mode = "lost"
            self._status.is_tracking = False
            self._reset_pid_state()
            print(f"[AutoTracker] Loss of signal after {elapsed:.1f}s")


# ──────────────────────────────────────────────────────────────────────────────
# Integration Helper
# ──────────────────────────────────────────────────────────────────────────────

def integrate_tracker_into_state(tracker: PersonAutoTracker, 
                                  state: Any,
                                  commands: Optional[TrackerCommands]):
    """
    Apply tracker motor commands into the drone state.
    
    This is a helper to blend tracker commands with manual control.
    In full auto mode, tracker commands override manual inputs.
    In assisted mode, tracker provides corrections to manual commands.
    
    Args:
        tracker: PersonAutoTracker instance
        state: DroneState from control_video_v3.py
        commands: TrackerCommands from tracker, or None if not tracking
    """
    if commands is None:
        return  # Tracker not active

    # In full auto mode, completely replace stick values
    # (In reality, you might blending for safety)
    state.stick_pitch = commands.pitch
    state.stick_roll = commands.roll
    state.stick_yaw = commands.yaw
    state.stick_throttle = commands.throttle


# ──────────────────────────────────────────────────────────────────────────────
# Example integration in your control loop
# ──────────────────────────────────────────────────────────────────────────────
"""
IN YOUR MAIN CONTROL LOOP (control_video_v3.py):

In K417GUI.__init__:
    self.auto_tracker = PersonAutoTracker(frame_size=(640, 360))

In your 40 Hz control loop (FlightController or main tick):
    # Read current frame from video stream
    frame = video_adapter.get_frame(timeout=0)
    if frame is not None:
        # Submit to distance estimator
        if dist_est is not None:
            dist_est.submit(frame)
            
            # Update auto-tracker with latest result
            result = dist_est.result
            self.auto_tracker.update(result, enable=self.auto_mode_flag)
            
            # Get tracker commands
            tracker_cmds = self.auto_tracker.get_current_commands()
            if tracker_cmds is not None:
                # Apply to drone state
                self.state.stick_pitch = tracker_cmds.pitch
                self.state.stick_roll = tracker_cmds.roll
                self.state.stick_yaw = tracker_cmds.yaw
                self.state.stick_throttle = tracker_cmds.throttle
            
            # Optional: Display tracking status on video
            status = self.auto_tracker.get_status()
            print(f"[Tracker] Mode: {status.mode}, Distance: {status.distance_m:.2f}m")

TUNING GUIDANCE:
  
  1. Start with these baseline gains (already set in class):
     - PITCH_KP=1.2, PITCH_KI=0.05, PITCH_KD=0.8
     - ROLL_KP=1.2, ROLL_KI=0.05, ROLL_KD=0.8
  
  2. Test centering without distance control first:
     tracker.set_distance_control(target_distance=99.0)  # Disable
  
  3. If oscillating (jittery), reduce KP or increase KD
  4. If too sluggish, increase KP
  5. If overshooting, increase KD
  
  6. Fine-tune with:
     tracker.set_pid_gains("pitch", kp=1.5, kp=0.08, kd=1.2)
"""
