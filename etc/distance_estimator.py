"""
distance_estimator.py  –  Drone-to-object distance estimation
==============================================================
Drop-in companion to control_video_v2.py (K417 drone controller).

METHOD STACK
------------
1. MiDaS v2.1-small  — monocular relative depth map every frame (~25 ms on CPU).
   Produces an inverse-depth map (disparity) in arbitrary units.

2. Lucas-Kanade sparse optical flow  — tracks feature points between frames.
   The apparent pixel motion of a scene point at known depth provides a
   scale factor that converts disparity → real-world distance.
   At 15 fps this is stable enough for 0.3–10 m range estimates.

3. YOLOv8n (COCO)  — when a person (≈ 1.75 m tall) is detected, its
   bounding-box height anchors the depth scale, overriding the flow-based
   estimate for that frame.  Detections are gated by a temporal IoU tracker
   (must appear in 2+ consecutive passes) to suppress false positives.
   Requires: pip install ultralytics

OUTPUT
------
Returns a DistanceResult dataclass every frame:
  .distance_m   float   estimated distance to scene centre [metres]
  .confidence   float   0–1 quality score
  .depth_map    ndarray H×W float32 metric depth [metres] (optional)
  .overlay      ndarray same as input frame with OSD burned in

INTEGRATION
-----------
See the two integration snippets at the bottom of this file.

INSTALL
-------
pip install torch torchvision timm opencv-python-headless pillow
pip install ultralytics          # for optional YOLO person anchor
"""

from __future__ import annotations

from pyexpat import model
import time
import threading
import warnings
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

# ── lazy imports so the rest of the controller loads without torch ─────────
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    from ultralytics import YOLO as _YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import huggingface_hub as _hf_hub   # noqa: F401 — presence check only
    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False


# ──────────────────────────────────────────────────────────────────────────────
# Data types
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class DistanceResult:
    distance_m:   float          = -1.0   # negative = unknown
    confidence:   float          = 0.0    # 0–1
    depth_map:    Optional[np.ndarray] = field(default=None, repr=False)
    overlay:      Optional[np.ndarray] = field(default=None, repr=False)
    method:       str            = "none"
    latency_ms:   float          = 0.0


# ──────────────────────────────────────────────────────────────────────────────
# MiDaS loader
# ──────────────────────────────────────────────────────────────────────────────

class _MiDaSBackend:
    """
    Wraps MiDaS v2.1 small (best CPU/15fps tradeoff).
    Falls back to DPT_Hybrid if VRAM allows.
    Outputs: disparity map (arbitrary units, higher = closer).
    """

    MODEL_NAMES = ["MiDaS_small", "DPT_Hybrid"]   # small tried first

    def __init__(self, model_name: str = "MiDaS_small", device: str = "auto"):
        if not TORCH_AVAILABLE:
            raise RuntimeError("torch is not installed. pip install torch torchvision timm")

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.model = torch.hub.load(
                "intel-isl/MiDaS", model_name,
                pretrained=True, trust_repo=True
            )
        self.model.to(self.device).eval()

        midas_transforms = torch.hub.load(
            "intel-isl/MiDaS", "transforms", trust_repo=True
        )
        if model_name == "MiDaS_small":
            self.transform = midas_transforms.small_transform
        else:
            self.transform = midas_transforms.dpt_transform

        self._lock = threading.Lock()
        print(f"[DistEst] MiDaS '{model_name}' loaded on {device}")

    # MiDaS inference resolution — smaller = faster, still accurate enough.
    # 256 gives ~2-3× speedup over 384 with negligible quality loss for ranging.
    INFER_SIZE = 256

    @torch.no_grad()
    def predict(self, bgr_frame: np.ndarray) -> np.ndarray:
        """
        Returns float32 disparity map at INFER_SIZE resolution (NOT full frame).
        Higher value = closer to camera.
        Callers must map pixel coordinates with map_coords() before sampling.
        """
        h, w = bgr_frame.shape[:2]
        # Downscale to INFER_SIZE on the shorter axis before handing to MiDaS.
        # This alone saves ~60% of inference time vs full 640×360.
        scale  = self.INFER_SIZE / min(h, w)
        new_w  = int(round(w * scale / 32) * 32)   # keep multiple of 32
        new_h  = int(round(h * scale / 32) * 32)
        small  = cv2.resize(bgr_frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        inp = self.transform(rgb).to(self.device)
        if inp.ndim == 3:
            inp = inp.unsqueeze(0)

        disp = self.model(inp).squeeze().cpu().numpy()   # shape: (H', W')

        # Normalise to [0, 1]
        mn, mx = disp.min(), disp.max()
        if mx > mn:
            disp = (disp - mn) / (mx - mn)
        return disp.astype(np.float32)   # small map, cheap to pass around

    def map_coords(self, bgr_frame: np.ndarray, disp: np.ndarray,
                   x: int, y: int) -> tuple[int, int]:
        """Map a pixel coordinate from the full frame into the small disp map."""
        fh, fw = bgr_frame.shape[:2]
        dh, dw = disp.shape
        sx = int(np.clip(x * dw / fw, 0, dw - 1))
        sy = int(np.clip(y * dh / fh, 0, dh - 1))
        return sx, sy


# ──────────────────────────────────────────────────────────────────────────────
# Scale calibrator  (disparity → metres)
# ──────────────────────────────────────────────────────────────────────────────

class _ScaleCalibrator:
    """
    Maintains a running estimate of the scene scale factor s such that:
        depth_m  ≈  s / disparity

    Two update sources (whichever fires):
      A) Lucas-Kanade optical flow between consecutive frames.
         When the drone translates by δ pixels in the image, and we know
         the relative depth change from MiDaS, we can estimate s up to
         an initial anchor. We bootstrap from a plausible physical assumption
         (scene at 3 m on first frame).

      B) YOLO anchor: known-height object (person, 1.75 m) visible →
         s = bounding_box_height_px * 1.75 * disp_at_bbox_centre / focal_px
         focal_px derived from FOV guess (70° typical consumer drone = 640/2/tan35°≈457).
    """

    PERSON_HEIGHT_M = 1.75
    FOCAL_PX        = 457.0    # 70° HFOV @ 640px — tune if you know the real value
    BOOTSTRAP_M     = 3.0      # assumed distance for first frame
    ALPHA           = 0.05     # EMA weight for scale updates (low = smoother)
    FLOW_GRID       = (8, 8)   # grid size for LK feature seeding

    def __init__(self):
        self._scale: Optional[float] = None
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_disp: Optional[np.ndarray] = None
        self._prev_pts:  Optional[np.ndarray] = None
        self._flow_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

    # ── public ───────────────────────────────────────────────────────────────

    def update(
        self,
        gray:  np.ndarray,         # current grayscale frame
        disp:  np.ndarray,         # current disparity (0-1)
        yolo_boxes: list | None,   # list of (x1,y1,x2,y2,cls,conf) for "person"
    ) -> Optional[float]:
        """
        Returns updated scale estimate (metres / disparity_unit),
        or None if not yet calibrated.
        """

        # --- YOLO anchor (highest priority) ---
        s_yolo = self._yolo_scale(disp, yolo_boxes)
        if s_yolo is not None:
            self._update_ema(s_yolo, weight=0.2)

        # --- Optical flow anchor ---
        s_flow = self._flow_scale(gray, disp)
        if s_flow is not None:
            self._update_ema(s_flow, weight=self.ALPHA)

        # --- Bootstrap (first frame) ---
        if self._scale is None:
            centre_disp = self._centre_disp(disp)
            if centre_disp > 0:
                self._scale = self.BOOTSTRAP_M * centre_disp

        # --- Update previous frame state ---
        self._prev_gray = gray.copy()
        self._prev_disp = disp.copy()

        return self._scale

    def disp_to_depth(self, disp: np.ndarray) -> Optional[np.ndarray]:
        """Convert disparity map to metric depth map [metres]."""
        if self._scale is None:
            return None
        safe_disp = np.where(disp > 1e-5, disp, 1e-5)
        return np.clip(self._scale / safe_disp, 0.1, 50.0)

    # ── private ──────────────────────────────────────────────────────────────

    def _centre_disp(self, disp: np.ndarray, frac: float = 0.2) -> float:
        h, w = disp.shape
        cy, cx = h // 2, w // 2
        dy, dx = int(h * frac / 2), int(w * frac / 2)
        patch = disp[cy - dy: cy + dy, cx - dx: cx + dx]
        return float(np.median(patch)) if patch.size > 0 else 0.0

    def _update_ema(self, new_val: float, weight: float):
        if self._scale is None:
            self._scale = new_val
        else:
            self._scale = (1 - weight) * self._scale + weight * new_val

    def _yolo_scale(
        self, disp: np.ndarray, boxes: list | None
    ) -> Optional[float]:
        if not boxes:
            return None
        best = None
        best_area = 0
        for box in boxes:
            x1, y1, x2, y2, cls, conf = box
            if cls != 0:   # class 0 = person in COCO
                continue
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area = area
                best = box

        if best is None:
            return None

        x1, y1, x2, y2, _, conf = best
        if conf < 0.5:
            return None

        box_h_px = y2 - y1
        if box_h_px < 20:
            return None

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        cx = np.clip(cx, 0, disp.shape[1] - 1)
        cy = np.clip(cy, 0, disp.shape[0] - 1)
        d = float(disp[cy, cx])
        if d < 1e-4:
            return None

        # depth_m = (H_real * focal) / box_h_px
        depth_m = (self.PERSON_HEIGHT_M * self.FOCAL_PX) / box_h_px
        # scale: depth_m = scale / d  ⟹ scale = depth_m * d
        return depth_m * d

    def _flow_scale(
        self, gray: np.ndarray, disp: np.ndarray
    ) -> Optional[float]:
        if self._prev_gray is None or self._prev_disp is None:
            return None

        h, w = gray.shape
        if self._prev_pts is None or len(self._prev_pts) < 8:
            self._prev_pts = self._seed_points(h, w)

        pts0 = self._prev_pts.reshape(-1, 1, 2).astype(np.float32)
        pts1, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, pts0, None, **self._flow_params
        )
        if pts1 is None:
            return None

        ok = status.ravel() == 1
        if ok.sum() < 4:
            return None

        p0 = pts0[ok].reshape(-1, 2)
        p1 = pts1[ok].reshape(-1, 2)

        # Gather disparities at tracked points from both frames
        def sample(pts, dmap, gray_shape):
            mp = self._map_pts_to_disp(pts, gray_shape, dmap.shape)
            return dmap[mp[:, 1], mp[:, 0]]

        d0 = sample(p0, self._prev_disp, gray.shape)
        d1 = sample(p1, disp, gray.shape)

        # Use points where disparity changed (camera moved toward/away from scene)
        dd = np.abs(d1 - d0)
        motion = np.linalg.norm(p1 - p0, axis=1)
        valid = (dd > 0.02) & (motion > 1.0) & (d0 > 0.05) & (d1 > 0.05)
        if valid.sum() < 4:
            # No useful depth change → keep previous scale
            self._prev_pts = self._seed_points(h, w)
            return None

        # The ratio of disparities encodes relative depth.
        # We anchor to the current scale: depth0 ≈ scale/d0
        # This provides a self-consistent update without ground truth.
        # Scale estimate: s = depth_anchor * d_curr for a stable region.
        if self._scale is None:
            return None

        depth0_est = self._scale / d0[valid]
        s_estimates = depth0_est * d1[valid]
        s_new = float(np.median(s_estimates))

        # Reject wild outliers
        if self._scale > 0 and (s_new / self._scale > 4 or s_new / self._scale < 0.25):
            return None

        # Update seed points for next frame
        self._prev_pts = self._seed_points(h, w)
        return s_new

    def _seed_points(self, h: int, w: int) -> np.ndarray:
        """Regular grid of seed points in FULL-FRAME coordinates (for LK flow)."""
        gx, gy = self.FLOW_GRID
        xs = np.linspace(w * 0.1, w * 0.9, gx)
        ys = np.linspace(h * 0.1, h * 0.9, gy)
        xx, yy = np.meshgrid(xs, ys)
        return np.stack([xx.ravel(), yy.ravel()], axis=1).astype(np.float32)

    def _map_pts_to_disp(self, pts: np.ndarray,
                          gray_shape: tuple, disp_shape: tuple) -> np.ndarray:
        """Remap full-frame pixel coords → small disparity map coords."""
        gh, gw = gray_shape
        dh, dw = disp_shape
        mapped = pts.copy()
        mapped[:, 0] = np.clip(pts[:, 0] * dw / gw, 0, dw - 1)
        mapped[:, 1] = np.clip(pts[:, 1] * dh / gh, 0, dh - 1)
        return mapped.astype(np.int32)


# ──────────────────────────────────────────────────────────────────────────────
# Main estimator
# ──────────────────────────────────────────────────────────────────────────────

# ──────────────────────────────────────────────────────────────────────────────
# Person detector loader
# ──────────────────────────────────────────────────────────────────────────────

def _load_person_detector():
    """
    Load YOLOv8n (COCO) as the single, robust person detector.

    YOLOv8n is chosen for this role because:
      - ~6 MB, runs at 30+ fps on CPU at 320px inference size
      - COCO class 0 = person, universally reliable label space
      - Handles oblique drone angles adequately at typical K417 flight heights
      - No external HuggingFace dependency — just `pip install ultralytics`

    The returned model has one extra attribute attached at load time:
      ._person_class_id  — always 0 for COCO; kept for API consistency with
                           the rest of the pipeline.
    """
    if not YOLO_AVAILABLE:
        print("[DistEst] ultralytics not installed — person detector disabled.")
        return None

    try:
        model = _YOLO("yolov8n.pt")          # downloads ~6 MB on first run, cached after
        #model = _YOLO('mshamrai/yolov8n-visdrone')
        model._person_class_id = 0            # COCO class 0 = person
        print("[DistEst] YOLOv8n (COCO) person detector loaded ✓")
        return model
    except Exception as e:
        print(f"[DistEst] YOLO load failed: {e}")
        return None


# ──────────────────────────────────────────────────────────────────────────────
# Temporal person tracker
# ──────────────────────────────────────────────────────────────────────────────

class _PersonTracker:
    """
    Lightweight IoU-based tracker that requires a detection to appear in
    CONFIRM_FRAMES consecutive YOLO passes before it is reported as confirmed.

    This is the final filter against one-shot false positives: even if a box
    passes all geometric checks, it must persist across multiple frames before
    we treat it as a real person.

    Each tracked candidate stores:
      box       — latest bounding box (x1,y1,x2,y2,cls,conf)
      hits      — how many consecutive YOLO passes it has been matched
      misses    — how many consecutive YOLO passes it was NOT matched
    """

    IOU_MATCH   = 0.25   # minimum IoU to associate a new detection with a track
    MAX_MISSES  = 5      # drop a track after this many unmatched passes
    CONFIRM     = 2      # passes needed before a track is reported

    def __init__(self):
        self._tracks: list[dict] = []

    def update(self, raw_boxes: list) -> list:
        """
        raw_boxes: list of (x1,y1,x2,y2,cls,conf) from this YOLO pass.
        Returns only confirmed boxes (hits >= CONFIRM, misses == 0).
        """
        matched_track_ids = set()
        matched_box_ids   = set()

        # Match each raw detection to the nearest existing track by IoU
        for bi, box in enumerate(raw_boxes):
            best_iou, best_ti = 0.0, -1
            for ti, track in enumerate(self._tracks):
                iou = self._iou(box, track["box"])
                if iou > best_iou:
                    best_iou, best_ti = iou, ti
            if best_iou >= self.IOU_MATCH:
                # Update existing track
                self._tracks[best_ti]["box"]    = box
                self._tracks[best_ti]["hits"]  += 1
                self._tracks[best_ti]["misses"] = 0
                matched_track_ids.add(best_ti)
                matched_box_ids.add(bi)

        # Unmatched detections → new candidate tracks (hits=1)
        for bi, box in enumerate(raw_boxes):
            if bi not in matched_box_ids:
                self._tracks.append({"box": box, "hits": 1, "misses": 0})

        # Age unmatched existing tracks
        for ti, track in enumerate(self._tracks):
            if ti not in matched_track_ids:
                track["misses"] += 1
                track["hits"]    = 0   # reset: must re-confirm after a gap

        # Prune dead tracks
        self._tracks = [t for t in self._tracks if t["misses"] < self.MAX_MISSES]

        # Return only confirmed, currently-visible tracks
        return [
            t["box"] for t in self._tracks
            if t["hits"] >= self.CONFIRM and t["misses"] == 0
        ]

    @staticmethod
    def _iou(a, b) -> float:
        ax1, ay1, ax2, ay2 = a[0], a[1], a[2], a[3]
        bx1, by1, bx2, by2 = b[0], b[1], b[2], b[3]
        ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
        iw  = max(0.0, ix2 - ix1)
        ih  = max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter == 0:
            return 0.0
        ua = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
        return inter / max(ua, 1e-6)


class DistanceEstimator:
    """
    Thread-safe distance estimator.

    Usage:
        est = DistanceEstimator()
        est.start()
        ...
        bgr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)
        result = est.process(bgr)
        print(result.distance_m)
        est.stop()

    Parameters
    ----------
    use_yolo : bool
        Enable YOLOv8n person-anchor (requires ultralytics).
    yolo_every_n : int
        Run YOLO only every N frames (default 5) to keep CPU load manageable.
    draw_overlay : bool
        Burn OSD onto the returned frame copy.
    depth_map_out : bool
        Include the metric depth map in the result (adds ~1ms copy overhead).
    """

    def __init__(
        self,
        use_yolo:      bool = False,
        yolo_every_n:  int  = 2,
        draw_overlay:  bool = True,
        depth_map_out: bool = False,
        midas_model:   str  = "MiDaS_small",
        device:        str  = "auto",
    ):
        self._use_yolo      = use_yolo and YOLO_AVAILABLE
        self._yolo_every_n  = max(1, yolo_every_n)
        self._draw_overlay  = draw_overlay
        self._depth_map_out = depth_map_out
        self._running       = False

        self._midas:    Optional[_MiDaSBackend]   = None
        self._yolo:     Optional[object]           = None
        self._calib:    _ScaleCalibrator           = _ScaleCalibrator()
        self._frame_n:  int                        = 0
        self._last_res:       DistanceResult             = DistanceResult()
        self._last_yolo_boxes: list | None                 = None
        self._person_tracker: _PersonTracker              = _PersonTracker()
        self._midas_model = midas_model
        self._device = device

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        """Load models (blocking, call from a background thread if needed)."""
        self._running = True
        if not TORCH_AVAILABLE:
            print("[DistEst] WARNING: torch not installed — distance estimation disabled.")
            self._running = False
            return
        try:
            self._midas = _MiDaSBackend(self._midas_model, self._device)
        except Exception as e:
            print(f"[DistEst] MiDaS load failed: {e}")
            self._running = False
            return

        if self._use_yolo:
            self._yolo = _load_person_detector()

        print("[DistEst] Ready ✓")

    def stop(self):
        self._running = False

    @property
    def ready(self) -> bool:
        return self._running and self._midas is not None

    # ── main API ───────────────────────────���──────────────────────────────────

    def process(self, bgr_frame: np.ndarray) -> DistanceResult:
        """
        Process one BGR frame. Returns a DistanceResult.
        Safe to call from the VideoWindow tick thread at 15 fps.
        """
        if not self.ready:
            return DistanceResult()

        t0 = time.perf_counter()
        self._frame_n += 1

        # ── 1. MiDaS depth ───────────────────────────────────────────────────
        disp = self._midas.predict(bgr_frame)          # float32 H×W, [0,1]
        gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)

        # ── 2. YOLO (every N frames) ──────────────────────────────────────────
        # Run on schedule, but KEEP the last result on off-frames so boxes
        # don't flicker — they stay visible until the next detection pass.
        if self._yolo is not None and self._frame_n % self._yolo_every_n == 0:
            self._last_yolo_boxes = self._run_yolo(bgr_frame)
        yolo_boxes = self._last_yolo_boxes

        # ── 3. Scale calibration ──────────────────────────────────────────────
        scale = self._calib.update(gray, disp, yolo_boxes)

        # ── 4. Compute distance to image centre ───────────────────────────────
        # _centre_disp works on the small disp map directly — no coord mapping needed
        centre_disp = self._calib._centre_disp(disp, frac=0.25)
        distance_m  = -1.0
        confidence  = 0.0
        method      = "none"

        if scale is not None and centre_disp > 1e-4:
            distance_m = np.clip(scale / centre_disp, 0.1, 50.0)
            # Confidence based on how many frames we've collected and disp quality
            disp_var = float(np.var(disp))
            confidence = min(1.0, self._frame_n / 30.0) * min(1.0, disp_var * 50)
            if yolo_boxes:
                method = "yolo+midas"
            elif self._frame_n > 5:
                method = "flow+midas"
            else:
                method = "midas(boot)"

        # ── 5. Metric depth map ────────────────────────────────────────────────
        depth_map = None
        if self._depth_map_out and scale is not None:
            depth_map = self._calib.disp_to_depth(disp)

        # ── 6. Overlay ─────────────────────────────────────────────────────────
        # Make ONE copy here; _draw_osd draws in-place on it (no internal copy).
        overlay = None
        if self._draw_overlay:
            overlay = bgr_frame.copy()
            self._draw_osd(overlay, distance_m, confidence, disp, yolo_boxes, method)

        latency_ms = (time.perf_counter() - t0) * 1000
        result = DistanceResult(
            distance_m  = float(distance_m),
            confidence  = float(confidence),
            depth_map   = depth_map,
            overlay     = overlay,
            method      = method,
            latency_ms  = latency_ms,
        )
        self._last_res = result
        return result

    # ── drawing ───────────────────────────────────────────────────────────────

    def _draw_osd(
        self,
        frame:      np.ndarray,
        distance_m: float,
        confidence: float,
        disp:       np.ndarray,
        boxes:      list | None,
        method:     str,
    ) -> np.ndarray:
        # Draw directly on the frame — caller already owns this copy.
        out = frame
        h, w = out.shape[:2]
        dh, dw = disp.shape   # small disparity map dimensions

        CYAN   = (0, 220, 255)
        GREEN  = (80, 255, 80)
        GREY   = (160, 160, 160)
        WHITE  = (230, 230, 230)
        DARK   = (18, 18, 18)

        # ── 1. Depth map thumbnail (top-right) ────────────────────────────────
        # disp is already small (INFER_SIZE), so this resize is cheap.
        thumb_h, thumb_w = 90, 160
        disp_u8 = (disp * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(disp_u8, cv2.COLORMAP_INFERNO)
        heatmap = cv2.resize(heatmap, (thumb_w, thumb_h), interpolation=cv2.INTER_NEAREST)
        tx, ty  = w - thumb_w - 8, 8
        roi = out[ty: ty + thumb_h, tx: tx + thumb_w]
        cv2.addWeighted(heatmap, 0.80, roi, 0.20, 0, roi)
        cv2.rectangle(out, (tx, ty), (tx + thumb_w, ty + thumb_h), (80, 80, 80), 1)
        cv2.putText(out, "depth map  (warm=close)",
                    (tx, ty + thumb_h + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, GREY, 1, cv2.LINE_AA)

        # ── 2. YOLO person boxes (drawn before reticle so reticle is on top) ──
        if boxes:
            for box in boxes:
                x1, y1, x2, y2, cls, conf = box
                if conf < 0.4:
                    continue
                bx1, by1, bx2, by2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(out, (bx1, by1), (bx2, by2), GREEN, 2)
                # Estimate this person's distance from the depth map
                pcx = np.clip(int((bx1 + bx2) / 2 * dw / w), 0, dw - 1)
                pcy = np.clip(int((by1 + by2) / 2 * dh / h), 0, dh - 1)
                pd  = float(disp[pcy, pcx])
                if self._calib._scale and pd > 1e-4:
                    pd_m = np.clip(self._calib._scale / pd, 0.1, 50.0)
                    label = f"person  {pd_m:.1f} m"
                else:
                    label = f"person  {conf:.0%}"
                # Label background pill
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                lx, ly = bx1, max(by1 - 6, lh + 4)
                cv2.rectangle(out, (lx - 2, ly - lh - 3), (lx + lw + 4, ly + 3),
                              DARK, -1)
                cv2.putText(out, label, (lx, ly),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, GREEN, 1, cv2.LINE_AA)

        # ── 3. Centre reticle + floating distance label ───────────────────────
        cx, cy = w // 2, h // 2
        arm = 20          # crosshair arm length
        gap = 6           # gap from centre

        # Draw crosshair manually so we can control the gap
        cv2.line(out, (cx - arm - gap, cy), (cx - gap, cy), CYAN, 1, cv2.LINE_AA)
        cv2.line(out, (cx + gap, cy), (cx + arm + gap, cy), CYAN, 1, cv2.LINE_AA)
        cv2.line(out, (cx, cy - arm - gap), (cx, cy - gap), CYAN, 1, cv2.LINE_AA)
        cv2.line(out, (cx, cy + gap), (cx, cy + arm + gap), CYAN, 1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), gap, CYAN, 1, cv2.LINE_AA)

        # Floating distance label just above the reticle centre
        if distance_m > 0:
            dist_txt = f"{distance_m:.2f} m to crosshair"
            (tw, th), _ = cv2.getTextSize(dist_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
            lx = cx - tw // 2
            ly = cy - arm - gap - 8
            # Dark backing so it's readable over any background
            cv2.rectangle(out, (lx - 4, ly - th - 3), (lx + tw + 4, ly + 4),
                          DARK, -1)
            cv2.putText(out, dist_txt, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, CYAN, 1, cv2.LINE_AA)

        # ── 4. Bottom status bar ──────────────────────────────────────────────
        bar_y = h - 36
        cv2.rectangle(out, (0, bar_y), (w, h), DARK, -1)

        if distance_m > 0:
            # Left: big distance value
            cv2.putText(out, f"Drone-to-target: {distance_m:.2f} m",
                        (10, bar_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, CYAN, 2, cv2.LINE_AA)
            # Right: method + confidence bar
            bar_max  = 80
            bar_fill = int(confidence * bar_max)
            bar_col  = (0, 200, 80) if confidence > 0.6 else (0, 160, 200) if confidence > 0.3 else (80, 80, 200)
            bx = w - bar_max - 90
            cv2.rectangle(out, (bx, bar_y + 8), (bx + bar_max, bar_y + 18), (60, 60, 60), -1)
            cv2.rectangle(out, (bx, bar_y + 8), (bx + bar_fill, bar_y + 18), bar_col, -1)
            cv2.putText(out, f"{method}  {confidence:.0%}",
                        (bx + bar_max + 4, bar_y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, GREY, 1, cv2.LINE_AA)
        else:
            cv2.putText(out, "Calibrating depth scale — keep moving…",
                        (10, bar_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (120, 120, 120), 1, cv2.LINE_AA)

        return out

    # ── YOLO helper ───────────────────────────────────────────────────────────

    # Detection thresholds.
    # VisDrone model uses lower conf (0.35) because it was trained on small
    # aerial persons and calibrated differently from the COCO model.
    # The temporal tracker is the main false-positive guard, not conf alone.
    CONF_THRESHOLD  = 0.20 # per-frame confidence gate
    MIN_BOX_PX      = 8     # minimum box dimension in pixels (aerial = tiny)
    MIN_AREA_PX2    = 80    # minimum box area — kills single-pixel noise
    # Aspect ratio: NOT enforced — from directly above, people look like blobs.
    # The VisDrone model handles this inherently.

    def _run_yolo(self, frame: np.ndarray) -> list:
        """
        Run the person detector and return temporally confirmed detections.

        Filter pipeline:
          1. Person class gate  — model restricted to person class at inference time
          2. Confidence         — CONF_THRESHOLD gate
          3. Minimum size       — boxes smaller than MIN_BOX_PX / MIN_AREA_PX2 are noise
          4. Temporal tracker   — must survive CONFIRM consecutive passes before reported
        """
        frame=cv2.GaussianBlur(frame, (3, 3), sigmaX=0.8)
        try:
            person_cls = self._yolo._person_class_id
            h, w = frame.shape[:2]
            # Upscale to at least 640px on the long side before inference
            if max(h, w) < 640:
                scale = 640 / max(h, w)
                frame = cv2.resize(frame, (int(w * scale), int(h * scale)),
                                interpolation=cv2.INTER_LINEAR)
            #frame = cv2.bilateralFilter(frame, d=5, sigmaColor=50, sigmaSpace=50) 
            results = self._yolo(frame, verbose=False,
                                 conf=self.CONF_THRESHOLD,
                                 classes=[person_cls])
            raw = []
            for r in results:
                for b in r.boxes:
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    conf = float(b.conf[0])
                    bw, bh = x2 - x1, y2 - y1
                    if bw < self.MIN_BOX_PX or bh < self.MIN_BOX_PX:
                        continue
                    if bw * bh < self.MIN_AREA_PX2:
                        continue
                    raw.append((x1, y1, x2, y2, 0, conf))
        except Exception as e:
            print(f"[DistEst] YOLO error: {e}")
            return []

        # Pass raw detections through the temporal tracker.
        # Only boxes confirmed across multiple consecutive passes are returned.
        return self._person_tracker.update(raw)


# ──────────────────────────────────────────────────────────────────────────────
# Async wrapper (runs inference in a background thread, returns last result)
# ──────────────────────────────────────────────────────────────────────────────

class AsyncDistanceEstimator:
    """
    Non-blocking wrapper: submit a frame, always get the most recent result.

    Design:
    - start() returns immediately; model loading runs in a background thread.
    - submit() is always non-blocking. If inference is still running, the
      pending frame is REPLACED (not queued) so we never accumulate lag.
    - The worker drains the queue before each inference pass so it always
      picks up the freshest frame, not a stale one from 300 ms ago.
    """

    def __init__(self, **kwargs):
        import queue
        self._est      = DistanceEstimator(**kwargs)
        self._queue    = queue.Queue(maxsize=2)
        self._result   = DistanceResult()
        self._lock     = threading.Lock()
        self._ready_ev = threading.Event()   # set when models are loaded
        self._stop_ev  = threading.Event()   # set when stop() is called

    def start(self):
        # Loader thread: init models, then signal ready and hand off to worker
        def _loader():
            self._est.start()          # torch hub load — takes a few seconds
            self._ready_ev.set()
            self._worker()             # becomes the inference loop after loading

        t = threading.Thread(target=_loader, daemon=True, name="DistEst-Loader")
        t.start()

    def stop(self):
        self._stop_ev.set()
        self._est.stop()
        # Unblock worker if it is waiting on the queue
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass

    def submit(self, bgr_frame: np.ndarray):
        """
        Submit a frame. Non-blocking always.
        Replaces any pending (not-yet-processed) frame so inference always
        works on the newest frame available, preventing lag build-up.
        """
        # Drain stale frames first, then enqueue the fresh one
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except Exception:
                break
        try:
            self._queue.put_nowait(bgr_frame)
        except Exception:
            pass

    @property
    def ready(self) -> bool:
        """True once models have finished loading."""
        return self._ready_ev.is_set()

    @property
    def result(self) -> DistanceResult:
        """Always returns the latest completed result (thread-safe)."""
        with self._lock:
            return self._result

    def _worker(self):
        import queue as _q
        print("[DistEst] Worker loop started")
        while not self._stop_ev.is_set():
            try:
                frame = self._queue.get(timeout=0.5)
            except _q.Empty:
                # No frame available, but keep listening
                continue

            if frame is None:
                # Explicit stop signal
                break

            if not self._est.ready:
                # Still loading models
                continue

            # Process the frame
            try:
                res = self._est.process(frame)
                with self._lock:
                    self._result = res
            except Exception as e:
                print(f"[DistEst] Process error: {e}")

        print("[DistEst] Worker loop exited")


# ──────────────────────────────────────────────────────────────────────────────
# ── INTEGRATION GUIDE ─────────────────────────────────────────────────────────
# ──────────────────────────────────────────────────────────────────────────────
#
# OPTION A — Async (recommended, never blocks VideoWindow):
# =========================================================
#
#   In K417GUI.__init__ or VideoWindow.__init__:
#
#       from distance_estimator import AsyncDistanceEstimator
#       self._dist_est = AsyncDistanceEstimator(
#           use_yolo     = False,   # set True if ultralytics is installed
#           draw_overlay = True,
#       )
#       self._dist_est.start()     # loads models in a background thread
#
#   In VideoWindow._display (right after you have a PIL image or numpy array):
#
#       # Convert JPEG to numpy BGR for the estimator
#       import numpy as np, cv2
#       nparr  = np.frombuffer(jpeg_bytes, np.uint8)
#       bgr    = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
#       if bgr is not None:
#           self._dist_est.submit(bgr)
#
#       # Use the latest result to annotate the frame
#       res = self._dist_est.result
#       if res.overlay is not None:
#           # Convert overlay back to PIL for Tkinter
#           rgb    = cv2.cvtColor(res.overlay, cv2.COLOR_BGR2RGB)
#           pil_img = Image.fromarray(rgb)
#       else:
#           # No result yet — show raw frame
#           pil_img = Image.open(io.BytesIO(jpeg_bytes))
#
#       # Update distance label in status bar
#       if res.distance_m > 0:
#           self._status_var.set(f"Dist: {res.distance_m:.2f} m  [{res.confidence:.0%}]")
#
#   On close:
#       self._dist_est.stop()
#
#
# OPTION B — Synchronous (simpler, may drop to 10 fps on slow CPUs):
# ===================================================================
#
#   from distance_estimator import DistanceEstimator
#   est = DistanceEstimator(draw_overlay=True)
#   est.start()
#
#   # Inside VideoWindow._tick, after decoding the JPEG:
#   bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
#   res = est.process(bgr)
#   # Use res.overlay and res.distance_m
#
# ──────────────────────────────────────────────────────────────────────────────


if __name__ == "__main__":
    # Quick smoke-test with webcam / video file
    import sys

    src   = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap   = cv2.VideoCapture(src)
    est   = DistanceEstimator(draw_overlay=True, use_yolo=YOLO_AVAILABLE)
    est.start()

    WIN = "Distance Estimator"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    print("Press Q to quit.")
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        result = est.process(frame)
        disp_frame = result.overlay if result.overlay is not None else frame
        cv2.imshow(WIN, disp_frame)
        cv2.setWindowTitle(
            WIN,
            f"Distance: {result.distance_m:.2f} m  [{result.method}  {result.latency_ms:.0f}ms]",
        )
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    est.stop()