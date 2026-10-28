"""Obstacle-guard logic (pure Python + numpy, no ROS imports).

Two pieces:
  * ``depth_to_meters`` / ``nearest_in_roi``: turn a depth image into one
    robust "nearest obstacle ahead" distance.
  * ``SpeedGate``: scale a velocity command from that distance, with a slow
    zone, a stop zone and hysteresis so the robot does not chatter at the edge.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def depth_to_meters(encoding: str, height: int, width: int, step: int,
                    data: bytes, is_bigendian: bool = False) -> np.ndarray:
    """Decode a sensor_msgs/Image depth buffer into a float32 array in metres.

    Supports ``16UC1`` / ``mono16`` (millimetres, RealSense/OAK default) and
    ``32FC1`` (metres). Invalid pixels become NaN.
    """
    enc = encoding.lower()
    if enc in ("16uc1", "mono16"):
        dtype, scale = np.dtype(">u2" if is_bigendian else "<u2"), 0.001
    elif enc == "32fc1":
        dtype, scale = np.dtype(">f4" if is_bigendian else "<f4"), 1.0
    else:
        raise ValueError(f"unsupported depth encoding '{encoding}'")

    row_items = step // dtype.itemsize
    arr = np.frombuffer(data, dtype=dtype, count=height * row_items)
    arr = arr.reshape(height, row_items)[:, :width].astype(np.float32) * scale
    arr[~np.isfinite(arr) | (arr <= 0.0)] = np.nan
    return arr


def nearest_in_roi(depth_m: np.ndarray,
                   roi: Tuple[float, float, float, float] = (0.3, 0.7, 0.2, 0.65),
                   min_valid: float = 0.15, max_valid: float = 4.0,
                   percentile: float = 5.0, min_pixels: int = 50) -> Optional[float]:
    """Robust nearest distance inside a region of interest.

    ``roi`` = (x_min, x_max, y_min, y_max) as fractions of width/height. The
    default keeps the centre and skips the lower part of the image, where the
    floor shows up. A low percentile instead of the minimum ignores single
    noisy pixels. Returns +inf when the ROI is clear (enough valid pixels, all
    beyond ``max_valid``) and None when there is not enough data to decide.
    """
    h, w = depth_m.shape
    x0, x1 = int(roi[0] * w), max(int(roi[1] * w), int(roi[0] * w) + 1)
    y0, y1 = int(roi[2] * h), max(int(roi[3] * h), int(roi[2] * h) + 1)
    patch = depth_m[y0:y1, x0:x1]
    finite = patch[np.isfinite(patch)]
    finite = finite[finite >= min_valid]
    if finite.size < min_pixels:
        return None
    near = finite[finite <= max_valid]
    if near.size < min_pixels:
        return math.inf
    return float(np.percentile(near, percentile))


@dataclass
class GateConfig:
    stop_distance: float = 0.5      # m, forward/backward motion blocked below this
    slow_distance: float = 1.0      # m, speed scaled linearly between stop and slow
    hysteresis: float = 0.1         # m, must clear stop + hysteresis to resume
    allow_lateral: bool = True      # keep vy when blocked
    allow_rotation: bool = True     # keep wz when blocked


class SpeedGate:
    """Stateful gate for one direction (front or back)."""

    def __init__(self, cfg: GateConfig):
        self.cfg = cfg
        self.blocked = False

    def update(self, distance: Optional[float], fail_safe: bool) -> float:
        """Return a speed scale in [0, 1] for this direction.

        ``distance`` None means "no valid data": blocked if ``fail_safe``.
        """
        c = self.cfg
        if distance is None:
            self.blocked = fail_safe
            return 0.0 if fail_safe else 1.0
        if self.blocked:
            if distance > c.stop_distance + c.hysteresis:
                self.blocked = False
        elif distance <= c.stop_distance:
            self.blocked = True
        if self.blocked:
            return 0.0
        if distance >= c.slow_distance or c.slow_distance <= c.stop_distance:
            return 1.0
        return max(0.0, min(1.0, (distance - c.stop_distance) /
                            (c.slow_distance - c.stop_distance)))


def gate_command(vx: float, vy: float, wz: float,
                 front_scale: float, back_scale: float,
                 front_blocked: bool, back_blocked: bool,
                 cfg: GateConfig) -> Tuple[float, float, float]:
    """Apply per-direction scales to a body-frame velocity command."""
    if vx > 0.0:
        vx *= front_scale
    elif vx < 0.0:
        vx *= back_scale
    if (front_blocked or back_blocked) and not cfg.allow_lateral:
        vy = 0.0
    if (front_blocked or back_blocked) and not cfg.allow_rotation:
        wz = 0.0
    return vx, vy, wz
