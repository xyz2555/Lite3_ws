"""Turn relative motions into timed velocity segments (pure Python).

A motion is a body-frame delta: rotate ``yaw`` degrees first (CCW positive),
then translate ``x`` metres forward and ``y`` metres left. Execution is
open-loop: duration = distance / speed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import List

MAX_VX = 1.0
MAX_VY = 0.5
MAX_WZ = 1.5
SETTLE_S = 0.3


@dataclass
class Segment:
    name: str
    vx: float
    vy: float
    wz: float
    duration: float

    @property
    def translates(self) -> bool:
        return abs(self.vx) > 1e-9 or abs(self.vy) > 1e-9


def motion_segments(name: str, x: float, y: float, yaw_deg: float,
                    lin_speed: float = 0.3, ang_speed: float = 0.5) -> List[Segment]:
    if lin_speed <= 0 or ang_speed <= 0:
        raise ValueError("speeds must be > 0")
    lin_speed = min(lin_speed, MAX_VX)
    ang_speed = min(ang_speed, MAX_WZ)
    segs: List[Segment] = []

    yaw = math.radians(yaw_deg)
    if abs(yaw) > 1e-3:
        segs.append(Segment(f"{name}:rotate", 0.0, 0.0,
                            math.copysign(ang_speed, yaw), abs(yaw) / ang_speed))
        segs.append(Segment(f"{name}:settle", 0.0, 0.0, 0.0, SETTLE_S))

    dist = math.hypot(x, y)
    if dist > 1e-3:
        vx, vy = x / dist * lin_speed, y / dist * lin_speed
        if abs(vy) > MAX_VY:                      # lateral is the tighter limit
            k = MAX_VY / abs(vy)
            vx, vy = vx * k, vy * k
        speed = math.hypot(vx, vy)
        segs.append(Segment(f"{name}:translate", vx, vy, 0.0, dist / speed))
        segs.append(Segment(f"{name}:settle", 0.0, 0.0, 0.0, SETTLE_S))
    return segs


def plan_from_dict(plan: dict) -> List[Segment]:
    v = float(plan.get("default_linear_speed", 0.3))
    w = float(plan.get("default_angular_speed", 0.5))
    wps = plan.get("waypoints", [])
    if not wps:
        raise ValueError("plan has no waypoints")
    segs: List[Segment] = []
    for i, wp in enumerate(wps):
        segs += motion_segments(
            wp.get("name", f"wp_{i}"),
            float(wp.get("x", 0.0)), float(wp.get("y", 0.0)), float(wp.get("yaw", 0.0)),
            float(wp.get("linear_speed", v)), float(wp.get("angular_speed", w)))
    return segs


def plan_from_file(path: str) -> List[Segment]:
    with open(path, "r", encoding="utf-8") as f:
        return plan_from_dict(json.load(f))
