"""Go-to-point control logic (pure Python, no ROS imports).

Same split as lite3_safety/guard_logic.py: all decisions live here so they can be
unit-tested (and simulated) without ROS or the robot. ``go_to_point_node.py`` is a
thin wrapper that feeds this class a pose, a "blocked" flag and a fault string at a
fixed rate and publishes the Twist it returns.

Behaviour (v1, face-and-go, no sideways motion):

    ROTATE -> MOVE -> ALIGN -> SUCCEEDED
                 \\-> WAIT_BLOCKED -> (clear) MOVE
                                   -> (timeout) ABORTED

* ROTATE       turn in place until facing the goal point
* MOVE         drive forward with P control on distance (slows down near the goal),
               small heading correction; turns back to ROTATE if the heading error
               grows large; waits if the obstacle guard reports "blocked"
* WAIT_BLOCKED stand still; resume when clear, abort after ``block_timeout``
* ALIGN        rotate in place to the requested final yaw (skipped if yaw is None)

Any state can end in ABORTED with a reason string (timeout, blocked_timeout,
no_progress, pose_invalid, pose_stale, not_standing, teleop_override, cancelled, ...).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

Pose2D = Tuple[float, float, float]  # x, y, yaw (rad) in the odom frame


# ---------------------------------------------------------------- helpers
def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def slew(current: float, target: float, max_delta: float) -> float:
    """Move ``current`` toward ``target`` by at most ``max_delta``."""
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


# ---------------------------------------------------------------- config
@dataclass
class GoToConfig:
    # limits (protocol allows 1.0 / 0.5 / 1.5 -- stay well below at first)
    max_vx: float = 0.3          # m/s
    max_wz: float = 0.5          # rad/s
    max_ax: float = 0.5          # m/s^2   linear acceleration limit
    max_aw: float = 1.0          # rad/s^2 angular acceleration limit
    min_vx: float = 0.05         # m/s     do not creep slower than this while moving
    min_wz: float = 0.10         # rad/s   same for pure rotation
    # gains
    k_lin: float = 0.8           # 1/s     vx = k_lin * distance
    k_ang: float = 1.2           # 1/s     wz = k_ang * angle error
    # tolerances
    xy_tol: float = 0.15         # m
    yaw_tol: float = 0.15        # rad (~8.6 deg)
    realign_factor: float = 1.5  # leave ALIGN again if pushed beyond xy_tol * this
    # rotate / move switching (hysteresis)
    rotate_enter: float = 0.52   # rad (~30 deg): stop and rotate in place above this
    rotate_exit: float = 0.17    # rad (~10 deg): start moving below this
    heading_freeze: float = 0.25 # m: no heading correction this close to the goal
    # failure handling
    block_timeout: float = 10.0        # s waiting while blocked before aborting
    no_progress_timeout: float = 5.0   # s without improvement before aborting
    progress_eps_m: float = 0.05       # m   improvement that counts as progress
    progress_eps_rad: float = 0.10     # rad improvement that counts as progress
    timeout_base: float = 10.0         # s   fixed part of the per-goal deadline
    timeout_factor: float = 2.5        # x   nominal travel + rotation time


@dataclass
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class State(str, Enum):
    IDLE = "IDLE"
    ROTATE = "ROTATE"
    MOVE = "MOVE"
    WAIT_BLOCKED = "WAIT_BLOCKED"
    ALIGN = "ALIGN"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"


_ACTIVE = (State.ROTATE, State.MOVE, State.WAIT_BLOCKED, State.ALIGN)


# ---------------------------------------------------------------- controller
class GoToPoint:
    def __init__(self, cfg: Optional[GoToConfig] = None):
        self.cfg = cfg or GoToConfig()
        self.state = State.IDLE
        self.reason = ""
        self.gx = self.gy = 0.0
        self.gyaw: Optional[float] = None
        self.dist = 0.0
        self.heading_err = 0.0
        self.yaw_err = 0.0
        self._cmd = Command()
        self._last_t: Optional[float] = None
        self._deadline = 0.0
        self._wait_start = 0.0
        self._best = math.inf
        self._best_t = 0.0

    # ------------------------------------------------------------ queries
    @property
    def active(self) -> bool:
        return self.state in _ACTIVE

    @property
    def finished(self) -> bool:
        return self.state in (State.SUCCEEDED, State.ABORTED)

    def status(self) -> str:
        if self.state == State.ABORTED:
            return f"ABORTED:{self.reason}"
        return self.state.value

    # ------------------------------------------------------------ commands
    def set_goal(self, x: float, y: float, yaw: Optional[float], pose: Pose2D,
                 now: float) -> None:
        """Start a new goal (replaces any running one). ``pose`` is the current pose."""
        c = self.cfg
        self.gx, self.gy = float(x), float(y)
        self.gyaw = None if yaw is None else wrap_angle(float(yaw))
        px, py, pyaw = pose
        dist = math.hypot(self.gx - px, self.gy - py)
        rot = 0.0
        if dist > c.xy_tol:
            rot += abs(wrap_angle(math.atan2(self.gy - py, self.gx - px) - pyaw))
            end_heading = math.atan2(self.gy - py, self.gx - px)
        else:
            end_heading = pyaw
        if self.gyaw is not None:
            rot += abs(wrap_angle(self.gyaw - end_heading))
        nominal = dist / c.max_vx + rot / c.max_wz
        self._deadline = now + c.timeout_base + c.timeout_factor * nominal
        self._cmd = Command()
        self._last_t = now
        self.reason = ""
        self._enter(State.ROTATE, now)

    def cancel(self, reason: str = "cancelled") -> None:
        if self.active:
            self._finish(State.ABORTED, reason)

    # ------------------------------------------------------------ internals
    def _enter(self, state: State, now: float) -> None:
        self.state = state
        self._best = math.inf
        self._best_t = now

    def _finish(self, state: State, reason: str = "") -> Command:
        self.state, self.reason = state, reason
        self._cmd = Command()
        return self._cmd

    def _rot_cmd(self, err: float, with_min: bool) -> float:
        c = self.cfg
        w = c.k_ang * err
        if with_min and abs(w) < c.min_wz:
            w = math.copysign(c.min_wz, err)
        return clamp(w, c.max_wz)

    def _after_arrival(self, now: float) -> None:
        if self.gyaw is None:
            self._finish(State.SUCCEEDED)
        else:
            self._enter(State.ALIGN, now)

    # ------------------------------------------------------------ main step
    def step(self, now: float, pose: Optional[Pose2D], blocked: bool = False,
             fault: Optional[str] = None) -> Command:
        """Advance one control tick and return the velocity command to publish."""
        c = self.cfg
        dt = 0.0 if self._last_t is None else min(max(now - self._last_t, 0.0), 0.2)
        self._last_t = now

        if not self.active:
            self._cmd = Command()
            return self._cmd
        if fault:
            return self._finish(State.ABORTED, fault)
        if pose is None or not all(math.isfinite(v) for v in pose):
            return self._finish(State.ABORTED, "pose_invalid")
        if self.state == State.WAIT_BLOCKED:
            self._deadline += dt            # waiting does not eat the goal deadline
        if now > self._deadline:
            return self._finish(State.ABORTED, "timeout")

        x, y, yaw = pose
        dx, dy = self.gx - x, self.gy - y
        self.dist = math.hypot(dx, dy)
        self.heading_err = wrap_angle(math.atan2(dy, dx) - yaw) if self.dist > 1e-6 else 0.0
        self.yaw_err = 0.0 if self.gyaw is None else wrap_angle(self.gyaw - yaw)

        vx = wz = 0.0
        hard_stop = False          # skip the acceleration limiter, command zero now
        metric = math.inf          # quantity that must keep improving

        for _ in range(4):         # a few chained transitions per tick are possible
            st = self.state
            if st == State.ROTATE:
                if self.dist <= c.xy_tol:
                    self._after_arrival(now)
                elif abs(self.heading_err) <= c.rotate_exit:
                    self._enter(State.MOVE, now)
                else:
                    wz = self._rot_cmd(self.heading_err, with_min=True)
                    metric = abs(self.heading_err)
                    break
            elif st == State.MOVE:
                if self.dist <= c.xy_tol:
                    self._after_arrival(now)
                elif abs(self.heading_err) > c.rotate_enter:
                    self._enter(State.ROTATE, now)
                elif blocked:
                    self._enter(State.WAIT_BLOCKED, now)
                    self._wait_start = now
                else:
                    vx = clamp(c.k_lin * self.dist, c.max_vx)
                    vx = max(vx, c.min_vx) * max(0.0, math.cos(self.heading_err))
                    if self.dist > c.heading_freeze:
                        wz = self._rot_cmd(self.heading_err, with_min=False)
                    metric = self.dist
                    break
            elif st == State.WAIT_BLOCKED:
                if self.dist <= c.xy_tol:
                    self._after_arrival(now)
                elif not blocked:
                    self._enter(State.MOVE, now)
                elif now - self._wait_start > c.block_timeout:
                    return self._finish(State.ABORTED, "blocked_timeout")
                else:
                    hard_stop = True
                    break
            elif st == State.ALIGN:
                if self.dist > c.xy_tol * c.realign_factor:
                    self._enter(State.ROTATE, now)
                elif abs(self.yaw_err) <= c.yaw_tol:
                    self._finish(State.SUCCEEDED)
                else:
                    wz = self._rot_cmd(self.yaw_err, with_min=True)
                    metric = abs(self.yaw_err)
                    break
            else:
                break

        if self.finished:
            self._cmd = Command()
            return self._cmd

        # no-progress watchdog
        if math.isfinite(metric):
            eps = c.progress_eps_m if self.state == State.MOVE else c.progress_eps_rad
            if metric < self._best - eps:
                self._best, self._best_t = metric, now
            elif self._best == math.inf:
                self._best, self._best_t = metric, now
            elif now - self._best_t > c.no_progress_timeout:
                return self._finish(State.ABORTED, "no_progress")

        if hard_stop:
            self._cmd = Command()
        else:
            self._cmd = Command(
                vx=slew(self._cmd.vx, vx, c.max_ax * dt),
                vy=0.0,
                wz=slew(self._cmd.wz, wz, c.max_aw * dt),
            )
        return self._cmd
