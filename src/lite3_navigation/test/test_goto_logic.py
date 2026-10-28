"""Offline tests for goto_logic: a small simulated robot closes the loop.

The simulated Lite3 is a body-frame velocity follower with a first-order lag,
optional pose noise and optional sensor delay. No ROS needed.
"""

import math
import random
from collections import deque

from lite3_navigation.goto_logic import (Command, GoToConfig, GoToPoint, State,
                                         wrap_angle, yaw_from_quat)

DT = 0.05  # 20 Hz control loop


class SimRobot:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, tau=0.3, gain=1.0,
                 noise_xy=0.0, noise_yaw=0.0, delay_steps=0, seed=1):
        self.x, self.y, self.yaw = x, y, yaw
        self.vx = self.vy = self.wz = 0.0
        self.tau, self.gain = tau, gain
        self.noise_xy, self.noise_yaw = noise_xy, noise_yaw
        self.rng = random.Random(seed)
        self.hist = deque([(x, y, yaw)] * (delay_steps + 1), maxlen=delay_steps + 1)

    def step(self, cmd, dt=DT):
        a = dt / (self.tau + dt)
        self.vx += a * (self.gain * cmd.vx - self.vx)
        self.vy += a * (self.gain * cmd.vy - self.vy)
        self.wz += a * (self.gain * cmd.wz - self.wz)
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw = wrap_angle(self.yaw + self.wz * dt)
        self.hist.append((self.x, self.y, self.yaw))

    def pose(self):
        x, y, yaw = self.hist[0]
        r = self.rng
        return (x + r.gauss(0, self.noise_xy), y + r.gauss(0, self.noise_xy),
                wrap_angle(yaw + r.gauss(0, self.noise_yaw)))


def run(ctrl, robot, goal, max_t=120.0, blocked_fn=None, fault_fn=None, yaw_goal=True):
    """Drive the loop until the goal finishes. Returns (log, elapsed seconds)."""
    gx, gy, gyaw = goal
    ctrl.set_goal(gx, gy, gyaw if yaw_goal else None, robot.pose(), 0.0)
    log, t = [], 0.0
    while t < max_t and not ctrl.finished:
        t += DT
        blocked = bool(blocked_fn(t)) if blocked_fn else False
        fault = fault_fn(t) if fault_fn else None
        cmd = ctrl.step(t, robot.pose(), blocked=blocked, fault=fault)
        robot.step(cmd)
        log.append((t, ctrl.state, cmd, (robot.x, robot.y, robot.yaw)))
    return log, t


def final_errors(robot, goal):
    return (math.hypot(goal[0] - robot.x, goal[1] - robot.y),
            abs(wrap_angle(goal[2] - robot.yaw)))


# ---------------------------------------------------------------- basics
def test_reaches_goal_ahead_with_final_yaw():
    cfg = GoToConfig()
    ctrl, robot = GoToPoint(cfg), SimRobot()
    goal = (1.0, 0.0, math.pi / 2)
    log, t = run(ctrl, robot, goal)
    assert ctrl.state == State.SUCCEEDED
    d, e = final_errors(robot, goal)
    assert d <= cfg.xy_tol + 0.05 and e <= cfg.yaw_tol + 0.05
    assert t < 40.0
    assert max(c.vx for _, _, c, _ in log) <= cfg.max_vx + 1e-9
    assert max(abs(c.wz) for _, _, c, _ in log) <= cfg.max_wz + 1e-9


def test_goal_behind_rotates_before_moving():
    ctrl, robot = GoToPoint(), SimRobot()
    goal = (-1.0, 0.0, math.pi)
    log, _ = run(ctrl, robot, goal)
    assert ctrl.state == State.SUCCEEDED
    # while the heading error is large the robot must not translate
    first_move = next(i for i, (_, s, _, _) in enumerate(log) if s == State.MOVE)
    assert all(c.vx == 0.0 for _, s, c, _ in log[:first_move])
    # and it never goes backwards (v1 is forward-only)
    assert min(c.vx for _, _, c, _ in log) >= 0.0


def test_yaw_only_goal_does_not_translate():
    ctrl, robot = GoToPoint(), SimRobot()
    goal = (0.0, 0.0, 1.2)
    log, _ = run(ctrl, robot, goal)
    assert ctrl.state == State.SUCCEEDED
    assert math.hypot(robot.x, robot.y) < 0.05
    assert abs(wrap_angle(1.2 - robot.yaw)) <= 0.2


def test_goal_without_yaw_stops_at_position():
    cfg = GoToConfig()
    ctrl, robot = GoToPoint(cfg), SimRobot()
    log, _ = run(ctrl, robot, (0.8, 0.5, 0.0), yaw_goal=False)
    assert ctrl.state == State.SUCCEEDED
    assert math.hypot(0.8 - robot.x, 0.5 - robot.y) <= cfg.xy_tol + 0.05


def test_already_at_goal_succeeds_immediately():
    ctrl, robot = GoToPoint(), SimRobot(x=1.0, y=1.0, yaw=0.3)
    log, t = run(ctrl, robot, (1.05, 1.0, 0.35))
    assert ctrl.state == State.SUCCEEDED
    assert t < 1.0


def test_yaw_wraparound_takes_the_short_way():
    ctrl, robot = GoToPoint(), SimRobot(yaw=3.1)
    log, _ = run(ctrl, robot, (0.0, 0.0, -3.1))
    assert ctrl.state == State.SUCCEEDED
    # the short way is ~0.18 rad; the long way would be ~6.1 rad
    travelled = sum(abs(c.wz) * DT for _, _, c, _ in log)
    assert travelled < 1.0


def test_acceleration_limits_are_respected():
    cfg = GoToConfig()
    ctrl, robot = GoToPoint(cfg), SimRobot()
    log, _ = run(ctrl, robot, (1.0, 0.5, 0.0))
    prev = Command()
    for _, _, c, _ in log[:-1]:   # the last tick is the deliberate hard stop at the goal
        assert abs(c.vx - prev.vx) <= cfg.max_ax * DT + 1e-9
        assert abs(c.wz - prev.wz) <= cfg.max_aw * DT + 1e-9
        prev = c


def test_no_overshoot_beyond_tolerance_and_no_oscillation():
    cfg = GoToConfig()
    ctrl, robot = GoToPoint(cfg), SimRobot()
    goal = (1.5, 0.0, 0.0)
    log, _ = run(ctrl, robot, goal)
    assert ctrl.state == State.SUCCEEDED
    assert max(p[0] for *_, p in log) <= goal[0] + 0.10
    # gentle approach: no sign flips of vx (forward only)
    assert all(c.vx >= 0.0 for _, _, c, _ in log)


def test_robust_to_noise_and_delay():
    cfg = GoToConfig()
    for seed in range(5):
        ctrl = GoToPoint(cfg)
        robot = SimRobot(noise_xy=0.01, noise_yaw=0.01, delay_steps=3, tau=0.4, seed=seed)
        goal = (1.0, -0.8, -1.0)
        log, t = run(ctrl, robot, goal)
        assert ctrl.state == State.SUCCEEDED, (seed, ctrl.status())
        d, e = final_errors(robot, goal)
        assert d <= cfg.xy_tol + 0.08 and e <= cfg.yaw_tol + 0.08
        assert t < 60.0


# ---------------------------------------------------------------- blocked
def test_blocked_forever_waits_then_aborts():
    cfg = GoToConfig(block_timeout=5.0)
    ctrl, robot = GoToPoint(cfg), SimRobot()
    # blocked from t=2 s on
    log, t = run(ctrl, robot, (2.0, 0.0, 0.0), blocked_fn=lambda t: t >= 2.0)
    assert ctrl.state == State.ABORTED and ctrl.reason == "blocked_timeout"
    # it waited about block_timeout, did not creep forward, and ended with zero command
    waits = [t for t, s, _, _ in log if s == State.WAIT_BLOCKED]
    assert 4.5 <= (waits[-1] - waits[0]) <= 5.5
    assert all(c.vx == 0.0 for _, s, c, _ in log if s == State.WAIT_BLOCKED)
    assert robot.x < 1.0


def test_blocked_briefly_resumes_and_succeeds():
    ctrl, robot = GoToPoint(), SimRobot()
    goal = (1.5, 0.0, 0.0)
    log, _ = run(ctrl, robot, goal, blocked_fn=lambda t: 3.0 <= t < 7.0)
    assert ctrl.state == State.SUCCEEDED
    assert any(s == State.WAIT_BLOCKED for _, s, _, _ in log)
    assert final_errors(robot, goal)[0] <= 0.25


def test_waiting_does_not_consume_goal_deadline():
    cfg = GoToConfig(timeout_base=4.0, timeout_factor=1.2, block_timeout=30.0)
    ctrl, robot = GoToPoint(cfg), SimRobot()
    goal = (1.0, 0.0, 0.0)
    # 20 s of blocking is far beyond the nominal deadline, yet the goal must still succeed
    log, _ = run(ctrl, robot, goal, blocked_fn=lambda t: 3.0 <= t < 23.0)
    assert ctrl.state == State.SUCCEEDED, ctrl.status()


def test_rotation_is_allowed_while_blocked():
    ctrl, robot = GoToPoint(), SimRobot()
    goal = (0.0, 0.0, 1.0)  # yaw-only: guard never blocks rotation
    log, _ = run(ctrl, robot, goal, blocked_fn=lambda t: True)
    assert ctrl.state == State.SUCCEEDED


# ---------------------------------------------------------------- failures
def test_unresponsive_robot_aborts_no_progress():
    cfg = GoToConfig(no_progress_timeout=4.0)
    ctrl, robot = GoToPoint(cfg), SimRobot(gain=0.0)
    log, t = run(ctrl, robot, (1.0, 0.0, 0.0))
    assert ctrl.state == State.ABORTED and ctrl.reason == "no_progress"
    assert t < 12.0
    assert log[-1][2].vx == 0.0 and log[-1][2].wz == 0.0


def test_total_timeout():
    cfg = GoToConfig(timeout_base=1.0, timeout_factor=0.1, no_progress_timeout=100.0)
    ctrl, robot = GoToPoint(cfg), SimRobot()
    log, _ = run(ctrl, robot, (3.0, 0.0, 0.0))
    assert ctrl.state == State.ABORTED and ctrl.reason == "timeout"


def test_fault_aborts_immediately_with_zero_command():
    ctrl, robot = GoToPoint(), SimRobot()
    log, t = run(ctrl, robot, (3.0, 0.0, 0.0),
                 fault_fn=lambda t: "pose_stale" if t >= 2.0 else None)
    assert ctrl.state == State.ABORTED and ctrl.reason == "pose_stale"
    assert 1.9 <= t <= 2.2
    last = log[-1][2]
    assert (last.vx, last.vy, last.wz) == (0.0, 0.0, 0.0)


def test_nan_pose_aborts():
    ctrl = GoToPoint()
    ctrl.set_goal(1.0, 0.0, 0.0, (0.0, 0.0, 0.0), 0.0)
    cmd = ctrl.step(0.05, (float("nan"), 0.0, 0.0))
    assert ctrl.state == State.ABORTED and ctrl.reason == "pose_invalid"
    assert (cmd.vx, cmd.wz) == (0.0, 0.0)
    ctrl.set_goal(1.0, 0.0, 0.0, (0.0, 0.0, 0.0), 1.0)
    ctrl.step(1.05, None)
    assert ctrl.reason == "pose_invalid"


def test_cancel_and_replace_goal():
    ctrl, robot = GoToPoint(), SimRobot()
    ctrl.set_goal(5.0, 0.0, 0.0, robot.pose(), 0.0)
    for k in range(1, 40):
        robot.step(ctrl.step(k * DT, robot.pose()))
    assert ctrl.active
    ctrl.cancel("teleop_override")
    assert ctrl.state == State.ABORTED and ctrl.reason == "teleop_override"
    cmd = ctrl.step(2.5, robot.pose())
    assert (cmd.vx, cmd.wz) == (0.0, 0.0)
    # a new goal works after a cancel
    log, _ = run(ctrl, robot, (robot.x + 0.5, robot.y, 0.0))
    assert ctrl.state == State.SUCCEEDED


def test_idle_outputs_zero():
    ctrl = GoToPoint()
    cmd = ctrl.step(0.0, (0.0, 0.0, 0.0))
    assert ctrl.state == State.IDLE and (cmd.vx, cmd.wz) == (0.0, 0.0)


def test_yaw_from_quat():
    assert math.isclose(yaw_from_quat(0, 0, math.sin(0.4), math.cos(0.4)), 0.8)
    assert math.isclose(yaw_from_quat(0, 0, 0, 1), 0.0)
