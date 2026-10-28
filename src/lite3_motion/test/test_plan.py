import math

import pytest

from lite3_motion.plan import MAX_VY, motion_segments, plan_from_dict


def test_rotate_then_translate():
    segs = motion_segments("a", 1.0, 0.0, 90, 0.5, 0.5)
    assert [s.name for s in segs] == ["a:rotate", "a:settle", "a:translate", "a:settle"]
    assert math.isclose(segs[0].wz, 0.5) and math.isclose(segs[0].duration, math.pi)
    assert math.isclose(segs[2].vx, 0.5) and math.isclose(segs[2].duration, 2.0)


def test_negative_yaw_turns_right():
    assert motion_segments("a", 0, 0, -45)[0].wz < 0


def test_lateral_limit_keeps_direction():
    seg = motion_segments("a", 0.0, 1.0, 0, lin_speed=0.9)[0]
    assert math.isclose(seg.vy, MAX_VY) and math.isclose(seg.duration, 1.0 / MAX_VY)


def test_plan_and_errors():
    segs = plan_from_dict({"waypoints": [{"x": 0.5}, {"yaw": 90}]})
    assert len(segs) == 4
    with pytest.raises(ValueError):
        plan_from_dict({"waypoints": []})
    with pytest.raises(ValueError):
        motion_segments("a", 1, 0, 0, lin_speed=0)
