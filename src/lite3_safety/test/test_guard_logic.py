import math

import numpy as np
import pytest

from lite3_safety.guard_logic import (GateConfig, SpeedGate, depth_to_meters,
                                      gate_command, nearest_in_roi)


def _img16(mm, h=48, w=64, pad=0):
    arr = np.full((h, w + pad), mm, dtype="<u2")
    return arr.tobytes(), (w + pad) * 2


def test_decode_16uc1_with_row_padding():
    data, step = _img16(1500, pad=4)
    d = depth_to_meters("16UC1", 48, 64, step, data)
    assert d.shape == (48, 64)
    assert np.allclose(d, 1.5)


def test_decode_zero_is_nan_and_32fc1():
    data, step = _img16(0)
    assert np.isnan(depth_to_meters("16UC1", 48, 64, step, data)).all()
    f = np.full((10, 10), 0.8, dtype="<f4")
    assert np.allclose(depth_to_meters("32FC1", 10, 10, 40, f.tobytes()), 0.8)


def test_decode_unsupported():
    with pytest.raises(ValueError):
        depth_to_meters("rgb8", 1, 1, 3, b"\x00\x00\x00")


def test_nearest_ignores_outlier_pixels():
    d = np.full((100, 100), 3.0, dtype=np.float32)
    d[40, 50] = 0.2                       # one noisy pixel
    d[30:50, 40:60] = 0.9                 # real obstacle, 400 px
    assert math.isclose(nearest_in_roi(d), 0.9, abs_tol=1e-6)


def test_nearest_clear_and_no_data():
    d = np.full((100, 100), 6.0, dtype=np.float32)
    assert nearest_in_roi(d, max_valid=4.0) == math.inf
    assert nearest_in_roi(np.full((100, 100), np.nan, dtype=np.float32)) is None


def test_gate_slow_stop_hysteresis():
    g = SpeedGate(GateConfig(stop_distance=0.5, slow_distance=1.0, hysteresis=0.1))
    assert g.update(2.0, True) == 1.0
    assert math.isclose(g.update(0.75, True), 0.5)
    assert g.update(0.45, True) == 0.0 and g.blocked
    assert g.update(0.55, True) == 0.0 and g.blocked      # inside hysteresis
    assert g.update(0.65, True) > 0.0 and not g.blocked


def test_gate_fail_safe():
    g = SpeedGate(GateConfig())
    assert g.update(None, True) == 0.0 and g.blocked
    assert g.update(None, False) == 1.0 and not g.blocked


def test_gate_command_keeps_turning_and_backing_off():
    cfg = GateConfig()
    vx, vy, wz = gate_command(0.3, 0.1, 0.5, 0.0, 1.0, True, False, cfg)
    assert (vx, vy, wz) == (0.0, 0.1, 0.5)
    vx, _, _ = gate_command(-0.2, 0.0, 0.0, 0.0, 1.0, True, False, cfg)
    assert vx == -0.2
    cfg_strict = GateConfig(allow_lateral=False, allow_rotation=False)
    assert gate_command(0.3, 0.1, 0.5, 0.0, 1.0, True, False, cfg_strict) == (0.0, 0.0, 0.0)
