import math
import struct

from lite3_driver import protocol as p


def _state_body(ultra=(0.8, 1.6), tail_pad=b"", rpy=(0.1, -0.2, 0.3), pos=(1.0, 2.0, 0.5)):
    body = struct.pack("<ii", 1, 2)
    body += struct.pack("<ddd", *rpy)
    body += struct.pack("<ddd", 0.0, 0.0, 0.0)
    body += struct.pack("<ddd", 0.0, 0.0, 9.81)
    body += struct.pack("<ddd", *pos)
    body += struct.pack("<ddd", 0.0, 0.0, 0.0)
    body += struct.pack("<ddd", 0.3, 0.0, 0.0)
    # fields between vel_body and ultrasound (layout varies by firmware)
    body += b"\x00" * 37
    body += struct.pack("<dd", *ultra)
    return body + tail_pad


def test_simple_packet_layout():
    pkt = p.pack_simple(p.CMD_HEARTBEAT)
    assert len(pkt) == 12
    assert struct.unpack("<III", pkt) == (p.CMD_HEARTBEAT, 0, 0)

def test_velocity_is_clamped():
    vx, vy, wz = p.pack_velocity(5.0, -3.0, 9.0)
    assert struct.unpack("<IIId", vx) == (p.CMD_VEL_X, 8, 1, 1.0)
    assert struct.unpack("<IIId", vy) == (p.CMD_VEL_Y, 8, 1, -0.5)
    assert struct.unpack("<IIId", wz) == (p.CMD_VEL_YAW, 8, 1, 1.5)

def test_split_rejects_short_and_truncated():
    assert p.split_packet(b"\x00" * 5) is None
    truncated = struct.pack("<III", p.RECV_ROBOT_STATE, 100, 1) + b"\x00" * 10
    assert p.split_packet(truncated) is None

def test_invert_wz_flips_only_the_yaw_command():
    vx, vy, wz = p.pack_velocity(0.2, -0.1, 0.5, invert_wz=True)
    assert struct.unpack("<IIId", vx)[3] == 0.2
    assert struct.unpack("<IIId", vy)[3] == -0.1
    assert struct.unpack("<IIId", wz)[3] == -0.5
    # clamping still applies after the flip
    assert struct.unpack("<IIId", p.pack_velocity(0, 0, 9.0, invert_wz=True)[2])[3] == -1.5

def test_parse_state_and_ultrasound_from_tail():
    body = _state_body()
    pkt = struct.pack("<III", p.RECV_ROBOT_STATE, len(body), 1) + body
    code, ctype, payload = p.split_packet(pkt)
    assert (code, ctype) == (p.RECV_ROBOT_STATE, 1)
    st = p.parse_robot_state(payload)
    assert st.basic_state == 1 and st.gait_state == 2
    assert st.pos_world == (1.0, 2.0, 0.5)
    assert st.vel_body[0] == 0.3
    assert st.ultrasound == (0.8, 1.6)

# def test_invalid_ultrasound_is_none():
#     assert p.parse_robot_state(_state_body(ultra=(0.0, 0.0))).ultrasound is None
#     assert p.parse_robot_state(_state_body(ultra=(float("nan"), 1.0))).ultrasound is None

def test_ultrasound_sensors_are_judged_independently():
    nan = float("nan")
    inf = float("inf")
    # no echo on one sensor must not blank the other one
    us = p.parse_robot_state(_state_body(ultra=(0.0, 1.2))).ultrasound
    assert math.isnan(us[0]) and us[1] == 1.2
    us = p.parse_robot_state(_state_body(ultra=(nan, 1.0))).ultrasound
    assert math.isnan(us[0]) and us[1] == 1.0
    # beyond max range = nothing in front = clear (inf), not "no data"
    us = p.parse_robot_state(_state_body(ultra=(5.0, 0.7)), max_ultrasound_range=4.0).ultrasound
    assert us == (inf, 0.7)
    us = p.parse_robot_state(_state_body(ultra=(0.9, 9.0)), max_ultrasound_range=4.0).ultrasound
    assert us == (0.9, inf)

def test_fixed_ultrasound_offset():
    body = _state_body(tail_pad=b"\xff" * 8)
    off = len(body) - 8 - 16
    assert p.parse_robot_state(body, ultrasound_offset=off).ultrasound == (0.8, 1.6)


def test_short_state_rejected():
    assert p.parse_robot_state(b"\x00" * 100) is None

REAL_0901 = bytes.fromhex(
    "0100000000000000000000a05fbfeabf000000409d80d33f00000040da9d6540"
    "000000203ede733f0000002069a260bf0000006089bf5b3f000000406090b4bf"
    "000000e0ecbac1bf00000080c4802340d57c20d95c9deb3ff4094ba1768dd8bf"
    "0852ab2e75f9d43fd2bb963468af6dbf0eec27ff7b5276bf077edd2cbde29ebf"
    "6686e8046d55683fece290920ffb753fa42b834a58f99ebf0000000000000000"
    "000000000000000000000000008040400000000000000000000000c09f1aeb3f"
    "000000a0703dea3f"
)

def test_real_lite3_packet_decodes_to_physical_values():
    st = p.parse_robot_state(REAL_0901)
    assert st is not None and len(REAL_0901) == 200
    assert abs(st.rpy[2] - 172.93) < 0.01            # yaw, degrees
    assert abs(st.xyz_acc[2] - 9.75) < 0.01          # gravity
    assert abs(st.pos_world[2] - 0.3277) < 0.001     # body height, not yaw
    assert st.ultrasound is not None and abs(st.ultrasound[0] - 0.847) < 0.001

def test_nan_state_rejected():
    body = bytearray(_state_body())
    body[p.OFF_POS_WORLD:p.OFF_POS_WORLD + 8] = struct.pack("<d", float("nan"))
    assert p.parse_robot_state(bytes(body)) is None

def test_short_state_rejected():
    assert p.parse_robot_state(b"\x00" * 100) is None

def test_quaternion_yaw_only():
    x, y, z, w = p.quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert abs(x) < 1e-12 and abs(y) < 1e-12
    assert math.isclose(z, math.sin(math.pi / 4))
    assert math.isclose(w, math.cos(math.pi / 4))
