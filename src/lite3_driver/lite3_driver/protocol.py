"""Lite3 motion-host UDP protocol (pure Python, no ROS imports).

Reference: DEEP Robotics Lite3 UDP command document V1.1.0.

Every packet starts with a 12-byte little-endian ``CommandHead``::

    uint32 code             command code
    uint32 parameters_size  value (simple command) or payload size (complex)
    uint32 type             0 = simple, 1 = complex

Kept free of rclpy so it can be unit-tested without a ROS install.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

# ---------------------------------------------------------------- commands
CMD_HEARTBEAT = 0x21040001
CMD_STAND_SIT = 0x21010202      # toggle
CMD_ESTOP = 0x21020C0E          # soft emergency stop
CMD_MOVE_MODE = 0x21010D06
CMD_NAVIGATION = 0x21010C03     # velocity commands only work in this mode
CMD_MANUAL = 0x21010C02

# complex commands, one little-endian double each (navigation mode)
CMD_VEL_X = 0x0140              # m/s    [-1.0, 1.0]
CMD_VEL_YAW = 0x0141            # rad/s  [-1.5, 1.5]
CMD_VEL_Y = 0x0145              # m/s    [-0.5, 0.5]

# robot -> host
RECV_ROBOT_STATE = 0x0901       # RobotStateUpload, ~50 Hz
RECV_JOINT_ANGLE = 0x0902
RECV_JOINT_VEL = 0x0903

MAX_VX = 1.0
MAX_VY = 0.5
MAX_WZ = 1.5

HEAD = struct.Struct("<III")

# RobotStateUpload offsets (bytes) that are stable across firmware builds
OFF_BASIC_STATE = 0
OFF_GAIT_STATE = 4
OFF_RPY = 8
OFF_RPY_VEL = 32
OFF_XYZ_ACC = 56
OFF_POS_WORLD = 80              # x, y, z (height) -- yaw comes from rpy[2]
OFF_VEL_WORLD = 104
OFF_VEL_BODY = 128
MIN_STATE_SIZE = OFF_VEL_BODY + 24

# ``double ultrasound[2]`` (front, back, metres) is the LAST field of the
# struct, so it is read from the tail of the payload. This avoids guessing
# the compiler padding of the bool/unsigned fields that sit before it.
ULTRASOUND_TAIL_BYTES = 16


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def pack_simple(code: int, value: int = 0) -> bytes:
    """Simple command (type 0). ``value`` travels in ``parameters_size``."""
    return HEAD.pack(code & 0xFFFFFFFF, value & 0xFFFFFFFF, 0)


def pack_double(code: int, value: float) -> bytes:
    """Complex command (type 1) carrying a single double."""
    payload = struct.pack("<d", float(value))
    return HEAD.pack(code & 0xFFFFFFFF, len(payload), 1) + payload

def pack_velocity(vx: float, vy: float, wz: float,invert_wz: bool = False) -> Tuple[bytes, bytes, bytes]:
    """Three velocity packets, clamped to the protocol limits.
    ``invert_wz``: the Lite3 yaw command (0x0141) is clockwise-positive, while ROS
    (REP-103) is counterclockwise-positive. Measured on Deeprcs 2.0.153: vx and vy
    follow ROS, but a positive yaw command makes the reported yaw rate negative.
    """
    if invert_wz:
        wz = -wz
    return (
        pack_double(CMD_VEL_X, clamp(vx, MAX_VX)),
        pack_double(CMD_VEL_Y, clamp(vy, MAX_VY)),
        pack_double(CMD_VEL_YAW, clamp(wz, MAX_WZ)),
    )


def split_packet(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    """Return (code, type, body) or None if the datagram is malformed."""
    if len(data) < HEAD.size:
        return None
    code, psize, ctype = HEAD.unpack_from(data, 0)
    body = data[HEAD.size:HEAD.size + psize] if ctype == 1 else b""
    if ctype == 1 and len(body) != psize:
        return None
    return code, ctype, body


@dataclass
class RobotState:
    basic_state: int
    gait_state: int
    rpy: Tuple[float, float, float]
    rpy_vel: Tuple[float, float, float]
    xyz_acc: Tuple[float, float, float]
    pos_world: Tuple[float, float, float]       # x, y, yaw
    vel_world: Tuple[float, float, float]
    vel_body: Tuple[float, float, float]
    ultrasound: Optional[Tuple[float, float]]   # front, back (m) or None

def _clean_range(v: float, max_range: float) -> float:
    """One ultrasound reading -> metres, inf (nothing within range = clear) or nan (no valid echo).

    Each sensor is judged on its own: a sensor that sees nothing must NOT blank the other one.
    """
    if math.isnan(v) or v <= 0.0:
        return math.nan
    if v > max_range:
        return math.inf
    return v

def _vec3(body: bytes, off: int) -> Tuple[float, float, float]:
    return struct.unpack_from("<ddd", body, off)


def parse_robot_state(body: bytes,
                      ultrasound_offset: int = -1,
                      max_ultrasound_range: float = 10.0) -> Optional[RobotState]:
    """Decode a 0x0901 payload.

    ``ultrasound_offset`` < 0 reads the last 16 bytes (default). Set a fixed
    byte offset if your firmware appends extra fields after ``ultrasound``.
    Each ultrasound value is cleaned on its own: above ``max_ultrasound_range`` -> inf
    (nothing in range, i.e. clear), zero / negative / NaN -> NaN (no valid echo).
    """
    if len(body) < MIN_STATE_SIZE:
        return None
    basic, gait= struct.unpack_from("<ii", body, 0)

    us = None
    off = len(body) - ULTRASOUND_TAIL_BYTES if ultrasound_offset < 0 else ultrasound_offset
    if off >= MIN_STATE_SIZE and off + ULTRASOUND_TAIL_BYTES <= len(body):
        front, back = struct.unpack_from("<dd", body, off)
        us = (_clean_range(front, max_ultrasound_range), _clean_range(back, max_ultrasound_range))
        
    state =  RobotState(
        basic_state=basic,
        gait_state=gait,
        # policy_state=policy,
        rpy=_vec3(body, OFF_RPY),
        rpy_vel=_vec3(body, OFF_RPY_VEL),
        xyz_acc=_vec3(body, OFF_XYZ_ACC),
        pos_world=_vec3(body, OFF_POS_WORLD),
        vel_world=_vec3(body, OFF_VEL_WORLD),
        vel_body=_vec3(body, OFF_VEL_BODY),
        ultrasound=us,
    )

    for vec in (state.rpy, state.rpy_vel, state.xyz_acc, state.pos_world, state.vel_world, state.vel_body):
        if not all(math.isfinite(v) and abs(v) < 1e6 for v in vec):
            return None
    return state


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """ZYX Euler -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
