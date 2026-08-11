"""Pure decoders for the Piper feedback frames used by the estimator."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import TypeAlias


POSITION_IDS = {0x2A5: 0, 0x2A6: 2, 0x2A7: 4}
HIGH_SPEED_IDS = {0x251 + index: index for index in range(6)}
LOW_SPEED_IDS = {0x261 + index: index for index in range(6)}
JOINT_COMMAND_IDS = {0x155: 0, 0x156: 2, 0x157: 4}
FIRMWARE_ID = 0x4AF
GRIPPER_ID = 0x2A8
EFFORT_NM_PER_RAW = (
    1.18125e-3,
    1.18125e-3,
    1.18125e-3,
    0.95844e-3,
    0.95844e-3,
    0.95844e-3,
)

_POSITION = struct.Struct(">ii")
_HIGH_SPEED = struct.Struct(">hhi")
_LOW_SPEED = struct.Struct(">HhbBH")
_GRIPPER = struct.Struct(">ihBB")


class FrameDecodeError(ValueError):
    """Raised when a recognized Piper frame cannot be decoded safely."""


@dataclass(frozen=True)
class HighSpeedSample:
    joint_index: int
    velocity_rad_s: float
    current_a: float
    effort_nm: float


@dataclass(frozen=True)
class LowSpeedSample:
    joint_index: int
    voltage_v: float
    foc_temp_c: int
    motor_temp_c: int
    status_code: int
    bus_current_a: float


@dataclass(frozen=True)
class JointCommandPair:
    first_joint: int
    positions_rad: tuple[float, float]


@dataclass(frozen=True)
class GripperSample:
    travel_mm: float
    torque_nm: float
    status_code: int


@dataclass(frozen=True)
class FirmwareFragment:
    data: bytes


PositionPair: TypeAlias = tuple[int, tuple[float, float]]
DecodedFrame: TypeAlias = (
    PositionPair
    | HighSpeedSample
    | LowSpeedSample
    | JointCommandPair
    | GripperSample
    | FirmwareFragment
)


def _require_payload(payload: bytes) -> None:
    if len(payload) != 8:
        raise FrameDecodeError(f"Piper feedback payload must contain 8 bytes, got {len(payload)}")


def decode_position_pair(can_id: int, payload: bytes) -> PositionPair:
    """Decode one of the three two-joint position feedback frames."""
    if can_id not in POSITION_IDS:
        raise FrameDecodeError(f"CAN identifier 0x{can_id:x} is not a position frame")
    _require_payload(payload)
    raw_first, raw_second = _POSITION.unpack(payload)
    scale = math.pi / (180.0 * 1000.0)
    return POSITION_IDS[can_id], (raw_first * scale, raw_second * scale)


def decode_high_speed(can_id: int, payload: bytes) -> HighSpeedSample:
    """Decode motor velocity and current-derived joint effort."""
    if can_id not in HIGH_SPEED_IDS:
        raise FrameDecodeError(f"CAN identifier 0x{can_id:x} is not a high-speed frame")
    _require_payload(payload)
    raw_speed, raw_current, _ = _HIGH_SPEED.unpack(payload)
    joint_index = HIGH_SPEED_IDS[can_id]
    return HighSpeedSample(
        joint_index=joint_index,
        velocity_rad_s=raw_speed * 1e-3,
        current_a=raw_current * 1e-3,
        effort_nm=raw_current * EFFORT_NM_PER_RAW[joint_index],
    )


def decode_low_speed(can_id: int, payload: bytes) -> LowSpeedSample:
    """Decode passive driver telemetry without affecting state coherence."""
    if can_id not in LOW_SPEED_IDS:
        raise FrameDecodeError(f"CAN identifier 0x{can_id:x} is not a low-speed frame")
    _require_payload(payload)
    voltage, foc_temp, motor_temp, status_code, bus_current = _LOW_SPEED.unpack(payload)
    return LowSpeedSample(
        joint_index=LOW_SPEED_IDS[can_id],
        voltage_v=voltage * 0.1,
        foc_temp_c=foc_temp,
        motor_temp_c=motor_temp,
        status_code=status_code,
        bus_current_a=bus_current * 1e-3,
    )


def decode_joint_command(can_id: int, payload: bytes) -> JointCommandPair:
    """Decode an observed follower joint command without transmitting."""
    if can_id not in JOINT_COMMAND_IDS:
        raise FrameDecodeError(f"CAN identifier 0x{can_id:x} is not a joint-command frame")
    _require_payload(payload)
    first, second = _POSITION.unpack(payload)
    scale = math.pi / (180.0 * 1000.0)
    return JointCommandPair(
        first_joint=JOINT_COMMAND_IDS[can_id],
        positions_rad=(first * scale, second * scale),
    )


def decode_gripper(can_id: int, payload: bytes) -> GripperSample:
    """Decode official 0x2A8 gripper travel, feedback torque, and status."""
    if can_id != GRIPPER_ID:
        raise FrameDecodeError(f"CAN identifier 0x{can_id:x} is not a gripper frame")
    _require_payload(payload)
    raw_travel, raw_torque, status_code, _reserved = _GRIPPER.unpack(payload)
    return GripperSample(
        travel_mm=raw_travel * 1e-3,
        torque_nm=raw_torque * 1e-3,
        status_code=status_code,
    )


def decode_frame(can_id: int, payload: bytes) -> DecodedFrame | None:
    """Decode a frame needed by the estimator and ignore every other ID."""
    if can_id in POSITION_IDS:
        return decode_position_pair(can_id, payload)
    if can_id in HIGH_SPEED_IDS:
        return decode_high_speed(can_id, payload)
    if can_id in LOW_SPEED_IDS:
        return decode_low_speed(can_id, payload)
    if can_id in JOINT_COMMAND_IDS:
        return decode_joint_command(can_id, payload)
    if can_id == GRIPPER_ID:
        return decode_gripper(can_id, payload)
    if can_id == FIRMWARE_ID:
        _require_payload(payload)
        return FirmwareFragment(bytes(payload))
    return None
