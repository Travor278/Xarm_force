"""Standalone Piper SDK teleoperation with shared dashboard telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Literal, Sequence

from .socketcan import DriverTelemetry, PiperState


class TeleopSafetyError(RuntimeError):
    """Raised when standalone teleoperation cannot start safely."""


@dataclass(frozen=True)
class ArmIdentity:
    name: str
    serial: str
    role: Literal["leader", "follower"]
    interface: str | None = None


@dataclass(frozen=True)
class TeleopPairConfig:
    name: str
    leader: ArmIdentity
    follower: ArmIdentity
    calibration_path: Path


@dataclass(frozen=True)
class OperatorTarget:
    timestamp_s: float
    frequency_hz: float
    joint_mdeg: tuple[int, ...]
    q_rad: tuple[float, ...]
    gripper_um: int
    gripper_mm: float


def _six_fields(value: object, prefix: str) -> tuple[object, ...]:
    try:
        result = tuple(getattr(value, f"{prefix}_{index}") for index in range(1, 7))
    except AttributeError as error:
        raise TeleopSafetyError(f"incomplete Piper SDK {prefix} data") from error
    return result


def _finite(values: Sequence[float], label: str) -> tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != 6 or not all(math.isfinite(value) for value in converted):
        raise TeleopSafetyError(f"{label} must contain six finite values")
    return converted


def operator_target(interface: object) -> OperatorTarget:
    """Copy the latest teaching-arm target out of one SDK interface."""
    joint_message = interface.GetArmJointCtrl()
    gripper_message = interface.GetArmGripperCtrl()
    joints = joint_message.joint_ctrl
    joint_mdeg = tuple(int(value) for value in _six_fields(joints, "joint"))
    q_rad = _finite(
        tuple(math.radians(value / 1000.0) for value in joint_mdeg),
        "operator target",
    )
    gripper_um = int(gripper_message.gripper_ctrl.grippers_angle)
    timestamp_s = min(
        float(joint_message.time_stamp), float(gripper_message.time_stamp)
    )
    frequency_hz = min(float(joint_message.Hz), float(gripper_message.Hz) * 2.0)
    if timestamp_s <= 0 or frequency_hz <= 0:
        raise TeleopSafetyError("operator target feedback is not live")
    return OperatorTarget(
        timestamp_s=timestamp_s,
        frequency_hz=frequency_hz,
        joint_mdeg=joint_mdeg,
        q_rad=q_rad,
        gripper_um=gripper_um,
        gripper_mm=abs(gripper_um) * 0.001,
    )


def follower_state(
    interface: object,
    identity: ArmIdentity,
    target: OperatorTarget,
    now_ns: int,
    *,
    firmware: str | None = None,
) -> PiperState:
    """Build dashboard input from the follower controller's existing SDK data."""
    joint_message = interface.GetArmJointMsgs()
    high_message = interface.GetArmHighSpdInfoMsgs()
    low_message = interface.GetArmLowSpdInfoMsgs()
    joints = _six_fields(joint_message.joint_state, "joint")
    q_rad = _finite(
        tuple(math.radians(float(value) / 1000.0) for value in joints),
        "follower position",
    )
    high_motors = _six_fields(high_message, "motor")
    qd = _finite(
        tuple(float(motor.motor_speed) * 0.001 for motor in high_motors),
        "follower velocity",
    )
    current = _finite(
        tuple(float(motor.current) * 0.001 for motor in high_motors),
        "follower current",
    )
    effort = _finite(
        tuple(float(motor.effort) * 0.001 for motor in high_motors),
        "follower effort",
    )
    low_motors = _six_fields(low_message, "motor")
    drivers = tuple(
        DriverTelemetry(
            voltage_v=float(motor.vol) * 0.1,
            foc_temp_c=int(motor.foc_temp),
            motor_temp_c=int(motor.motor_temp),
            status_code=int(motor.foc_status_code),
            bus_current_a=float(motor.bus_current) * 0.001,
            timestamp_ns=now_ns,
        )
        for motor in low_motors
    )
    frequency_hz = min(float(joint_message.Hz), float(high_message.Hz))
    if frequency_hz <= 0:
        raise TeleopSafetyError("follower feedback is not live")
    return PiperState(
        interface=identity.interface or "",
        adapter_serial=identity.serial,
        timestamp_ns=now_ns,
        q_rad=q_rad,
        qd_rad_s=qd,
        tau_measured_nm=effort,
        frequency_hz=frequency_hz,
        complete=True,
        fresh=True,
        reason=None,
        current_a=current,
        driver=drivers,
        driver_fresh=(True,) * 6,
        q_command_rad=target.q_rad,
        command_fresh=True,
        firmware=firmware,
    )


def require_aligned(
    operator_q_rad: Sequence[float],
    follower_q_rad: Sequence[float],
    operator_gripper_mm: float,
    follower_gripper_mm: float,
    *,
    max_joint_deg: float = 15.0,
    max_gripper_mm: float = 10.0,
) -> None:
    operator = _finite(operator_q_rad, "operator alignment position")
    follower = _finite(follower_q_rad, "follower alignment position")
    for index, (leader, work) in enumerate(zip(operator, follower, strict=True), 1):
        delta_deg = abs(math.degrees(leader - work))
        if delta_deg > max_joint_deg:
            raise TeleopSafetyError(
                f"joint {index} alignment error {delta_deg:.3f} deg exceeds "
                f"{max_joint_deg:.3f} deg"
            )
    gripper_delta = abs(float(operator_gripper_mm) - float(follower_gripper_mm))
    if not math.isfinite(gripper_delta) or gripper_delta > max_gripper_mm:
        raise TeleopSafetyError(
            f"gripper alignment error {gripper_delta:.3f} mm exceeds "
            f"{max_gripper_mm:.3f} mm"
        )


_FIRMWARE = re.compile(r"^S-V(\d+)\.(\d+)-(\d+)$")


def parse_firmware_version(value: object) -> tuple[int, int, int]:
    match = _FIRMWARE.fullmatch(str(value))
    if match is None:
        raise TeleopSafetyError(f"unrecognized Piper firmware {value!r}")
    return tuple(int(component) for component in match.groups())  # type: ignore[return-value]


def validate_pair_configs(configs: Sequence[TeleopPairConfig]) -> None:
    if len(configs) != 2 or {config.name for config in configs} != {"left", "right"}:
        raise TeleopSafetyError("standalone teleop requires left and right pairs")
    identities = [
        identity
        for config in configs
        for identity in (config.leader, config.follower)
    ]
    if any(not identity.name or not identity.serial for identity in identities):
        raise TeleopSafetyError("arm identities must include names and serials")
    if any(config.leader.role != "leader" or config.follower.role != "follower" for config in configs):
        raise TeleopSafetyError("each pair must contain one leader and one follower")
    if len({identity.serial for identity in identities}) != 4:
        raise TeleopSafetyError("standalone teleop requires four distinct USB-CAN adapters")

