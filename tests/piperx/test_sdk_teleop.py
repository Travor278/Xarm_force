from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from robot_control.piperx.sdk_teleop import (
    ArmIdentity,
    TeleopPairConfig,
    TeleopSafetyError,
    follower_state,
    operator_target,
    parse_firmware_version,
    require_aligned,
    validate_pair_configs,
)


def _motors(**values):
    return NS(**{f"motor_{index}": NS(**values) for index in range(1, 7)})


class FakeSdk:
    def __init__(self):
        self.joint_ctrl = NS(
            time_stamp=12.5,
            Hz=198.0,
            joint_ctrl=NS(
                joint_1=1000,
                joint_2=-2000,
                joint_3=3000,
                joint_4=-4000,
                joint_5=5000,
                joint_6=-6000,
            ),
        )
        self.gripper_ctrl = NS(
            time_stamp=12.5,
            Hz=100.0,
            gripper_ctrl=NS(grippers_angle=30_000),
        )
        self.joints = NS(
            time_stamp=13.0,
            Hz=190.0,
            joint_state=NS(
                joint_1=1000,
                joint_2=-2000,
                joint_3=3000,
                joint_4=-4000,
                joint_5=5000,
                joint_6=-6000,
            ),
        )
        self.high = _motors(motor_speed=250, current=-100, effort=-118.125)
        self.high.time_stamp = 13.0
        self.high.Hz = 200.0
        self.low = _motors(
            vol=500,
            foc_temp=41,
            motor_temp=36,
            foc_status_code=0x40,
            bus_current=123,
        )
        self.low.time_stamp = 13.0
        self.low.Hz = 10.0

    def GetArmJointCtrl(self):
        return self.joint_ctrl

    def GetArmGripperCtrl(self):
        return self.gripper_ctrl

    def GetArmJointMsgs(self):
        return self.joints

    def GetArmHighSpdInfoMsgs(self):
        return self.high

    def GetArmLowSpdInfoMsgs(self):
        return self.low


def test_operator_target_converts_official_control_units():
    target = operator_target(FakeSdk())

    assert target.timestamp_s == 12.5
    assert target.frequency_hz == 198.0
    assert target.joint_mdeg == (1000, -2000, 3000, -4000, 5000, -6000)
    assert target.q_rad == pytest.approx(
        tuple(math.radians(value / 1000.0) for value in target.joint_mdeg)
    )
    assert target.gripper_um == 30_000
    assert target.gripper_mm == pytest.approx(30.0)


def test_follower_state_uses_same_sdk_feedback_for_dashboard():
    identity = ArmIdentity("left", "follower-serial", "follower", "can2")
    target = operator_target(FakeSdk())

    state = follower_state(FakeSdk(), identity, target, now_ns=9_000_000_000)

    assert state.interface == "can2"
    assert state.adapter_serial == "follower-serial"
    assert state.timestamp_ns == 9_000_000_000
    assert state.q_rad == pytest.approx(target.q_rad)
    assert state.qd_rad_s == pytest.approx((0.25,) * 6)
    assert state.current_a == pytest.approx((-0.1,) * 6)
    assert state.tau_measured_nm == pytest.approx((-0.118125,) * 6)
    assert state.frequency_hz == pytest.approx(190.0)
    assert state.q_command_rad == pytest.approx(target.q_rad)
    assert state.command_fresh
    assert all(state.driver_fresh)
    assert state.driver[0].voltage_v == pytest.approx(50.0)
    assert state.driver[0].foc_temp_c == 41
    assert state.driver[0].motor_temp_c == 36
    assert state.driver[0].status_code == 0x40
    assert state.driver[0].bus_current_a == pytest.approx(0.123)


def test_alignment_accepts_exact_limits_and_rejects_excess():
    q_zero = (0.0,) * 6
    q_at_limit = (math.radians(15.0),) + (0.0,) * 5

    require_aligned(q_at_limit, q_zero, 20.0, 10.0)

    with pytest.raises(TeleopSafetyError, match="joint 1"):
        require_aligned((math.radians(15.001),) + (0.0,) * 5, q_zero, 20.0, 10.0)
    with pytest.raises(TeleopSafetyError, match="gripper"):
        require_aligned(q_zero, q_zero, 20.001, 10.0)


def test_firmware_and_four_arm_identity_gates(tmp_path: Path):
    assert parse_firmware_version("S-V1.8-9") == (1, 8, 9)
    assert parse_firmware_version("S-V1.9-0") == (1, 9, 0)
    with pytest.raises(TeleopSafetyError, match="firmware"):
        parse_firmware_version("unknown")

    left = TeleopPairConfig(
        "left",
        ArmIdentity("left-leader", "serial-0", "leader"),
        ArmIdentity("left-follower", "serial-2", "follower"),
        tmp_path / "left.json",
    )
    right = TeleopPairConfig(
        "right",
        ArmIdentity("right-leader", "serial-1", "leader"),
        ArmIdentity("right-follower", "serial-3", "follower"),
        tmp_path / "right.json",
    )
    validate_pair_configs((left, right))

    duplicate = TeleopPairConfig(
        "right",
        ArmIdentity("right-leader", "serial-0", "leader"),
        right.follower,
        right.calibration_path,
    )
    with pytest.raises(TeleopSafetyError, match="four distinct"):
        validate_pair_configs((left, duplicate))

