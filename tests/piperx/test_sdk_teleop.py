from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from robot_control.piperx.estimator import Estimate
from robot_control.piperx.monitoring import LatestEventHub
from robot_control.piperx.sdk_teleop import (
    ArmIdentity,
    StandaloneTeleopCoordinator,
    StandaloneTeleopRuntime,
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


def test_operator_target_uses_joint_liveness_and_holds_stale_gripper():
    sdk = FakeSdk()
    sdk.gripper_ctrl.time_stamp = 1.0
    sdk.gripper_ctrl.Hz = 0.0

    target = operator_target(sdk, fallback_gripper_um=12_345)

    assert target.timestamp_s == 12.5
    assert target.frequency_hz == 198.0
    assert target.gripper_um == 12_345
    assert target.gripper_mm == pytest.approx(12.345)


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


class FullFakeSdk(FakeSdk):
    def __init__(self, interface: str, *, operator: bool):
        super().__init__()
        self.interface = interface
        self.operator = operator
        self.calls: list[tuple] = []
        self.firmware = "S-V1.8-9"
        self.gripper = NS(
            time_stamp=13.0,
            Hz=100.0,
            gripper_state=NS(grippers_angle=30_000),
        )
        self.auto_advance = False
        self.ack_role_commands = True
        self._response_clock = 20.0
        self.response = NS(
            time_stamp=0.0,
            instruction_response=NS(instruction_index=-1),
        )

    def ConnectPort(self, **kwargs):
        self.calls.append(("ConnectPort", kwargs))

    def DisconnectPort(self):
        self.calls.append(("DisconnectPort",))

    def SearchPiperFirmwareVersion(self):
        self.calls.append(("SearchPiperFirmwareVersion",))

    def GetPiperFirmwareVersion(self):
        return self.firmware

    def MasterSlaveConfig(self, *args):
        self.calls.append(("MasterSlaveConfig", *args))
        if args[0] == 0xFA and self.ack_role_commands:
            self._response_clock += 0.001
            self.response.time_stamp = self._response_clock
            self.response.instruction_response.instruction_index = 0x70

    def ClearRespSetInstruction(self):
        self.calls.append(("ClearRespSetInstruction",))
        self.response.time_stamp = 0.0
        self.response.instruction_response.instruction_index = -1

    def GetRespInstruction(self):
        return self.response

    def MotionCtrl_1(self, *args):
        self.calls.append(("MotionCtrl_1", *args))

    def ModeCtrl(self, *args):
        self.calls.append(("ModeCtrl", *args))

    def EnableArm(self, *args):
        self.calls.append(("EnableArm", *args))
        return True

    def JointCtrl(self, *args):
        self.calls.append(("JointCtrl", *args))

    def GripperCtrl(self, *args):
        self.calls.append(("GripperCtrl", *args))

    def GetArmGripperMsgs(self):
        return self.gripper

    def GetArmJointCtrl(self):
        if self.auto_advance:
            self.joint_ctrl.time_stamp += 0.005
        return super().GetArmJointCtrl()


def _pair_configs(tmp_path: Path):
    return (
        TeleopPairConfig(
            "left",
            ArmIdentity("left-leader", "serial-0", "leader", "can0"),
            ArmIdentity("left-follower", "serial-2", "follower", "can2"),
            tmp_path / "left.json",
        ),
        TeleopPairConfig(
            "right",
            ArmIdentity("right-leader", "serial-1", "leader", "can1"),
            ArmIdentity("right-follower", "serial-3", "follower", "can3"),
            tmp_path / "right.json",
        ),
    )


def _sdk_fixture():
    return {
        "can0": FullFakeSdk("can0", operator=True),
        "can1": FullFakeSdk("can1", operator=True),
        "can2": FullFakeSdk("can2", operator=False),
        "can3": FullFakeSdk("can3", operator=False),
    }


def test_coordinator_owns_each_sdk_once_and_shares_follower_feedback(tmp_path: Path):
    sdks = _sdk_fixture()
    factory_calls = []

    def factory(interface, *, judge_flag, can_auto_init):
        factory_calls.append((interface, judge_flag, can_auto_init))
        return sdks[interface]

    coordinator = StandaloneTeleopCoordinator(
        _pair_configs(tmp_path),
        sdk_factory=factory,
        clock_ns=time.monotonic_ns,
        sleeper=lambda _duration: None,
    )

    coordinator.connect()
    sdks["can0"].joint_ctrl.time_stamp += 0.005
    sdks["can1"].joint_ctrl.time_stamp += 0.005
    states = coordinator.step()
    coordinator.close()

    assert sorted(factory_calls) == [
        ("can0", True, True),
        ("can1", True, True),
        ("can2", True, True),
        ("can3", True, True),
    ]
    for sdk in sdks.values():
        assert ("ConnectPort", {"piper_init": False}) in sdk.calls
        assert sdk.calls[-1] == ("DisconnectPort",)
        assert sdk.calls.count(("SearchPiperFirmwareVersion",)) == 1
    assert ("MasterSlaveConfig", 0xFA, 0, 0, 0) in sdks["can0"].calls
    assert ("MotionCtrl_1", 0x02, 0, 0) in sdks["can0"].calls
    assert sdks["can0"].calls.index(("MasterSlaveConfig", 0xFA, 0, 0, 0)) < sdks[
        "can0"
    ].calls.index(("MotionCtrl_1", 0x02, 0, 0))
    assert ("MasterSlaveConfig", 0xFC, 0, 0, 0) not in sdks["can0"].calls
    assert ("MasterSlaveConfig", 0xFC, 0, 0, 0) not in sdks["can1"].calls
    assert not any(call[0] == "MotionCtrl_1" for call in sdks["can2"].calls)
    assert not any(call[0] == "MotionCtrl_1" for call in sdks["can3"].calls)
    assert ("MasterSlaveConfig", 0xFC, 0, 0, 0) in sdks["can2"].calls
    assert ("ModeCtrl", 1, 1, 10, 0xAD) in sdks["can2"].calls
    assert ("EnableArm", 7, 0x02) in sdks["can2"].calls
    assert ("JointCtrl", 1000, -2000, 3000, -4000, 5000, -6000) in sdks["can2"].calls
    assert ("GripperCtrl", 30_000, 1000, 0x01, 0) in sdks["can2"].calls
    assert states["left"].interface == "can2"
    assert states["left"].current_a == pytest.approx((-0.1,) * 6)


def test_coordinator_watchdog_holds_then_fails_without_disabling(tmp_path: Path):
    sdks = _sdk_fixture()
    now = [1_000_000_000]
    coordinator = StandaloneTeleopCoordinator(
        _pair_configs(tmp_path),
        sdk_factory=lambda interface, **_kwargs: sdks[interface],
        clock_ns=lambda: now[0],
        sleeper=lambda _duration: None,
    )
    coordinator.connect()
    sdks["can0"].ack_role_commands = False
    sdks["can1"].ack_role_commands = False
    initial_joint_commands = sum(
        call[0] == "JointCtrl" for call in sdks["can2"].calls
    )

    now[0] += 300_000_000
    coordinator.step()
    held_joint_commands = sum(call[0] == "JointCtrl" for call in sdks["can2"].calls)
    assert held_joint_commands == initial_joint_commands

    now[0] += 701_000_000
    with pytest.raises(TeleopSafetyError, match="leader liveness timeout"):
        coordinator.step()
    assert not any(call[0] == "DisableArm" for call in sdks["can2"].calls)
    coordinator.close()


def test_coordinator_accepts_role_ack_as_stationary_leader_liveness(tmp_path: Path):
    sdks = _sdk_fixture()
    now = [1_000_000_000]
    coordinator = StandaloneTeleopCoordinator(
        _pair_configs(tmp_path),
        sdk_factory=lambda interface, **_kwargs: sdks[interface],
        clock_ns=lambda: now[0],
        sleeper=lambda _duration: None,
    )
    coordinator.connect()
    initial_commands = sum(call[0] == "JointCtrl" for call in sdks["can2"].calls)

    now[0] += 1_100_000_000
    states = coordinator.step()

    assert set(states) == {"left", "right"}
    assert sum(call[0] == "JointCtrl" for call in sdks["can2"].calls) > initial_commands
    assert ("ClearRespSetInstruction",) in sdks["can0"].calls
    coordinator.close()


def test_coordinator_joint_frames_keep_watchdog_alive_when_gripper_is_quiet(
    tmp_path: Path,
):
    sdks = _sdk_fixture()
    now = [1_000_000_000]
    coordinator = StandaloneTeleopCoordinator(
        _pair_configs(tmp_path),
        sdk_factory=lambda interface, **_kwargs: sdks[interface],
        clock_ns=lambda: now[0],
        sleeper=lambda _duration: None,
    )
    coordinator.connect()

    now[0] += 1_100_000_000
    sdks["can0"].joint_ctrl.time_stamp += 0.005
    sdks["can1"].joint_ctrl.time_stamp += 0.005
    sdks["can0"].gripper_ctrl.Hz = 0.0
    sdks["can1"].gripper_ctrl.Hz = 0.0

    states = coordinator.step()

    assert set(states) == {"left", "right"}
    coordinator.close()


class ZeroEstimator:
    def estimate(self, state):
        zeros = np.zeros(6)
        return Estimate(
            interface=state.interface,
            adapter_serial=state.adapter_serial,
            timestamp_ns=state.timestamp_ns,
            q_rad=np.asarray(state.q_rad),
            qd_filtered_rad_s=np.asarray(state.qd_rad_s),
            qdd_rad_s2=zeros,
            tau_measured_nm=np.asarray(state.tau_measured_nm),
            tau_model_nm=zeros,
            tau_bias_nm=zeros,
            tau_external_nm=-np.asarray(state.tau_measured_nm),
            valid=True,
            calibrated=True,
            reason=None,
        )


def test_runtime_publishes_two_arms_and_propagates_clean_stop(tmp_path: Path):
    sdks = _sdk_fixture()
    sdks["can0"].auto_advance = True
    sdks["can1"].auto_advance = True
    with pytest.raises(ValueError, match="event queue"):
        StandaloneTeleopRuntime(
            _pair_configs(tmp_path),
            estimators={"left": ZeroEstimator(), "right": ZeroEstimator()},
            hub=LatestEventHub(queue_size=1),
            sdk_factory=lambda interface, **_kwargs: sdks[interface],
        )

    hub = LatestEventHub(queue_size=2)
    subscriber = hub.subscribe()
    runtime = StandaloneTeleopRuntime(
        _pair_configs(tmp_path),
        estimators={"left": ZeroEstimator(), "right": ZeroEstimator()},
        hub=hub,
        sdk_factory=lambda interface, **_kwargs: sdks[interface],
        control_rate_hz=200.0,
        ui_rate_hz=25.0,
    )

    runtime.start()
    seen = {subscriber.get(timeout=1.0)["arm"] for _ in range(2)}
    for _ in range(100):
        if runtime.healthy():
            break
        threading.Event().wait(0.005)
    runtime.stop()

    assert seen == {"left", "right"}
    assert runtime.status()["mode"] == "standalone_teleop"
    assert set(runtime.snapshot()["arms"]) == {"left", "right"}
    assert not runtime.healthy()
    assert all(sdk.calls[-1] == ("DisconnectPort",) for sdk in sdks.values())


def test_runtime_start_surfaces_connect_failure_and_closes_all_sdks(tmp_path: Path):
    sdks = _sdk_fixture()
    sdks["can2"].firmware = "S-V1.8-2"
    runtime = StandaloneTeleopRuntime(
        _pair_configs(tmp_path),
        estimators={"left": ZeroEstimator(), "right": ZeroEstimator()},
        hub=LatestEventHub(queue_size=2),
        sdk_factory=lambda interface, **_kwargs: sdks[interface],
    )

    with pytest.raises(TeleopSafetyError, match="below required"):
        runtime.start()

    assert not runtime.healthy()
    assert all(sdk.calls[-1] == ("DisconnectPort",) for sdk in sdks.values())
