"""Standalone Piper SDK teleoperation with shared dashboard telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import threading
import time
from typing import Callable, Literal, Mapping, Protocol, Sequence

from .estimator import Estimate
from .monitoring import SCHEMA_VERSION, LatestEventHub, serialize_snapshot
from .socketcan import DriverTelemetry, PiperState
from .socketcan import discover_interface


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


def operator_target(
    interface: object,
    *,
    fallback_gripper_um: int | None = None,
) -> OperatorTarget:
    """Copy the latest teaching-arm target out of one SDK interface."""
    joint_message = interface.GetArmJointCtrl()
    gripper_message = interface.GetArmGripperCtrl()
    joints = joint_message.joint_ctrl
    joint_mdeg = tuple(int(value) for value in _six_fields(joints, "joint"))
    q_rad = _finite(
        tuple(math.radians(value / 1000.0) for value in joint_mdeg),
        "operator target",
    )
    timestamp_s = float(joint_message.time_stamp)
    frequency_hz = float(joint_message.Hz)
    if timestamp_s <= 0 or frequency_hz <= 0:
        raise TeleopSafetyError("operator target feedback is not live")
    gripper_is_live = (
        float(gripper_message.time_stamp) > 0 and float(gripper_message.Hz) > 0
    )
    if gripper_is_live:
        gripper_um = int(gripper_message.gripper_ctrl.grippers_angle)
    elif fallback_gripper_um is not None:
        gripper_um = int(fallback_gripper_um)
    else:
        raise TeleopSafetyError("operator gripper feedback is not live")
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


class _Estimator(Protocol):
    def estimate(self, state: PiperState) -> Estimate: ...


@dataclass
class _Endpoint:
    identity: ArmIdentity
    interface: str
    sdk: object
    firmware: str | None = None


@dataclass
class _PairSession:
    config: TeleopPairConfig
    leader: _Endpoint
    follower: _Endpoint
    target: OperatorTarget
    last_sdk_timestamp_s: float
    last_command_ns: int


def _feedback_pose(interface: object) -> tuple[tuple[float, ...], tuple[int, ...], float]:
    joint_message = interface.GetArmJointMsgs()
    gripper_message = interface.GetArmGripperMsgs()
    if float(joint_message.Hz) <= 0 or float(gripper_message.Hz) <= 0:
        raise TeleopSafetyError("Piper feedback is not live")
    joint_mdeg = tuple(
        int(value) for value in _six_fields(joint_message.joint_state, "joint")
    )
    q_rad = _finite(
        tuple(math.radians(value / 1000.0) for value in joint_mdeg),
        "Piper feedback position",
    )
    gripper_mm = abs(float(gripper_message.gripper_state.grippers_angle)) * 0.001
    if not math.isfinite(gripper_mm):
        raise TeleopSafetyError("Piper gripper feedback is not finite")
    return q_rad, joint_mdeg, gripper_mm


def _command_follower(interface: object, target: OperatorTarget, effort: int) -> None:
    interface.JointCtrl(*target.joint_mdeg)
    interface.GripperCtrl(target.gripper_um, effort, 0x01, 0)


def _default_sdk_factory(interface: str, *, judge_flag: bool, can_auto_init: bool):
    try:
        from piper_sdk import C_PiperInterface_V2
    except ImportError as error:
        raise TeleopSafetyError("piper_sdk is required for standalone teleop") from error
    return C_PiperInterface_V2(
        interface,
        judge_flag=judge_flag,
        can_auto_init=can_auto_init,
    )


class StandaloneTeleopCoordinator:
    """Own four SDK interfaces and tee follower feedback into the estimator path."""

    def __init__(
        self,
        pair_configs: Sequence[TeleopPairConfig],
        *,
        sdk_factory: Callable[..., object] = _default_sdk_factory,
        interface_resolver: Callable[[str], str] = discover_interface,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        sleeper: Callable[[float], None] = time.sleep,
        speed_ratio: int = 10,
        gripper_effort: int = 1000,
        hold_after_ms: float = 250.0,
        fail_after_ms: float = 1000.0,
    ) -> None:
        validate_pair_configs(pair_configs)
        if not 1 <= speed_ratio <= 100:
            raise ValueError("speed_ratio must be in [1, 100]")
        if not 0 <= gripper_effort <= 5000:
            raise ValueError("gripper_effort must be in [0, 5000]")
        if not 0 < hold_after_ms < fail_after_ms:
            raise ValueError("watchdog thresholds must be positive and ordered")
        self.configs = tuple(pair_configs)
        self.sdk_factory = sdk_factory
        self.interface_resolver = interface_resolver
        self.clock_ns = clock_ns
        self.sleeper = sleeper
        self.speed_ratio = speed_ratio
        self.gripper_effort = gripper_effort
        self.hold_after_ns = int(hold_after_ms * 1e6)
        self.fail_after_ns = int(fail_after_ms * 1e6)
        self._endpoints: list[_Endpoint] = []
        self._sessions: dict[str, _PairSession] = {}
        self._connected = False

    def _open_endpoint(self, identity: ArmIdentity) -> _Endpoint:
        interface = identity.interface or self.interface_resolver(identity.serial)
        sdk = self.sdk_factory(interface, judge_flag=True, can_auto_init=True)
        endpoint = _Endpoint(
            identity=ArmIdentity(identity.name, identity.serial, identity.role, interface),
            interface=interface,
            sdk=sdk,
        )
        self._endpoints.append(endpoint)
        sdk.ConnectPort(piper_init=False)
        return endpoint

    def _wait_firmware(self) -> None:
        for endpoint in self._endpoints:
            endpoint.sdk.SearchPiperFirmwareVersion()
        pending = list(self._endpoints)
        for _attempt in range(300):
            next_pending = []
            for endpoint in pending:
                value = endpoint.sdk.GetPiperFirmwareVersion()
                try:
                    version = parse_firmware_version(value)
                except TeleopSafetyError:
                    next_pending.append(endpoint)
                    continue
                if version < (1, 8, 9):
                    raise TeleopSafetyError(
                        f"Piper firmware {value} is below required S-V1.8-9"
                    )
                endpoint.firmware = str(value)
            if not next_pending:
                return
            pending = next_pending
            self.sleeper(0.01)
        names = ", ".join(endpoint.identity.name for endpoint in pending)
        raise TeleopSafetyError(f"timed out reading Piper firmware for {names}")

    def _wait_pose(self, endpoint: _Endpoint) -> tuple[tuple[float, ...], tuple[int, ...], float]:
        for _attempt in range(300):
            try:
                return _feedback_pose(endpoint.sdk)
            except TeleopSafetyError:
                self.sleeper(0.01)
        raise TeleopSafetyError(
            f"timed out waiting for feedback from {endpoint.identity.name}"
        )

    def _wait_operator_target(self, endpoint: _Endpoint) -> OperatorTarget:
        for _attempt in range(300):
            try:
                operator_target(endpoint.sdk)
                self.sleeper(0.1)
                return operator_target(endpoint.sdk)
            except TeleopSafetyError:
                self.sleeper(0.01)
        raise TeleopSafetyError(
            f"timed out waiting for teaching frames from {endpoint.identity.name}; "
            "set it to leader mode (0xFA) and power-cycle that leader arm"
        )

    def _wait_enabled(self, endpoint: _Endpoint) -> None:
        for attempt in range(300):
            if attempt % 20 == 0:
                endpoint.sdk.EnableArm(7, 0x02)
            low = endpoint.sdk.GetArmLowSpdInfoMsgs()
            motors = _six_fields(low, "motor")
            if all(int(motor.foc_status_code) & 0x40 for motor in motors):
                return
            self.sleeper(0.01)
        raise TeleopSafetyError(
            f"timed out enabling all motors on {endpoint.identity.name}"
        )

    def connect(self) -> None:
        if self._connected:
            return
        try:
            pairs: list[tuple[TeleopPairConfig, _Endpoint, _Endpoint]] = []
            for config in self.configs:
                leader = self._open_endpoint(config.leader)
                follower = self._open_endpoint(config.follower)
                pairs.append((config, leader, follower))
            interfaces = [endpoint.interface for endpoint in self._endpoints]
            if len(set(interfaces)) != 4:
                raise TeleopSafetyError("four arm serials resolved to duplicate CAN interfaces")
            self._wait_firmware()
            for _config, _leader, follower in pairs:
                follower.sdk.MasterSlaveConfig(0xFC, 0, 0, 0)
            for _config, leader, _follower in pairs:
                leader.sdk.MasterSlaveConfig(0xFA, 0, 0, 0)
                leader.sdk.MotionCtrl_1(0x02, 0, 0)
            for config, leader, follower in pairs:
                target = self._wait_operator_target(leader)
                follower_q, _follower_mdeg, follower_gripper = self._wait_pose(follower)
                require_aligned(
                    target.q_rad,
                    follower_q,
                    target.gripper_mm,
                    follower_gripper,
                )
                follower.sdk.ModeCtrl(1, 1, self.speed_ratio, 0xAD)
                self._wait_enabled(follower)
                _command_follower(follower.sdk, target, self.gripper_effort)
                now_ns = self.clock_ns()
                self._sessions[config.name] = _PairSession(
                    config=config,
                    leader=leader,
                    follower=follower,
                    target=target,
                    last_sdk_timestamp_s=target.timestamp_s,
                    last_command_ns=now_ns,
                )
            self._connected = True
        except Exception:
            self.close()
            raise

    def step(self) -> dict[str, PiperState]:
        if not self._connected:
            raise TeleopSafetyError("standalone teleop is not connected")
        now_ns = self.clock_ns()
        states: dict[str, PiperState] = {}
        for name, session in self._sessions.items():
            new_target: OperatorTarget | None = None
            try:
                candidate = operator_target(
                    session.leader.sdk,
                    fallback_gripper_um=session.target.gripper_um,
                )
                if candidate.timestamp_s > session.last_sdk_timestamp_s:
                    new_target = candidate
            except TeleopSafetyError:
                new_target = None
            if new_target is not None:
                session.target = new_target
                session.last_sdk_timestamp_s = new_target.timestamp_s
                session.last_command_ns = now_ns
                _command_follower(
                    session.follower.sdk, session.target, self.gripper_effort
                )
            else:
                age_ns = now_ns - session.last_command_ns
                if age_ns > self.fail_after_ns:
                    raise TeleopSafetyError(f"{name} leader command timeout")
                if age_ns <= self.hold_after_ns:
                    _command_follower(
                        session.follower.sdk, session.target, self.gripper_effort
                    )
            states[name] = follower_state(
                session.follower.sdk,
                session.follower.identity,
                session.target,
                now_ns,
                firmware=session.follower.firmware,
            )
        return states

    def close(self) -> None:
        for endpoint in reversed(self._endpoints):
            try:
                endpoint.sdk.DisconnectPort()
            except Exception:
                pass
        self._endpoints.clear()
        self._sessions.clear()
        self._connected = False


class _ArmView:
    def __init__(self, runtime: "StandaloneTeleopRuntime", name: str) -> None:
        self.runtime = runtime
        self.name = name

    @property
    def latest(self) -> dict[str, object] | None:
        with self.runtime._lock:
            return self.runtime._latest.get(self.name)

    def status(self) -> dict[str, object]:
        with self.runtime._lock:
            latest = self.runtime._latest.get(self.name)
            return {
                "arm": self.name,
                "running": self.runtime._running,
                "error": self.runtime._error,
                "sequence": self.runtime._sequences[self.name],
                "latest_timestamp_ns": (
                    latest.get("timestamp_ns") if latest is not None else None
                ),
            }


class StandaloneTeleopRuntime:
    """Run control and Web publication in the same failure domain."""

    def __init__(
        self,
        pair_configs: Sequence[TeleopPairConfig],
        *,
        estimators: Mapping[str, _Estimator],
        hub: LatestEventHub,
        sdk_factory: Callable[..., object] = _default_sdk_factory,
        interface_resolver: Callable[[str], str] = discover_interface,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        control_rate_hz: float = 200.0,
        ui_rate_hz: float = 25.0,
        speed_ratio: int = 10,
        gripper_effort: int = 1000,
    ) -> None:
        validate_pair_configs(pair_configs)
        names = {config.name for config in pair_configs}
        if set(estimators) != names:
            raise ValueError("estimators must exactly match teleop pair names")
        if not 1 <= control_rate_hz <= 500:
            raise ValueError("control_rate_hz must be in [1, 500]")
        if not 1 <= ui_rate_hz <= control_rate_hz:
            raise ValueError("ui_rate_hz must be in [1, control_rate_hz]")
        if hub.queue_size < len(pair_configs):
            raise ValueError("event queue must hold one event per teleop pair")
        self.hub = hub
        self.estimators = dict(estimators)
        self.clock_ns = clock_ns
        self.control_period_ns = int(1e9 / control_rate_hz)
        self.ui_period_ns = int(1e9 / ui_rate_hz)
        self.coordinator = StandaloneTeleopCoordinator(
            pair_configs,
            sdk_factory=sdk_factory,
            interface_resolver=interface_resolver,
            clock_ns=clock_ns,
            speed_ratio=speed_ratio,
            gripper_effort=gripper_effort,
        )
        ordered_names = [config.name for config in pair_configs]
        self.workers = tuple(_ArmView(self, name) for name in ordered_names)
        self._latest: dict[str, dict[str, object]] = {}
        self._sequences = {name: 0 for name in ordered_names}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False
        self._error: str | None = None
        self._startup_error: Exception | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self._startup_error = None
        self._thread = threading.Thread(
            target=self._run,
            name="piperx-standalone-teleop-web",
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(timeout=15.0):
            self.stop()
            raise TeleopSafetyError("standalone teleop startup timed out")
        if self._startup_error is not None:
            self._thread.join(timeout=1.0)
            raise self._startup_error

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=4.0)

    def _run(self) -> None:
        with self._lock:
            self._running = True
            self._error = None
        last_publish_ns: int | None = None
        next_cycle_ns = self.clock_ns()
        try:
            self.coordinator.connect()
            self._ready.set()
            while not self._stop.is_set():
                states = self.coordinator.step()
                now_ns = self.clock_ns()
                if (
                    last_publish_ns is None
                    or now_ns - last_publish_ns >= self.ui_period_ns
                ):
                    for name, state in states.items():
                        estimate = self.estimators[name].estimate(state)
                        self._sequences[name] += 1
                        event = serialize_snapshot(
                            name, state, estimate, self._sequences[name]
                        )
                        event["runtime_mode"] = "standalone_teleop"
                        with self._lock:
                            self._latest[name] = event
                        self.hub.publish(event)
                    last_publish_ns = now_ns
                next_cycle_ns += self.control_period_ns
                delay_s = max(0.0, (next_cycle_ns - self.clock_ns()) * 1e-9)
                self._stop.wait(delay_s)
        except Exception as error:
            with self._lock:
                self._error = f"{type(error).__name__}: {error}"
                if not self._ready.is_set():
                    self._startup_error = error
        finally:
            self._ready.set()
            self.coordinator.close()
            with self._lock:
                self._running = False

    def status(self) -> dict[str, object]:
        return {
            "schema": SCHEMA_VERSION,
            "mode": "standalone_teleop",
            "arms": [worker.status() for worker in self.workers],
            "subscribers": self.hub.subscriber_count,
        }

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "schema": SCHEMA_VERSION,
                "mode": "standalone_teleop",
                "arms": dict(self._latest),
            }

    def healthy(self) -> bool:
        with self._lock:
            return self._running and self._error is None and len(self._latest) == 2
