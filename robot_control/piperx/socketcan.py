"""Stable CAN discovery and receive-only PiperX state assembly."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import re
import socket
import struct
import subprocess
import time
from typing import Callable, Mapping, Protocol

import numpy as np

from .protocol import (
    FirmwareFragment,
    HighSpeedSample,
    JointCommandPair,
    LowSpeedSample,
    decode_frame,
)


CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FLAGS = 0xE0000000
_CAN_FRAME = struct.Struct("=IB3x8s")


class CanDiscoveryError(RuntimeError):
    """Raised when a stable adapter serial cannot identify one interface."""


class CanReceiveError(RuntimeError):
    """Raised when a kernel CAN frame is malformed."""


def _udev_properties(path: Path) -> dict[str, str]:
    result = subprocess.run(
        ["udevadm", "info", "--query=property", f"--path={path}"],
        check=True,
        capture_output=True,
        text=True,
    )
    properties: dict[str, str] = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            properties[key] = value
    return properties


def discover_interface(
    serial: str,
    sys_class_net: str | Path = "/sys/class/net",
    property_loader: Callable[[Path], Mapping[str, str]] = _udev_properties,
) -> str:
    """Resolve exactly one network interface using a USB adapter serial."""
    if not serial:
        raise CanDiscoveryError("adapter serial must not be empty")
    root = Path(sys_class_net)
    matches: list[str] = []
    for interface_path in sorted(root.iterdir(), key=lambda path: path.name):
        try:
            properties = property_loader(interface_path)
        except (OSError, subprocess.SubprocessError):
            continue
        if serial in (
            properties.get("ID_SERIAL_SHORT"),
            properties.get("ID_SERIAL"),
        ):
            matches.append(interface_path.name)
    if not matches:
        raise CanDiscoveryError(f"CAN adapter serial {serial!r} was not found")
    if len(matches) != 1:
        raise CanDiscoveryError(
            f"multiple CAN interfaces match adapter serial {serial!r}: {matches}"
        )
    return matches[0]


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    payload: bytes
    timestamp_ns: int


class _SocketLike(Protocol):
    def bind(self, address: tuple[str]) -> None: ...
    def settimeout(self, timeout: float) -> None: ...
    def recv(self, size: int) -> bytes: ...
    def close(self) -> None: ...


class ReadOnlySocketCan:
    """A deliberately receive-only raw SocketCAN handle."""

    def __init__(
        self,
        interface: str,
        *,
        timeout_s: float = 1.0,
        socket_factory: Callable[..., _SocketLike] = socket.socket,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        af_can = getattr(socket, "AF_CAN", 29)
        can_raw = getattr(socket, "CAN_RAW", 1)
        self._socket = socket_factory(af_can, socket.SOCK_RAW, can_raw)
        self._socket.bind((interface,))
        self._socket.settimeout(timeout_s)
        self._clock_ns = clock_ns

    def recv_frame(self) -> CanFrame:
        packet = self._socket.recv(_CAN_FRAME.size)
        if len(packet) != _CAN_FRAME.size:
            raise CanReceiveError(
                f"kernel CAN frame must contain 16 bytes, got {len(packet)}"
            )
        raw_can_id, data_length, payload = _CAN_FRAME.unpack(packet)
        if data_length > 8:
            raise CanReceiveError(f"invalid CAN payload length {data_length}")
        timestamp_ns = self._clock_ns()
        if raw_can_id & CAN_FRAME_FLAGS:
            return CanFrame(can_id=-1, payload=b"", timestamp_ns=timestamp_ns)
        return CanFrame(
            can_id=raw_can_id & CAN_EFF_MASK,
            payload=payload[:data_length],
            timestamp_ns=timestamp_ns,
        )

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "ReadOnlySocketCan":
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close()


@dataclass(frozen=True)
class DriverTelemetry:
    voltage_v: float
    foc_temp_c: int
    motor_temp_c: int
    status_code: int
    bus_current_a: float
    timestamp_ns: int


@dataclass(frozen=True)
class PiperState:
    interface: str
    adapter_serial: str
    timestamp_ns: int
    q_rad: tuple[float, ...]
    qd_rad_s: tuple[float, ...]
    tau_measured_nm: tuple[float, ...]
    frequency_hz: float | None
    complete: bool
    fresh: bool
    reason: str | None
    current_a: tuple[float, ...] = (float("nan"),) * 6
    driver: tuple[DriverTelemetry | None, ...] = (None,) * 6
    driver_fresh: tuple[bool, ...] = (False,) * 6
    q_command_rad: tuple[float, ...] | None = None
    command_fresh: bool = False
    firmware: str | None = None


class PiperStateAssembler:
    """Assemble one coherent state after every required feedback ID advances."""

    def __init__(
        self,
        interface: str,
        adapter_serial: str,
        *,
        max_skew_ns: int = 10_000_000,
        stale_after_ns: int = 100_000_000,
        auxiliary_stale_after_ns: int = 500_000_000,
    ) -> None:
        self.interface = interface
        self.adapter_serial = adapter_serial
        self.max_skew_ns = max_skew_ns
        self.stale_after_ns = stale_after_ns
        self.auxiliary_stale_after_ns = auxiliary_stale_after_ns
        self._q = np.zeros(6, dtype=np.float64)
        self._qd = np.zeros(6, dtype=np.float64)
        self._current = np.zeros(6, dtype=np.float64)
        self._tau = np.zeros(6, dtype=np.float64)
        self._driver: list[DriverTelemetry | None] = [None] * 6
        self._driver_timestamps = np.zeros(6, dtype=np.int64)
        self._q_command = np.zeros(6, dtype=np.float64)
        self._command_timestamps = np.zeros(3, dtype=np.int64)
        self._firmware_buffer = bytearray()
        self._firmware: str | None = None
        self._position_revision = np.zeros(3, dtype=np.int64)
        self._motor_revision = np.zeros(6, dtype=np.int64)
        self._emitted_position_revision = np.zeros(3, dtype=np.int64)
        self._emitted_motor_revision = np.zeros(6, dtype=np.int64)
        self._position_timestamps = np.zeros(3, dtype=np.int64)
        self._motor_timestamps = np.zeros(6, dtype=np.int64)
        self._last_state: PiperState | None = None
        self._previous_emit_ns: int | None = None

    def update(self, can_id: int, payload: bytes, timestamp_ns: int) -> PiperState | None:
        decoded = decode_frame(can_id, payload)
        if decoded is None:
            return None
        if isinstance(decoded, HighSpeedSample):
            index = decoded.joint_index
            self._qd[index] = decoded.velocity_rad_s
            self._current[index] = decoded.current_a
            self._tau[index] = decoded.effort_nm
            self._motor_revision[index] += 1
            self._motor_timestamps[index] = timestamp_ns
        elif isinstance(decoded, LowSpeedSample):
            index = decoded.joint_index
            self._driver[index] = DriverTelemetry(
                voltage_v=decoded.voltage_v,
                foc_temp_c=decoded.foc_temp_c,
                motor_temp_c=decoded.motor_temp_c,
                status_code=decoded.status_code,
                bus_current_a=decoded.bus_current_a,
                timestamp_ns=timestamp_ns,
            )
            self._driver_timestamps[index] = timestamp_ns
            return None
        elif isinstance(decoded, JointCommandPair):
            first = decoded.first_joint
            self._q_command[first : first + 2] = decoded.positions_rad
            self._command_timestamps[first // 2] = timestamp_ns
            return None
        elif isinstance(decoded, FirmwareFragment):
            self._firmware_buffer.extend(decoded.data)
            if len(self._firmware_buffer) > 64:
                del self._firmware_buffer[:-64]
            match = re.search(rb"S-V\d+(?:\.\d+)*(?:-\d+)?", self._firmware_buffer)
            if match is not None:
                self._firmware = match.group().decode("ascii")
            return None
        else:
            first_joint, positions = decoded
            pair_index = first_joint // 2
            self._q[first_joint : first_joint + 2] = positions
            self._position_revision[pair_index] += 1
            self._position_timestamps[pair_index] = timestamp_ns

        if not np.all(self._position_revision > self._emitted_position_revision):
            return None
        if not np.all(self._motor_revision > self._emitted_motor_revision):
            return None
        timestamps = np.concatenate((self._position_timestamps, self._motor_timestamps))
        if int(np.max(timestamps) - np.min(timestamps)) > self.max_skew_ns:
            return None
        if not np.all(np.isfinite(np.concatenate((self._q, self._qd, self._tau)))):
            return None

        emit_ns = int(np.max(timestamps))
        driver_fresh = tuple(
            bool(timestamp > 0 and emit_ns - timestamp <= self.auxiliary_stale_after_ns)
            for timestamp in self._driver_timestamps
        )
        have_complete_command = bool(np.all(self._command_timestamps > 0))
        command_fresh = bool(
            have_complete_command
            and emit_ns - int(np.min(self._command_timestamps))
            <= self.auxiliary_stale_after_ns
        )
        frequency_hz = None
        if self._previous_emit_ns is not None and emit_ns > self._previous_emit_ns:
            frequency_hz = 1e9 / (emit_ns - self._previous_emit_ns)
        state = PiperState(
            interface=self.interface,
            adapter_serial=self.adapter_serial,
            timestamp_ns=emit_ns,
            q_rad=tuple(float(value) for value in self._q),
            qd_rad_s=tuple(float(value) for value in self._qd),
            current_a=tuple(float(value) for value in self._current),
            tau_measured_nm=tuple(float(value) for value in self._tau),
            frequency_hz=frequency_hz,
            complete=True,
            fresh=True,
            reason=None,
            driver=tuple(self._driver),
            driver_fresh=driver_fresh,
            q_command_rad=(
                tuple(float(value) for value in self._q_command)
                if have_complete_command
                else None
            ),
            command_fresh=command_fresh,
            firmware=self._firmware,
        )
        self._emitted_position_revision[:] = self._position_revision
        self._emitted_motor_revision[:] = self._motor_revision
        self._previous_emit_ns = emit_ns
        self._last_state = state
        return state

    def snapshot(self, now_ns: int) -> PiperState | None:
        """Return the last state, marking it invalid when feedback has stopped."""
        if self._last_state is None:
            return None
        if now_ns - self._last_state.timestamp_ns <= self.stale_after_ns:
            return self._last_state
        return replace(self._last_state, fresh=False, reason="stale_feedback")
