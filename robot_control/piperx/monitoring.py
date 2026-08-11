"""Finite snapshots and bounded fan-out for the passive web monitor."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import queue
import re
import threading
from typing import Any

import numpy as np

from .estimator import Estimate
from .socketcan import DriverTelemetry, PiperState


SCHEMA_VERSION = "piperx-monitor-v1"
_FIRMWARE_PATTERN = re.compile(r"^S-V(\d+)\.(\d+)-(\d+)$")


@dataclass(frozen=True)
class FirmwareEffortStatus:
    version: str | None
    status: str
    scale_verified: bool
    joint_1_3_factor: float | None


def firmware_effort_status(version: str | None) -> FirmwareEffortStatus:
    """Classify whether SDK fixed-coefficient effort needs legacy correction."""
    match = _FIRMWARE_PATTERN.fullmatch(version or "")
    if match is None:
        return FirmwareEffortStatus(version, "unknown", False, None)
    parsed = tuple(int(part) for part in match.groups())
    if parsed <= (1, 8, 2):
        return FirmwareEffortStatus(version, "legacy", False, 4.0)
    return FirmwareEffortStatus(version, "current", True, 1.0)


def _finite_scalar(value: Any) -> float | int | str | bool | None:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, np.integer)):
        return int(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _finite_list(values: Any) -> list[float | int | None]:
    return [_finite_scalar(value) for value in values]


def _driver_payload(
    driver: DriverTelemetry | None, fresh: bool
) -> dict[str, object] | None:
    if driver is None:
        return None
    status = driver.status_code
    return {
        "voltage_v": _finite_scalar(driver.voltage_v),
        "foc_temp_c": driver.foc_temp_c,
        "motor_temp_c": driver.motor_temp_c,
        "bus_current_a": _finite_scalar(driver.bus_current_a),
        "status_code": status,
        "voltage_low": bool(status & (1 << 0)),
        "motor_overheat": bool(status & (1 << 1)),
        "overcurrent": bool(status & (1 << 2)),
        "driver_overheat": bool(status & (1 << 3)),
        "collision": bool(status & (1 << 4)),
        "driver_error": bool(status & (1 << 5)),
        "enabled": bool(status & (1 << 6)),
        "stall": bool(status & (1 << 7)),
        "fresh": bool(fresh),
        "timestamp_ns": driver.timestamp_ns,
    }


def serialize_snapshot(
    arm_name: str,
    state: PiperState,
    estimate: Estimate,
    sequence: int,
    *,
    firmware_override: str | None = None,
) -> dict[str, object]:
    """Convert one matched state/estimate pair to strict JSON-compatible data."""
    version = firmware_override or state.firmware
    firmware = firmware_effort_status(version)
    q_command = (
        _finite_list(state.q_command_rad) if state.q_command_rad is not None else None
    )
    tracking_error = None
    if state.q_command_rad is not None:
        tracking_error = _finite_list(
            np.asarray(state.q_command_rad, dtype=np.float64)
            - np.asarray(state.q_rad, dtype=np.float64)
        )
    external = _finite_list(estimate.tau_external_nm)
    return {
        "schema": SCHEMA_VERSION,
        "arm": arm_name,
        "sequence": int(sequence),
        "interface": state.interface,
        "adapter_serial": state.adapter_serial,
        "timestamp_ns": int(state.timestamp_ns),
        "frequency_hz": _finite_scalar(state.frequency_hz),
        "valid": bool(estimate.valid),
        "calibrated": bool(estimate.calibrated),
        "reason": estimate.reason,
        "q_rad": _finite_list(state.q_rad),
        "qd_rad_s": _finite_list(estimate.qd_filtered_rad_s),
        "qdd_rad_s2": _finite_list(estimate.qdd_rad_s2),
        "current_a": _finite_list(state.current_a),
        "tau_effort_nm": _finite_list(state.tau_measured_nm),
        "tau_model_nm": _finite_list(estimate.tau_model_nm),
        "tau_bias_nm": _finite_list(estimate.tau_bias_nm),
        "tau_external_nm": external,
        "q_command_rad": q_command,
        "tracking_error_rad": tracking_error,
        "command_fresh": bool(state.command_fresh),
        "driver": [
            _driver_payload(driver, fresh)
            for driver, fresh in zip(state.driver, state.driver_fresh, strict=True)
        ],
        "firmware": {
            "version": firmware.version,
            "status": firmware.status,
            "scale_verified": firmware.scale_verified,
            "joint_1_3_factor": firmware.joint_1_3_factor,
        },
    }


class LatestEventHub:
    """Fan out newest-state events without letting clients block acquisition."""

    def __init__(self, queue_size: int = 1) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        self._queue_size = queue_size
        self._subscribers: set[queue.Queue[dict[str, object]]] = set()
        self._lock = threading.Lock()

    @property
    def subscriber_count(self) -> int:
        with self._lock:
            return len(self._subscribers)

    @property
    def queue_size(self) -> int:
        return self._queue_size

    def subscribe(self) -> queue.Queue[dict[str, object]]:
        subscriber: queue.Queue[dict[str, object]] = queue.Queue(self._queue_size)
        with self._lock:
            self._subscribers.add(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[dict[str, object]]) -> None:
        with self._lock:
            self._subscribers.discard(subscriber)

    def publish(self, event: dict[str, object]) -> None:
        with self._lock:
            subscribers = tuple(self._subscribers)
        for subscriber in subscribers:
            if subscriber.full():
                try:
                    subscriber.get_nowait()
                except queue.Empty:
                    pass
            try:
                subscriber.put_nowait(event)
            except queue.Full:
                pass


def sse_message(event: str, data: object, event_id: int | None = None) -> str:
    """Encode one strict-JSON server-sent event."""
    lines: list[str] = []
    if event_id is not None:
        lines.append(f"id: {event_id}")
    lines.append(f"event: {event}")
    lines.append(
        "data: "
        + json.dumps(
            data,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
    )
    return "\n".join(lines) + "\n\n"
