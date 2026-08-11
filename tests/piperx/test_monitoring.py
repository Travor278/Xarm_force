from __future__ import annotations

import json
import math

import numpy as np
import pytest

from robot_control.piperx.estimator import Estimate
from robot_control.piperx.gripper_force import fit_gripper_force_calibration
from robot_control.piperx.monitoring import (
    LatestEventHub,
    firmware_effort_status,
    serialize_snapshot,
    sse_message,
)
from robot_control.piperx.socketcan import DriverTelemetry, GripperTelemetry, PiperState


def _state(*, firmware="S-V1.9-0") -> PiperState:
    return PiperState(
        interface="can2",
        adapter_serial="serial-left",
        timestamp_ns=1_000_000_000,
        q_rad=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        qd_rad_s=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        tau_measured_nm=(0.5, 0.6, 0.7, 0.8, 0.9, 1.0),
        frequency_hz=198.5,
        complete=True,
        fresh=True,
        reason=None,
        current_a=(-1.0, -0.5, 0.0, 0.5, 1.0, 1.5),
        driver=(
            DriverTelemetry(49.8, 40, 35, 0x50, 0.1, 999_000_000),
            None, None, None, None, None,
        ),
        driver_fresh=(True, False, False, False, False, False),
        q_command_rad=(0.11, 0.22, 0.33, 0.44, 0.55, 0.66),
        command_fresh=True,
        firmware=firmware,
        gripper=GripperTelemetry(35.0, -0.5, 0xC0, 999_500_000),
        gripper_fresh=True,
    )


def _estimate(*, valid=True, external=None) -> Estimate:
    if external is None:
        external = (
            np.arange(6, dtype=float) - 2 if valid else np.full(6, np.nan)
        )
    return Estimate(
        interface="can2",
        adapter_serial="serial-left",
        timestamp_ns=1_000_000_000,
        q_rad=np.arange(6, dtype=float) / 10,
        qd_filtered_rad_s=np.arange(1, 7, dtype=float),
        qdd_rad_s2=np.zeros(6),
        tau_measured_nm=np.arange(6, dtype=float),
        tau_model_nm=np.arange(6, dtype=float) + 1,
        tau_bias_nm=np.full(6, 0.1),
        tau_external_nm=np.asarray(external, dtype=float),
        valid=valid,
        calibrated=True,
        reason=None if valid else "outside_calibrated_workspace",
    )


@pytest.mark.parametrize(
    ("version", "status", "verified", "factor"),
    [
        (None, "unknown", False, None),
        ("not-a-version", "unknown", False, None),
        ("S-V1.8-2", "legacy", False, 4.0),
        ("S-V1.8-1", "legacy", False, 4.0),
        ("S-V1.9-0", "current", True, 1.0),
    ],
)
def test_firmware_status_prevents_unverified_effort_scaling(version, status, verified, factor):
    result = firmware_effort_status(version)

    assert result.status == status
    assert result.scale_verified is verified
    assert result.joint_1_3_factor == factor


def test_snapshot_has_versioned_finite_six_joint_payload_and_status_bits():
    payload = serialize_snapshot("left", _state(), _estimate(), sequence=7)

    assert payload["schema"] == "piperx-monitor-v2"
    assert payload["arm"] == "left"
    assert payload["sequence"] == 7
    assert payload["current_a"] == pytest.approx([-1.0, -0.5, 0.0, 0.5, 1.0, 1.5])
    assert len(payload["tau_external_nm"]) == 6
    assert payload["tracking_error_rad"] == pytest.approx([0.01, 0.02, 0.03, 0.04, 0.05, 0.06])
    assert payload["driver"][0]["enabled"] is True
    assert payload["driver"][0]["collision"] is True
    assert payload["driver"][0]["fresh"] is True
    assert payload["firmware"] == {
        "version": "S-V1.9-0",
        "status": "current",
        "scale_verified": True,
        "joint_1_3_factor": 1.0,
    }
    assert payload["gripper"]["travel_mm"] == pytest.approx(35.0)
    assert payload["gripper"]["feedback_torque_nm"] == pytest.approx(-0.5)
    assert payload["gripper"]["enabled"] is True
    assert payload["gripper"]["homed"] is True
    assert payload["gripper"]["force_n"] is None
    assert payload["gripper"]["force_reason"] == "uncalibrated"
    assert "NaN" not in json.dumps(payload, allow_nan=False)


def test_invalid_snapshot_breaks_external_trace_and_sanitizes_nonfinite_values():
    estimate = _estimate(valid=False)
    estimate.tau_model_nm[2] = math.inf

    payload = serialize_snapshot("left", _state(firmware=None), estimate, sequence=8)

    assert payload["valid"] is False
    assert payload["reason"] == "outside_calibrated_workspace"
    assert payload["tau_external_nm"] == [None] * 6
    assert payload["tau_model_nm"][2] is None
    json.dumps(payload, allow_nan=False)


def test_workspace_invalid_snapshot_preserves_finite_extrapolated_torque():
    external = np.linspace(-0.5, 0.5, 6)

    payload = serialize_snapshot(
        "left",
        _state(),
        _estimate(valid=False, external=external),
        sequence=9,
    )

    assert payload["valid"] is False
    assert payload["reason"] == "outside_calibrated_workspace"
    assert payload["tau_external_nm"] == pytest.approx(external)
    json.dumps(payload, allow_nan=False)


def test_snapshot_includes_calibrated_gripper_force_and_fault_validity():
    calibration = fit_gripper_force_calibration(
        torque_nm=np.linspace(-0.1, -1.0, 20),
        force_n=np.linspace(5.0, 50.0, 20),
        travel_mm=np.linspace(10.0, 60.0, 20),
        adapter_serial="serial-left",
        source_sha256="a" * 64,
    )

    payload = serialize_snapshot(
        "left", _state(), _estimate(), sequence=10,
        gripper_calibration=calibration,
    )

    assert payload["gripper"]["force_n"] == pytest.approx(25.0)
    assert payload["gripper"]["force_valid"] is True
    assert payload["gripper"]["force_calibrated"] is True
    assert payload["gripper"]["force_reason"] is None


def test_latest_event_hub_drops_old_event_for_slow_subscriber():
    hub = LatestEventHub(queue_size=1)
    subscriber = hub.subscribe()

    hub.publish({"sequence": 1})
    hub.publish({"sequence": 2})

    assert subscriber.get_nowait() == {"sequence": 2}
    hub.unsubscribe(subscriber)
    assert hub.subscriber_count == 0


def test_sse_message_is_strict_json_with_event_identity():
    message = sse_message("snapshot", {"sequence": 3}, event_id=3)

    assert message == 'id: 3\nevent: snapshot\ndata: {"sequence":3}\n\n'
