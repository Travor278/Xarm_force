from __future__ import annotations

import numpy as np
import pytest

from robot_control.piperx.gripper_force import (
    GripperForceCalibration,
    GripperForceMismatchError,
    fit_gripper_force_calibration,
)
from robot_control.piperx.socketcan import GripperTelemetry


def _fit() -> GripperForceCalibration:
    torque = np.linspace(-0.1, -1.0, 60)
    force = 42.0 * (-torque) + 1.5
    travel = np.linspace(10.0, 60.0, torque.size)
    return fit_gripper_force_calibration(
        torque_nm=torque,
        force_n=force,
        travel_mm=travel,
        adapter_serial="follower-left",
        source_sha256="a" * 64,
    )


def _sample(*, torque=-0.5, travel=35.0, status=0xC0):
    return GripperTelemetry(
        travel_mm=travel,
        torque_nm=torque,
        status_code=status,
        timestamp_ns=123,
    )


def test_fit_recovers_negative_torque_to_positive_force_mapping():
    artifact = _fit()
    prediction = artifact.predict(
        _sample(torque=-0.5), fresh=True, adapter_serial="follower-left"
    )

    assert artifact.direction == -1
    assert artifact.slope_n_per_nm == pytest.approx(42.0)
    assert artifact.intercept_n == pytest.approx(1.5)
    assert artifact.validation_mae_n < 1e-10
    assert prediction.valid
    assert prediction.force_n == pytest.approx(22.5)


def test_calibration_round_trip_preserves_prediction(tmp_path):
    artifact = _fit()
    path = tmp_path / "gripper-force.json"
    artifact.save(path)

    loaded = GripperForceCalibration.load(path)

    assert loaded.predict(
        _sample(), fresh=True, adapter_serial="follower-left"
    ).force_n == pytest.approx(artifact.predict(
        _sample(), fresh=True, adapter_serial="follower-left"
    ).force_n)


@pytest.mark.parametrize(
    ("sample", "fresh", "serial", "reason"),
    [
        (_sample(), False, "follower-left", "stale"),
        (_sample(status=0x80), True, "follower-left", "not_enabled"),
        (_sample(status=0xC1), True, "follower-left", "low_voltage"),
        (_sample(status=0xD0), True, "follower-left", "sensor_abnormal"),
        (_sample(travel=70.0), True, "follower-left", "outside_calibrated_range"),
        (_sample(torque=-1.2), True, "follower-left", "outside_calibrated_range"),
    ],
)
def test_prediction_fails_closed_for_invalid_feedback(sample, fresh, serial, reason):
    prediction = _fit().predict(sample, fresh=fresh, adapter_serial=serial)

    assert not prediction.valid
    assert prediction.force_n is None
    assert prediction.reason == reason


def test_prediction_rejects_adapter_identity_mismatch():
    with pytest.raises(GripperForceMismatchError, match="adapter serial"):
        _fit().predict(_sample(), fresh=True, adapter_serial="other")


def test_fit_rejects_noninformative_force_data():
    with pytest.raises(ValueError, match="vary"):
        fit_gripper_force_calibration(
            torque_nm=np.ones(10),
            force_n=np.ones(10),
            travel_mm=np.ones(10) * 30,
            adapter_serial="left",
            source_sha256="a" * 64,
        )
