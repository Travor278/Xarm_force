from __future__ import annotations

import numpy as np
import pytest

from robot_control.piperx.calibration import CalibrationPrediction
from robot_control.piperx.estimator import ExternalTorqueEstimator
from robot_control.piperx.filtering import DerivativeEstimate
from robot_control.piperx.socketcan import PiperState


class _Dynamics:
    urdf_sha256 = "a" * 64
    base_rpy = (0.0, 0.0, 0.0)

    def __init__(self, torque):
        self.torque = np.asarray(torque, dtype=float)

    def compute(self, q, qd, qdd):
        assert np.asarray(q).shape == (6,)
        assert np.asarray(qd).shape == (6,)
        assert np.asarray(qdd).shape == (6,)
        return self.torque.copy()


class _Derivative:
    def __init__(self, valid=True, reason=None):
        self.valid = valid
        self.reason = reason

    def update(self, _timestamp_ns, qd):
        return DerivativeEstimate(
            qd_filtered_rad_s=np.asarray(qd, dtype=float),
            qdd_rad_s2=np.zeros(6),
            valid=self.valid,
            reason=self.reason,
        )


class _Calibration:
    def __init__(self, bias, valid=True, reason=None):
        self.bias = np.asarray(bias, dtype=float)
        self.valid = valid
        self.reason = reason
        self.validated = None

    def validate_runtime(self, serial, urdf_hash, base_rpy):
        self.validated = (serial, urdf_hash, tuple(base_rpy))

    def predict(self, _q, _qd):
        return CalibrationPrediction(self.bias.copy(), self.valid, self.reason)


def _state(*, measured=None, complete=True, fresh=True, reason=None):
    return PiperState(
        interface="can2",
        adapter_serial="serial-left",
        timestamp_ns=1_000_000_000,
        q_rad=(0.0,) * 6,
        qd_rad_s=(0.0,) * 6,
        tau_measured_nm=tuple(
            np.asarray(np.zeros(6) if measured is None else measured, dtype=float)
        ),
        frequency_hz=200.0,
        complete=complete,
        fresh=fresh,
        reason=reason,
    )


def test_estimator_uses_documented_external_torque_sign():
    model = np.full(6, 2.0)
    bias = np.full(6, 0.5)
    measured = np.array([3.0, 1.0, 2.5, 2.0, 4.0, 0.0])
    calibration = _Calibration(bias)
    estimator = ExternalTorqueEstimator(
        _Dynamics(model), _Derivative(), calibration
    )

    estimate = estimator.estimate(_state(measured=measured))

    assert estimate.valid
    assert estimate.calibrated
    assert estimate.tau_external_nm == pytest.approx(
        [-0.5, 1.5, 0.0, 0.5, -1.5, 2.5]
    )
    assert calibration.validated == (
        "serial-left",
        "a" * 64,
        (0.0, 0.0, 0.0),
    )


def test_estimator_preserves_measured_diagnostics_for_stale_state():
    measured = np.arange(6, dtype=float)
    estimator = ExternalTorqueEstimator(
        _Dynamics(np.zeros(6)), _Derivative(), _Calibration(np.zeros(6))
    )

    estimate = estimator.estimate(
        _state(measured=measured, fresh=False, reason="stale_feedback")
    )

    assert not estimate.valid
    assert estimate.reason == "stale_feedback"
    assert estimate.tau_measured_nm == pytest.approx(measured)
    assert np.all(np.isnan(estimate.tau_external_nm))


def test_estimator_propagates_derivative_invalidity():
    estimator = ExternalTorqueEstimator(
        _Dynamics(np.zeros(6)),
        _Derivative(valid=False, reason="acceleration_limit"),
        _Calibration(np.zeros(6)),
    )

    estimate = estimator.estimate(_state())

    assert not estimate.valid
    assert estimate.reason == "acceleration_limit"
    assert np.all(np.isnan(estimate.tau_model_nm))


def test_estimator_reports_workspace_invalidity_without_hiding_raw_result():
    estimator = ExternalTorqueEstimator(
        _Dynamics(np.full(6, 2.0)),
        _Derivative(),
        _Calibration(np.full(6, 0.5), False, "outside_calibrated_workspace"),
    )

    estimate = estimator.estimate(_state(measured=np.ones(6)))

    assert not estimate.valid
    assert estimate.reason == "outside_calibrated_workspace"
    assert estimate.tau_external_nm == pytest.approx(np.full(6, 1.5))


def test_uncalibrated_estimator_outputs_model_residual_for_recording():
    estimator = ExternalTorqueEstimator(
        _Dynamics(np.full(6, 2.0)), _Derivative(), calibration=None
    )

    estimate = estimator.estimate(_state(measured=np.ones(6)))

    assert estimate.valid
    assert not estimate.calibrated
    assert estimate.tau_bias_nm == pytest.approx(np.zeros(6))
    assert estimate.tau_external_nm == pytest.approx(np.ones(6))

