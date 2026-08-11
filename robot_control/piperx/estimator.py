"""Composition of state validation, inverse dynamics, and calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .calibration import (
    CalibrationArtifact,
    CalibrationMismatchError,
    CalibrationPrediction,
)
from .dynamics import DynamicsError
from .filtering import DerivativeEstimate, VelocityDerivativeFilter
from .socketcan import PiperState


class DynamicsBackend(Protocol):
    urdf_sha256: str
    base_rpy: tuple[float, float, float]

    def compute(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray: ...


@dataclass(frozen=True)
class Estimate:
    interface: str
    adapter_serial: str
    timestamp_ns: int
    q_rad: np.ndarray
    qd_filtered_rad_s: np.ndarray
    qdd_rad_s2: np.ndarray
    tau_measured_nm: np.ndarray
    tau_model_nm: np.ndarray
    tau_bias_nm: np.ndarray
    tau_external_nm: np.ndarray
    valid: bool
    calibrated: bool
    reason: str | None


class ExternalTorqueEstimator:
    """Estimate follower external torque using the approved sign convention."""

    def __init__(
        self,
        dynamics: DynamicsBackend,
        derivative: VelocityDerivativeFilter,
        calibration: CalibrationArtifact | None,
    ) -> None:
        self.dynamics = dynamics
        self.derivative = derivative
        self.calibration = calibration

    @staticmethod
    def _nan() -> np.ndarray:
        return np.full(6, np.nan, dtype=np.float64)

    def _invalid_state(self, state: PiperState, reason: str) -> Estimate:
        q = np.asarray(state.q_rad, dtype=np.float64)
        qd = np.asarray(state.qd_rad_s, dtype=np.float64)
        return Estimate(
            interface=state.interface,
            adapter_serial=state.adapter_serial,
            timestamp_ns=state.timestamp_ns,
            q_rad=q,
            qd_filtered_rad_s=qd,
            qdd_rad_s2=self._nan(),
            tau_measured_nm=np.asarray(state.tau_measured_nm, dtype=np.float64),
            tau_model_nm=self._nan(),
            tau_bias_nm=self._nan(),
            tau_external_nm=self._nan(),
            valid=False,
            calibrated=self.calibration is not None,
            reason=reason,
        )

    def estimate(self, state: PiperState) -> Estimate:
        if not state.complete:
            return self._invalid_state(state, state.reason or "incomplete_feedback")
        if not state.fresh:
            return self._invalid_state(state, state.reason or "stale_feedback")
        q = np.asarray(state.q_rad, dtype=np.float64)
        measured = np.asarray(state.tau_measured_nm, dtype=np.float64)
        derivative: DerivativeEstimate = self.derivative.update(
            state.timestamp_ns, state.qd_rad_s
        )
        if not derivative.valid:
            invalid = self._invalid_state(
                state, derivative.reason or "invalid_state_derivative"
            )
            return Estimate(
                **{
                    **invalid.__dict__,
                    "qd_filtered_rad_s": derivative.qd_filtered_rad_s,
                    "qdd_rad_s2": derivative.qdd_rad_s2,
                }
            )
        try:
            model = self.dynamics.compute(
                q, derivative.qd_filtered_rad_s, derivative.qdd_rad_s2
            )
        except DynamicsError as error:
            return self._invalid_state(state, f"dynamics_error: {error}")

        calibrated = self.calibration is not None
        if self.calibration is None:
            prediction = CalibrationPrediction(np.zeros(6), True, None)
        else:
            try:
                self.calibration.validate_runtime(
                    state.adapter_serial,
                    self.dynamics.urdf_sha256,
                    self.dynamics.base_rpy,
                    getattr(self.dynamics, "payload_sha256", None),
                )
            except CalibrationMismatchError as error:
                return Estimate(
                    interface=state.interface,
                    adapter_serial=state.adapter_serial,
                    timestamp_ns=state.timestamp_ns,
                    q_rad=q,
                    qd_filtered_rad_s=derivative.qd_filtered_rad_s,
                    qdd_rad_s2=derivative.qdd_rad_s2,
                    tau_measured_nm=measured,
                    tau_model_nm=model,
                    tau_bias_nm=self._nan(),
                    tau_external_nm=self._nan(),
                    valid=False,
                    calibrated=True,
                    reason=f"calibration_mismatch: {error}",
                )
            prediction = self.calibration.predict(q, derivative.qd_filtered_rad_s)
        external = model + prediction.bias_nm - measured
        finite = all(
            np.all(np.isfinite(array))
            for array in (q, measured, model, prediction.bias_nm, external)
        )
        reason = prediction.reason
        if not finite:
            reason = reason or "nonfinite_estimate"
        return Estimate(
            interface=state.interface,
            adapter_serial=state.adapter_serial,
            timestamp_ns=state.timestamp_ns,
            q_rad=q,
            qd_filtered_rad_s=derivative.qd_filtered_rad_s,
            qdd_rad_s2=derivative.qdd_rad_s2,
            tau_measured_nm=measured,
            tau_model_nm=model,
            tau_bias_nm=prediction.bias_nm,
            tau_external_nm=external,
            valid=bool(prediction.valid and finite),
            calibrated=calibrated,
            reason=reason,
        )


def bounded_for_display(values: np.ndarray, limit_nm: float = 20.0) -> np.ndarray:
    """Bound presentation values without modifying raw estimate storage."""
    if limit_nm <= 0:
        raise ValueError("display limit must be positive")
    return np.clip(np.asarray(values, dtype=np.float64), -limit_nm, limit_nm)
