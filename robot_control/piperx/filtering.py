"""Bounded velocity filtering and acceleration estimation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class DerivativeEstimate:
    qd_filtered_rad_s: np.ndarray
    qdd_rad_s2: np.ndarray
    valid: bool
    reason: str | None


class VelocityDerivativeFilter:
    """Low-pass motor velocity and its derivative with fail-closed bounds."""

    def __init__(
        self,
        *,
        velocity_time_constant_s: float = 0.02,
        acceleration_time_constant_s: float = 0.04,
        startup_samples: int = 3,
        max_gap_s: float = 0.1,
        max_acceleration_rad_s2: Iterable[float] = (40.0,) * 6,
    ) -> None:
        if velocity_time_constant_s < 0 or acceleration_time_constant_s < 0:
            raise ValueError("filter time constants must be nonnegative")
        if startup_samples < 2 or max_gap_s <= 0:
            raise ValueError("startup_samples must be >= 2 and max_gap_s must be positive")
        limits = np.asarray(max_acceleration_rad_s2, dtype=np.float64)
        if limits.shape != (6,) or not np.all(np.isfinite(limits)) or np.any(limits <= 0):
            raise ValueError("max_acceleration_rad_s2 must be six positive finite values")
        self.velocity_time_constant_s = velocity_time_constant_s
        self.acceleration_time_constant_s = acceleration_time_constant_s
        self.startup_samples = startup_samples
        self.max_gap_s = max_gap_s
        self.max_acceleration_rad_s2 = limits
        self._timestamp_ns: int | None = None
        self._qd_filtered = np.zeros(6, dtype=np.float64)
        self._qdd_filtered = np.zeros(6, dtype=np.float64)
        self._sample_count = 0

    @staticmethod
    def _alpha(dt_s: float, time_constant_s: float) -> float:
        if time_constant_s == 0:
            return 1.0
        return dt_s / (time_constant_s + dt_s)

    @staticmethod
    def _result(qd: np.ndarray, qdd: np.ndarray, valid: bool, reason: str | None) -> DerivativeEstimate:
        return DerivativeEstimate(qd.copy(), qdd.copy(), valid, reason)

    def _restart(self, timestamp_ns: int, qd: np.ndarray) -> None:
        self._timestamp_ns = timestamp_ns
        self._qd_filtered = qd.copy()
        self._qdd_filtered.fill(0.0)
        self._sample_count = 1

    def update(self, timestamp_ns: int, qd_rad_s: Iterable[float]) -> DerivativeEstimate:
        qd = np.asarray(qd_rad_s, dtype=np.float64)
        if qd.shape != (6,) or not np.all(np.isfinite(qd)):
            return self._result(qd, np.zeros(6), False, "nonfinite_velocity")
        if self._timestamp_ns is None:
            self._restart(timestamp_ns, qd)
            return self._result(self._qd_filtered, self._qdd_filtered, False, "derivative_startup")

        delta_ns = timestamp_ns - self._timestamp_ns
        if delta_ns <= 0:
            return self._result(self._qd_filtered, self._qdd_filtered, False, "non_monotonic_timestamp")
        dt_s = delta_ns * 1e-9
        if dt_s > self.max_gap_s:
            self._restart(timestamp_ns, qd)
            return self._result(self._qd_filtered, self._qdd_filtered, False, "excessive_timestamp_gap")

        velocity_alpha = self._alpha(dt_s, self.velocity_time_constant_s)
        new_qd_filtered = self._qd_filtered + velocity_alpha * (qd - self._qd_filtered)
        raw_qdd = (new_qd_filtered - self._qd_filtered) / dt_s
        acceleration_alpha = self._alpha(dt_s, self.acceleration_time_constant_s)
        new_qdd_filtered = self._qdd_filtered + acceleration_alpha * (
            raw_qdd - self._qdd_filtered
        )
        self._timestamp_ns = timestamp_ns
        self._qd_filtered = new_qd_filtered
        self._qdd_filtered = new_qdd_filtered
        self._sample_count += 1
        if np.any(np.abs(new_qdd_filtered) > self.max_acceleration_rad_s2):
            self._sample_count = 1
            return self._result(new_qd_filtered, new_qdd_filtered, False, "acceleration_limit")
        if self._sample_count < self.startup_samples:
            return self._result(new_qd_filtered, new_qdd_filtered, False, "derivative_startup")
        return self._result(new_qd_filtered, new_qdd_filtered, True, None)
