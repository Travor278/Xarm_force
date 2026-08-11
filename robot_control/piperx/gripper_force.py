"""Empirical Piper gripper feedback-torque to fingertip-force calibration."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from typing import Mapping

import numpy as np

from .socketcan import GripperTelemetry


SCHEMA_VERSION = 1
MODEL_VERSION = "piper-gripper-affine-v1"
_FAULT_REASONS = (
    (0, "low_voltage"),
    (1, "motor_overheat"),
    (2, "driver_overcurrent"),
    (3, "driver_overheat"),
    (4, "sensor_abnormal"),
    (5, "driver_error"),
)


class GripperForceMismatchError(RuntimeError):
    """Raised when a force artifact belongs to a different device."""


@dataclass(frozen=True)
class GripperForcePrediction:
    force_n: float | None
    valid: bool
    reason: str | None


@dataclass(frozen=True)
class GripperForceCalibration:
    adapter_serial: str
    direction: int
    slope_n_per_nm: float
    intercept_n: float
    travel_min_mm: float
    travel_max_mm: float
    torque_min_nm: float
    torque_max_nm: float
    sample_count: int
    validation_mae_n: float
    source_sha256: str
    created_at: str
    schema_version: int = SCHEMA_VERSION
    model_version: str = MODEL_VERSION

    def validate(self) -> None:
        finite = (
            self.slope_n_per_nm,
            self.intercept_n,
            self.travel_min_mm,
            self.travel_max_mm,
            self.torque_min_nm,
            self.torque_max_nm,
            self.validation_mae_n,
        )
        if not self.adapter_serial:
            raise ValueError("adapter serial must not be empty")
        if self.direction not in (-1, 1):
            raise ValueError("force direction must be -1 or 1")
        if not np.all(np.isfinite(finite)):
            raise ValueError("force calibration values must be finite")
        if self.slope_n_per_nm <= 0:
            raise ValueError("force calibration slope must be positive")
        if self.travel_min_mm > self.travel_max_mm:
            raise ValueError("force calibration travel range is invalid")
        if self.torque_min_nm > self.torque_max_nm:
            raise ValueError("force calibration torque range is invalid")
        if self.sample_count < 3 or self.validation_mae_n < 0:
            raise ValueError("force calibration metrics are invalid")

    def predict(
        self,
        telemetry: GripperTelemetry,
        *,
        fresh: bool,
        adapter_serial: str,
    ) -> GripperForcePrediction:
        if adapter_serial != self.adapter_serial:
            raise GripperForceMismatchError(
                f"adapter serial mismatch: expected {self.adapter_serial!r}, got {adapter_serial!r}"
            )
        if not fresh:
            return GripperForcePrediction(None, False, "stale")
        status = telemetry.status_code
        for bit, reason in _FAULT_REASONS:
            if status & (1 << bit):
                return GripperForcePrediction(None, False, reason)
        if not status & (1 << 6):
            return GripperForcePrediction(None, False, "not_enabled")
        values = (telemetry.travel_mm, telemetry.torque_nm)
        if not np.all(np.isfinite(values)):
            return GripperForcePrediction(None, False, "nonfinite_feedback")
        if not (
            self.travel_min_mm <= telemetry.travel_mm <= self.travel_max_mm
            and self.torque_min_nm <= telemetry.torque_nm <= self.torque_max_nm
        ):
            return GripperForcePrediction(None, False, "outside_calibrated_range")
        force = self.slope_n_per_nm * self.direction * telemetry.torque_nm + self.intercept_n
        if not np.isfinite(force):
            return GripperForcePrediction(None, False, "nonfinite_force")
        return GripperForcePrediction(max(0.0, float(force)), True, None)

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "model_version": self.model_version,
            "adapter_serial": self.adapter_serial,
            "direction": self.direction,
            "slope_n_per_nm": self.slope_n_per_nm,
            "intercept_n": self.intercept_n,
            "workspace": {
                "travel_min_mm": self.travel_min_mm,
                "travel_max_mm": self.travel_max_mm,
                "torque_min_nm": self.torque_min_nm,
                "torque_max_nm": self.torque_max_nm,
            },
            "metrics": {
                "sample_count": self.sample_count,
                "validation_mae_n": self.validation_mae_n,
            },
            "source_sha256": self.source_sha256,
            "created_at": self.created_at,
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> "GripperForceCalibration":
        if document.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported gripper force schema version")
        if document.get("model_version") != MODEL_VERSION:
            raise ValueError("unsupported gripper force model version")
        workspace = document.get("workspace")
        metrics = document.get("metrics")
        if not isinstance(workspace, Mapping) or not isinstance(metrics, Mapping):
            raise ValueError("gripper force artifact is malformed")
        artifact = cls(
            schema_version=int(document["schema_version"]),
            model_version=str(document["model_version"]),
            adapter_serial=str(document["adapter_serial"]),
            direction=int(document["direction"]),
            slope_n_per_nm=float(document["slope_n_per_nm"]),
            intercept_n=float(document["intercept_n"]),
            travel_min_mm=float(workspace["travel_min_mm"]),
            travel_max_mm=float(workspace["travel_max_mm"]),
            torque_min_nm=float(workspace["torque_min_nm"]),
            torque_max_nm=float(workspace["torque_max_nm"]),
            sample_count=int(metrics["sample_count"]),
            validation_mae_n=float(metrics["validation_mae_n"]),
            source_sha256=str(document["source_sha256"]),
            created_at=str(document["created_at"]),
        )
        artifact.validate()
        return artifact

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self.to_dict(), indent=2, sort_keys=True), encoding="utf-8"
        )
        os.replace(temporary, destination)

    @classmethod
    def load(cls, path: str | Path) -> "GripperForceCalibration":
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(document, Mapping):
            raise ValueError("gripper force artifact must be an object")
        return cls.from_dict(document)


def fit_gripper_force_calibration(
    *,
    torque_nm: np.ndarray,
    force_n: np.ndarray,
    travel_mm: np.ndarray,
    adapter_serial: str,
    source_sha256: str,
) -> GripperForceCalibration:
    torque = np.asarray(torque_nm, dtype=np.float64)
    force = np.asarray(force_n, dtype=np.float64)
    travel = np.asarray(travel_mm, dtype=np.float64)
    if torque.ndim != 1 or torque.shape != force.shape or torque.shape != travel.shape:
        raise ValueError("torque, force, and travel must be equal one-dimensional arrays")
    if torque.size < 5 or not all(
        np.all(np.isfinite(array)) for array in (torque, force, travel)
    ):
        raise ValueError("force calibration requires at least five finite samples")
    if np.ptp(torque) <= 1e-6 or np.ptp(force) <= 1e-6:
        raise ValueError("force and torque must vary during calibration")
    covariance = float(np.cov(torque, force, ddof=0)[0, 1])
    direction = 1 if covariance >= 0 else -1
    directed = direction * torque
    validation_mask = np.arange(torque.size) % 5 == 0
    train_mask = ~validation_mask
    design = np.column_stack((directed[train_mask], np.ones(np.count_nonzero(train_mask))))
    slope, intercept = np.linalg.lstsq(design, force[train_mask], rcond=None)[0]
    if slope <= 0 or not np.all(np.isfinite((slope, intercept))):
        raise ValueError("force calibration slope is not physically usable")
    predicted = slope * directed[validation_mask] + intercept
    mae = float(np.mean(np.abs(predicted - force[validation_mask])))
    artifact = GripperForceCalibration(
        adapter_serial=adapter_serial,
        direction=direction,
        slope_n_per_nm=float(slope),
        intercept_n=float(intercept),
        travel_min_mm=float(np.min(travel)),
        travel_max_mm=float(np.max(travel)),
        torque_min_nm=float(np.min(torque)),
        torque_max_nm=float(np.max(torque)),
        sample_count=int(torque.size),
        validation_mae_n=mae,
        source_sha256=source_sha256,
        created_at=datetime.now(timezone.utc).isoformat(),
    )
    artifact.validate()
    return artifact
