"""Per-arm no-contact residual calibration for PiperX effort feedback."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np


SCHEMA_VERSION = 1
FEATURE_VERSION = "piperx-residual-v1"


class CalibrationMismatchError(RuntimeError):
    """Raised when a calibration cannot safely describe the runtime arm."""


def build_features(
    q_rad: Iterable[float] | np.ndarray,
    qd_rad_s: Iterable[float] | np.ndarray,
    *,
    friction_velocity_scale: float,
) -> np.ndarray:
    q = np.asarray(q_rad, dtype=np.float64)
    qd = np.asarray(qd_rad_s, dtype=np.float64)
    single = q.ndim == 1
    if single:
        q = q.reshape(1, -1)
        qd = qd.reshape(1, -1)
    if q.ndim != 2 or q.shape[1] != 6 or qd.shape != q.shape:
        raise ValueError("q and qd must have shape (6,) or (samples, 6)")
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
        raise ValueError("calibration features must be finite")
    if not np.isfinite(friction_velocity_scale) or friction_velocity_scale <= 0:
        raise ValueError("friction_velocity_scale must be positive and finite")
    chain = np.cumsum(q[:, 1:], axis=1)
    features = np.column_stack(
        (
            np.ones(q.shape[0]),
            np.sin(q),
            np.cos(q),
            np.sin(chain),
            np.cos(chain),
            np.tanh(qd / friction_velocity_scale),
        )
    )
    return features[0] if single else features


def split_contiguous_groups(
    group_ids: Iterable[object] | np.ndarray, *, validation_fraction: float
) -> tuple[np.ndarray, np.ndarray]:
    groups = np.asarray(group_ids)
    if groups.ndim != 1:
        raise ValueError("group_ids must be one-dimensional")
    if not 0 < validation_fraction < 1:
        raise ValueError("validation_fraction must be between zero and one")
    ordered_groups: list[object] = []
    for group in groups.tolist():
        if not ordered_groups or group != ordered_groups[-1]:
            if group in ordered_groups:
                raise ValueError("group_ids must form contiguous, non-repeating groups")
            ordered_groups.append(group)
    if len(ordered_groups) < 2:
        raise ValueError("calibration requires at least two contiguous groups")
    validation_count = max(1, int(np.ceil(len(ordered_groups) * validation_fraction)))
    validation_count = min(validation_count, len(ordered_groups) - 1)
    validation_groups = ordered_groups[-validation_count:]
    validation_mask = np.isin(groups, validation_groups)
    return ~validation_mask, validation_mask


def _error_metrics(error_nm: np.ndarray) -> dict[str, object]:
    return {
        "count": int(error_nm.shape[0]),
        "signed_mean_nm": np.mean(error_nm, axis=0).tolist(),
        "mae_nm": np.mean(np.abs(error_nm), axis=0).tolist(),
        "rmse_nm": np.sqrt(np.mean(np.square(error_nm), axis=0)).tolist(),
        "std_nm": np.std(error_nm, axis=0).tolist(),
        "p95_abs_nm": np.percentile(np.abs(error_nm), 95, axis=0).tolist(),
    }


@dataclass(frozen=True)
class CalibrationPrediction:
    bias_nm: np.ndarray
    valid: bool
    reason: str | None


@dataclass(frozen=True)
class CalibrationArtifact:
    adapter_serial: str
    urdf_sha256: str
    payload_sha256: str | None
    base_rpy: tuple[float, float, float]
    coefficients: np.ndarray
    ridge: float
    friction_velocity_scale: float
    metrics: Mapping[str, object]
    q_min_rad: np.ndarray
    q_max_rad: np.ndarray
    qd_min_rad_s: np.ndarray
    qd_max_rad_s: np.ndarray
    created_at: str
    source_log_sha256: str
    schema_version: int = SCHEMA_VERSION
    feature_version: str = FEATURE_VERSION

    def validate_runtime(
        self,
        adapter_serial: str,
        urdf_sha256: str,
        base_rpy: Iterable[float],
        payload_sha256: str | None = None,
    ) -> None:
        if adapter_serial != self.adapter_serial:
            raise CalibrationMismatchError(
                f"adapter serial mismatch: expected {self.adapter_serial!r}, got {adapter_serial!r}"
            )
        if urdf_sha256 != self.urdf_sha256:
            raise CalibrationMismatchError("URDF SHA-256 does not match calibration")
        if payload_sha256 != self.payload_sha256:
            raise CalibrationMismatchError("payload SHA-256 does not match calibration")
        runtime_rpy = np.asarray(tuple(base_rpy), dtype=np.float64)
        if runtime_rpy.shape != (3,) or not np.allclose(
            runtime_rpy, self.base_rpy, atol=1e-12, rtol=0.0
        ):
            raise CalibrationMismatchError("base orientation does not match calibration")

    def predict(
        self, q_rad: Iterable[float], qd_rad_s: Iterable[float]
    ) -> CalibrationPrediction:
        q = np.asarray(q_rad, dtype=np.float64)
        qd = np.asarray(qd_rad_s, dtype=np.float64)
        if q.shape != (6,) or qd.shape != (6,) or not (
            np.all(np.isfinite(q)) and np.all(np.isfinite(qd))
        ):
            return CalibrationPrediction(np.full(6, np.nan), False, "invalid_calibration_input")
        features = build_features(
            q, qd, friction_velocity_scale=self.friction_velocity_scale
        )
        bias = self.coefficients @ features
        inside = (
            np.all(q >= self.q_min_rad)
            and np.all(q <= self.q_max_rad)
            and np.all(qd >= self.qd_min_rad_s)
            and np.all(qd <= self.qd_max_rad_s)
        )
        return CalibrationPrediction(
            bias_nm=bias,
            valid=bool(inside),
            reason=None if inside else "outside_calibrated_workspace",
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "feature_version": self.feature_version,
            "adapter_serial": self.adapter_serial,
            "urdf_sha256": self.urdf_sha256,
            "payload_sha256": self.payload_sha256,
            "base_rpy": list(self.base_rpy),
            "coefficients": self.coefficients.tolist(),
            "ridge": self.ridge,
            "friction_velocity_scale": self.friction_velocity_scale,
            "metrics": dict(self.metrics),
            "workspace": {
                "q_min_rad": self.q_min_rad.tolist(),
                "q_max_rad": self.q_max_rad.tolist(),
                "qd_min_rad_s": self.qd_min_rad_s.tolist(),
                "qd_max_rad_s": self.qd_max_rad_s.tolist(),
            },
            "created_at": self.created_at,
            "source_log_sha256": self.source_log_sha256,
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> "CalibrationArtifact":
        if document.get("schema_version") != SCHEMA_VERSION:
            raise CalibrationMismatchError("unsupported calibration schema version")
        if document.get("feature_version") != FEATURE_VERSION:
            raise CalibrationMismatchError("unsupported calibration feature version")
        workspace = document["workspace"]
        if not isinstance(workspace, Mapping):
            raise CalibrationMismatchError("calibration workspace is malformed")
        artifact = cls(
            schema_version=int(document["schema_version"]),
            feature_version=str(document["feature_version"]),
            adapter_serial=str(document["adapter_serial"]),
            urdf_sha256=str(document["urdf_sha256"]),
            payload_sha256=(
                str(document["payload_sha256"])
                if document.get("payload_sha256") is not None
                else None
            ),
            base_rpy=tuple(float(value) for value in document["base_rpy"]),
            coefficients=np.asarray(document["coefficients"], dtype=np.float64),
            ridge=float(document["ridge"]),
            friction_velocity_scale=float(document["friction_velocity_scale"]),
            metrics=document["metrics"],
            q_min_rad=np.asarray(workspace["q_min_rad"], dtype=np.float64),
            q_max_rad=np.asarray(workspace["q_max_rad"], dtype=np.float64),
            qd_min_rad_s=np.asarray(workspace["qd_min_rad_s"], dtype=np.float64),
            qd_max_rad_s=np.asarray(workspace["qd_max_rad_s"], dtype=np.float64),
            created_at=str(document["created_at"]),
            source_log_sha256=str(document["source_log_sha256"]),
        )
        artifact._validate_shape()
        return artifact

    def _validate_shape(self) -> None:
        if self.coefficients.shape != (6, 29):
            raise CalibrationMismatchError("calibration coefficients must have shape (6, 29)")
        arrays = (self.q_min_rad, self.q_max_rad, self.qd_min_rad_s, self.qd_max_rad_s)
        if any(array.shape != (6,) for array in arrays):
            raise CalibrationMismatchError("calibration workspace arrays must have shape (6,)")
        if not all(np.all(np.isfinite(array)) for array in (*arrays, self.coefficients)):
            raise CalibrationMismatchError("calibration contains non-finite values")

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self.to_dict(), indent=2, sort_keys=True), encoding="utf-8"
        )
        os.replace(temporary, destination)

    @classmethod
    def load(cls, path: str | Path) -> "CalibrationArtifact":
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(document, Mapping):
            raise CalibrationMismatchError("calibration document must be an object")
        return cls.from_dict(document)


def fit_calibration(
    *,
    q_rad: np.ndarray,
    qd_rad_s: np.ndarray,
    tau_measured_nm: np.ndarray,
    tau_model_nm: np.ndarray,
    group_ids: np.ndarray,
    adapter_serial: str,
    urdf_sha256: str,
    payload_sha256: str | None = None,
    base_rpy: Iterable[float],
    source_log_sha256: str,
    ridge: float = 1e-3,
    validation_fraction: float = 0.2,
    friction_velocity_scale: float = 0.2,
) -> CalibrationArtifact:
    q = np.asarray(q_rad, dtype=np.float64)
    qd = np.asarray(qd_rad_s, dtype=np.float64)
    measured = np.asarray(tau_measured_nm, dtype=np.float64)
    model = np.asarray(tau_model_nm, dtype=np.float64)
    groups = np.asarray(group_ids)
    if q.ndim != 2 or q.shape[1:] != (6,) or qd.shape != q.shape:
        raise ValueError("q and qd must have shape (samples, 6)")
    if measured.shape != q.shape or model.shape != q.shape or groups.shape != (q.shape[0],):
        raise ValueError("torque arrays and group_ids must share the sample count")
    if q.shape[0] < 2 or not all(
        np.all(np.isfinite(array)) for array in (q, qd, measured, model)
    ):
        raise ValueError("calibration arrays need at least two finite samples")
    if not np.isfinite(ridge) or ridge <= 0:
        raise ValueError("ridge must be positive and finite")
    train_mask, validation_mask = split_contiguous_groups(
        groups, validation_fraction=validation_fraction
    )
    features = build_features(
        q, qd, friction_velocity_scale=friction_velocity_scale
    )
    target = measured - model
    x_train = features[train_mask]
    y_train = target[train_mask]
    regularizer = np.eye(features.shape[1]) * ridge
    regularizer[0, 0] = 0.0
    weights = np.linalg.solve(
        x_train.T @ x_train + regularizer, x_train.T @ y_train
    )
    prediction = features @ weights
    rpy = tuple(float(value) for value in base_rpy)
    if len(rpy) != 3 or not np.all(np.isfinite(rpy)):
        raise ValueError("base_rpy must contain three finite values")
    artifact = CalibrationArtifact(
        adapter_serial=adapter_serial,
        urdf_sha256=urdf_sha256,
        payload_sha256=payload_sha256,
        base_rpy=rpy,
        coefficients=weights.T,
        ridge=float(ridge),
        friction_velocity_scale=float(friction_velocity_scale),
        metrics={
            "train": _error_metrics(prediction[train_mask] - target[train_mask]),
            "validation": _error_metrics(
                prediction[validation_mask] - target[validation_mask]
            ),
        },
        q_min_rad=np.min(q, axis=0),
        q_max_rad=np.max(q, axis=0),
        qd_min_rad_s=np.min(qd, axis=0),
        qd_max_rad_s=np.max(qd, axis=0),
        created_at=datetime.now(timezone.utc).isoformat(),
        source_log_sha256=source_log_sha256,
    )
    artifact._validate_shape()
    return artifact
