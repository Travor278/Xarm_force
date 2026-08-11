"""Pinocchio rigid-body dynamics boundary for PiperX."""

from __future__ import annotations

import hashlib
import importlib
from pathlib import Path
from typing import Iterable

import numpy as np


class DynamicsError(RuntimeError):
    """Raised when the robot model or a dynamics result is unsafe to use."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _rotation_matrix_from_rpy(base_rpy: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in base_rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def gravity_in_base(base_rpy: Iterable[float]) -> np.ndarray:
    """Express world gravity in coordinates of a fixed, rotated robot base."""
    rotation_world_base = _rotation_matrix_from_rpy(base_rpy)
    return rotation_world_base.T @ np.array([0.0, 0.0, -9.81])


class PinocchioDynamics:
    """Validated six-joint RNEA backend loaded from an explicit URDF."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        base_rpy: Iterable[float] = (0.0, 0.0, 0.0),
    ) -> None:
        path = Path(urdf_path).expanduser().resolve(strict=True)
        try:
            pin = importlib.import_module("pinocchio")
        except ImportError as error:
            raise DynamicsError(
                "Pinocchio is required for PiperX dynamics; install package 'pin'"
            ) from error
        model = pin.buildModelFromUrdf(str(path))
        if model.nq != 6 or model.nv != 6:
            raise DynamicsError(
                f"PiperX model must contain six actuated joints, got nq={model.nq}, nv={model.nv}"
            )
        rpy = tuple(float(value) for value in base_rpy)
        if len(rpy) != 3 or not np.all(np.isfinite(rpy)):
            raise DynamicsError("base_rpy must contain three finite values")
        model.gravity.linear = gravity_in_base(rpy)
        self._pin = pin
        self.model = model
        self.data = model.createData()
        self.urdf_path = path
        self.urdf_sha256 = sha256_file(path)
        self.base_rpy = rpy

    def compute(
        self,
        q_rad: Iterable[float],
        qd_rad_s: Iterable[float],
        qdd_rad_s2: Iterable[float],
    ) -> np.ndarray:
        vectors = tuple(
            np.asarray(value, dtype=np.float64)
            for value in (q_rad, qd_rad_s, qdd_rad_s2)
        )
        if any(vector.shape != (6,) for vector in vectors):
            raise DynamicsError("q, qd, and qdd must each have shape (6,)")
        if not all(np.all(np.isfinite(vector)) for vector in vectors):
            raise DynamicsError("dynamics inputs must be finite")
        torque = np.asarray(
            self._pin.rnea(self.model, self.data, *vectors), dtype=np.float64
        ).reshape(-1)
        if torque.shape != (6,) or not np.all(np.isfinite(torque)):
            raise DynamicsError("RNEA returned an invalid six-joint torque")
        return torque.copy()
