"""Validated rigid end-effector payloads for PiperX inverse dynamics."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Mapping

import numpy as np


SCHEMA_VERSION = 1


class PayloadError(ValueError):
    """Raised when a payload document cannot safely enter the dynamics model."""


@dataclass(frozen=True)
class RigidPayload:
    name: str
    parent_joint: str
    reference_opening_m: float
    mass_kg: float
    com_m: np.ndarray
    inertia_kg_m2: np.ndarray
    source: Mapping[str, str]
    schema_version: int = SCHEMA_VERSION

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> "RigidPayload":
        if document.get("schema_version") != SCHEMA_VERSION:
            raise PayloadError("unsupported payload schema version")
        source = document.get("source")
        if not isinstance(source, Mapping):
            raise PayloadError("payload source must be an object")
        try:
            payload = cls(
                schema_version=int(document["schema_version"]),
                name=str(document["name"]),
                parent_joint=str(document["parent_joint"]),
                reference_opening_m=float(document["reference_opening_m"]),
                mass_kg=float(document["mass_kg"]),
                com_m=np.asarray(document["com_m"], dtype=np.float64),
                inertia_kg_m2=np.asarray(
                    document["inertia_kg_m2"], dtype=np.float64
                ),
                source={str(key): str(value) for key, value in source.items()},
            )
        except (KeyError, TypeError, ValueError) as error:
            raise PayloadError(f"payload document is malformed: {error}") from error
        payload.validate()
        return payload

    @classmethod
    def load(cls, path: str | Path) -> "RigidPayload":
        try:
            document = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise PayloadError(f"cannot load payload document: {error}") from error
        if not isinstance(document, Mapping):
            raise PayloadError("payload document must be an object")
        return cls.from_dict(document)

    def validate(self) -> None:
        if not self.name or not self.parent_joint:
            raise PayloadError("payload name and parent joint must not be empty")
        if not np.isfinite(self.mass_kg) or self.mass_kg <= 0:
            raise PayloadError("payload mass must be positive and finite")
        if not np.isfinite(self.reference_opening_m) or self.reference_opening_m < 0:
            raise PayloadError("payload reference opening must be finite and non-negative")
        if self.com_m.shape != (3,) or not np.all(np.isfinite(self.com_m)):
            raise PayloadError("payload COM must contain three finite values")
        inertia = self.inertia_kg_m2
        if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
            raise PayloadError("payload inertia must be a finite 3x3 matrix")
        if not np.allclose(inertia, inertia.T, atol=1e-12, rtol=0.0):
            raise PayloadError("payload inertia must be symmetric")
        if float(np.min(np.linalg.eigvalsh(inertia))) <= 0:
            raise PayloadError("payload inertia must be positive definite")
        required_source = {"repository", "commit", "path"}
        if not required_source.issubset(self.source) or any(
            not self.source[key] for key in required_source
        ):
            raise PayloadError("payload source is incomplete")

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "parent_joint": self.parent_joint,
            "reference_opening_m": self.reference_opening_m,
            "mass_kg": self.mass_kg,
            "com_m": self.com_m.tolist(),
            "inertia_kg_m2": self.inertia_kg_m2.tolist(),
            "source": dict(self.source),
        }

    @property
    def sha256(self) -> str:
        canonical = json.dumps(
            self.to_dict(), sort_keys=True, separators=(",", ":"), ensure_ascii=False
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()
