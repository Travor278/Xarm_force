"""Deterministic NPZ torque logs with a verified JSON metadata sidecar."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Mapping

import numpy as np


LOG_SCHEMA_VERSION = 1
REQUIRED_ARRAYS = (
    "timestamp_ns",
    "q",
    "qd",
    "qdd",
    "tau_measured",
    "tau_model",
    "tau_bias",
    "tau_external",
    "valid",
    "group_id",
    "reason",
)
_VECTOR_ARRAYS = (
    "q",
    "qd",
    "qdd",
    "tau_measured",
    "tau_model",
    "tau_bias",
    "tau_external",
)


class RecordError(RuntimeError):
    """Raised when a torque log is partial or dimensionally unsafe."""


@dataclass
class TorqueLog:
    arrays: dict[str, np.ndarray]
    metadata: dict[str, object]


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate_arrays(arrays: Mapping[str, np.ndarray]) -> int:
    missing = set(REQUIRED_ARRAYS) - set(arrays)
    if missing:
        raise RecordError(f"torque log is missing required arrays: {sorted(missing)}")
    timestamp = np.asarray(arrays["timestamp_ns"])
    if timestamp.ndim != 1:
        raise RecordError("timestamp_ns must be one-dimensional")
    count = timestamp.shape[0]
    for name in _VECTOR_ARRAYS:
        if np.asarray(arrays[name]).shape != (count, 6):
            raise RecordError(
                f"array {name!r} does not share sample count {count} and shape (6,)"
            )
    for name in ("valid", "group_id", "reason"):
        if np.asarray(arrays[name]).shape != (count,):
            raise RecordError(f"array {name!r} does not share sample count {count}")
    return count


def _sidecar_path(path: Path) -> Path:
    return path.with_suffix(".json")


def save_torque_log(path: str | Path, log: TorqueLog) -> None:
    destination = Path(path)
    if destination.suffix.lower() != ".npz":
        raise RecordError("torque log path must end in .npz")
    destination.parent.mkdir(parents=True, exist_ok=True)
    arrays = {name: np.asarray(log.arrays[name]) for name in REQUIRED_ARRAYS}
    count = _validate_arrays(arrays)
    npz_handle = tempfile.NamedTemporaryFile(
        mode="wb", prefix=destination.name + ".", suffix=".tmp", dir=destination.parent, delete=False
    )
    npz_temporary = Path(npz_handle.name)
    json_temporary = _sidecar_path(destination).with_suffix(".json.tmp")
    try:
        with npz_handle:
            np.savez_compressed(npz_handle, **arrays)
        metadata = dict(log.metadata)
        metadata.update(
            {
                "schema_version": LOG_SCHEMA_VERSION,
                "sample_count": count,
                "npz_sha256": file_sha256(npz_temporary),
            }
        )
        json_temporary.write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )
        os.replace(npz_temporary, destination)
        os.replace(json_temporary, _sidecar_path(destination))
    finally:
        npz_temporary.unlink(missing_ok=True)
        json_temporary.unlink(missing_ok=True)


def load_torque_log(path: str | Path) -> TorqueLog:
    source = Path(path)
    sidecar = _sidecar_path(source)
    if not source.is_file() or not sidecar.is_file():
        raise RecordError("torque log requires both NPZ data and JSON sidecar")
    metadata = json.loads(sidecar.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict) or metadata.get("schema_version") != LOG_SCHEMA_VERSION:
        raise RecordError("unsupported or malformed torque log sidecar")
    if metadata.get("npz_sha256") != file_sha256(source):
        raise RecordError("torque log NPZ SHA-256 does not match sidecar")
    try:
        with np.load(source, allow_pickle=False) as archive:
            arrays = {name: archive[name].copy() for name in REQUIRED_ARRAYS}
    except (KeyError, OSError, ValueError) as error:
        raise RecordError(f"unable to load torque log arrays: {error}") from error
    count = _validate_arrays(arrays)
    if metadata.get("sample_count") != count:
        raise RecordError("sidecar sample count does not match NPZ arrays")
    return TorqueLog(arrays=arrays, metadata=metadata)


def finite_valid_mask(log: TorqueLog) -> np.ndarray:
    count = _validate_arrays(log.arrays)
    mask = np.asarray(log.arrays["valid"], dtype=bool).copy()
    finite = np.ones(count, dtype=bool)
    for name in _VECTOR_ARRAYS:
        finite &= np.all(np.isfinite(log.arrays[name]), axis=1)
    return mask & finite
