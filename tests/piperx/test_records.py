from __future__ import annotations

import json

import numpy as np
import pytest

from robot_control.piperx.records import (
    REQUIRED_ARRAYS,
    RecordError,
    TorqueLog,
    file_sha256,
    finite_valid_mask,
    load_torque_log,
    save_torque_log,
)


def _log(count=3):
    arrays = {
        "timestamp_ns": np.arange(count, dtype=np.int64),
        "q": np.zeros((count, 6)),
        "qd": np.zeros((count, 6)),
        "qdd": np.zeros((count, 6)),
        "tau_measured": np.ones((count, 6)),
        "tau_model": np.full((count, 6), 0.5),
        "tau_bias": np.zeros((count, 6)),
        "tau_external": np.full((count, 6), -0.5),
        "valid": np.ones(count, dtype=bool),
        "group_id": np.arange(count, dtype=np.int64),
        "reason": np.full(count, "", dtype="U32"),
    }
    return TorqueLog(arrays=arrays, metadata={"adapter_serial": "serial-left"})


def test_npz_and_sidecar_round_trip_with_verified_hash(tmp_path):
    path = tmp_path / "static.npz"
    original = _log()

    save_torque_log(path, original)
    loaded = load_torque_log(path)

    assert set(loaded.arrays) == set(REQUIRED_ARRAYS)
    assert loaded.metadata["adapter_serial"] == "serial-left"
    assert loaded.metadata["npz_sha256"] == file_sha256(path)
    assert loaded.arrays["tau_external"] == pytest.approx(
        original.arrays["tau_external"]
    )


def test_save_rejects_inconsistent_row_counts(tmp_path):
    log = _log()
    log.arrays["qdd"] = np.zeros((2, 6))

    with pytest.raises(RecordError, match="sample count"):
        save_torque_log(tmp_path / "broken.npz", log)


def test_load_rejects_missing_sidecar(tmp_path):
    path = tmp_path / "partial.npz"
    save_torque_log(path, _log())
    path.with_suffix(".json").unlink()

    with pytest.raises(RecordError, match="sidecar"):
        load_torque_log(path)


def test_load_rejects_npz_tampering(tmp_path):
    path = tmp_path / "tampered.npz"
    save_torque_log(path, _log())
    with path.open("ab") as stream:
        stream.write(b"changed")

    with pytest.raises(RecordError, match="SHA-256"):
        load_torque_log(path)


def test_finite_valid_mask_excludes_invalid_and_nonfinite_rows():
    log = _log()
    log.arrays["valid"][1] = False
    log.arrays["tau_model"][2, 0] = np.nan

    assert finite_valid_mask(log).tolist() == [True, False, False]


def test_sidecar_is_json_and_contains_required_metadata(tmp_path):
    path = tmp_path / "metadata.npz"
    save_torque_log(path, _log())

    document = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))

    assert document["schema_version"] == 1
    assert document["sample_count"] == 3
    assert len(document["npz_sha256"]) == 64
