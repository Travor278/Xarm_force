from __future__ import annotations

import json

import numpy as np
import pytest

from robot_control.piperx.calibration import (
    FEATURE_VERSION,
    CalibrationArtifact,
    CalibrationMismatchError,
    build_features,
    fit_calibration,
    split_contiguous_groups,
)


def _independent_features(q, qd, velocity_scale=0.2):
    rows = []
    for position, velocity in zip(q, qd):
        chain = np.cumsum(position[1:])
        rows.append(
            np.concatenate(
                (
                    [1.0],
                    np.sin(position),
                    np.cos(position),
                    np.sin(chain),
                    np.cos(chain),
                    np.tanh(velocity / velocity_scale),
                )
            )
        )
    return np.asarray(rows)


def test_features_have_versioned_literal_layout():
    features = build_features(np.zeros(6), np.zeros(6), friction_velocity_scale=0.2)

    assert FEATURE_VERSION == "piperx-residual-v1"
    assert features.shape == (29,)
    assert features == pytest.approx(
        [1.0] + [0.0] * 6 + [1.0] * 6 + [0.0] * 5 + [1.0] * 5 + [0.0] * 6
    )


def test_group_split_holds_out_whole_trailing_groups():
    groups = np.repeat(np.arange(5), 3)

    train, validation = split_contiguous_groups(groups, validation_fraction=0.4)

    assert set(groups[train]) == {0, 1, 2}
    assert set(groups[validation]) == {3, 4}
    assert not np.any(train & validation)


def _synthetic_fit():
    rng = np.random.default_rng(4)
    count = 600
    q = rng.uniform(-1.0, 1.0, size=(count, 6))
    qd = rng.uniform(-0.4, 0.4, size=(count, 6))
    independent_x = _independent_features(q, qd)
    weights = rng.normal(scale=0.1, size=(independent_x.shape[1], 6))
    target_bias = independent_x @ weights
    tau_model = rng.normal(scale=0.2, size=(count, 6))
    tau_measured = tau_model + target_bias
    groups = np.repeat(np.arange(12), count // 12)
    artifact = fit_calibration(
        q_rad=q,
        qd_rad_s=qd,
        tau_measured_nm=tau_measured,
        tau_model_nm=tau_model,
        group_ids=groups,
        adapter_serial="serial-left",
        urdf_sha256="a" * 64,
        payload_sha256="p" * 64,
        base_rpy=(0.0, 0.0, 0.0),
        source_log_sha256="b" * 64,
        ridge=1e-9,
        validation_fraction=0.25,
        friction_velocity_scale=0.2,
    )
    return artifact, q, qd, target_bias


def test_ridge_fit_recovers_synthetic_no_contact_bias_on_held_out_groups():
    artifact, q, qd, expected = _synthetic_fit()

    predicted = np.asarray([artifact.predict(qi, qdi).bias_nm for qi, qdi in zip(q, qd)])

    assert np.max(np.abs(predicted - expected)) < 1e-5
    assert max(artifact.metrics["validation"]["mae_nm"]) < 1e-5
    assert artifact.coefficients.shape == (6, 29)


def test_artifact_json_round_trip_preserves_predictions(tmp_path):
    artifact, q, qd, _ = _synthetic_fit()
    path = tmp_path / "left.json"

    artifact.save(path)
    loaded = CalibrationArtifact.load(path)

    assert loaded.adapter_serial == artifact.adapter_serial
    assert loaded.created_at == artifact.created_at
    assert loaded.predict(q[0], qd[0]).bias_nm == pytest.approx(
        artifact.predict(q[0], qd[0]).bias_nm
    )


@pytest.mark.parametrize(
    ("serial", "urdf_hash", "base_rpy", "payload_hash", "message"),
    [
        ("other", "a" * 64, (0.0, 0.0, 0.0), "p" * 64, "adapter serial"),
        ("serial-left", "c" * 64, (0.0, 0.0, 0.0), "p" * 64, "URDF"),
        ("serial-left", "a" * 64, (0.1, 0.0, 0.0), "p" * 64, "base orientation"),
        ("serial-left", "a" * 64, (0.0, 0.0, 0.0), "q" * 64, "payload"),
    ],
)
def test_artifact_rejects_runtime_identity_mismatch(
    serial, urdf_hash, base_rpy, payload_hash, message
):
    artifact, *_ = _synthetic_fit()

    with pytest.raises(CalibrationMismatchError, match=message):
        artifact.validate_runtime(serial, urdf_hash, base_rpy, payload_hash)


def test_payload_calibration_fails_closed_when_runtime_omits_payload():
    artifact, *_ = _synthetic_fit()

    with pytest.raises(CalibrationMismatchError, match="payload"):
        artifact.validate_runtime(
            "serial-left", "a" * 64, (0.0, 0.0, 0.0), None
        )


def test_load_rejects_unknown_feature_version(tmp_path):
    artifact, *_ = _synthetic_fit()
    document = artifact.to_dict()
    document["feature_version"] = "future-layout"
    path = tmp_path / "future.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(CalibrationMismatchError, match="feature version"):
        CalibrationArtifact.load(path)


def test_prediction_fails_closed_outside_calibrated_workspace():
    artifact, q, qd, _ = _synthetic_fit()

    prediction = artifact.predict(q[0] + 10.0, qd[0])

    assert not prediction.valid
    assert prediction.reason == "outside_calibrated_workspace"
    assert prediction.bias_nm.shape == (6,)


def test_fit_rejects_random_frame_split_without_two_groups():
    arrays = np.zeros((10, 6))

    with pytest.raises(ValueError, match="two contiguous groups"):
        fit_calibration(
            q_rad=arrays,
            qd_rad_s=arrays,
            tau_measured_nm=arrays,
            tau_model_nm=arrays,
            group_ids=np.zeros(10),
            adapter_serial="serial-left",
            urdf_sha256="a" * 64,
            base_rpy=(0.0, 0.0, 0.0),
            source_log_sha256="b" * 64,
        )
