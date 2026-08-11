from __future__ import annotations

import numpy as np
from pathlib import Path
import subprocess
import sys

from robot_control.piperx.records import TorqueLog, save_torque_log
from scripts.piperx_external_torque import (
    PiperTorqueCliError,
    _validate_record_confirmation,
    build_parser,
    compute_statistics,
    main,
    score_known_load,
)
import pytest


def _offline_log(count=100, *, measured_offset=0.0, serial="serial-left"):
    phase = np.linspace(-0.5, 0.5, count)
    q = np.column_stack([phase * (index + 1) / 6 for index in range(6)])
    qd = np.zeros((count, 6))
    model = np.column_stack([np.sin(q[:, index]) for index in range(6)])
    measured = model + measured_offset
    return TorqueLog(
        arrays={
            "timestamp_ns": np.arange(count, dtype=np.int64) * 5_000_000,
            "q": q,
            "qd": qd,
            "qdd": np.zeros((count, 6)),
            "tau_measured": measured,
            "tau_model": model,
            "tau_bias": np.zeros((count, 6)),
            "tau_external": model - measured,
            "valid": np.ones(count, dtype=bool),
            "group_id": np.repeat(np.arange(5), count // 5),
            "reason": np.full(count, "", dtype="U64"),
        },
        metadata={
            "adapter_serial": serial,
            "urdf_sha256": "a" * 64,
            "base_rpy": [0.0, 0.0, 0.0],
            "label": "no_contact",
        },
    )


def test_parser_exposes_all_validation_workflows():
    parser = build_parser()

    for command in ("discover", "record", "fit", "monitor", "evaluate", "known-load"):
        assert command in parser.format_help()


def test_script_help_runs_directly_without_editable_install():
    repository = Path(__file__).resolve().parents[2]

    result = subprocess.run(
        [sys.executable, "scripts/piperx_external_torque.py", "--help"],
        cwd=repository,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "known-load" in result.stdout


def test_record_refuses_to_run_without_explicit_no_contact_confirmation(tmp_path):
    exit_code = main(
        [
            "record",
            "--arm",
            "left",
            "--serial",
            "serial-left",
            "--urdf",
            str(tmp_path / "missing.urdf"),
            "--seconds",
            "1",
            "--output",
            str(tmp_path / "data.npz"),
        ]
    )

    assert exit_code == 2


@pytest.mark.parametrize(
    ("label", "confirm_no_contact", "confirm_known_load"),
    [
        ("no_contact", True, False),
        ("known_load", False, True),
    ],
)
def test_record_label_requires_matching_explicit_confirmation(
    label, confirm_no_contact, confirm_known_load
):
    assert (
        _validate_record_confirmation(
            label,
            confirm_no_contact=confirm_no_contact,
            confirm_known_load=confirm_known_load,
        )
        == label
    )


@pytest.mark.parametrize(
    ("label", "confirm_no_contact", "confirm_known_load"),
    [
        ("no_contact", False, False),
        ("known_load", False, False),
        ("known_load", True, False),
        ("no_contact", False, True),
        ("no_contact", True, True),
    ],
)
def test_record_label_rejects_missing_wrong_or_ambiguous_confirmation(
    label, confirm_no_contact, confirm_known_load
):
    with pytest.raises(PiperTorqueCliError, match="confirmation"):
        _validate_record_confirmation(
            label,
            confirm_no_contact=confirm_no_contact,
            confirm_known_load=confirm_known_load,
        )


def test_fit_and_stationary_evaluate_pass_for_zero_no_contact_residual(tmp_path):
    log_path = tmp_path / "no-contact.npz"
    artifact_path = tmp_path / "left.json"
    save_torque_log(log_path, _offline_log())

    assert main(["fit", "--input", str(log_path), "--output", str(artifact_path)]) == 0
    assert artifact_path.is_file()
    assert (
        main(
            [
                "evaluate",
                "--input",
                str(log_path),
                "--calibration",
                str(artifact_path),
                "--stationary",
            ]
        )
        == 0
    )


def test_stationary_evaluate_returns_failure_for_one_nm_residual(tmp_path):
    calibration_log = tmp_path / "calibration.npz"
    validation_log = tmp_path / "validation.npz"
    artifact_path = tmp_path / "left.json"
    save_torque_log(calibration_log, _offline_log())
    save_torque_log(validation_log, _offline_log(measured_offset=1.0))
    assert main(["fit", "--input", str(calibration_log), "--output", str(artifact_path)]) == 0

    result = main(
        [
            "evaluate",
            "--input",
            str(validation_log),
            "--calibration",
            str(artifact_path),
            "--stationary",
        ]
    )

    assert result == 2


def test_evaluate_rejects_adapter_metadata_mismatch(tmp_path):
    source = tmp_path / "source.npz"
    other = tmp_path / "other.npz"
    artifact = tmp_path / "left.json"
    save_torque_log(source, _offline_log())
    save_torque_log(other, _offline_log(serial="serial-right"))
    assert main(["fit", "--input", str(source), "--output", str(artifact)]) == 0

    assert (
        main(
            [
                "evaluate",
                "--input",
                str(other),
                "--calibration",
                str(artifact),
            ]
        )
        == 2
    )


def test_evaluate_rejects_known_load_label(tmp_path):
    source = tmp_path / "source.npz"
    loaded = tmp_path / "loaded.npz"
    artifact = tmp_path / "left.json"
    save_torque_log(source, _offline_log())
    loaded_log = _offline_log()
    loaded_log.metadata["label"] = "known_load"
    save_torque_log(loaded, loaded_log)
    assert main(["fit", "--input", str(source), "--output", str(artifact)]) == 0

    assert (
        main(
            [
                "evaluate",
                "--input",
                str(loaded),
                "--calibration",
                str(artifact),
            ]
        )
        == 2
    )


def test_known_load_rejects_no_contact_label(tmp_path):
    source = tmp_path / "source.npz"
    artifact = tmp_path / "left.json"
    save_torque_log(source, _offline_log())
    assert main(["fit", "--input", str(source), "--output", str(artifact)]) == 0

    assert (
        main(
            [
                "known-load",
                "--input",
                str(source),
                "--calibration",
                str(artifact),
                "--expected-torque",
                "0,0,0,0,0,0",
            ]
        )
        == 2
    )


def test_statistics_report_literal_per_joint_errors():
    values = np.array([[1.0] * 6, [-1.0] * 6, [0.0] * 6])

    report = compute_statistics(values)

    assert report["count"] == 3
    assert report["signed_mean_nm"] == [0.0] * 6
    assert report["mae_nm"] == [2.0 / 3.0] * 6


def test_known_load_score_checks_magnitude_and_direction():
    expected = np.array([1.0, -2.0, 0.0, 0.0, 0.0, 0.0])
    passing_samples = np.tile(expected, (100, 1))
    passing_samples[:, 0] += 0.05
    passing_samples[:, 1] -= 0.05

    passing = score_known_load(passing_samples, expected)
    wrong_direction = score_known_load(-passing_samples, expected)

    assert passing["passed"]
    assert passing["direction_agreement"][0:2] == [1.0, 1.0]
    assert not wrong_direction["passed"]
