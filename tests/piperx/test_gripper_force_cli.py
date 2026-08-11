from __future__ import annotations

import csv

import pytest

from robot_control.piperx.gripper_force import GripperForceCalibration
from scripts.piperx_gripper_force import main


def _write_samples(path):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=("force_n", "torque_nm", "travel_mm")
        )
        writer.writeheader()
        for index in range(20):
            torque = -0.1 - index * 0.04
            writer.writerow(
                {
                    "force_n": 50.0 * (-torque),
                    "torque_nm": torque,
                    "travel_mm": 20.0 + index,
                }
            )


def test_fit_and_evaluate_known_force_csv(tmp_path):
    samples = tmp_path / "samples.csv"
    artifact_path = tmp_path / "force.json"
    _write_samples(samples)

    assert main([
        "fit", "--input", str(samples), "--serial", "left-follower",
        "--output", str(artifact_path),
    ]) == 0
    artifact = GripperForceCalibration.load(artifact_path)
    assert artifact.adapter_serial == "left-follower"
    assert artifact.slope_n_per_nm == pytest.approx(50.0)
    assert main([
        "evaluate", "--input", str(samples), "--calibration", str(artifact_path)
    ]) == 0


def test_capture_requires_explicit_known_force_confirmation(tmp_path):
    assert main([
        "capture", "--serial", "left-follower", "--known-force", "10",
        "--seconds", "1", "--output", str(tmp_path / "samples.csv")
    ]) == 2
