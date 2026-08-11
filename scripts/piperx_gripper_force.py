#!/usr/bin/env python3
"""Receive-only known-force capture and Piper fingertip-force calibration."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import socket
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.piperx.gripper_force import (
    GripperForceCalibration,
    fit_gripper_force_calibration,
)
from robot_control.piperx.protocol import GRIPPER_ID, decode_gripper
from robot_control.piperx.records import file_sha256
from robot_control.piperx.socketcan import (
    CanDiscoveryError,
    GripperTelemetry,
    ReadOnlySocketCan,
    discover_interface,
)


class GripperForceCliError(RuntimeError):
    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate Piper gripper feedback torque to fingertip force"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    capture = commands.add_parser("capture", help="capture one known-force point")
    capture.add_argument("--serial", required=True)
    capture.add_argument("--known-force", type=float, required=True, metavar="N")
    capture.add_argument("--seconds", type=float, default=2.0)
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--confirm-known-force", action="store_true")

    fit = commands.add_parser("fit", help="fit a force artifact from captured CSV")
    fit.add_argument("--input", type=Path, required=True)
    fit.add_argument("--serial", required=True)
    fit.add_argument("--output", type=Path, required=True)

    evaluate = commands.add_parser("evaluate", help="evaluate an artifact on known-force CSV")
    evaluate.add_argument("--input", type=Path, required=True)
    evaluate.add_argument("--calibration", type=Path, required=True)
    evaluate.add_argument("--max-mae", type=float, default=5.0, metavar="N")
    return parser


def _load_samples(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        force = np.asarray([float(row["force_n"]) for row in rows], dtype=np.float64)
        torque = np.asarray([float(row["torque_nm"]) for row in rows], dtype=np.float64)
        travel = np.asarray([float(row["travel_mm"]) for row in rows], dtype=np.float64)
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise GripperForceCliError(f"cannot read force sample CSV: {error}") from error
    if force.size < 5:
        raise GripperForceCliError("force sample CSV needs at least five rows")
    return force, torque, travel


def _run_capture(args: argparse.Namespace) -> int:
    if not args.confirm_known_force:
        raise GripperForceCliError(
            "capture requires --confirm-known-force after a force gauge is installed"
        )
    if args.seconds <= 0 or not np.isfinite(args.known_force) or args.known_force < 0:
        raise GripperForceCliError("capture duration and known force must be valid")
    interface = discover_interface(args.serial)
    samples = []
    deadline = time.monotonic() + args.seconds
    with ReadOnlySocketCan(interface, timeout_s=0.1) as reader:
        while time.monotonic() < deadline:
            try:
                frame = reader.recv_frame()
            except socket.timeout:
                continue
            if frame.can_id != GRIPPER_ID:
                continue
            sample = decode_gripper(frame.can_id, frame.payload)
            if sample.status_code & 0x3F or not sample.status_code & (1 << 6):
                continue
            samples.append(sample)
    if not samples:
        raise GripperForceCliError("no enabled fault-free 0x2A8 gripper samples received")
    torque = float(np.median([sample.torque_nm for sample in samples]))
    travel = float(np.median([sample.travel_mm for sample in samples]))
    destination = args.output
    destination.parent.mkdir(parents=True, exist_ok=True)
    new_file = not destination.exists() or destination.stat().st_size == 0
    with destination.open("a", newline="", encoding="utf-8") as stream:
        fields = ("force_n", "torque_nm", "travel_mm", "sample_count", "status_code")
        writer = csv.DictWriter(stream, fieldnames=fields)
        if new_file:
            writer.writeheader()
        writer.writerow(
            {
                "force_n": float(args.known_force),
                "torque_nm": torque,
                "travel_mm": travel,
                "sample_count": len(samples),
                "status_code": samples[-1].status_code,
            }
        )
    print(json.dumps({
        "output": str(destination), "samples": len(samples),
        "known_force_n": args.known_force, "torque_nm": torque, "travel_mm": travel,
    }, indent=2))
    return 0


def _run_fit(args: argparse.Namespace) -> int:
    force, torque, travel = _load_samples(args.input)
    artifact = fit_gripper_force_calibration(
        torque_nm=torque,
        force_n=force,
        travel_mm=travel,
        adapter_serial=args.serial,
        source_sha256=file_sha256(args.input),
    )
    artifact.save(args.output)
    print(json.dumps({
        "output": str(args.output),
        "direction": artifact.direction,
        "slope_n_per_nm": artifact.slope_n_per_nm,
        "intercept_n": artifact.intercept_n,
        "validation_mae_n": artifact.validation_mae_n,
    }, indent=2))
    return 0


def _run_evaluate(args: argparse.Namespace) -> int:
    if args.max_mae <= 0:
        raise GripperForceCliError("--max-mae must be positive")
    force, torque, travel = _load_samples(args.input)
    artifact = GripperForceCalibration.load(args.calibration)
    predicted = []
    for torque_value, travel_value in zip(torque, travel, strict=True):
        result = artifact.predict(
            GripperTelemetry(float(travel_value), float(torque_value), 0xC0, 0),
            fresh=True,
            adapter_serial=artifact.adapter_serial,
        )
        if not result.valid or result.force_n is None:
            raise GripperForceCliError(f"sample is outside calibration: {result.reason}")
        predicted.append(result.force_n)
    error = np.asarray(predicted) - force
    report = {
        "count": int(force.size),
        "mae_n": float(np.mean(np.abs(error))),
        "rmse_n": float(np.sqrt(np.mean(np.square(error)))),
        "max_abs_n": float(np.max(np.abs(error))),
        "threshold_n": float(args.max_mae),
    }
    report["passed"] = report["mae_n"] <= args.max_mae
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 2


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "capture":
            return _run_capture(args)
        if args.command == "fit":
            return _run_fit(args)
        if args.command == "evaluate":
            return _run_evaluate(args)
        raise GripperForceCliError(f"unknown command {args.command!r}")
    except (CanDiscoveryError, GripperForceCliError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
