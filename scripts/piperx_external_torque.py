#!/usr/bin/env python3
"""Receive-only PiperX external joint-torque calibration and validation CLI."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import socket
import sys
import threading
import time
from typing import Iterable

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.piperx.calibration import (
    CalibrationArtifact,
    CalibrationMismatchError,
    fit_calibration,
)
from robot_control.piperx.dynamics import DynamicsError, PinocchioDynamics
from robot_control.piperx.estimator import Estimate, ExternalTorqueEstimator, bounded_for_display
from robot_control.piperx.filtering import VelocityDerivativeFilter
from robot_control.piperx.records import (
    RecordError,
    TorqueLog,
    file_sha256,
    finite_valid_mask,
    load_torque_log,
    save_torque_log,
)
from robot_control.piperx.socketcan import (
    CanDiscoveryError,
    PiperStateAssembler,
    ReadOnlySocketCan,
    discover_interface,
)


STATIONARY_LIMIT_NM = np.array([0.15, 0.30, 0.30, 0.15, 0.15, 0.15])
FREE_MOTION_P95_LIMIT_NM = 0.50


class PiperTorqueCliError(RuntimeError):
    pass


def _add_base_rpy(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--base-rpy",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("ROLL", "PITCH", "YAW"),
        help="fixed base orientation in world coordinates, radians",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Passive PiperX follower external joint-torque tools"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    discover = subparsers.add_parser("discover", help="resolve a CAN interface by USB serial")
    discover.add_argument("--serial", required=True)

    record = subparsers.add_parser("record", help="record an explicitly no-contact follower log")
    record.add_argument("--arm", required=True)
    record.add_argument("--serial", required=True)
    record.add_argument("--urdf", type=Path, required=True)
    record.add_argument("--seconds", type=float, required=True)
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--group-seconds", type=float, default=2.0)
    record.add_argument(
        "--label", choices=("no_contact", "known_load"), default="no_contact"
    )
    record.add_argument("--confirm-no-contact", action="store_true")
    record.add_argument("--confirm-known-load", action="store_true")
    _add_base_rpy(record)

    fit = subparsers.add_parser("fit", help="fit one arm's no-contact residual artifact")
    fit.add_argument("--input", type=Path, required=True)
    fit.add_argument("--output", type=Path, required=True)
    fit.add_argument("--ridge", type=float, default=1e-3)
    fit.add_argument("--validation-fraction", type=float, default=0.2)
    fit.add_argument("--friction-velocity-scale", type=float, default=0.2)

    monitor = subparsers.add_parser("monitor", help="monitor one or both calibrated followers")
    monitor.add_argument(
        "--arm",
        action="append",
        required=True,
        metavar="NAME,SERIAL,CALIBRATION",
    )
    monitor.add_argument("--urdf", type=Path, required=True)
    monitor.add_argument("--seconds", type=float, default=0.0)
    _add_base_rpy(monitor)

    evaluate = subparsers.add_parser("evaluate", help="score a no-contact replay log")
    evaluate.add_argument("--input", type=Path, required=True)
    evaluate.add_argument("--calibration", type=Path, required=True)
    evaluate.add_argument("--stationary", action="store_true")

    known = subparsers.add_parser("known-load", help="score a known external joint-torque interval")
    known.add_argument("--input", type=Path, required=True)
    known.add_argument("--calibration", type=Path, required=True)
    known.add_argument("--expected-torque", required=True, metavar="NM,NM,NM,NM,NM,NM")
    return parser


def compute_statistics(values_nm: np.ndarray) -> dict[str, object]:
    values = np.asarray(values_nm, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 6 or values.shape[0] == 0:
        raise PiperTorqueCliError("statistics require at least one finite six-joint sample")
    if not np.all(np.isfinite(values)):
        raise PiperTorqueCliError("statistics input contains non-finite values")
    return {
        "count": int(values.shape[0]),
        "signed_mean_nm": np.mean(values, axis=0).tolist(),
        "mae_nm": np.mean(np.abs(values), axis=0).tolist(),
        "rmse_nm": np.sqrt(np.mean(np.square(values), axis=0)).tolist(),
        "std_nm": np.std(values, axis=0).tolist(),
        "p95_abs_nm": np.percentile(np.abs(values), 95, axis=0).tolist(),
    }


def score_known_load(samples_nm: np.ndarray, expected_nm: np.ndarray) -> dict[str, object]:
    samples = np.asarray(samples_nm, dtype=np.float64)
    expected = np.asarray(expected_nm, dtype=np.float64)
    if expected.shape != (6,) or not np.all(np.isfinite(expected)):
        raise PiperTorqueCliError("expected torque must contain six finite values")
    error = samples - expected
    statistics = compute_statistics(error)
    thresholds = np.maximum(0.30, 0.15 * np.abs(expected))
    direction: list[float | None] = []
    direction_pass = True
    for joint in range(6):
        if expected[joint] == 0:
            direction.append(None)
            continue
        agreement = float(np.mean(np.sign(samples[:, joint]) == np.sign(expected[joint])))
        direction.append(agreement)
        direction_pass &= agreement >= 0.95
    magnitude_pass = bool(np.all(np.asarray(statistics["mae_nm"]) <= thresholds))
    return {
        "passed": bool(magnitude_pass and direction_pass),
        "expected_nm": expected.tolist(),
        "error_threshold_nm": thresholds.tolist(),
        "error_statistics": statistics,
        "direction_agreement": direction,
    }


def _metadata_identity(log: TorqueLog) -> tuple[str, str, tuple[float, float, float]]:
    try:
        serial = str(log.metadata["adapter_serial"])
        urdf_hash = str(log.metadata["urdf_sha256"])
        base_rpy = tuple(float(value) for value in log.metadata["base_rpy"])
    except (KeyError, TypeError, ValueError) as error:
        raise PiperTorqueCliError("log metadata lacks arm/URDF/base identity") from error
    if len(base_rpy) != 3:
        raise PiperTorqueCliError("log base_rpy metadata must contain three values")
    return serial, urdf_hash, base_rpy


def _replay_external(
    log: TorqueLog, artifact: CalibrationArtifact
) -> tuple[np.ndarray, int, int]:
    serial, urdf_hash, base_rpy = _metadata_identity(log)
    artifact.validate_runtime(serial, urdf_hash, base_rpy)
    source_mask = finite_valid_mask(log)
    indices = np.flatnonzero(source_mask)
    estimates: list[np.ndarray] = []
    invalid_workspace = 0
    for index in indices:
        prediction = artifact.predict(log.arrays["q"][index], log.arrays["qd"][index])
        if not prediction.valid:
            invalid_workspace += 1
            continue
        estimates.append(
            log.arrays["tau_model"][index]
            + prediction.bias_nm
            - log.arrays["tau_measured"][index]
        )
    if not estimates:
        raise PiperTorqueCliError("no valid in-workspace samples remain for evaluation")
    return np.asarray(estimates), int(indices.size), invalid_workspace


def _run_fit(args: argparse.Namespace) -> int:
    log = load_torque_log(args.input)
    if log.metadata.get("label") != "no_contact":
        raise PiperTorqueCliError("calibration input must be explicitly labelled no_contact")
    serial, urdf_hash, base_rpy = _metadata_identity(log)
    mask = finite_valid_mask(log)
    if np.count_nonzero(mask) < 2:
        raise PiperTorqueCliError("calibration log has fewer than two valid finite samples")
    artifact = fit_calibration(
        q_rad=log.arrays["q"][mask],
        qd_rad_s=log.arrays["qd"][mask],
        tau_measured_nm=log.arrays["tau_measured"][mask],
        tau_model_nm=log.arrays["tau_model"][mask],
        group_ids=log.arrays["group_id"][mask],
        adapter_serial=serial,
        urdf_sha256=urdf_hash,
        base_rpy=base_rpy,
        source_log_sha256=file_sha256(args.input),
        ridge=args.ridge,
        validation_fraction=args.validation_fraction,
        friction_velocity_scale=args.friction_velocity_scale,
    )
    artifact.save(args.output)
    print(json.dumps({"output": str(args.output), "metrics": artifact.metrics}, indent=2))
    return 0


def _run_evaluate(args: argparse.Namespace) -> int:
    log = load_torque_log(args.input)
    artifact = CalibrationArtifact.load(args.calibration)
    external, source_count, invalid_workspace = _replay_external(log, artifact)
    statistics = compute_statistics(external)
    if args.stationary:
        passed = bool(np.all(np.asarray(statistics["mae_nm"]) <= STATIONARY_LIMIT_NM))
        threshold: object = STATIONARY_LIMIT_NM.tolist()
        criterion = "stationary_mae_nm"
    else:
        passed = bool(
            np.all(np.asarray(statistics["p95_abs_nm"]) <= FREE_MOTION_P95_LIMIT_NM)
        )
        threshold = FREE_MOTION_P95_LIMIT_NM
        criterion = "free_motion_p95_abs_nm"
    report = {
        "passed": passed,
        "criterion": criterion,
        "threshold_nm": threshold,
        "source_valid_count": source_count,
        "outside_workspace_count": invalid_workspace,
        "statistics": statistics,
    }
    print(json.dumps(report, indent=2))
    return 0 if passed else 2


def _parse_expected(value: str) -> np.ndarray:
    try:
        values = np.asarray([float(item) for item in value.split(",")], dtype=np.float64)
    except ValueError as error:
        raise PiperTorqueCliError("--expected-torque contains a non-number") from error
    if values.shape != (6,):
        raise PiperTorqueCliError("--expected-torque must contain exactly six comma-separated values")
    return values


def _run_known_load(args: argparse.Namespace) -> int:
    log = load_torque_log(args.input)
    artifact = CalibrationArtifact.load(args.calibration)
    external, source_count, invalid_workspace = _replay_external(log, artifact)
    report = score_known_load(external, _parse_expected(args.expected_torque))
    report["source_valid_count"] = source_count
    report["outside_workspace_count"] = invalid_workspace
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 2


def _estimate_rows(estimates: list[Estimate], start_ns: int, group_seconds: float) -> TorqueLog:
    if not estimates:
        raise PiperTorqueCliError("no complete Piper feedback state was received")
    group_ns = int(group_seconds * 1e9)
    if group_ns <= 0:
        raise PiperTorqueCliError("--group-seconds must be positive")
    return TorqueLog(
        arrays={
            "timestamp_ns": np.asarray([item.timestamp_ns for item in estimates], dtype=np.int64),
            "q": np.asarray([item.q_rad for item in estimates]),
            "qd": np.asarray([item.qd_filtered_rad_s for item in estimates]),
            "qdd": np.asarray([item.qdd_rad_s2 for item in estimates]),
            "tau_measured": np.asarray([item.tau_measured_nm for item in estimates]),
            "tau_model": np.asarray([item.tau_model_nm for item in estimates]),
            "tau_bias": np.asarray([item.tau_bias_nm for item in estimates]),
            "tau_external": np.asarray([item.tau_external_nm for item in estimates]),
            "valid": np.asarray([item.valid for item in estimates], dtype=bool),
            "group_id": np.asarray(
                [(item.timestamp_ns - start_ns) // group_ns for item in estimates], dtype=np.int64
            ),
            "reason": np.asarray([item.reason or "" for item in estimates], dtype="U128"),
        },
        metadata={},
    )


def _validate_record_confirmation(
    label: str, *, confirm_no_contact: bool, confirm_known_load: bool
) -> str:
    confirmations = int(confirm_no_contact) + int(confirm_known_load)
    if confirmations != 1:
        raise PiperTorqueCliError(
            "record confirmation must select exactly one of --confirm-no-contact or --confirm-known-load"
        )
    expected = "no_contact" if confirm_no_contact else "known_load"
    if label != expected:
        raise PiperTorqueCliError(
            f"record confirmation for {expected!r} does not match --label {label!r}"
        )
    return label


def _run_record(args: argparse.Namespace) -> int:
    label = _validate_record_confirmation(
        args.label,
        confirm_no_contact=args.confirm_no_contact,
        confirm_known_load=args.confirm_known_load,
    )
    if args.seconds <= 0:
        raise PiperTorqueCliError("--seconds must be positive")
    dynamics = PinocchioDynamics(args.urdf, base_rpy=args.base_rpy)
    interface = discover_interface(args.serial)
    assembler = PiperStateAssembler(interface, args.serial)
    estimator = ExternalTorqueEstimator(
        dynamics, VelocityDerivativeFilter(), calibration=None
    )
    estimates: list[Estimate] = []
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(args.seconds * 1e9)
    with ReadOnlySocketCan(interface, timeout_s=0.1) as reader:
        while time.monotonic_ns() < deadline_ns:
            try:
                frame = reader.recv_frame()
            except socket.timeout:
                continue
            state = assembler.update(frame.can_id, frame.payload, frame.timestamp_ns)
            if state is not None:
                estimates.append(estimator.estimate(state))
    log = _estimate_rows(estimates, start_ns, args.group_seconds)
    valid_count = int(np.count_nonzero(finite_valid_mask(log)))
    log.metadata.update(
        {
            "arm": args.arm,
            "adapter_serial": args.serial,
            "interface": interface,
            "urdf_path": str(dynamics.urdf_path),
            "urdf_sha256": dynamics.urdf_sha256,
            "base_rpy": list(dynamics.base_rpy),
            "label": label,
            "receive_only": True,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "complete_sample_count": len(estimates),
            "valid_sample_count": valid_count,
        }
    )
    save_torque_log(args.output, log)
    duration_s = max((estimates[-1].timestamp_ns - estimates[0].timestamp_ns) * 1e-9, 0.0)
    rate = (len(estimates) - 1) / duration_s if duration_s > 0 and len(estimates) > 1 else 0.0
    print(
        json.dumps(
            {
                "output": str(args.output),
                "interface": interface,
                "samples": len(estimates),
                "valid_samples": valid_count,
                "complete_rate_hz": rate,
            },
            indent=2,
        )
    )
    return 0


def _parse_arm_spec(spec: str) -> tuple[str, str, Path]:
    parts = spec.split(",", 2)
    if len(parts) != 3 or not all(parts):
        raise PiperTorqueCliError(
            "--arm must use NAME,SERIAL,CALIBRATION format"
        )
    return parts[0], parts[1], Path(parts[2])


def _monitor_worker(
    name: str,
    serial: str,
    artifact_path: Path,
    urdf: Path,
    base_rpy: Iterable[float],
    stop: threading.Event,
    output: queue.Queue[tuple[str, Estimate | Exception]],
) -> None:
    try:
        interface = discover_interface(serial)
        dynamics = PinocchioDynamics(urdf, base_rpy=base_rpy)
        artifact = CalibrationArtifact.load(artifact_path)
        artifact.validate_runtime(serial, dynamics.urdf_sha256, dynamics.base_rpy)
        estimator = ExternalTorqueEstimator(dynamics, VelocityDerivativeFilter(), artifact)
        assembler = PiperStateAssembler(interface, serial)
        with ReadOnlySocketCan(interface, timeout_s=0.1) as reader:
            while not stop.is_set():
                try:
                    frame = reader.recv_frame()
                except socket.timeout:
                    continue
                state = assembler.update(frame.can_id, frame.payload, frame.timestamp_ns)
                if state is not None:
                    output.put((name, estimator.estimate(state)))
    except Exception as error:
        output.put((name, error))


def _run_monitor(args: argparse.Namespace) -> int:
    if args.seconds < 0:
        raise PiperTorqueCliError("--seconds must be nonnegative")
    specifications = [_parse_arm_spec(spec) for spec in args.arm]
    names = [item[0] for item in specifications]
    if len(set(names)) != len(names):
        raise PiperTorqueCliError("monitor arm names must be unique")
    stop = threading.Event()
    output: queue.Queue[tuple[str, Estimate | Exception]] = queue.Queue()
    threads = [
        threading.Thread(
            target=_monitor_worker,
            args=(*spec, args.urdf, args.base_rpy, stop, output),
            daemon=True,
            name=f"piperx-monitor-{spec[0]}",
        )
        for spec in specifications
    ]
    for thread in threads:
        thread.start()
    deadline = time.monotonic() + args.seconds if args.seconds else None
    last_display: dict[str, float] = {}
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                name, item = output.get(timeout=0.2)
            except queue.Empty:
                if not any(thread.is_alive() for thread in threads):
                    break
                continue
            if isinstance(item, Exception):
                raise PiperTorqueCliError(f"{name} monitor failed: {item}")
            now = time.monotonic()
            if now - last_display.get(name, 0.0) >= 0.1:
                values = bounded_for_display(item.tau_external_nm).round(3).tolist()
                print(json.dumps({"arm": name, "valid": item.valid, "reason": item.reason, "tau_external_nm": values}))
                last_display[name] = now
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=1.0)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "discover":
            print(json.dumps({"serial": args.serial, "interface": discover_interface(args.serial)}))
            return 0
        if args.command == "record":
            return _run_record(args)
        if args.command == "fit":
            return _run_fit(args)
        if args.command == "monitor":
            return _run_monitor(args)
        if args.command == "evaluate":
            return _run_evaluate(args)
        if args.command == "known-load":
            return _run_known_load(args)
        raise PiperTorqueCliError(f"unsupported command {args.command!r}")
    except (
        PiperTorqueCliError,
        CalibrationMismatchError,
        CanDiscoveryError,
        DynamicsError,
        RecordError,
        OSError,
        ValueError,
    ) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
