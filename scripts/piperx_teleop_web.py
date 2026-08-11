#!/usr/bin/env python3
"""Run standalone bimanual PiperX teleop and the torque dashboard together."""

from __future__ import annotations

import argparse
import ipaddress
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.piperx.calibration import CalibrationArtifact
from robot_control.piperx.dynamics import PinocchioDynamics
from robot_control.piperx.estimator import ExternalTorqueEstimator
from robot_control.piperx.filtering import VelocityDerivativeFilter
from robot_control.piperx.monitoring import LatestEventHub
from robot_control.piperx.sdk_teleop import (
    ArmIdentity,
    StandaloneTeleopRuntime,
    TeleopPairConfig,
    TeleopSafetyError,
    validate_pair_configs,
)
from robot_control.piperx.web_monitor import create_app


class PiperTeleopWebCliError(RuntimeError):
    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Standalone bimanual PiperX teleop with shared torque dashboard"
    )
    parser.add_argument(
        "--pair",
        action="append",
        required=True,
        metavar="NAME,LEADER_SERIAL,FOLLOWER_SERIAL,CALIBRATION",
        help="configure exactly the left and right PiperX pairs",
    )
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--base-rpy", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--speed-ratio", type=int, default=10)
    parser.add_argument("--gripper-effort", type=int, default=1000)
    parser.add_argument("--control-rate", type=float, default=200.0)
    parser.add_argument("--ui-rate", type=float, default=25.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18765)
    return parser


def parse_pair_specs(specs: list[str]) -> list[TeleopPairConfig]:
    configs: list[TeleopPairConfig] = []
    for spec in specs:
        parts = [part.strip() for part in spec.split(",")]
        if len(parts) != 4 or not all(parts):
            raise PiperTeleopWebCliError(
                "--pair must contain NAME,LEADER_SERIAL,FOLLOWER_SERIAL,CALIBRATION"
            )
        name, leader_serial, follower_serial, calibration = parts
        configs.append(
            TeleopPairConfig(
                name=name,
                leader=ArmIdentity(f"{name}-leader", leader_serial, "leader"),
                follower=ArmIdentity(f"{name}-follower", follower_serial, "follower"),
                calibration_path=Path(calibration),
            )
        )
    try:
        validate_pair_configs(configs)
    except TeleopSafetyError as error:
        raise PiperTeleopWebCliError(str(error)) from error
    return configs


def validate_bind_host(host: str) -> str:
    if host == "localhost":
        return host
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise PiperTeleopWebCliError("--host must be a loopback IP address") from error
    if not address.is_loopback:
        raise PiperTeleopWebCliError(
            "non-loopback HTTP bind refused; use SSH forwarding"
        )
    return host


def build_runtime(args: argparse.Namespace) -> StandaloneTeleopRuntime:
    pairs = parse_pair_specs(args.pair)
    estimators = {}
    for pair in pairs:
        calibration = CalibrationArtifact.load(pair.calibration_path)
        dynamics = PinocchioDynamics(args.urdf, base_rpy=args.base_rpy)
        estimators[pair.name] = ExternalTorqueEstimator(
            dynamics,
            VelocityDerivativeFilter(),
            calibration,
        )
    return StandaloneTeleopRuntime(
        pairs,
        estimators=estimators,
        hub=LatestEventHub(queue_size=len(pairs)),
        speed_ratio=args.speed_ratio,
        gripper_effort=args.gripper_effort,
        control_rate_hz=args.control_rate,
        ui_rate_hz=args.ui_rate,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        host = validate_bind_host(args.host)
        if not 1 <= args.port <= 65535:
            raise PiperTeleopWebCliError("--port must be in [1, 65535]")
        runtime = build_runtime(args)
        assets = Path(__file__).resolve().parents[1] / "web" / "piperx_monitor"
        app = create_app(runtime, assets)
        import uvicorn

        uvicorn.run(app, host=host, port=args.port, log_level="info")
        return 0
    except (OSError, ValueError, PiperTeleopWebCliError, TeleopSafetyError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
