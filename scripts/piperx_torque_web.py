#!/usr/bin/env python3
"""Run the receive-only PiperX torque dashboard on a loopback socket."""

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
from robot_control.piperx.web_monitor import (
    ArmAcquisitionWorker,
    ArmMonitorConfig,
    MonitorRuntime,
    create_app,
)


class PiperWebCliError(RuntimeError):
    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receive-only PiperX external torque web monitor"
    )
    parser.add_argument(
        "--arm",
        action="append",
        required=True,
        metavar="NAME,SERIAL,CALIBRATION",
        help="repeat once per follower arm",
    )
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--base-rpy", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--ui-rate", type=float, default=50.0)
    parser.add_argument(
        "--firmware",
        action="append",
        default=[],
        metavar="NAME=S-VERSION",
        help="optional per-arm firmware override; no firmware query is transmitted",
    )
    parser.add_argument(
        "--allow-network-bind",
        action="store_true",
        help="explicitly allow a non-loopback HTTP bind",
    )
    return parser


def parse_arm_specs(specs: list[str]) -> list[ArmMonitorConfig]:
    configs: list[ArmMonitorConfig] = []
    names: set[str] = set()
    for spec in specs:
        parts = [part.strip() for part in spec.split(",")]
        if len(parts) != 3 or not all(parts):
            raise PiperWebCliError(
                "--arm must contain exactly NAME,SERIAL,CALIBRATION"
            )
        name, serial, path = parts
        if name in names:
            raise PiperWebCliError(f"duplicate arm name {name!r}")
        names.add(name)
        configs.append(ArmMonitorConfig(name, serial, Path(path)))
    return configs


def _parse_firmware(values: list[str], arm_names: set[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        name, separator, version = value.partition("=")
        if not separator or name not in arm_names or not version.startswith("S-V"):
            raise PiperWebCliError(
                "--firmware must be NAME=S-VERSION for a configured arm"
            )
        overrides[name] = version
    return overrides


def validate_bind_host(host: str, *, allow_network_bind: bool) -> str:
    if host == "localhost":
        return host
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise PiperWebCliError("--host must be an IP address or localhost") from error
    if not address.is_loopback and not allow_network_bind:
        raise PiperWebCliError(
            "non-loopback HTTP bind refused; use SSH forwarding or explicitly pass "
            "--allow-network-bind"
        )
    return host


def build_runtime(args: argparse.Namespace) -> MonitorRuntime:
    configs = parse_arm_specs(args.arm)
    firmware = _parse_firmware(args.firmware, {config.name for config in configs})
    hub = LatestEventHub()
    workers = []
    for config in configs:
        calibration = CalibrationArtifact.load(config.calibration_path)
        dynamics = PinocchioDynamics(args.urdf, base_rpy=args.base_rpy)
        estimator = ExternalTorqueEstimator(
            dynamics,
            VelocityDerivativeFilter(),
            calibration,
        )
        configured = ArmMonitorConfig(
            config.name,
            config.serial,
            config.calibration_path,
            firmware_override=firmware.get(config.name),
        )
        workers.append(
            ArmAcquisitionWorker(
                configured,
                estimator=estimator,
                hub=hub,
                ui_rate_hz=args.ui_rate,
            )
        )
    return MonitorRuntime(workers, hub)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        host = validate_bind_host(
            args.host, allow_network_bind=args.allow_network_bind
        )
        if not 1 <= args.port <= 65535:
            raise PiperWebCliError("--port must be in [1, 65535]")
        runtime = build_runtime(args)
        assets = Path(__file__).resolve().parents[1] / "web" / "piperx_monitor"
        app = create_app(runtime, assets)
        import uvicorn

        uvicorn.run(app, host=host, port=args.port, log_level="info")
        return 0
    except (OSError, ValueError, PiperWebCliError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
