from __future__ import annotations

from pathlib import Path

import pytest

from scripts.piperx_torque_web import (
    PiperWebCliError,
    build_parser,
    parse_arm_specs,
    validate_bind_host,
)


def test_parser_accepts_two_arm_loopback_configuration(tmp_path):
    parser = build_parser()
    args = parser.parse_args(
        [
            "--arm", f"left,serial-left,{tmp_path / 'left.json'}",
            "--arm", f"right,serial-right,{tmp_path / 'right.json'}",
            "--urdf", str(tmp_path / "piper.urdf"),
        ]
    )

    configs = parse_arm_specs(args.arm)

    assert [config.name for config in configs] == ["left", "right"]
    assert configs[0].calibration_path == tmp_path / "left.json"
    assert args.host == "127.0.0.1"
    assert args.port == 8765


@pytest.mark.parametrize(
    "specs",
    [
        ["left,serial-only"],
        ["left,serial,a.json", "left,serial,b.json"],
        [",serial,a.json"],
    ],
)
def test_arm_specs_reject_malformed_or_duplicate_identity(specs):
    with pytest.raises(PiperWebCliError):
        parse_arm_specs(specs)


def test_network_bind_requires_explicit_authorization():
    assert validate_bind_host("127.0.0.1", allow_network_bind=False) == "127.0.0.1"
    assert validate_bind_host("::1", allow_network_bind=False) == "::1"

    with pytest.raises(PiperWebCliError, match="loopback"):
        validate_bind_host("0.0.0.0", allow_network_bind=False)

    assert validate_bind_host("0.0.0.0", allow_network_bind=True) == "0.0.0.0"
