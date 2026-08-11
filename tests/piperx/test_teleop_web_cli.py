from __future__ import annotations

from pathlib import Path

import pytest

from scripts.piperx_teleop_web import (
    PiperTeleopWebCliError,
    build_parser,
    parse_pair_specs,
    validate_bind_host,
)


def test_parser_accepts_exact_bimanual_standalone_configuration(tmp_path: Path):
    parser = build_parser()
    args = parser.parse_args(
        [
            "--pair",
            f"left,leader-left,follower-left,{tmp_path / 'left.json'}",
            "--pair",
            f"right,leader-right,follower-right,{tmp_path / 'right.json'}",
            "--urdf",
            str(tmp_path / "piper.urdf"),
        ]
    )

    pairs = parse_pair_specs(args.pair)

    assert [pair.name for pair in pairs] == ["left", "right"]
    assert pairs[0].leader.serial == "leader-left"
    assert pairs[0].follower.serial == "follower-left"
    assert args.control_rate == 200.0
    assert args.ui_rate == 25.0
    assert args.speed_ratio == 10
    assert args.gripper_effort == 1000


@pytest.mark.parametrize(
    "specs",
    [
        ["left,leader,follower"],
        ["left,a,b,l.json", "left,c,d,r.json"],
        ["left,a,b,l.json", "right,a,d,r.json"],
        ["left,a,b,l.json"],
    ],
)
def test_pair_specs_fail_closed_on_malformed_or_duplicate_identity(specs):
    with pytest.raises((PiperTeleopWebCliError, RuntimeError)):
        parse_pair_specs(specs)


def test_teleop_web_bind_remains_loopback_only():
    assert validate_bind_host("127.0.0.1") == "127.0.0.1"
    assert validate_bind_host("::1") == "::1"
    with pytest.raises(PiperTeleopWebCliError, match="loopback"):
        validate_bind_host("0.0.0.0")


def test_remote_wrapper_has_exclusive_controller_and_cleanup_guards():
    root = Path(__file__).resolve().parents[2]
    remote = (root / "scripts" / "run_piperx_teleop_web_remote.sh").read_text(
        encoding="utf-8"
    )
    launcher = (root / "scripts" / "open_piperx_teleop_monitor.ps1").read_text(
        encoding="utf-8"
    )
    combined = remote + launcher

    assert "/client/status" in remote
    assert '"idle"' in remote
    assert "systemctl stop evostudio-client" in remote
    assert "trap cleanup" in remote
    assert "systemctl start evostudio-client" in remote
    assert "systemd-run" in remote
    assert "piperx-teleop-guard" in remote
    assert "kill -0" in remote
    assert "ip link show dev" in remote
    assert "type can bitrate 1000000" in remote
    assert 'ip link set dev "$interface" up' in remote
    assert 'ip link set dev "$interface" down' not in remote
    assert "/home/dell/anaconda3/envs/evo-rl/bin/python" in remote
    assert "conda run" not in combined
    assert "run_piperx_teleop_web_remote.sh" in launcher
    assert "-tt" in launcher
    assert 'Start-Process -FilePath "powershell.exe"' in launcher
    assert "-EncodedCommand" in launcher
    assert "-WindowStyle Normal" in launcher
    assert "password=" not in combined.lower()
    assert "-pw " not in combined.lower()
