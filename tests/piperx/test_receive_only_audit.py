from pathlib import Path


PROHIBITED_TOKENS = (
    ".send(",
    ".sendto(",
    ".sendmsg(",
    "ip link",
    "EnableArm",
    "MotionCtrl",
    "JointCtrl",
    "GripperCtrl",
    "JointMitCtrl",
)


def test_estimator_sources_contain_no_can_transmit_or_motion_control_path():
    repository = Path(__file__).resolve().parents[2]
    sources = list((repository / "robot_control" / "piperx").glob("*.py"))
    sources.append(repository / "scripts" / "piperx_external_torque.py")

    violations = []
    for source in sources:
        text = source.read_text(encoding="utf-8")
        for token in PROHIBITED_TOKENS:
            if token in text:
                violations.append(f"{source.relative_to(repository)}: {token}")

    assert violations == []
