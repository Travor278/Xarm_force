import robot_control.piperx as piperx


def test_package_exports_supported_library_entrypoints():
    expected = {
        "CalibrationArtifact",
        "ExternalTorqueEstimator",
        "GripperForceCalibration",
        "LatestEventHub",
        "PinocchioDynamics",
        "PiperStateAssembler",
        "ReadOnlySocketCan",
        "RigidPayload",
        "TorqueLog",
        "VelocityDerivativeFilter",
        "discover_interface",
        "fit_calibration",
        "fit_gripper_force_calibration",
        "load_torque_log",
        "save_torque_log",
        "serialize_snapshot",
    }

    assert set(piperx.__all__) == expected
    assert all(hasattr(piperx, name) for name in expected)
