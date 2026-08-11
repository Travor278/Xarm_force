from __future__ import annotations

import hashlib
import math

import numpy as np
import pytest

from robot_control.piperx.dynamics import (
    DynamicsError,
    PinocchioDynamics,
    gravity_in_base,
    sha256_file,
)


def test_sha256_file_hashes_exact_bytes(tmp_path):
    path = tmp_path / "model.urdf"
    path.write_bytes(b"piperx-model\n")

    assert sha256_file(path) == hashlib.sha256(b"piperx-model\n").hexdigest()


def test_gravity_is_expressed_in_rotated_base_coordinates():
    assert gravity_in_base((0.0, 0.0, 0.0)) == pytest.approx([0.0, 0.0, -9.81])
    assert gravity_in_base((math.pi, 0.0, 0.0)) == pytest.approx(
        [0.0, 0.0, 9.81], abs=1e-12
    )
    assert gravity_in_base((0.0, math.pi / 2, 0.0)) == pytest.approx(
        [9.81, 0.0, 0.0], abs=1e-12
    )


_SIX_JOINT_URDF = """<?xml version="1.0"?>
<robot name="six_joint_test">
  <link name="base"/>
  {links}
</robot>
"""


def _joint_fragment(index: int) -> str:
    parent = "base" if index == 1 else f"link{index - 1}"
    return f"""
  <link name="link{index}">
    <inertial><origin xyz="0 0 0"/><mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint{index}" type="revolute">
    <parent link="{parent}"/><child link="link{index}"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>
  </joint>"""


def _write_urdf(tmp_path, joint_count=6):
    path = tmp_path / "arm.urdf"
    path.write_text(
        _SIX_JOINT_URDF.format(
            links="\n".join(_joint_fragment(index) for index in range(1, joint_count + 1))
        ),
        encoding="utf-8",
    )
    return path


def test_pinocchio_backend_computes_finite_six_joint_torque(tmp_path):
    pytest.importorskip("pinocchio")
    backend = PinocchioDynamics(_write_urdf(tmp_path), base_rpy=(0.0, 0.0, 0.0))

    torque = backend.compute(np.zeros(6), np.zeros(6), np.zeros(6))

    assert torque.shape == (6,)
    assert np.all(np.isfinite(torque))


def test_pinocchio_backend_rejects_non_six_joint_model(tmp_path):
    pytest.importorskip("pinocchio")

    with pytest.raises(DynamicsError, match="six actuated joints"):
        PinocchioDynamics(_write_urdf(tmp_path, joint_count=5))


def test_pinocchio_backend_rejects_wrong_state_shape(tmp_path):
    pytest.importorskip("pinocchio")
    backend = PinocchioDynamics(_write_urdf(tmp_path))

    with pytest.raises(DynamicsError, match="shape"):
        backend.compute(np.zeros(5), np.zeros(6), np.zeros(6))

