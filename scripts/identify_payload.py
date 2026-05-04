#!/usr/bin/env python3
"""
xArm6 Payload / Mass Parameter Identification
===============================================
Identifies the missing gripper mass and COM by modifying the URDF
and minimizing gravity residuals across calibration poses.

Usage:
  python scripts/identify_payload.py
"""

import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, "/home/mrblue/Projects/robot_control")
from robot_control.constants import N_JOINTS, XARM6_URDF


def make_modified_urdf(mass, com_xyz, base_urdf=XARM6_URDF):
    """Create a temporary URDF with modified link_eef mass and COM."""
    tree = ET.parse(base_urdf)
    root = tree.getroot()

    for link in root.findall("link"):
        if link.get("name") == "link_eef":
            inertial = link.find("inertial")
            if inertial is None:
                inertial = ET.SubElement(link, "inertial")

            # Set mass
            mass_el = inertial.find("mass")
            if mass_el is None:
                mass_el = ET.SubElement(inertial, "mass")
            mass_el.set("value", f"{mass:.6f}")

            # Set COM origin
            origin = inertial.find("origin")
            if origin is None:
                origin = ET.SubElement(inertial, "origin")
            origin.set("xyz", f"{com_xyz[0]:.6f} {com_xyz[1]:.6f} {com_xyz[2]:.6f}")
            origin.set("rpy", "0 0 0")

            # Set inertia (small but nonzero)
            inertia = inertial.find("inertia")
            if inertia is None:
                inertia = ET.SubElement(inertial, "inertia")
            for attr in ["ixx", "iyy", "izz"]:
                inertia.set(attr, "0.001")
            for attr in ["ixy", "ixz", "iyz"]:
                inertia.set(attr, "0")
            break

    fd, path = tempfile.mkstemp(suffix=".urdf")
    os.close(fd)
    tree.write(path)
    return path


def compute_gravity(q_data, urdf_path):
    """Compute G(q) for all poses using a given URDF."""
    import pybullet as pb
    from robot_control.constants import PB_JOINT_INDICES

    cid = pb.connect(pb.DIRECT)
    pb.setGravity(0, 0, -9.81, physicsClientId=cid)
    robot = pb.loadURDF(urdf_path, useFixedBase=True, physicsClientId=cid)
    for j in PB_JOINT_INDICES:
        pb.changeDynamics(robot, j, linearDamping=0, angularDamping=0,
                          physicsClientId=cid)

    z = [0.0] * N_JOINTS
    G = np.zeros((len(q_data), N_JOINTS))
    for i, q_deg in enumerate(q_data):
        q_rad = np.deg2rad(q_deg[:N_JOINTS])
        for k, j in enumerate(PB_JOINT_INDICES):
            pb.resetJointState(robot, j, q_rad[k], physicsClientId=cid)
        tau = pb.calculateInverseDynamics(robot, list(q_rad), z, z,
                                          physicsClientId=cid)
        G[i] = np.array(tau[:N_JOINTS])

    pb.disconnect(cid)
    return G


def main():
    # Load calibration data
    data = np.load("assets/gravity_bias_model_data.npz")
    q_data = data["q"][:36]
    res_data = data["residual"][:36]

    # Compute current G(q) and τ_meas
    G_current = compute_gravity(q_data, XARM6_URDF)
    tau_meas = G_current + res_data

    print("=== Current URDF residual ===")
    for j in range(N_JOINTS):
        rmse = np.sqrt(np.mean(res_data[:, j] ** 2))
        print(f"  J{j+1}: RMSE={rmse:.3f} Nm")
    print()

    # Weights: emphasize J2, J5; reduce J3 (sensor noise)
    W = np.array([1.0, 3.0, 0.5, 1.0, 3.0, 1.0])

    eval_count = [0]

    def cost(params):
        mass = params[0]
        com = params[1:4]
        urdf_path = make_modified_urdf(mass, com)
        try:
            G_mod = compute_gravity(q_data, urdf_path)
            residual = tau_meas - G_mod
            err = np.sum((residual * W) ** 2)
        finally:
            os.unlink(urdf_path)
        eval_count[0] += 1
        if eval_count[0] % 10 == 0:
            print(f"  eval {eval_count[0]}: mass={mass:.3f} "
                  f"com=[{com[0]:.3f},{com[1]:.3f},{com[2]:.3f}] "
                  f"cost={err:.1f}")
        return err

    # Initial guess: xArm gripper ~0.85 kg, COM at ~6cm below flange
    x0 = [0.85, 0.0, 0.0, -0.06]
    bounds = [(0.1, 2.5), (-0.15, 0.15), (-0.15, 0.15), (-0.20, 0.05)]

    print("Optimizing payload parameters...")
    result = minimize(cost, x0, method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 100, "ftol": 1e-6})

    m_opt = result.x[0]
    com_opt = result.x[1:4]
    print(f"\n=== Identified payload ===")
    print(f"  Mass: {m_opt:.4f} kg")
    print(f"  COM:  [{com_opt[0]:.4f}, {com_opt[1]:.4f}, {com_opt[2]:.4f}] m (in link_eef frame)")
    print(f"  Evaluations: {eval_count[0]}")
    print()

    # Final residuals
    urdf_opt = make_modified_urdf(m_opt, com_opt)
    G_opt = compute_gravity(q_data, urdf_opt)
    res_opt = tau_meas - G_opt
    os.unlink(urdf_opt)

    print("=== Residual comparison ===")
    for j in range(N_JOINTS):
        rmse_before = np.sqrt(np.mean(res_data[:, j] ** 2))
        rmse_after = np.sqrt(np.mean(res_opt[:, j] ** 2))
        improve = (1 - rmse_after / max(rmse_before, 1e-6)) * 100
        print(f"  J{j+1}: {rmse_before:.3f} → {rmse_after:.3f} Nm  ({improve:+.0f}%)")

    # Save
    np.savez("assets/payload_params.npz",
             mass=m_opt, com=com_opt)
    print(f"\n[OK] Saved to assets/payload_params.npz")
    print(f"\nTo apply: update link_eef in URDF:")
    print(f'  <mass value="{m_opt:.4f}"/>')
    print(f'  <origin xyz="{com_opt[0]:.4f} {com_opt[1]:.4f} {com_opt[2]:.4f}"/>')


if __name__ == "__main__":
    main()
