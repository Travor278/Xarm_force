#!/usr/bin/env python3
"""
xArm6 Gravity Bias Calibration
===============================
Moves the robot to a grid of static poses, records the raw residual
(τ_meas − G_model) at each pose, and fits a position-dependent bias model.

Uses raw torque residuals instead of the momentum observer to avoid
J3 sensor jump accumulation issues during calibration.

Usage:
  python scripts/calibrate_gravity.py --ip 192.168.1.199
  python scripts/calibrate_gravity.py --ip 192.168.1.199 --settle 1.5 --record 1.5
"""

import argparse
import itertools
import os
import sys
import time

import numpy as np

sys.path.insert(0, "/home/mrblue/Projects/robot_control")
from robot_control.constants import HOME_JOINTS_DEG, N_JOINTS
from robot_control.dynamics import DynamicsModel
from robot_control.gravity_calibration import GravityBiasModel
from robot_control.xarm_utils import connect, go_home, read_state

# Calibration grid: J1 (left/right), J2/J3 (small vertical), J4/J5 (wrist)
J1_GRID = [-20, 0, 20, 40]
J2_GRID = [-12, -4, 4]
J3_GRID = [-30, -22, -15]
J4_GRID = [180, 200, 220]
J5_GRID = [50, 65, 80]

# J3 sensor jump: known ~5.28 Nm discrete offset
J3_JUMP_SIZE = 5.28
J3_JUMP_HALF = J3_JUMP_SIZE / 2


def generate_poses():
    """Generate calibration poses: cycle J4/J5 while sweeping J1/J2/J3."""
    home = list(HOME_JOINTS_DEG)
    poses = []
    j45_combos = list(itertools.product(J4_GRID, J5_GRID))
    idx45 = 0
    for q1, q2, q3 in itertools.product(J1_GRID, J2_GRID, J3_GRID):
        q4, q5 = j45_combos[idx45 % len(j45_combos)]
        idx45 += 1
        pose = home.copy()
        pose[0] = q1; pose[1] = q2; pose[2] = q3
        pose[3] = q4; pose[4] = q5
        poses.append(pose)
    return poses


def collect_raw(arm, dyn, dt, record_time):
    """Collect τ_meas and G(q) at current static pose.

    Returns (q_mean, residual_mean, tau_meas3_median).
    """
    samples_q = []
    samples_res = []
    samples_tau3 = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < record_time:
        q, qd, tau_meas, ok = read_state(arm)
        if ok:
            g = dyn.gravity(q)
            residual = tau_meas - g
            samples_q.append(q.copy())
            samples_res.append(residual.copy())
            samples_tau3.append(tau_meas[2])
        time.sleep(dt)
    return (np.mean(samples_q, axis=0),
            np.mean(samples_res, axis=0),
            np.median(samples_tau3))


def main():
    p = argparse.ArgumentParser(description="xArm6 gravity bias calibration")
    p.add_argument("--ip", default="192.168.1.199")
    p.add_argument("--speed", type=float, default=15,
                   help="Joint move speed (deg/s, default: 15)")
    p.add_argument("--settle", type=float, default=1.0,
                   help="Settle time after reaching pose (s, default: 1.0)")
    p.add_argument("--record", type=float, default=1.5,
                   help="Recording time per pose (s, default: 1.5)")
    p.add_argument("--freq", type=int, default=100,
                   help="Sampling frequency (Hz)")
    p.add_argument("--output", default=None,
                   help="Output model path (default: assets/gravity_bias_model.npz)")
    args = p.parse_args()

    if args.output is None:
        args.output = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            os.pardir, "assets", "gravity_bias_model.npz",
        )

    poses = generate_poses()
    print(f"[CAL] {len(poses)} calibration poses")
    est = len(poses) * (args.settle + args.record + 3)
    print(f"[CAL] Estimated time: {est:.0f}s (~{est/60:.1f} min)")
    print()

    arm = connect(args.ip)
    dyn = DynamicsModel()
    dt = 1.0 / args.freq

    go_home(arm, speed=args.speed)
    time.sleep(1.0)

    all_q = []
    all_res = []
    all_tau3_median = []

    try:
        for idx, pose in enumerate(poses):
            print(f"[CAL] Pose {idx+1}/{len(poses)}: "
                  f"J1={pose[0]:+.0f} J2={pose[1]:+.0f} J3={pose[2]:+.0f} "
                  f"J4={pose[3]:+.0f} J5={pose[4]:+.0f}  ",
                  end="", flush=True)

            arm.set_mode(0)
            arm.set_state(0)
            arm.set_servo_angle(
                angle=pose, speed=args.speed, mvacc=200,
                is_radian=False, wait=True,
            )
            time.sleep(args.settle)

            q_mean, res_mean, tau3_med = collect_raw(arm, dyn, dt, args.record)
            all_q.append(q_mean)
            all_res.append(res_mean)
            all_tau3_median.append(tau3_med)
            print(f"res=[{', '.join(f'{v:+.2f}' for v in res_mean)}]  "
                  f"tau3_raw={tau3_med:+.2f}")

        all_q = np.array(all_q)
        all_res = np.array(all_res)
        all_tau3_median = np.array(all_tau3_median)

        # Post-process J3: normalize sensor state
        # The J3 sensor has two states separated by ~5.28 Nm.
        # Use the median τ_meas3 at each pose to detect which state.
        # Strategy: use the first pose as reference, shift others to match.
        ref_tau3 = all_tau3_median[0]
        j3_corrections = np.zeros(len(all_q))
        for i in range(1, len(all_q)):
            # Expected τ_meas3 difference from gravity model
            g3_ref = dyn.gravity(all_q[0])[2]
            g3_i = dyn.gravity(all_q[i])[2]
            expected_delta = g3_i - g3_ref

            # Actual τ_meas3 difference
            actual_delta = all_tau3_median[i] - ref_tau3

            # If the difference from expected is close to ±5.28, it's a jump
            jump_residual = actual_delta - expected_delta
            n_jumps = round(jump_residual / J3_JUMP_SIZE)
            if n_jumps != 0:
                j3_corrections[i] = n_jumps * J3_JUMP_SIZE

        n_corrected = np.count_nonzero(j3_corrections)
        if n_corrected > 0:
            print(f"\n[CAL] J3 sensor state correction: {n_corrected}/{len(all_q)} poses adjusted")
            all_res[:, 2] -= j3_corrections

        # Fit model
        print(f"\n[CAL] Fitting model on {len(all_q)} poses ...")
        model = GravityBiasModel()
        model.fit(all_q, all_res)
        model.summary()

        # Show residuals
        print("\nPer-joint fit residuals:")
        for j in range(N_JOINTS):
            preds = np.array([model.predict(q)[j] for q in all_q])
            rmse = np.sqrt(np.mean((all_res[:, j] - preds) ** 2))
            maxe = np.abs(all_res[:, j] - preds).max()
            print(f"  J{j+1}: RMSE={rmse:.4f}  max={maxe:.4f}")

        # Save
        model.save(args.output)
        print(f"\n[CAL] Model saved to {os.path.abspath(args.output)}")
        raw_path = args.output.replace(".npz", "_data.npz")
        np.savez(raw_path, q=all_q, residual=all_res,
                 tau3_median=all_tau3_median, j3_corrections=j3_corrections)
        print(f"[CAL] Raw data saved to {os.path.abspath(raw_path)}")

    finally:
        print("\n[CAL] Returning home ...")
        arm.set_mode(0)
        arm.set_state(0)
        go_home(arm, speed=args.speed)
        dyn.close()
        arm.disconnect()
        print("[DONE] Calibration complete.")


if __name__ == "__main__":
    main()
