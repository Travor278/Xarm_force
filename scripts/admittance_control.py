#!/usr/bin/env python3
"""
xArm6 Admittance Control — Spring-Back Compliant Behavior
==========================================================
Push the end-effector and the robot yields; release it and it springs
back to the home position.

Uses a momentum observer (De Luca 2005) for external torque estimation —
no numerical differentiation of q̇, giving fast response without noise.

Usage:
  python scripts/admittance_control.py --ip 192.168.1.199
  python scripts/admittance_control.py --ip 192.168.1.199 --damping 2.0 --stiffness 0.15
  python scripts/admittance_control.py --ip 192.168.1.199 --stiffness 0   # free-float
"""

import argparse
import os
import signal
import sys
import time

import numpy as np

from robot_control.constants import (
    DAMPING_SCALE,
    HOME_JOINTS_DEG,
    JOINT_LIMITS_HIGH,
    JOINT_LIMITS_LOW,
    LIMIT_BUFFER_DEG,
    N_JOINTS,
    STIFFNESS_SCALE,
    TORQUE_DEAD_ZONE,
)
from robot_control.dynamics import DynamicsModel, MomentumObserver
from robot_control.xarm_utils import connect, go_home, read_state


def run(arm, dyn: DynamicsModel, obs: MomentumObserver, args):
    """Main admittance control loop.

    Model per joint:  B·q̇ = τ_ext − K·(q − q_home)
    """
    B = args.damping * DAMPING_SCALE
    K = args.stiffness * STIFFNESS_SCALE
    v_max = np.full(N_JOINTS, args.max_vel)
    alpha_tau = args.filter_alpha
    q_home = np.array(HOME_JOINTS_DEG, dtype=np.float64)

    arm.set_mode(4)
    arm.set_state(0)
    time.sleep(0.2)

    dt = 1.0 / args.freq
    tau_filt = np.zeros(N_JOINTS)
    consecutive_errors = 0
    running = True

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    print(
        f"[ADM] Active — damping={args.damping:.1f}  stiffness={args.stiffness:.2f}"
        f"  max_vel={args.max_vel:.0f} deg/s  freq={args.freq} Hz  gain={args.gain:.0f}"
    )
    print("[ADM] Push the robot to feel compliance.  Ctrl-C to stop.\n")

    try:
        while running:
            t0 = time.monotonic()

            # Error recovery
            if arm.has_err_warn:
                print("[ADM] Error/warning — recovering ...")
                arm.vc_set_joint_velocity([0] * 7, is_radian=False)
                arm.clean_error()
                arm.clean_warn()
                arm.motion_enable(True)
                arm.set_mode(4)
                arm.set_state(0)
                time.sleep(0.5)
                consecutive_errors += 1
                if consecutive_errors > 3:
                    print("[ADM] Too many errors — stopping.")
                    break
                continue
            consecutive_errors = 0

            # 1) Read state
            q, qd, tau_meas, ok = read_state(arm)
            if not ok:
                time.sleep(dt)
                continue

            # 2) External torque via momentum observer (no q̈ needed)
            tau_ext = obs.update(q, qd, tau_meas, dt)

            # 3) Dead-zone
            for i in range(N_JOINTS):
                if abs(tau_ext[i]) < TORQUE_DEAD_ZONE[i]:
                    tau_ext[i] = 0.0
                else:
                    tau_ext[i] -= np.sign(tau_ext[i]) * TORQUE_DEAD_ZONE[i]

            # 4) Low-pass filter
            tau_filt = alpha_tau * tau_ext + (1.0 - alpha_tau) * tau_filt

            # 5) Admittance law:  q̇ = (τ_ext − K·Δq) / B
            delta_q = q - q_home
            qd_cmd = (tau_filt - K * delta_q) / B

            # 6) Soft joint-limit repulsion
            for i in range(N_JOINTS):
                if q[i] < JOINT_LIMITS_LOW[i] + LIMIT_BUFFER_DEG:
                    qd_cmd[i] = max(qd_cmd[i], 0.0)
                elif q[i] > JOINT_LIMITS_HIGH[i] - LIMIT_BUFFER_DEG:
                    qd_cmd[i] = min(qd_cmd[i], 0.0)

            # 7) Safety clip + send
            qd_cmd = np.clip(qd_cmd, -v_max, v_max)
            arm.vc_set_joint_velocity(list(qd_cmd) + [0.0],
                                      is_radian=False, duration=-1)

            # 8) Maintain loop rate
            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    finally:
        arm.vc_set_joint_velocity([0] * 7, is_radian=False)
        time.sleep(0.1)
        arm.set_mode(0)
        arm.set_state(0)
        print("\n[ADM] Stopped.")
        if args.return_home:
            go_home(arm)


def main():
    p = argparse.ArgumentParser(
        description="xArm6 spring-back admittance control")
    p.add_argument("--ip", default="192.168.1.199")
    p.add_argument("--damping", type=float, default=3.0,
                   help="Base damping [Nm/(deg/s)] (default: 3.0)")
    p.add_argument("--stiffness", type=float, default=0.3,
                   help="Spring stiffness [Nm/deg] (default: 0.3, 0=free-float)")
    p.add_argument("--max-vel", type=float, default=5.0,
                   help="Max joint velocity [deg/s] (default: 5)")
    p.add_argument("--freq", type=int, default=100,
                   help="Control loop frequency [Hz] (default: 100)")
    p.add_argument("--filter-alpha", type=float, default=0.3,
                   help="Torque EMA filter (0-1, default: 0.3)")
    p.add_argument("--gain", type=float, default=50.0,
                   help="Observer gain (higher=faster, default: 50)")
    p.add_argument("--bias-model", default="auto", metavar="FILE",
                   help="Gravity bias model .npz (default: auto-detect, 'none' to skip)")
    p.add_argument("--cal-time", type=float, default=2.0,
                   help="Zero-point calibration duration if no model (default: 2)")
    p.add_argument("--no-cal", action="store_true",
                   help="Skip all bias compensation")
    p.add_argument("--return-home", action=argparse.BooleanOptionalAction,
                   default=True, help="Return home on exit (default: True)")
    args = p.parse_args()

    # Load gravity bias model if available
    bias_model = None
    if not args.no_cal and args.bias_model != "none":
        model_path = args.bias_model
        if model_path == "auto":
            model_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, "assets", "gravity_bias_model.npz",
            )
        if os.path.exists(model_path):
            from robot_control.gravity_calibration import GravityBiasModel
            bias_model = GravityBiasModel.load(model_path)
            print(f"[CAL] Loaded gravity bias model from {model_path}")

    arm = connect(args.ip)
    dyn = DynamicsModel()
    obs = MomentumObserver(dyn, gain=args.gain, bias_model=bias_model)
    print("[ADM] Dynamics model + momentum observer loaded.")

    try:
        go_home(arm)

        # Fallback: constant zero-point calibration if no model
        dt = 1.0 / args.freq
        if not args.no_cal and bias_model is None:
            print(f"[CAL] No bias model found, constant calibrating {args.cal_time:.1f}s ...")
            cal_samples = []
            cal_start = time.monotonic()
            while time.monotonic() - cal_start < args.cal_time:
                q, qd, tau_meas, ok = read_state(arm)
                if ok:
                    tau_ext = obs.update(q, qd, tau_meas, dt)
                    cal_samples.append(tau_ext.copy())
                time.sleep(dt)
            n_skip = len(cal_samples) // 2
            if n_skip > 0:
                obs.calibrate(cal_samples[n_skip:])

        run(arm, dyn, obs, args)
    finally:
        dyn.close()
        arm.disconnect()
        print("[DONE] Disconnected.")


if __name__ == "__main__":
    main()
