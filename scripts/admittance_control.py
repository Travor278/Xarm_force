#!/usr/bin/env python3
"""
xArm6 Admittance Control — Spring-Back Compliant Behavior
==========================================================
Push the end-effector and the robot yields; release it and it springs
back to the home position.

Uses direct subtraction with smooth blend factor and online firmware
bias detection for external torque estimation (same as torque_monitor).

Usage:
  python scripts/admittance_control.py --ip 192.168.1.199
  python scripts/admittance_control.py --ip 192.168.1.199 --damping 2.0 --stiffness 0.15
  python scripts/admittance_control.py --ip 192.168.1.199 --stiffness 0   # free-float
"""

import argparse
import signal
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
from robot_control.dynamics import DynamicsModel
from robot_control.xarm_utils import connect, go_home, read_state

# -- Force-estimation parameters (from torque_monitor) -------------------------

# Speed threshold for binary motion detector input to EMA (deg/s).
MOTION_SPEED_THR = 2.0

# EMA smoothing factor for blend factor lambda.
BLEND_ALPHA = 0.2

# Firmware discrete bias levels per joint (from k-means on 18 stop events).
#                   LOW        HIGH
BIAS_LEVELS = {
    1: (  0.0000,   0.0000),
    2: ( -4.5056,  +4.0151),
    3: ( -2.4267,  +3.3561),
    4: ( +0.0857,  +0.0857),
    5: ( -1.9928,  +0.9617),
    6: ( +0.1845,  +0.1845),
}
BIAS_MIDPOINTS = np.array([
    (BIAS_LEVELS[j+1][0] + BIAS_LEVELS[j+1][1]) / 2 for j in range(N_JOINTS)
])
BIAS_IS_BIMODAL = np.array([
    BIAS_LEVELS[j+1][0] != BIAS_LEVELS[j+1][1] for j in range(N_JOINTS)
])

# Detection parameters
SETTLE_LAM_THR = 0.05
DETECT_EMA_ALPHA = 0.05


def run(arm, dyn: DynamicsModel, args):
    """Main admittance control loop.

    Model per joint:  B·q̇ = τ_ext − K·(q − q_home)

    External torque estimation uses direct subtraction with smooth blend
    and online firmware bias detection (same as torque_monitor).
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

    # Force-estimation state (from torque_monitor)
    lam = 0.0
    detected_bias = np.zeros(N_JOINTS)
    resid_ema = np.zeros(N_JOINTS)
    ema_initialized = False

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    print(
        f"[ADM] Active — damping={args.damping:.1f}  stiffness={args.stiffness:.2f}"
        f"  max_vel={args.max_vel:.0f} deg/s  freq={args.freq} Hz"
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

            # 2) External torque via direct subtraction + online bias
            tau_model = dyn.gravity(q) + dyn.coriolis(q, qd)
            resid = tau_meas - tau_model

            # Smooth blend factor lambda
            max_speed = np.abs(qd).max()
            is_moving = float(max_speed > MOTION_SPEED_THR)
            lam = (1.0 - BLEND_ALPHA) * lam + BLEND_ALPHA * is_moving

            # Online bias detection (continuous)
            if lam < SETTLE_LAM_THR:
                if not ema_initialized:
                    resid_ema[:] = resid
                    ema_initialized = True
                else:
                    resid_ema += DETECT_EMA_ALPHA * (resid - resid_ema)

                for i in range(N_JOINTS):
                    if BIAS_IS_BIMODAL[i]:
                        lo, hi = BIAS_LEVELS[i + 1]
                        if resid_ema[i] > BIAS_MIDPOINTS[i]:
                            detected_bias[i] = hi
                        else:
                            detected_bias[i] = lo
                    else:
                        detected_bias[i] = BIAS_LEVELS[i + 1][0]
            else:
                ema_initialized = False

            tau_jump = detected_bias * (1.0 - lam)
            tau_ext = tau_model + tau_jump - tau_meas

            # 3) Dead-zone
            for i in range(N_JOINTS):
                if abs(tau_ext[i]) < TORQUE_DEAD_ZONE[i]:
                    tau_ext[i] = 0.0
                else:
                    tau_ext[i] -= np.sign(tau_ext[i]) * TORQUE_DEAD_ZONE[i]

            # 4) Low-pass filter
            tau_filt = alpha_tau * tau_ext + (1.0 - alpha_tau) * tau_filt

            # 5) Admittance law:  q̇ = (τ_ext − K·Δq) / B_eff
            delta_q = q - q_home
            # Gaussian damping: B increases near home to suppress oscillation
            #   B_eff = B * (1 + gain * exp(-Δq² / (2σ²)))
            gauss = np.exp(-delta_q**2 / (2.0 * args.damping_sigma**2))
            B_eff = B * (1.0 + args.damping_gain * gauss)
            qd_cmd = (tau_filt - K * delta_q) / B_eff

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
    p.add_argument("--damping-gain", type=float, default=3.0,
                   help="Extra damping multiplier at home (default: 3.0)")
    p.add_argument("--damping-sigma", type=float, default=5.0,
                   help="Gaussian width for extra damping [deg] (default: 5.0)")
    p.add_argument("--return-home", action=argparse.BooleanOptionalAction,
                   default=True, help="Return home on exit (default: True)")
    args = p.parse_args()

    arm = connect(args.ip)
    dyn = DynamicsModel()
    print("[ADM] Dynamics model loaded.")
    print(f"[ADM] Bimodal joints: {[j+1 for j in range(N_JOINTS) if BIAS_IS_BIMODAL[j]]}")

    try:
        go_home(arm)
        run(arm, dyn, args)
    finally:
        dyn.close()
        arm.disconnect()
        print("[DONE] Disconnected.")


if __name__ == "__main__":
    main()
