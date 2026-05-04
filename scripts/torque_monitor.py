#!/usr/bin/env python3
"""
xArm6 Real-Time Torque Monitor
===============================
Displays compensated joint torques, position, and velocity in the terminal.

Uses a momentum observer (De Luca 2005) instead of numerical differentiation
for external torque estimation — zero delay, low noise.

The displayed τ_ext should stay near zero regardless of pose or motion.
Non-zero readings indicate actual external forces on the robot.

Usage:
  python scripts/torque_monitor.py --ip 192.168.1.199
  python scripts/torque_monitor.py --ip 192.168.1.199 --log          # auto-named CSV
  python scripts/torque_monitor.py --ip 192.168.1.199 --log my.csv   # custom name
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, "/home/mrblue/Projects/robot_control")
from robot_control.constants import N_JOINTS
from robot_control.dynamics import DynamicsModel, MomentumObserver
from robot_control.xarm_utils import connect, read_state

from rich.live import Live
from rich.table import Table
from rich.text import Text


def color_torque(val: float, threshold: float = 0.5) -> Text:
    """Color-code a torque value: green near zero, yellow medium, red high."""
    s = f"{val:+8.3f}"
    if abs(val) < threshold:
        return Text(s, style="green")
    elif abs(val) < threshold * 3:
        return Text(s, style="yellow")
    else:
        return Text(s, style="bold red")


def build_table(q, qd, tau_meas, tau_grav, tau_cori, tau_ext, freq_actual: float):
    """Build a rich Table for one display frame."""
    tau_other = tau_meas - tau_grav - tau_cori - tau_ext

    t = Table(
        title=f"xArm6 Torque Monitor  ({freq_actual:.0f} Hz)  —  Ctrl-C to stop",
        show_lines=True,
    )
    t.add_column("Joint", style="bold", width=6)
    t.add_column("q (°)", justify="right", width=9)
    t.add_column("q̇ (°/s)", justify="right", width=9)
    t.add_column("τ_meas", justify="right", width=9)
    t.add_column("G(q)", justify="right", width=9)
    t.add_column("C(q,q̇)", justify="right", width=9)
    t.add_column("τ_other", justify="right", width=9)
    t.add_column("τ_ext", justify="right", width=9)

    for i in range(N_JOINTS):
        t.add_row(
            f"J{i+1}",
            f"{q[i]:+8.2f}",
            f"{qd[i]:+8.2f}",
            f"{tau_meas[i]:+8.3f}",
            f"{tau_grav[i]:+8.3f}",
            f"{tau_cori[i]:+8.3f}",
            f"{tau_other[i]:+8.3f}",
            color_torque(tau_ext[i]),
        )
    return t


LOG_HEADER = (
    ["timestamp", "elapsed_s"]
    + [f"q{i+1}_deg" for i in range(N_JOINTS)]
    + [f"qd{i+1}_dps" for i in range(N_JOINTS)]
    + [f"tau_meas{i+1}" for i in range(N_JOINTS)]
    + [f"tau_grav{i+1}" for i in range(N_JOINTS)]
    + [f"tau_cori{i+1}" for i in range(N_JOINTS)]
    + [f"tau_ext{i+1}" for i in range(N_JOINTS)]
)


def main():
    p = argparse.ArgumentParser(description="xArm6 real-time torque monitor")
    p.add_argument("--ip", default="192.168.1.199")
    p.add_argument("--freq", type=int, default=100,
                   help="Internal loop frequency (Hz)")
    p.add_argument("--display-rate", type=int, default=20,
                   help="Terminal refresh rate (Hz)")
    p.add_argument("--gain", type=float, default=50.0,
                   help="Observer gain (higher=faster response, default: 50)")
    p.add_argument("--bias-model", default="auto", metavar="FILE",
                   help="Gravity bias model .npz (default: auto-detect, 'none' to skip)")
    p.add_argument("--cal-time", type=float, default=2.0,
                   help="Zero-point calibration duration if no model (default: 2)")
    p.add_argument("--no-cal", action="store_true",
                   help="Skip all bias compensation")
    p.add_argument("--log", nargs="?", const="auto", default=None,
                   metavar="FILE",
                   help="Log data to CSV (default: auto-named in logs/)")
    args = p.parse_args()

    # Set up logging
    log_file = None
    csv_writer = None
    if args.log is not None:
        if args.log == "auto":
            log_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), os.pardir, "logs"
            )
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(log_dir, f"monitor_{stamp}.csv")
        else:
            log_path = args.log
        log_file = open(log_path, "w", newline="")
        csv_writer = csv.writer(log_file)
        csv_writer.writerow(LOG_HEADER)
        print(f"[LOG] Recording to {os.path.abspath(log_path)}")

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
    print("[MONITOR] Dynamics model + momentum observer loaded.")

    # Fallback: constant zero-point calibration if no model
    dt = 1.0 / args.freq
    if not args.no_cal and bias_model is None:
        print(f"[CAL] Calibrating for {args.cal_time:.1f}s — keep robot still ...")
        cal_samples = []
        cal_start = time.monotonic()
        while time.monotonic() - cal_start < args.cal_time:
            q, qd, tau_meas, ok = read_state(arm)
            if ok:
                tau_ext = obs.update(q, qd, tau_meas, dt)
                cal_samples.append(tau_ext.copy())
            time.sleep(dt)
        # Use last half (after observer converges)
        n_skip = len(cal_samples) // 2
        if n_skip > 0:
            obs.calibrate(cal_samples[n_skip:])
    print()

    display_interval = 1.0 / args.display_rate
    last_display = 0.0
    freq_actual = 0.0
    loop_count = 0
    t_freq_start = time.monotonic()
    t_start = time.monotonic()

    z = np.zeros(N_JOINTS)
    d_q = z.copy()
    d_qd = z.copy()
    d_tau_meas = z.copy()
    d_tau_grav = z.copy()
    d_tau_cori = z.copy()
    d_tau_ext = z.copy()

    running = True
    logged_rows = 0

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    try:
        with Live(build_table(z, z, z, z, z, z, 0),
                  refresh_per_second=args.display_rate) as live:
            while running:
                t0 = time.monotonic()

                q, qd, tau_meas, ok = read_state(arm)
                if not ok:
                    time.sleep(dt)
                    continue

                # Momentum observer → τ_ext (no q̈ needed)
                tau_ext = obs.update(q, qd, tau_meas, dt)

                # Analytical decomposition for display
                tau_grav = dyn.gravity(q)
                tau_cori = dyn.coriolis(q, qd)

                d_q = q
                d_qd = qd
                d_tau_meas = tau_meas
                d_tau_grav = tau_grav
                d_tau_cori = tau_cori
                d_tau_ext = tau_ext

                # Log every sample
                if csv_writer is not None:
                    elapsed = t0 - t_start
                    row = (
                        [f"{t0:.6f}", f"{elapsed:.4f}"]
                        + [f"{v:.4f}" for v in q]
                        + [f"{v:.4f}" for v in qd]
                        + [f"{v:.4f}" for v in tau_meas]
                        + [f"{v:.4f}" for v in tau_grav]
                        + [f"{v:.4f}" for v in tau_cori]
                        + [f"{v:.4f}" for v in tau_ext]
                    )
                    csv_writer.writerow(row)
                    logged_rows += 1

                # Frequency measurement
                loop_count += 1
                elapsed_freq = t0 - t_freq_start
                if elapsed_freq >= 1.0:
                    freq_actual = loop_count / elapsed_freq
                    loop_count = 0
                    t_freq_start = t0

                # Update display at lower rate
                if t0 - last_display >= display_interval:
                    live.update(build_table(
                        d_q, d_qd,
                        d_tau_meas, d_tau_grav, d_tau_cori,
                        d_tau_ext, freq_actual,
                    ))
                    last_display = t0

                elapsed = time.monotonic() - t0
                if elapsed < dt:
                    time.sleep(dt - elapsed)

    finally:
        if log_file is not None:
            log_file.close()
            print(f"[LOG] Saved {logged_rows} rows.")
        dyn.close()
        arm.disconnect()
        print("\n[MONITOR] Stopped.")


if __name__ == "__main__":
    main()
