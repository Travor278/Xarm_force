#!/usr/bin/env python3
"""
xArm6 Real-Time Force Monitor
==============================
Estimates external joint torques via direct subtraction:

  tau_ext = tau_api - tau_model(q, qd) - tau_comp(q, qd) - tau_jump

All parameters are fixed and hardcoded. No runtime calibration.
Model output is deterministic -- depends only on (q, qd) read from API.

The firmware uses different internal compensation when the arm is moving
vs stationary. The total compensation splits into:
  tau_comp: base continuous compensation (always active, depends on q/qd)
  tau_jump: online-detected discrete firmware bias (two levels per joint)

After each motion->static transition, the firmware settles into one of
two discrete compensation levels for J2, J3, J5. Online detection reads
the raw residual after settling and snaps to the nearest known level.

Usage:
  python scripts/torque_monitor.py --ip 192.168.1.199
  python scripts/torque_monitor.py --ip 192.168.1.199 --log
"""

import argparse
import csv
import os
import signal
import time
from datetime import datetime

import numpy as np

from robot_control.constants import N_JOINTS
from robot_control.dynamics import DynamicsModel
from robot_control.xarm_utils import connect, read_state

from rich.live import Live
from rich.table import Table
from rich.text import Text

# -- Fixed parameters (hardcoded) ------------------------------------------

# Speed threshold for binary motion detector input to EMA (deg/s).
MOTION_SPEED_THR = 2.0

# EMA smoothing factor for blend factor lambda.
BLEND_ALPHA = 0.2

# Firmware discrete bias levels per joint.
# Bimodal joints (J2, J3, J5) have two known levels; unimodal joints use
# a single fixed value. Levels from k-means clustering of static residuals
# across 18 stop events.
#                   LOW        HIGH
BIAS_LEVELS = {
    1: (  0.0000,   0.0000),  # unimodal, ~0
    2: ( -4.5056,  +4.0151),  # bimodal, gap=8.52 Nm
    3: ( -2.4267,  +3.3561),  # bimodal, gap=5.78 Nm
    4: ( +0.0857,  +0.0857),  # unimodal (q-dependent, handled by tau_comp)
    5: ( -1.9928,  +0.9617),  # bimodal, gap=2.95 Nm
    6: ( +0.1845,  +0.1845),  # unimodal
}
# Midpoint for classification: (LOW + HIGH) / 2
BIAS_MIDPOINTS = np.array([
    (BIAS_LEVELS[j+1][0] + BIAS_LEVELS[j+1][1]) / 2 for j in range(N_JOINTS)
])
# Whether the joint is bimodal (needs online detection)
BIAS_IS_BIMODAL = np.array([
    BIAS_LEVELS[j+1][0] != BIAS_LEVELS[j+1][1] for j in range(N_JOINTS)
])

# Detection parameters
SETTLE_LAM_THR = 0.05      # lambda threshold for "settled"
DETECT_EMA_ALPHA = 0.05    # EMA smoothing for residual tracking in static state

# ---------------------------------------------------------------------------


def color_torque(val: float, threshold: float = 0.5) -> Text:
    s = f"{val:+8.3f}"
    if abs(val) < threshold:
        return Text(s, style="green")
    elif abs(val) < threshold * 3:
        return Text(s, style="yellow")
    else:
        return Text(s, style="bold red")


def build_table(q, tau_api, tau_model, tau_comp, tau_jump, tau_ext, lam, freq_actual: float):
    t = Table(
        title=f"xArm6 Force Monitor  ({freq_actual:.0f} Hz)  "
              f"[lambda={lam:.2f}]  --  Ctrl-C to stop",
        show_lines=True,
    )
    t.add_column("Joint", style="bold", width=6)
    t.add_column("q (deg)", justify="right", width=9)
    t.add_column("tau_api", justify="right", width=9)
    t.add_column("tau_model", justify="right", width=9)
    t.add_column("tau_comp", justify="right", width=9)
    t.add_column("tau_jump", justify="right", width=9)
    t.add_column("tau_ext", justify="right", width=9)

    for i in range(N_JOINTS):
        t.add_row(
            f"J{i+1}",
            f"{q[i]:+8.2f}",
            f"{tau_api[i]:+8.3f}",
            f"{tau_model[i]:+8.3f}",
            f"{tau_comp[i]:+8.3f}",
            f"{tau_jump[i]:+8.3f}",
            color_torque(tau_ext[i]),
        )
    return t


LOG_HEADER = (
    ["timestamp", "elapsed_s", "lambda"]
    + [f"q{i+1}_deg" for i in range(N_JOINTS)]
    + [f"tau_api{i+1}" for i in range(N_JOINTS)]
    + [f"tau_model{i+1}" for i in range(N_JOINTS)]
    + [f"tau_comp{i+1}" for i in range(N_JOINTS)]
    + [f"tau_jump{i+1}" for i in range(N_JOINTS)]
    + [f"tau_ext{i+1}" for i in range(N_JOINTS)]
)


def main():
    p = argparse.ArgumentParser(description="xArm6 real-time force monitor")
    p.add_argument("--ip", default="192.168.1.199")
    p.add_argument("--freq", type=int, default=100,
                   help="Internal loop frequency (Hz)")
    p.add_argument("--display-rate", type=int, default=20,
                   help="Terminal refresh rate (Hz)")
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

    arm = connect(args.ip)
    dyn = DynamicsModel()
    print("[MONITOR] Dynamics model loaded.")
    print(f"[MONITOR] Motion threshold = {MOTION_SPEED_THR} deg/s")
    print(f"[MONITOR] Blend alpha = {BLEND_ALPHA}")
    print(f"[MONITOR] Bimodal joints: {[j+1 for j in range(N_JOINTS) if BIAS_IS_BIMODAL[j]]}")
    print()

    dt = 1.0 / args.freq

    # Smooth blend factor (EMA state)
    lam = 0.0

    # Online bias detection state: continuous EMA tracking of residual in static
    detected_bias = np.zeros(N_JOINTS)
    resid_ema = np.zeros(N_JOINTS)  # smoothed residual for classification
    ema_initialized = False

    display_interval = 1.0 / args.display_rate
    last_display = 0.0
    freq_actual = 0.0
    loop_count = 0
    t_freq_start = time.monotonic()
    t_start = time.monotonic()

    z = np.zeros(N_JOINTS)
    d_q = z.copy()
    d_tau_api = z.copy()
    d_tau_model = z.copy()
    d_tau_comp = z.copy()
    d_tau_jump = z.copy()
    d_tau_ext = z.copy()
    d_lam = 0.0

    running = True
    logged_rows = 0

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    try:
        with Live(build_table(z, z, z, z, z, z, 0.0, 0),
                  refresh_per_second=args.display_rate) as live:
            while running:
                t0 = time.monotonic()

                q, qd, tau_meas, ok = read_state(arm)
                if not ok:
                    time.sleep(dt)
                    continue

                tau_api = tau_meas

                # --- Smooth blend factor lambda ---
                max_speed = np.abs(qd).max()
                is_moving = float(max_speed > MOTION_SPEED_THR)
                lam = (1.0 - BLEND_ALPHA) * lam + BLEND_ALPHA * is_moving

                # --- Force estimation ---
                tau_model = dyn.gravity(q) + dyn.coriolis(q, qd)
                resid = tau_api - tau_model

                # --- Online bias detection (continuous) ---
                if lam < SETTLE_LAM_THR:
                    # Track residual with EMA while static
                    if not ema_initialized:
                        resid_ema[:] = resid
                        ema_initialized = True
                    else:
                        resid_ema += DETECT_EMA_ALPHA * (resid - resid_ema)

                    # Classify bimodal joints every frame
                    for j in range(N_JOINTS):
                        if BIAS_IS_BIMODAL[j]:
                            lo, hi = BIAS_LEVELS[j+1]
                            if resid_ema[j] > BIAS_MIDPOINTS[j]:
                                detected_bias[j] = hi
                            else:
                                detected_bias[j] = lo
                        else:
                            detected_bias[j] = BIAS_LEVELS[j+1][0]
                else:
                    ema_initialized = False

                # tau_jump: detected bias in static, blends to 0 in motion
                # (1 - lam) = 1 in static, 0 in motion
                tau_jump = detected_bias * (1.0 - lam)

                # tau_comp: DISABLED for now — isolating tau_jump behavior
                tau_comp = np.zeros(N_JOINTS)

                tau_ext = tau_api - tau_model - tau_comp - tau_jump

                d_q = q
                d_tau_api = tau_api
                d_tau_model = tau_model
                d_tau_comp = tau_comp
                d_tau_jump = tau_jump
                d_tau_ext = tau_ext
                d_lam = lam

                # Log
                if csv_writer is not None:
                    elapsed = t0 - t_start
                    row = (
                        [f"{t0:.6f}", f"{elapsed:.4f}", f"{lam:.4f}"]
                        + [f"{v:.4f}" for v in q]
                        + [f"{v:.4f}" for v in tau_api]
                        + [f"{v:.4f}" for v in tau_model]
                        + [f"{v:.4f}" for v in tau_comp]
                        + [f"{v:.4f}" for v in tau_jump]
                        + [f"{v:.4f}" for v in tau_ext]
                    )
                    csv_writer.writerow(row)
                    logged_rows += 1

                # Frequency
                loop_count += 1
                elapsed_freq = t0 - t_freq_start
                if elapsed_freq >= 1.0:
                    freq_actual = loop_count / elapsed_freq
                    loop_count = 0
                    t_freq_start = t0

                # Display
                if t0 - last_display >= display_interval:
                    live.update(build_table(
                        d_q, d_tau_api, d_tau_model,
                        d_tau_comp, d_tau_jump, d_tau_ext, d_lam, freq_actual,
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
