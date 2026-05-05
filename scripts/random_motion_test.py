#!/usr/bin/env python3
"""
Random Motion Test with Torque Monitoring
==========================================
Moves xArm6 slowly within a 15cm sphere around home position while
recording full force estimation data for validation.

Usage:
  python scripts/random_motion_test.py --duration 120
"""

import argparse
import csv
import os
import signal
import threading
import time
from datetime import datetime

import numpy as np

from robot_control.constants import N_JOINTS
from robot_control.dynamics import DynamicsModel
from robot_control.xarm_utils import connect, go_home, read_state

# Import fixed parameters from monitor
from torque_monitor import (
    MOTION_SPEED_THR, BLEND_ALPHA, COMP_COEFS,
)


LOG_HEADER = (
    ["timestamp", "elapsed_s", "lambda"]
    + [f"q{i+1}_deg" for i in range(N_JOINTS)]
    + [f"qd{i+1}_dps" for i in range(N_JOINTS)]
    + [f"tau_api{i+1}" for i in range(N_JOINTS)]
    + [f"tau_model{i+1}" for i in range(N_JOINTS)]
    + [f"tau_comp{i+1}" for i in range(N_JOINTS)]
    + [f"tau_ext{i+1}" for i in range(N_JOINTS)]
)


def monitor_loop(arm, dyn, csv_writer, dt, stop_event, t_start):
    """Background thread: read sensors, compute model, log data."""
    lam = 0.0

    while not stop_event.is_set():
        t0 = time.monotonic()
        q, qd, tau_meas, ok = read_state(arm)
        if not ok:
            time.sleep(dt)
            continue

        tau_api = tau_meas

        # Smooth blend factor
        max_speed = np.abs(qd).max()
        is_moving = float(max_speed > MOTION_SPEED_THR)
        lam = (1.0 - BLEND_ALPHA) * lam + BLEND_ALPHA * is_moving

        tau_model = dyn.gravity(q) + dyn.coriolis(q, qd)

        # Unified model
        feat = np.concatenate([q, qd, [lam], q * lam, qd * lam, [1.0]])
        tau_comp = COMP_COEFS @ feat
        tau_ext = tau_api - tau_model - tau_comp

        elapsed = t0 - t_start
        row = (
            [f"{t0:.6f}", f"{elapsed:.4f}", f"{lam:.4f}"]
            + [f"{v:.4f}" for v in q]
            + [f"{v:.4f}" for v in qd]
            + [f"{v:.4f}" for v in tau_api]
            + [f"{v:.4f}" for v in tau_model]
            + [f"{v:.4f}" for v in tau_comp]
            + [f"{v:.4f}" for v in tau_ext]
        )
        csv_writer.writerow(row)

        sleep_time = dt - (time.monotonic() - t0)
        if sleep_time > 0:
            time.sleep(sleep_time)


def random_cartesian_motion(arm, duration, radius_mm=150, speed=30):
    """Move end-effector randomly within a sphere around home position."""
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.3)
    code, home_pose = arm.get_position()
    if code != 0:
        print(f"[ERROR] get_position failed: {code}")
        return
    home_xyz = np.array(home_pose[:3])
    home_rpy = home_pose[3:6]
    print(f"[MOTION] Home TCP: x={home_xyz[0]:.1f} y={home_xyz[1]:.1f} z={home_xyz[2]:.1f} mm")
    print(f"[MOTION] Moving within {radius_mm}mm sphere for {duration}s at {speed}mm/s")

    t_start = time.monotonic()
    waypoint_count = 0

    while time.monotonic() - t_start < duration:
        direction = np.random.randn(3)
        direction /= np.linalg.norm(direction)
        r = radius_mm * np.cbrt(np.random.uniform())
        target_xyz = home_xyz + direction * r

        code = arm.set_position(
            x=target_xyz[0], y=target_xyz[1], z=target_xyz[2],
            roll=home_rpy[0], pitch=home_rpy[1], yaw=home_rpy[2],
            speed=speed, mvacc=100, wait=True, timeout=10,
        )
        waypoint_count += 1
        elapsed = time.monotonic() - t_start
        if waypoint_count % 5 == 0:
            print(f"[MOTION] {elapsed:.0f}s/{duration}s  waypoints={waypoint_count}")

        if code != 0:
            print(f"[MOTION] set_position returned {code}, retrying...")
            arm.clean_error()
            arm.clean_warn()
            arm.set_mode(0)
            arm.set_state(0)
            time.sleep(1)

    print("[MOTION] Returning to home...")
    go_home(arm)


def main():
    p = argparse.ArgumentParser(description="Random motion test with torque monitoring")
    p.add_argument("--ip", default="192.168.1.199")
    p.add_argument("--duration", type=int, default=120, help="Test duration (s)")
    p.add_argument("--radius", type=float, default=150, help="Sphere radius (mm)")
    p.add_argument("--speed", type=float, default=30, help="Motion speed (mm/s)")
    p.add_argument("--freq", type=int, default=100, help="Monitor frequency (Hz)")
    args = p.parse_args()

    arm = connect(args.ip)
    dyn = DynamicsModel()

    go_home(arm)

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(log_dir, f"random_motion_{stamp}.csv")
    log_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(log_file)
    csv_writer.writerow(LOG_HEADER)
    print(f"[LOG] Recording to {os.path.abspath(log_path)}")

    stop_event = threading.Event()
    t_start = time.monotonic()

    monitor_thread = threading.Thread(
        target=monitor_loop,
        args=(arm, dyn, csv_writer, 1.0 / args.freq, stop_event, t_start),
        daemon=True,
    )
    monitor_thread.start()

    def _stop(sig, frame):
        stop_event.set()
    signal.signal(signal.SIGINT, _stop)

    try:
        random_cartesian_motion(arm, args.duration, args.radius, args.speed)
    finally:
        stop_event.set()
        monitor_thread.join(timeout=2)
        log_file.close()
        dyn.close()
        arm.disconnect()
        print(f"[DONE] Log saved to {log_path}")


if __name__ == "__main__":
    main()
