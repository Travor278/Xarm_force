#!/usr/bin/env python3
"""
Random Motion Test with Torque Monitoring
==========================================
Moves xArm6 slowly within a 15cm sphere around home position while
recording torque monitor data. Used to evaluate force estimation quality
across varied poses.

Usage:
  python scripts/random_motion_test.py --duration 180
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time
from datetime import datetime

import numpy as np

from robot_control.constants import HOME_JOINTS_DEG, N_JOINTS
from robot_control.dynamics import DynamicsModel, MomentumObserver
from robot_control.xarm_utils import connect, go_home, read_state


LOG_HEADER = (
    ["timestamp", "elapsed_s"]
    + [f"q{i+1}_deg" for i in range(N_JOINTS)]
    + [f"qd{i+1}_dps" for i in range(N_JOINTS)]
    + [f"tau_meas{i+1}" for i in range(N_JOINTS)]
    + [f"tau_grav{i+1}" for i in range(N_JOINTS)]
    + [f"tau_cori{i+1}" for i in range(N_JOINTS)]
    + [f"tau_ext{i+1}" for i in range(N_JOINTS)]
)


def monitor_loop(arm, dyn, obs, csv_writer, dt, stop_event, t_start):
    """Background thread: read sensors, run observer, log data."""
    cal_samples = []
    cal_done = False
    cal_duration = 2.0

    while not stop_event.is_set():
        t0 = time.monotonic()
        q, qd, tau_meas, ok = read_state(arm)
        if not ok:
            time.sleep(dt)
            continue

        tau_ext = obs.update(q, qd, tau_meas, dt)

        # Calibration during first 2s
        elapsed = t0 - t_start
        if not cal_done:
            cal_samples.append(tau_ext.copy())
            if elapsed >= cal_duration:
                n_skip = len(cal_samples) // 2
                if n_skip > 0:
                    obs.calibrate(cal_samples[n_skip:])
                cal_done = True
                print("[CAL] Calibration done.")

        tau_grav = dyn.gravity(q)
        tau_cori = dyn.coriolis(q, qd)

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

        sleep_time = dt - (time.monotonic() - t0)
        if sleep_time > 0:
            time.sleep(sleep_time)


def random_cartesian_motion(arm, duration, radius_mm=150, speed=30):
    """Move end-effector randomly within a sphere around home position."""
    # Get home TCP position
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
        # Random point within sphere
        direction = np.random.randn(3)
        direction /= np.linalg.norm(direction)
        r = radius_mm * np.cbrt(np.random.uniform())  # uniform in volume
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

    # Return home
    print("[MOTION] Returning to home...")
    go_home(arm)


def main():
    p = argparse.ArgumentParser(description="Random motion test with torque monitoring")
    p.add_argument("--ip", default="192.168.1.199")
    p.add_argument("--duration", type=int, default=180, help="Test duration (s)")
    p.add_argument("--radius", type=float, default=150, help="Sphere radius (mm)")
    p.add_argument("--speed", type=float, default=30, help="Motion speed (mm/s)")
    p.add_argument("--freq", type=int, default=100, help="Monitor frequency (Hz)")
    p.add_argument("--gain", type=float, default=50.0, help="Observer gain")
    args = p.parse_args()

    arm = connect(args.ip)
    dyn = DynamicsModel()
    obs = MomentumObserver(dyn, gain=args.gain)

    # Go home first
    go_home(arm)

    # Set up logging
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

    # Start monitor thread
    monitor_thread = threading.Thread(
        target=monitor_loop,
        args=(arm, dyn, obs, csv_writer, 1.0 / args.freq, stop_event, t_start),
        daemon=True,
    )
    monitor_thread.start()

    def _stop(sig, frame):
        stop_event.set()
    signal.signal(signal.SIGINT, _stop)

    try:
        # Wait for calibration
        time.sleep(3)
        # Run random motion
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
