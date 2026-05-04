#!/usr/bin/env python3
"""Diagnose J3/J5 torque sensor jumping issue."""
import sys
import time

import numpy as np

sys.path.insert(0, "/home/mrblue/Projects/robot_control")
from robot_control.xarm_utils import connect


def fmt(vals):
    return ", ".join("%+8.4f" % v for v in vals[:6])


def main():
    arm = connect("192.168.1.199")

    # 1. Firmware versions
    print("=== Firmware Versions ===")
    code, ver = arm.get_version()
    print("  Arm:", ver)
    for i in range(1, 7):
        code, sv = arm.get_servo_version(servo_id=i)
        print("  Servo J%d: %s" % (i, sv))
    code, gv = arm.get_gripper_version()
    print("  Gripper:", gv)
    print()

    # 2. Current reporting mode
    print("=== Torque/Current Report Mode ===")
    code, mode = arm.get_report_tau_or_i()
    print("  Mode: %d (0=torque, 1=current)" % mode)
    print()

    # 3. Read torque (mode=0)
    print("=== Torque mode (tau_or_i=0) ===")
    arm.set_report_tau_or_i(0)
    time.sleep(0.5)
    for _ in range(3):
        code, tau = arm.get_joints_torque()
        print("  tau: [%s]" % fmt(tau))
        time.sleep(0.1)
    print()

    # 4. Switch to current mode
    print("=== Current mode (tau_or_i=1) ===")
    arm.set_report_tau_or_i(1)
    time.sleep(0.5)
    for _ in range(3):
        code, cur = arm.get_joints_torque()
        print("  cur: [%s]" % fmt(cur))
        time.sleep(0.1)
    print()

    # 5. Switch back to torque
    print("=== Back to torque mode ===")
    arm.set_report_tau_or_i(0)
    time.sleep(0.5)
    for _ in range(3):
        code, tau = arm.get_joints_torque()
        print("  tau: [%s]" % fmt(tau))
        time.sleep(0.1)
    print()

    # 6. Current mode stability (10s)
    print("=== Current mode stability (10s) ===")
    arm.set_report_tau_or_i(1)
    time.sleep(0.3)
    readings = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        code, cur = arm.get_joints_torque()
        if code == 0:
            readings.append(list(cur[:6]))
        time.sleep(0.01)
    readings = np.array(readings)
    print("  Samples: %d" % len(readings))
    for i in range(6):
        vals = readings[:, i]
        n_unique = len(np.unique(vals.round(4)))
        diffs = np.abs(np.diff(vals))
        jumps = (diffs > 0.5).sum()
        print("  J%d: mean=%+8.4f  std=%.4f  unique=%d  jumps(>0.5)=%d" % (
            i + 1, vals.mean(), vals.std(), n_unique, jumps))
    print()

    # 7. Torque mode stability (10s)
    print("=== Torque mode stability (10s) ===")
    arm.set_report_tau_or_i(0)
    time.sleep(0.3)
    readings = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        code, tau = arm.get_joints_torque()
        if code == 0:
            readings.append(list(tau[:6]))
        time.sleep(0.01)
    readings = np.array(readings)
    print("  Samples: %d" % len(readings))
    for i in range(6):
        vals = readings[:, i]
        n_unique = len(np.unique(vals.round(4)))
        diffs = np.abs(np.diff(vals))
        jumps = (diffs > 0.5).sum()
        print("  J%d: mean=%+8.4f  std=%.4f  unique=%d  jumps(>0.5)=%d" % (
            i + 1, vals.mean(), vals.std(), n_unique, jumps))

    # Restore and disconnect
    arm.set_report_tau_or_i(0)
    arm.disconnect()
    print("\n[DONE]")


if __name__ == "__main__":
    main()
