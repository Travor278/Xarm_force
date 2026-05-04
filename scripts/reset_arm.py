#!/usr/bin/env python3
"""Reset xArm6 to the home position and open gripper."""
import argparse
import sys

sys.path.insert(0, "/home/mrblue/Projects/robot_control")
from robot_control.xarm_utils import connect, go_home


def main():
    p = argparse.ArgumentParser(description="Reset xArm6 to home position")
    p.add_argument("--ip", default="192.168.1.199", help="xArm IP")
    p.add_argument("--speed", type=float, default=20, help="Joint move speed (deg/s)")
    p.add_argument("--mvacc", type=float, default=200, help="Joint move acceleration")
    args = p.parse_args()

    arm = connect(args.ip)
    try:
        go_home(arm, speed=args.speed, mvacc=args.mvacc)
    finally:
        arm.disconnect()
        print("[DONE] Disconnected.")


if __name__ == "__main__":
    main()
