"""xArm6 SDK helpers — connection, homing, and state reading."""

import sys
import time

import numpy as np

sys.path.insert(0, "/home/mrblue/Projects/TeleOp-Mouse/scripts/xArm-Python-SDK")
from xarm.wrapper import XArmAPI

from .constants import GRIPPER_OPEN, HOME_JOINTS_DEG, N_JOINTS


def connect(ip: str) -> XArmAPI:
    """Connect to xArm, enable motion, and turn on torque reporting."""
    arm = XArmAPI(ip, do_not_open=True)
    arm.connect()
    if not arm.connected:
        raise RuntimeError(f"Cannot reach xArm at {ip}")

    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_report_tau_or_i(tau_or_i=0)
    return arm


def go_home(arm: XArmAPI, speed: float = 20, mvacc: float = 200):
    """Move to the TeleOp-Mouse home pose and open the gripper."""
    arm.set_mode(0)
    arm.set_state(0)
    print(f"[HOME] Moving to {HOME_JOINTS_DEG} ...")
    arm.set_servo_angle(
        angle=HOME_JOINTS_DEG,
        speed=speed, mvacc=mvacc,
        is_radian=False, wait=True,
    )
    arm.set_gripper_enable(True)
    arm.set_gripper_position(
        pos=GRIPPER_OPEN, wait=True, speed=2400.0, auto_enable=True,
    )
    time.sleep(0.5)
    print("[HOME] Ready.\n")


def read_state(arm: XArmAPI):
    """Read (q_deg, qd_dps, tau_Nm) from the robot in one shot.

    Returns
    -------
    q   : np.ndarray  — joint angles (deg)
    qd  : np.ndarray  — joint velocities (deg/s)
    tau : np.ndarray  — joint torques (Nm)
    ok  : bool        — True if all reads succeeded
    """
    ret_q = arm.get_servo_angle(is_radian=False)
    code_tau, tau_raw = arm.get_joints_torque()

    if ret_q[0] != 0 or code_tau != 0:
        z = np.zeros(N_JOINTS)
        return z, z, z, False

    q = np.array(ret_q[1][:N_JOINTS])
    qd = np.array(arm.realtime_joint_speeds[:N_JOINTS])
    tau = np.array(tau_raw[:N_JOINTS])
    return q, qd, tau, True
