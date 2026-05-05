#!/usr/bin/env python3
"""Query xArm hardware info: tcp_load, gravity direction, version, etc."""

import sys
sys.path.insert(0, "/home/mrblue/Projects/TeleOp-Mouse/scripts/xArm-Python-SDK")
from xarm.wrapper import XArmAPI

arm = XArmAPI("192.168.1.199", do_not_open=True)
arm.connect()

print("=" * 50)
print("xArm Hardware Info")
print("=" * 50)

print(f"\nSerial Number:    {arm.sn}")
print(f"Control Box SN:   {arm.control_box_sn}")
print(f"Version:          {arm.version}")
print(f"Device Type:      {arm.device_type}")

print(f"\n--- TCP Load (as set in controller) ---")
print(f"  tcp_load:       {arm.tcp_load}")
print(f"  (weight kg, center_of_gravity [x,y,z] mm)")

print(f"\n--- Gravity Direction ---")
print(f"  gravity_direction: {arm.gravity_direction}")

print(f"\n--- Collision Sensitivity ---")
print(f"  collision_sensitivity: {arm.collision_sensitivity}")

print(f"\n--- Servo Info ---")
for i in range(1, 7):
    ver = arm.get_servo_version(servo_id=i)
    harmonic = arm.get_harmonic_type(servo_id=i)
    print(f"  J{i}: version={ver}, harmonic_type={harmonic}")

arm.disconnect()
print("\nDone.")
