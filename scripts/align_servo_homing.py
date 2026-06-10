#!/usr/bin/env python3
"""Align a follower's servo Homing_Offset registers to the calibration JSON.

The rosetta/ROS path leaves whatever Homing_Offset is in the servo, while
lerobot effectively uses the calibration JSON's homing_offset. When they
differ, the same policy command lands at a different physical pose through
rosetta than through lerobot-record (the few-inch offset). This writes the
JSON homing_offset into each servo so both paths agree.

Run with ROS bringup / rosetta STOPPED (needs the serial port).
Usage: align_servo_homing.py [port] [id]
"""
import json, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM3"
ID = sys.argv[2] if len(sys.argv) > 2 else "gix-follower4"
CAL = f"/home/ubuntu/techin517_final/huggingface/lerobot/calibration/robots/so101_follower/{ID}.json"
sys.path.insert(0, "/home/ubuntu/techin517_final/lerobot/src")

cal = json.load(open(CAL))
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

r = SO101Follower(SO101FollowerConfig(port=PORT, id=ID))
r.connect(calibrate=False)
bus = r.bus

print(f"{'joint':14} {'before':>8} {'json_HO':>8} {'after':>8}")
for j, c in cal.items():
    target = int(c["homing_offset"])
    before = bus.sync_read("Homing_Offset", [j], normalize=False)[j]
    bus.write("Homing_Offset", j, target, normalize=False)
    after = bus.sync_read("Homing_Offset", [j], normalize=False)[j]
    flag = "" if after == target else "  <-- WRITE FAILED"
    print(f"{j:14} {before:>8} {target:>8} {after:>8}{flag}")

r.disconnect()
print("\nDone. Servo homing offsets now match the calibration JSON.")
print("Re-run diag_offset.py to confirm servo_HO == json_HO.")
