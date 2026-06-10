#!/usr/bin/env python3
"""Diagnose the rosetta vs lerobot joint offset.

Reads, for the right follower, at the CURRENT physical pose:
  - the servo's raw Present_Position (ticks) and written Homing_Offset register
  - lerobot's normalized observation
  - what the ROS converter would reconstruct (tick = rad/RAD_PER_TICK + 2048)
and compares against the calibration JSON.

Run with ROS bringup / rosetta STOPPED (needs the serial port). Arm resting
stable. Right follower port from system_check (currently /dev/ttyACM3).
"""
import json, math, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM3"
ID = "gix-follower4"
CAL = f"/home/ubuntu/techin517_final/huggingface/lerobot/calibration/robots/so101_follower/{ID}.json"
SRC = "/home/ubuntu/techin517_final/lerobot/src"
sys.path.insert(0, SRC)

cal = json.load(open(CAL))
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

r = SO101Follower(SO101FollowerConfig(port=PORT, id=ID))
r.connect(calibrate=False)
bus = r.bus

print(f"{'joint':14} {'raw_tick':>9} {'servo_HO':>9} {'json_HO':>9} {'json_min':>9} {'json_max':>9} {'lerobot_norm':>13}")
raw = bus.sync_read("Present_Position", normalize=False)
try:
    ho = bus.sync_read("Homing_Offset", normalize=False)
except Exception as e:
    ho = {m: "n/a" for m in raw}
norm = bus.sync_read("Present_Position", normalize=True)

for j in ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]:
    c = cal[j]
    print(f"{j:14} {raw.get(j,'?'):>9} {str(ho.get(j,'?')):>9} {c['homing_offset']:>9} "
          f"{c['range_min']:>9} {c['range_max']:>9} {round(float(norm.get(j,0)),2):>13}")

r.disconnect()
print("\nIf servo_HO != json_HO, the servo's written homing offset drifted from "
      "the calibration the converter trusts -> that's the offset source.")
