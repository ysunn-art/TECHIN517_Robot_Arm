"""Command a known pose through the ROS converter math, read where it lands
via lerobot. Difference = the true per-joint offset, whatever causes it.
Run with bringup/rosetta STOPPED. Arm will move to a mid pose. Keep clear."""
import sys, time, json, math
sys.path.insert(0, "/home/ubuntu/techin517_final/lerobot/src")
PORT, ID = "/dev/ttyACM3", "gix-follower4"
CAL = f"/home/ubuntu/techin517_final/huggingface/lerobot/calibration/robots/so101_follower/{ID}.json"
cal = json.load(open(CAL))
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
r = SO101Follower(SO101FollowerConfig(port=PORT, id=ID)); r.connect(calibrate=False)
# command each arm joint to normalized 0.0 (gripper 50) = mid of its range
tgt = {f"{j}.pos": (50.0 if j=="gripper" else 0.0) for j in
       ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]}
print("commanding all joints to mid (norm 0 / gripper 50)...")
r.send_action(tgt); time.sleep(2.0)
obs = r.get_observation()
print(f"{'joint':14} {'commanded':>10} {'landed':>10} {'err_norm':>10}")
for j in ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]:
    c = 50.0 if j=="gripper" else 0.0
    got = float(obs[f"{j}.pos"]); print(f"{j:14} {c:>10.1f} {got:>10.2f} {got-c:>10.2f}")
r.disconnect()
