# Terminal Commands Reference

---

## Setup

```bash
sudo chmod a+rw /dev/video* /dev/ttyACM*
source ~/techin517_final/ros2_ws/install/setup.bash
```

---

## System Check

Verifies all cameras, arm serial ports, and game keyboards are connected and
resolves the current `/dev/videoX` / `/dev/ttyACMx` numbers (which shuffle on
every reboot/replug). Run this first after any boot or replug.

```bash
python3 ~/techin517_final/scripts/system_check.py
```

Identifies hardware by **stable identity** (USB bus/port + device name), not by
device number, and prints the resolved mapping. Exits non-zero if anything is
missing or moved, so you can gate a launch on it:

```bash
python3 ~/techin517_final/scripts/system_check.py && \
ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false
```

Expected mapping (cables in their usual USB ports):

| Role | Identified by | Typical node |
|---|---|---|
| right wrist cam | XWF-1080P @ bus `0c:00.3-2`, MJPG | `/dev/video2` |
| left wrist cam | XWF-1080P @ bus `0d:00.0-1`, MJPG | `/dev/video0` |
| overhead cam | RealSense color (YUYV-only node) | `/dev/video8` |
| left leader | USB port `1-1.1.1` | `/dev/ttyACM0` |
| left follower | USB port `1-1.1.2` | `/dev/ttyACM1` |
| right leader | USB port `1-1.1.3` | `/dev/ttyACM2` |
| right follower | USB port `1-1.1.4` | `/dev/ttyACM3` |
| main keyboard (`` ` `` = HIT/DEAL) | `Chicony_HP_Elite_USB_Keyboard` | — |
| numpad (`1` = STAND/END) | `magic_force` numpad | — |

If you re-cable to different physical USB ports, update the `WRIST_BUS` /
`ARM_USB_PORT` / `KEYBOARD_ID` maps at the top of `scripts/system_check.py`.
The `/dev/...` numbers shift run-to-run — always trust the script's output
over the table above.

---

## Calibration

```bash
# Left follower
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src python3.12 -m lerobot.scripts.lerobot_calibrate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=gix-follower3

# Left leader
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src python3.12 -m lerobot.scripts.lerobot_calibrate \
  --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=gix-leader3

# Right follower
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src python3.12 -m lerobot.scripts.lerobot_calibrate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM3 --robot.id=gix-follower4

# Right leader
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src python3.12 -m lerobot.scripts.lerobot_calibrate \
  --teleop.type=so101_leader --teleop.port=/dev/ttyACM2 --teleop.id=gix-leader4
```

Press **Enter** to reuse existing calibration, **c + Enter** to redo from scratch.

---

## Teleop Both Arms

**Terminal 1 — left arm**
```bash
sudo chmod a+rw /dev/video* /dev/ttyACM* && \
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-teleop \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=gix-follower3 \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=gix-leader3
```

**Terminal 2 — right arm**
```bash
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-teleop \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM3 \
  --robot.id=gix-follower4 \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM2 \
  --teleop.id=gix-leader4
```

---

## Record Waypoints

### Terminal 1 — bimanual bringup + teleop
```bash
source ~/techin517_final/ros2_ws/install/setup.bash
ros2 launch soa_bringup bi_soa_bringup.launch.py leader:=true cameras:=false
```

### Terminal 2 — record both arms simultaneously (auto-saves every 1s, Enter to stop)
```bash
python3 ~/techin517_final/scripts/record_waypoints.py waypoints/<name>.csv
```

Record single arm only:
```bash
python3 ~/techin517_final/scripts/record_waypoints.py waypoints/<name>.csv --arm left
python3 ~/techin517_final/scripts/record_waypoints.py waypoints/<name>.csv --arm right
```

Change capture interval (default 0.3s — keep replay `--duration` equal to this):
```bash
python3 ~/techin517_final/scripts/record_waypoints.py waypoints/<name>.csv --interval 1.0
```

### Waypoint files
| File | Arms | Used in |
|---|---|---|
| `waypoints/phase1_initialize.csv` | both | game loop phase 1 |
| `waypoints/phase2_deal_player.csv` | right only | player hit |
| `waypoints/phase2_flip_dealer.csv` | left only | flip dealer hole card |
| `waypoints/phase2_deal_dealer.csv` | left only | dealer hit |

Record flip dealer hole card (left arm):
```bash
python3 ~/techin517_final/scripts/record_waypoints.py waypoints/phase2_flip_dealer.csv --arm left
```

---

## Record Dataset — right_pay_out_chip (right arm, teleop)

```bash
rm -rf /home/ubuntu/techin517_final/data/right_pay_out_chip
```
### Terminal 1 — record
```bash
sudo chmod a+rw /dev/video* /dev/ttyACM* && \
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM3 \
  --robot.id=gix-follower4 \
  --robot.cameras='{"wrist_right": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 10}, "overhead": {"type": "opencv", "index_or_path": "/dev/video8", "width": 640, "height": 480, "fps": 30, "warmup_s": 15}}' \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM2 \
  --teleop.id=gix-leader4 \
  --dataset.repo_id=aarony8881/right_pay_out_chip \
  --dataset.root=/home/ubuntu/techin517_final/data/right_pay_out_chip \
  --dataset.single_task="pay out chip to player" \
  --dataset.num_episodes=30 \
  --dataset.episode_time_s=60 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --play_sounds=false
```

Append more episodes to an existing dataset: add `--resume true` and bump
`--dataset.num_episodes` to the new total.

---

## Train Policy — right_pay_out_chip

ACT:
```bash
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-train \
  --dataset.repo_id=aarony8881/right_pay_out_chip \
  --dataset.root=/home/ubuntu/techin517_final/data/right_pay_out_chip \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=/home/ubuntu/techin517_final/data/right_pay_out_chip/policy \
  --job_name=act \
  --batch_size=32 \
  --steps=100000 \
  --save_freq=20000 \
  --log_freq=200 \
  --save_checkpoint=true \
  --num_workers=4 \
  --seed=42 \
  --wandb.enable=false
```

SmolVLA (separate output dir so it doesn't clobber the ACT checkpoints):
```bash
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-train \
  --dataset.repo_id=aarony8881/right_pay_out_chip \
  --dataset.root=/home/ubuntu/techin517_final/data/right_pay_out_chip \
  --policy.type=smolvla \
  --policy.device=cuda \
  --policy.load_vlm_weights=true \
  --policy.push_to_hub=false \
  --output_dir=/home/ubuntu/techin517_final/data/right_pay_out_chip/policy_smolvla \
  --job_name=smolvla \
  --batch_size=16 \
  --steps=100000 \
  --save_freq=20000 \
  --log_freq=200 \
  --save_checkpoint=true \
  --num_workers=4 \
  --seed=42 \
  --wandb.enable=false
```

Run the trained policy:
```bash
rm -rf ~/.cache/huggingface/lerobot/aarony8881/eval_right_pay_out_chip && \
sudo chmod a+rw /dev/video* /dev/ttyACM* && \
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM3 \
  --robot.id=gix-follower4 \
  --robot.cameras='{"wrist_right": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 10}, "overhead": {"type": "opencv", "index_or_path": "/dev/video8", "width": 640, "height": 480, "fps": 30, "warmup_s": 15}}' \
  --policy.path=/home/ubuntu/techin517_final/data/right_pay_out_chip/policy_smolvla/checkpoints/100000/pretrained_model \
  --dataset.repo_id=aarony8881/eval_right_pay_out_chip \
  --dataset.single_task="pay out chip to player" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=13 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --play_sounds=false
```

Always pass `--play_sounds=false` (the `spd-say` hook crashes at the end otherwise).

---

## Record Dataset — right_collect_chip (right arm, teleop)

```bash
rm -rf /home/ubuntu/techin517_final/data/right_collect_chip
```
### Terminal 1 — record
```bash
sudo chmod a+rw /dev/video* /dev/ttyACM* && \
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM3 \
  --robot.id=gix-follower4 \
  --robot.cameras='{"wrist_right": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 10}, "overhead": {"type": "opencv", "index_or_path": "/dev/video8", "width": 640, "height": 480, "fps": 30, "warmup_s": 15}}' \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM2 \
  --teleop.id=gix-leader4 \
  --dataset.repo_id=aarony8881/right_collect_chip \
  --dataset.root=/home/ubuntu/techin517_final/data/right_collect_chip \
  --dataset.single_task="collect chip from player" \
  --dataset.num_episodes=30 \
  --dataset.episode_time_s=60 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --play_sounds=false
```

---

## Train Policy — right_collect_chip

ACT:
```bash
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-train \
  --dataset.repo_id=aarony8881/right_collect_chip \
  --dataset.root=/home/ubuntu/techin517_final/data/right_collect_chip \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir=/home/ubuntu/techin517_final/data/right_collect_chip/policy_act \
  --job_name=act \
  --batch_size=32 \
  --steps=100000 \
  --save_freq=20000 \
  --log_freq=200 \
  --save_checkpoint=true \
  --num_workers=4 \
  --seed=42 \
  --wandb.enable=false
```

SmolVLA (separate output dir so it doesn't clobber the ACT checkpoints):
```bash
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-train \
  --dataset.repo_id=aarony8881/right_collect_chip \
  --dataset.root=/home/ubuntu/techin517_final/data/right_collect_chip \
  --policy.type=smolvla \
  --policy.device=cuda \
  --policy.load_vlm_weights=true \
  --policy.push_to_hub=false \
  --output_dir=/home/ubuntu/techin517_final/data/right_collect_chip/policy_smolvla \
  --job_name=smolvla \
  --batch_size=16 \
  --steps=100000 \
  --save_freq=20000 \
  --log_freq=200 \
  --save_checkpoint=true \
  --num_workers=4 \
  --seed=42 \
  --wandb.enable=false
```

run policy
```bash
rm -rf ~/.cache/huggingface/lerobot/aarony8881/eval_right_collect_chip && \
sudo chmod a+rw /dev/video* /dev/ttyACM* && \
PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
/home/ubuntu/techin517_final/lerobot/.venv/bin/lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM3 \
  --robot.id=gix-follower4 \
  --robot.cameras='{"wrist_right": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 10}, "overhead": {"type": "opencv", "index_or_path": "/dev/video8", "width": 640, "height": 480, "fps": 30, "warmup_s": 15}}' \
  --policy.path=/home/ubuntu/techin517_final/data/right_collect_chip/policy_smolvla/checkpoints/100000/pretrained_model \
  --dataset.repo_id=aarony8881/eval_right_collect_chip \
  --dataset.single_task="collect chip from player" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=13 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --play_sounds=false
```

---

## Save / Load Policy + Dataset (Hugging Face Hub)

GitHub can't hold the large checkpoints — use HF Hub. Login once (`hf auth login`),
then upload. HF namespace is `aarony630`.

```bash
# Upload policy
hf upload aarony630/<name>_policy /home/ubuntu/techin517_final/data/<name>/policy_smolvla .

# Upload dataset
hf upload aarony630/<name> /home/ubuntu/techin517_final/data/<name> .

# Download a policy onto another machine
hf download aarony630/<name>_policy --local-dir /home/ubuntu/techin517_final/data/<name>/policy_smolvla
```

---

## Replay Waypoints

### Terminal 1 — bimanual bringup (JTC mode, no leader needed)
```bash
source ~/techin517_final/ros2_ws/install/setup.bash
ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false
```

### Terminal 2 — replay (auto-detects bimanual vs single-arm from CSV)
```bash
python3 ~/techin517_final/scripts/replay_waypoints.py waypoints/<name>.csv
```

Specific waypoints:
```bash
python3 ~/techin517_final/scripts/replay_waypoints.py waypoints/right_draw_cards.csv
```

Specify arm for single-arm CSVs (when using bimanual bringup):
```bash
python3 ~/techin517_final/scripts/replay_waypoints.py waypoints/<name>.csv --arm left
python3 ~/techin517_final/scripts/replay_waypoints.py waypoints/<name>.csv --arm right
```

Single-arm bringup (required for no-prefix CSVs without --arm):
```bash
source ~/techin517_final/ros2_ws/install/setup.bash
ros2 launch soa_bringup soa_bringup.launch.py controller:=jtc cameras:=false arm:=right
```

Change speed (default 2.0s per waypoint):
```bash
python3 ~/techin517_final/scripts/replay_waypoints.py waypoints/<name>.csv --duration 1.5
```

---

## Run Policy (left_deal_face_up)

Drives the left arm with a trained ACT checkpoint. Uses `lerobot-record` with `--policy.path` (the eval entry point in this lerobot version is for sim only).

Clean up previous eval cache first (mkdir fails with `FileExistsError` otherwise):
```bash
rm -rf ~/.cache/huggingface/lerobot/aarony8881/eval_left_deal_face_up
```

Run:
```bash
sudo chmod a+rw /dev/video* /dev/ttyACM* && \
source ~/lerobot-venv/bin/activate && \
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=gix-follower3 \
  --robot.cameras='{"wrist_left": {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}, "overhead": {"type": "opencv", "index_or_path": "/dev/video2", "width": 640, "height": 480, "fps": 30}}' \
  --policy.path="$HOME/Desktop/Final Project/blackjack_ws/data/left_deal_face_up/policy/checkpoints/060000/pretrained_model" \
  --dataset.repo_id=aarony8881/eval_left_deal_face_up \
  --dataset.single_task="deal card face up to dealer" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=13 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

Bump checkpoint number (`060000`) to the highest available in `data/left_deal_face_up/policy/checkpoints/`.

---

## Run Policy (Rosetta / ROS2)

Use the **3.10** lerobot at `third_party/lerobot` and NO active `(lerobot)` venv
(the 3.12 venv breaks rclpy). All terminals: `source` the ws setup first.

### Terminal 1 — bringup (forward controllers + cameras)
```bash
source ~/techin517_final/ros2_ws/install/setup.bash
ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=forward cameras:=true
```

### Terminal 2 — rosetta client (auto-launches its own policy server)
```bash
source ~/techin517_final/ros2_ws/install/setup.bash && \
PYTHONPATH=/home/ubuntu/techin517_final/third_party/lerobot/src:$PYTHONPATH \
ros2 launch rosetta rosetta_client_launch.py \
  contract_path:=/home/ubuntu/techin517_final/ros2_ws/src/soa_ros2/soa_bringup/rosetta_contracts/right_arm/deal_face_up.yaml \
  pretrained_name_or_path:=/home/ubuntu/techin517_final/data/right_deal_face_up/policy/checkpoints/100000/pretrained_model \
  chunk_size_threshold:=0.0 \
  actions_per_chunk:=100
```

### Terminal 3 — trigger
```bash
source ~/techin517_final/ros2_ws/install/setup.bash
ros2 action send_goal /rosetta_client/run_policy \
  rosetta_interfaces/action/RunPolicy "{prompt: 'deal card face up'}"
```

Contracts: `ros2_ws/src/soa_ros2/soa_bringup/rosetta_contracts/{left_arm,right_arm}/*.yaml`
(bimanual: prefixed joints + per-arm `*_arm_fwd_controller` topics).

---

## Game Loop

### Terminal 1 — bimanual bringup (if not already running)
```bash
source ~/techin517_final/ros2_ws/install/setup.bash
ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false
```

### Terminal 2 — casino dashboard (optional, recommended)
Live web UI: player/dealer cards (suit + value), totals, status, result, plus
the 3 table cameras. Start it any time — the game pushes to it best-effort, so
it's safe whether or not the dashboard is up.
```bash
python3 ~/techin517_final/scripts/dashboard_server.py
```
Then open **http://localhost:8000** (or `http://<host-ip>:8000` from another
device on the network).

- All three cameras stream live — the server owns them and the card detector
  reads its wrist frames *from the server* (no V4L2 contention). Defaults:
  `OVERHEAD_CAM=/dev/video8`, `PLAYER_CAM=/dev/video0`, `DEALER_CAM=/dev/video2`
  — override the env vars if `system_check.py` resolved different numbers.
- If the dashboard isn't running, the detector falls back to opening the wrist
  devices directly (game behaves exactly as before, just no live wrist tiles).
- Disable state pushes for a run with `DASHBOARD_OFF=1`, or force the detector to
  always open devices directly with `CARD_CAM_HUB=` (empty), in Terminal 3.

### Terminal 3 — game loop
```bash
python3 ~/techin517_final/scripts/blackjack_game_loop.py
```

Flow:
1. **Phase 1** — runs `phase1_initialize.csv` (both arms, once)
2. **Player turn** — press `` ` `` (backtick, below Esc) to HIT, `1` to STAND
3. **Dealer turn** — press `` ` `` to DEAL, `1` to END

---

## Overhead Camera

```bash
# Live view (use the device system_check reports for overhead)
ffplay -video_size 640x480 -framerate 30 /dev/video8

# Single snapshot (opens in VS Code file explorer)
python3 -c "
import cv2
cap = cv2.VideoCapture(8)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
if ret: cv2.imwrite('/tmp/overhead_check.jpg', frame)
cap.release()
"

# List video devices
v4l2-ctl --list-devices

# Check supported resolutions
v4l2-ctl -d /dev/video8 --list-formats-ext
```

Device numbers shuffle — run `scripts/system_check.py` for the current mapping.

---

## ROS2 Utilities

```bash
# Check which topics are publishing
ros2 topic list

# View camera image topics
ros2 run rqt_image_view rqt_image_view

# Check joint states
ros2 topic echo /follower/joint_states

# List active controllers
ros2 control list_controllers
```

---

## Arm Config Reference
python3 ~/techin517_final/scripts/system_check.py

Edit `ros2_ws/src/soa_ros2/soa_bringup/config/soa_params.yaml` to switch single-arm bringup:

| Arm | follower id | follower port | leader id | leader port |
|---|---|---|---|---|
| Left | `gix-follower3` | `/dev/ttyACM1` | `gix-leader3` | `/dev/ttyACM0` |
| Right | `gix-follower4` | `/dev/ttyACM3` | `gix-leader4` | `/dev/ttyACM2` |


