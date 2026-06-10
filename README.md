# TECHIN 517 Blackjack

## Project description

Two SO-101 robot arms play a full game of blackjack against a human player. One arm acts as the dealer and one handles chip payout and collection. The system deals cards, flips the dealer's hole card, detects card values with computer vision, and physically pays out or collects chips based on the game outcome. The chip-handling arm is driven by a learned visuomotor policy (ACT or SmolVLA) trained with [LeRobot](https://github.com/huggingface/lerobot). Card dealing uses pre-recorded waypoints under ROS 2 Humble joint trajectory control.

## Video demo

[Watch the demo on Google Drive](https://drive.google.com/drive/folders/1-JW1nn_67j9yA2Atp-RZBGfFr8Vs1K2I)

## Quantitative results

We evaluate the system as **three phases** (one per sub-system), **10 trials each (30 total)**. Full write-up, experiment design, and failure analysis: [`quantitative_results.md`](quantitative_results.md).

| Phase (state) | Sub-system | Trials | Success | Mean time (success) | Std |
|---|---|---|---|---|---|
| **I — Initial Deal** | FK waypoints + YOLO | 10 | **8 / 10 (80%)** | 105.6 s | 2.5 s |
| **II — Decision** | FK loop + YOLO | 10 | **9 / 10 (90%)** | 48.3 s | 20.9 s |
| **III — Settlement** | SmolVLA policy (VLA) | 10 | **10 / 10 (100%)** | 30.0 s | 1.7 s |
| **Overall** | — | 30 | **27 / 30 (90%)** | — | — |

Timing means/standard deviations are over successful trials only. All 3 failures share a single root cause — overlapping cards confusing YOLO rank classification; no failures came from the arms, grippers, decision logic, or the VLA settlement policy.

## Setup instructions

### Prerequisites

- Ubuntu 22.04 with an NVIDIA GPU (CUDA 12.8 recommended)
- [Docker](https://docs.docker.com/engine/install/ubuntu/) with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Dev container

1. Clone the repo and open it in VS Code.
2. When prompted, click **Reopen in Container** (or run `Dev Containers: Reopen in Container` from the command palette).
3. VS Code will build the Docker image and run `docker/setup.sh`, which installs ROS 2 Humble, LeRobot, PyTorch 2.7.1 + CUDA 12.8, and builds the ROS 2 workspace.
4. Source the workspace after the container starts:
   ```bash
   source ./ros2_ws/install/setup.bash
   ```

> The repo is mounted at `/home/ubuntu/techin517_final` inside the container.

### Pre-trained models

Models are hosted on Hugging Face: [https://huggingface.co/aarony630](https://huggingface.co/aarony630)

Download a policy:
```bash
hf download aarony630/<model_name>_policy --local-dir ./data/<model_name>/policy_smolvla
```

### Arm calibration (first time only)

```bash
# Left follower
PYTHONPATH=./third_party/lerobot/src python3.12 -m lerobot.scripts.lerobot_calibrate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=gix-follower3

# Right follower
PYTHONPATH=./third_party/lerobot/src python3.12 -m lerobot.scripts.lerobot_calibrate \
  --robot.type=so101_follower --robot.port=/dev/ttyACM3 --robot.id=gix-follower4
```

Press **Enter** to reuse existing calibration or **c + Enter** to redo from scratch.

## Usage instructions

### 1. System check

Run this after every boot or hardware replug to verify all cameras and arms are detected.

```bash
python3 ./scripts/system_check.py
```

### 2. Grant device permissions

```bash
sudo chmod a+rw /dev/video* /dev/ttyACM*
```

### 3. Start ROS 2 (Terminal 1)

Required only for the waypoint-based game loop (`blackjack_game_loop.py`). The
full game loop with the learned chip policy (`blackjack_game_loop_with_lerobot.py`)
manages its own bringup, so you can skip this step for it.

```bash
source ./ros2_ws/install/setup.bash
ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false
```

### 4. Start the dashboard (Terminal 2, optional)

Live web UI at **http://localhost:8000** showing cards, game state, and camera feeds.

```bash
python3 ./scripts/dashboard_server.py
```

### 5. Run the game

Card drawing only (waypoint-based):
```bash
python3 ./scripts/blackjack_game_loop.py
```

Full game loop with learned chip policy:
```bash
python3 ./scripts/blackjack_game_loop_with_lerobot.py
```

**Controls:** press `0` to STAND, any other number key to HIT.