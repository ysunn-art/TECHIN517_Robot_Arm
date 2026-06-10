#!/usr/bin/env python3
"""
Auto-record joint waypoints during teleoperation — captures a pose at a fixed interval.

Bimanual mode (default) — records both arms simultaneously from bi_soa_bringup:
    python3 scripts/record_waypoints.py waypoints/phase1.csv

Single-arm mode — records one arm from soa_bringup:
    python3 scripts/record_waypoints.py waypoints/<primitive>.csv --arm left
    python3 scripts/record_waypoints.py waypoints/<primitive>.csv --arm right

Optional: change capture interval (default 0.3 s):
    python3 scripts/record_waypoints.py waypoints/phase1.csv --interval 1.0

The replay duration per waypoint (replay_waypoints.py / blackjack_game_loop.py)
should match this interval to reproduce the demonstrated motion at recorded speed.

Press Enter to stop recording and save the CSV.
Ctrl+C also stops and saves.

Bimanual CSV columns:
    left_shoulder_pan, left_shoulder_lift, left_elbow_flex, left_wrist_flex,
    left_wrist_roll, left_gripper, right_shoulder_pan, ..., right_gripper

Single-arm CSV columns (no prefix):
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
"""

import argparse
import csv
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
BIMANUAL_JOINTS = (
    [f'left_{j}' for j in ARM_JOINTS] + [f'right_{j}' for j in ARM_JOINTS]
)
TOPIC = '/follower/joint_states'


class WaypointRecorder(Node):

    def __init__(self, csv_path: str, arm: str | None, interval: float):
        super().__init__('waypoint_recorder')
        self.csv_path = csv_path
        self.arm = arm  # None = bimanual, 'left'/'right' = single
        self._latest: dict[str, float] = {}
        self._waypoints: list[dict] = []
        self._lock = threading.Lock()
        self._stopped = False

        if arm:
            self._columns = ARM_JOINTS
            self._prefix = f'{arm}_'
        else:
            self._columns = BIMANUAL_JOINTS
            self._prefix = None

        self.create_subscription(JointState, TOPIC, self._joint_cb, 10)
        self.create_timer(interval, self._auto_capture)

        mode = f'{arm.upper()} arm only' if arm else 'BIMANUAL (both arms)'
        self.get_logger().info(f'Recording {mode} from {TOPIC} every {interval}s')
        self.get_logger().info('Press Enter to stop and save.')

    def _joint_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if self._prefix:
                    if name.startswith(self._prefix):
                        self._latest[name.removeprefix(self._prefix)] = pos
                else:
                    self._latest[name] = pos

    def _auto_capture(self):
        if self._stopped:
            return
        with self._lock:
            snapshot = dict(self._latest)

        missing = [c for c in self._columns if c not in snapshot]
        if missing:
            return  # joint states not yet received

        row = {c: snapshot[c] for c in self._columns}
        self._waypoints.append(row)
        idx = len(self._waypoints)
        # Print compact summary
        if self._prefix is None:
            l = ', '.join(f'{snapshot[f"left_{j}"]:.3f}' for j in ARM_JOINTS)
            r = ', '.join(f'{snapshot[f"right_{j}"]:.3f}' for j in ARM_JOINTS)
            print(f'[{idx}] L:[{l}]  R:[{r}]')
        else:
            vals = ', '.join(f'{snapshot[j]:.3f}' for j in ARM_JOINTS)
            print(f'[{idx}] [{vals}]')

    def stop(self):
        self._stopped = True

    def save(self):
        if not self._waypoints:
            print('No waypoints recorded — nothing saved.')
            return
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._columns)
            writer.writeheader()
            writer.writerows(self._waypoints)
        print(f'Saved {len(self._waypoints)} waypoints to {self.csv_path}')


def _stop_on_enter(recorder: WaypointRecorder, stop_event: threading.Event):
    try:
        input()
    except EOFError:
        pass
    recorder.stop()
    stop_event.set()


def main():
    parser = argparse.ArgumentParser(description='Auto-record joint waypoints during teleoperation')
    parser.add_argument('csv_path', help='Output CSV file path')
    parser.add_argument('--arm', choices=['left', 'right'], default=None,
                        help='Single-arm mode: which arm to record (omit for bimanual)')
    parser.add_argument('--interval', type=float, default=0.3,
                        help='Capture interval in seconds (default: 0.3)')
    args = parser.parse_args()

    rclpy.init()
    recorder = WaypointRecorder(args.csv_path, args.arm, args.interval)

    stop_event = threading.Event()
    t = threading.Thread(target=_stop_on_enter, args=(recorder, stop_event), daemon=True)
    t.start()

    try:
        while not stop_event.is_set():
            rclpy.spin_once(recorder, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        recorder.save()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
