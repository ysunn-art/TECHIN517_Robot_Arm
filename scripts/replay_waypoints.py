#!/usr/bin/env python3
"""
Replay joint waypoints via JointTrajectoryController. No MoveIt required.

Auto-detects bimanual vs single-arm from the CSV column names:
  - CSV has 'left_shoulder_pan' → bimanual: drives both arms simultaneously
  - CSV has 'shoulder_pan' (no prefix) → single arm (use --arm to pick controller)

Bimanual replay (both arms move together per waypoint):
    python3 scripts/replay_waypoints.py waypoints/phase1.csv

Single-arm replay:
    python3 scripts/replay_waypoints.py waypoints/<primitive>.csv --arm left
    python3 scripts/replay_waypoints.py waypoints/<primitive>.csv --arm right

Optional: seconds per waypoint (default 2.0):
    python3 scripts/replay_waypoints.py waypoints/phase1.csv --duration 3.0

Bringup prerequisite (bimanual):
    ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false

Bringup prerequisite (single arm):
    ros2 launch soa_bringup soa_bringup.launch.py controller:=jtc cameras:=false
"""

import argparse
import csv
import sys
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def load_csv(path: str) -> tuple[list[dict], bool]:
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    bimanual = 'left_shoulder_pan' in rows[0]
    return rows, bimanual


def secs_to_duration(secs: float) -> Duration:
    msg = Duration()
    msg.sec = int(secs)
    msg.nanosec = int((secs - msg.sec) * 1e9)
    return msg


def make_arm_goal(joint_names: list[str], positions: list[float], duration_s: float) -> FollowJointTrajectory.Goal:
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = secs_to_duration(duration_s)
    traj = JointTrajectory()
    traj.joint_names = joint_names
    traj.points = [point]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


class WaypointReplayer(Node):

    def __init__(self, bimanual: bool, arm: str | None):
        super().__init__('waypoint_replayer')
        self._bimanual = bimanual

        if bimanual:
            self._left_arm_joints = [f'left_{j}' for j in ARM_JOINTS]
            self._right_arm_joints = [f'right_{j}' for j in ARM_JOINTS]
            self._left_arm = ActionClient(self, FollowJointTrajectory,
                                          '/follower/left_arm_controller/follow_joint_trajectory')
            self._right_arm = ActionClient(self, FollowJointTrajectory,
                                           '/follower/right_arm_controller/follow_joint_trajectory')
            self._left_gripper = ActionClient(self, GripperCommand,
                                              '/follower/left_gripper_controller/gripper_cmd')
            self._right_gripper = ActionClient(self, GripperCommand,
                                               '/follower/right_gripper_controller/gripper_cmd')
        else:
            prefix = f'{arm}_' if arm else ''
            ns = arm or 'arm'
            self._arm_joints = [f'{prefix}{j}' for j in ARM_JOINTS]
            self._gripper_joint = f'{prefix}gripper'
            if arm:
                # bi bringup: gripper is a separate GripperActionController, driven via gripper_cmd
                self._arm_client = ActionClient(self, FollowJointTrajectory,
                                                f'/follower/{arm}_arm_controller/follow_joint_trajectory')
                self._gripper_client = ActionClient(self, GripperCommand,
                                                    f'/follower/{arm}_gripper_controller/gripper_cmd')
                self._single_gripper_mode = 'gripper_cmd'
            else:
                self._arm_client = ActionClient(self, FollowJointTrajectory,
                                                '/follower/arm_controller/follow_joint_trajectory')
                self._gripper_client = ActionClient(self, FollowJointTrajectory,
                                                    '/follower/gripper_controller/follow_joint_trajectory')
                self._single_gripper_mode = 'jtc'

    def _wait_servers(self, timeout: float = 10.0) -> bool:
        clients = []
        if self._bimanual:
            clients = [self._left_arm, self._right_arm, self._left_gripper, self._right_gripper]
        else:
            clients = [c for c in [self._arm_client, self._gripper_client] if c is not None]

        self.get_logger().info('Waiting for action servers...')
        for c in clients:
            if not c.wait_for_server(timeout_sec=timeout):
                self.get_logger().error(f'Server not available: {c._action_name}')
                return False
        self.get_logger().info('All action servers ready.')
        return True

    def _send_arm_goal(self, client: ActionClient, joint_names: list[str],
                       positions: list[float], duration_s: float):
        goal = make_arm_goal(joint_names, positions, duration_s)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        return client.send_goal_async(goal)

    def _send_gripper_goal(self, client: ActionClient, position: float, mode: str):
        if mode == 'gripper_cmd':
            goal = GripperCommand.Goal()
            goal.command.position = position
            goal.command.max_effort = 0.0
            return client.send_goal_async(goal)
        else:
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.time_from_start = secs_to_duration(1.0)
            traj = JointTrajectory()
            traj.joint_names = [self._gripper_joint]
            traj.points = [point]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            return client.send_goal_async(goal)

    def _spin_futures(self, *futures):
        for f in futures:
            rclpy.spin_until_future_complete(self, f)

    def _get_result(self, handle_future):
        result_future = handle_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def replay(self, rows: list[dict], duration_s: float):
        if not self._wait_servers():
            return

        self.get_logger().info(f'Replaying {len(rows)} waypoints ({duration_s}s each)')

        for i, row in enumerate(rows):
            self.get_logger().info(f'Waypoint {i+1}/{len(rows)}')

            if self._bimanual:
                # Send both arm goals simultaneously
                lf = self._send_arm_goal(
                    self._left_arm, self._left_arm_joints,
                    [float(row[j]) for j in self._left_arm_joints], duration_s)
                rf = self._send_arm_goal(
                    self._right_arm, self._right_arm_joints,
                    [float(row[j]) for j in self._right_arm_joints], duration_s)
                self._spin_futures(lf, rf)

                if not lf.result().accepted or not rf.result().accepted:
                    self.get_logger().error(f'Arm goal rejected at waypoint {i+1} — stopping')
                    return

                # Wait for both arms to finish
                lr = lf.result().get_result_async()
                rr = rf.result().get_result_async()
                self._spin_futures(lr, rr)

                # Send both grippers simultaneously
                lgf = self._send_gripper_goal(self._left_gripper, float(row['left_gripper']), 'gripper_cmd')
                rgf = self._send_gripper_goal(self._right_gripper, float(row['right_gripper']), 'gripper_cmd')
                self._spin_futures(lgf, rgf)

            else:
                arm_positions = [float(row[j]) for j in ARM_JOINTS]
                f = self._send_arm_goal(self._arm_client, self._arm_joints, arm_positions, duration_s)
                rclpy.spin_until_future_complete(self, f)
                if not f.result().accepted:
                    self.get_logger().error(f'Arm goal rejected at waypoint {i+1} — stopping')
                    return
                result_f = f.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_f)

                if self._gripper_client is not None:
                    gf = self._send_gripper_goal(self._gripper_client, float(row['gripper']),
                                                  self._single_gripper_mode)
                    rclpy.spin_until_future_complete(self, gf)

        self.get_logger().info('Replay complete')


def main():
    parser = argparse.ArgumentParser(description='Replay waypoints via JointTrajectoryController')
    parser.add_argument('csv_path', help='Waypoint CSV file')
    parser.add_argument('--arm', choices=['left', 'right'], default=None,
                        help='Single-arm override (auto-detected from CSV if omitted)')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Seconds per waypoint (default: 2.0)')
    parser.add_argument('--start', type=int, default=1,
                        help='First waypoint to replay, 1-indexed (default: 1)')
    parser.add_argument('--end', type=int, default=None,
                        help='Last waypoint to replay, 1-indexed inclusive (default: last)')
    args = parser.parse_args()

    if not Path(args.csv_path).exists():
        print(f'ERROR: {args.csv_path} not found')
        sys.exit(1)

    rows, bimanual = load_csv(args.csv_path)
    rows = rows[args.start - 1: args.end]
    mode = 'BIMANUAL' if bimanual else f'single arm ({args.arm or "from bringup"})'
    print(f'Loaded {len(rows)} waypoints from {args.csv_path} [{mode}]')

    rclpy.init()
    node = WaypointReplayer(bimanual=bimanual, arm=args.arm)
    try:
        node.replay(rows, args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
