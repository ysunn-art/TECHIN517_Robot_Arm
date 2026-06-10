#!/usr/bin/env python3
"""
Raise Max_Torque_Limit and Torque_Limit for motor ID 4 (wrist_flex) on both arms.
Max value is 1000. Default is typically 300-500. Increase carefully.

Usage:
    PYTHONPATH=/home/ubuntu/techin517_final/lerobot/src \
    python3 scripts/set_wrist_torque.py --value 800
"""
import argparse
import sys

sys.path.insert(0, '/home/ubuntu/techin517_final/lerobot/src')

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

LEFT_PORT  = '/dev/ttyACM1'   # left follower
RIGHT_PORT = '/dev/ttyACM3'   # right follower
MOTOR_ID   = 4                # wrist_flex

MOTORS = {
    'shoulder_pan':  Motor(1, 'sts3215', MotorNormMode.RANGE_M100_100),
    'shoulder_lift': Motor(2, 'sts3215', MotorNormMode.RANGE_M100_100),
    'elbow_flex':    Motor(3, 'sts3215', MotorNormMode.RANGE_M100_100),
    'wrist_flex':    Motor(4, 'sts3215', MotorNormMode.RANGE_M100_100),
    'wrist_roll':    Motor(5, 'sts3215', MotorNormMode.RANGE_M100_100),
    'gripper':       Motor(6, 'sts3215', MotorNormMode.RANGE_0_100),
}


def set_torque(port: str, arm_name: str, value: int) -> None:
    bus = FeetechMotorsBus(port=port, motors=MOTORS)
    bus.connect()
    try:
        before = bus.read('Torque_Limit', 'wrist_flex')
        print(f'  [{arm_name}] wrist_flex Torque_Limit before: {before}')
        bus.write('Max_Torque_Limit', 'wrist_flex', value)
        bus.write('Torque_Limit',     'wrist_flex', value)
        after = bus.read('Torque_Limit', 'wrist_flex')
        print(f'  [{arm_name}] wrist_flex Torque_Limit after:  {after}')
    finally:
        bus.disconnect()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--value', type=int, default=800,
                        help='Torque limit to set (0-1000, default 800)')
    args = parser.parse_args()

    if not 0 <= args.value <= 1000:
        print('ERROR: value must be 0-1000')
        sys.exit(1)

    print(f'Setting wrist_flex (motor 4) torque limit to {args.value} on both arms...')
    set_torque(LEFT_PORT,  'left',  args.value)
    set_torque(RIGHT_PORT, 'right', args.value)
    print('Done.')


if __name__ == '__main__':
    main()
