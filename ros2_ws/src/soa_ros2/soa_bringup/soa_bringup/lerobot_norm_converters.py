"""Rosetta custom converters that mirror lerobot's per-joint range normalization.

Use these in rosetta_contracts to make the SO-101 follower's Rosetta
pipeline numerically equivalent to running a policy through
`lerobot.SO101Follower.send_action` / `get_observation`.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray


_ARM_JOINTS = (
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
)
_GRIPPER = 'gripper'
_ALL_JOINTS = _ARM_JOINTS + (_GRIPPER,)


def _base_joint(joint: str) -> str:
    """Strip a bimanual ``left_``/``right_`` prefix to get the calibration key.

    Bimanual bringup publishes/consumes prefixed joint names
    (``right_shoulder_pan``) while the lerobot calibration JSON keys are
    unprefixed (``shoulder_pan``). Single-arm bringup uses unprefixed names,
    which pass through unchanged.
    """
    for prefix in ('left_', 'right_'):
        if joint.startswith(prefix):
            return joint[len(prefix):]
    return joint

_TICKS_PER_REV = 4096
_CENTER_TICK = 2048
_RAD_PER_TICK = 2.0 * math.pi / _TICKS_PER_REV

# Empirical per-joint command correction (radians), keyed by base joint name.
# Verified by measurement: with arm ports correct, the converter math already
# matches lerobot (shoulder_pan: lerobot norm 35.78 -> 0.406 rad vs ROS 0.405
# rad, ~0.06 deg). No correction needed. The earlier "+30 deg" was an artifact
# of the left-leader/right-follower port swap, not the converter. Leave empty.
_JOINT_CORRECTION_RAD: dict[str, float] = {}

_RANGES_CACHE: dict[str, tuple[int, int, int]] | None = None


def _load_ranges() -> dict[str, tuple[int, int, int]]:
    bringup_share = get_package_share_directory('soa_bringup')
    params_path = os.path.join(bringup_share, 'config', 'soa_params.yaml')
    with open(params_path) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    follower = params['follower']
    cal_path = os.path.join(follower['calibration_dir'], f"{follower['id']}.json")
    with open(cal_path) as f:
        cal = json.load(f)

    ranges: dict[str, tuple[int, int, int]] = {}
    for j in _ALL_JOINTS:
        if j not in cal:
            raise KeyError(
                f"Calibration file {cal_path} is missing joint '{j}'. "
                f"Found joints: {sorted(cal.keys())}"
            )
        entry = cal[j]
        for field in ('range_min', 'range_max'):
            if field not in entry:
                raise KeyError(
                    f"Calibration file {cal_path} joint '{j}' is missing "
                    f"required field '{field}' (got: {sorted(entry.keys())})"
                )
        ranges[j] = (
            int(entry['range_min']),
            int(entry['range_max']),
            int(entry.get('drive_mode', 0)),
        )
    return ranges


def _ranges() -> dict[str, tuple[int, int, int]]:
    global _RANGES_CACHE
    if _RANGES_CACHE is None:
        _RANGES_CACHE = _load_ranges()
    return _RANGES_CACHE


def _rad_to_tick(rad: float) -> float:
    return rad / _RAD_PER_TICK + _CENTER_TICK


def _tick_to_rad(tick: float) -> float:
    return (tick - _CENTER_TICK) * _RAD_PER_TICK


def _tick_to_norm(joint: str, tick: float) -> float:
    lo, hi, drive = _ranges()[joint]
    bounded = min(hi, max(lo, tick))
    if joint == _GRIPPER:
        norm = (bounded - lo) / (hi - lo) * 100.0
        return 100.0 - norm if drive else norm
    norm = (bounded - lo) / (hi - lo) * 200.0 - 100.0
    return -norm if drive else norm


def _norm_to_tick(joint: str, val: float) -> float:
    lo, hi, drive = _ranges()[joint]
    if joint == _GRIPPER:
        if drive:
            val = 100.0 - val
        bounded = min(100.0, max(0.0, val))
        return bounded / 100.0 * (hi - lo) + lo
    if drive:
        val = -val
    bounded = min(100.0, max(-100.0, val))
    return (bounded + 100.0) / 200.0 * (hi - lo) + lo


def decode_joint_state_norm(msg: Any, spec: Any) -> np.ndarray:
    if not spec.names:
        raise ValueError('decode_joint_state_norm requires selector.names; got empty list.')

    name_to_idx = {n: i for i, n in enumerate(msg.name)}
    out = np.empty(len(spec.names), dtype=np.float64)
    for i, selector in enumerate(spec.names):
        field, joint = (selector.split('.', 1) if '.' in selector else ('position', selector))
        if field != 'position':
            raise ValueError(f"decode_joint_state_norm only supports 'position.*' selectors; got '{selector}'.")
        base = _base_joint(joint)
        if base not in _ALL_JOINTS:
            raise ValueError(f"Unknown joint '{joint}' in selector '{selector}'. Expected one of: {list(_ALL_JOINTS)}")
        if joint not in name_to_idx:
            raise ValueError(f"Joint '{joint}' not in JointState.name (got: {list(msg.name)}).")
        rad = float(msg.position[name_to_idx[joint]])
        out[i] = _tick_to_norm(base, _rad_to_tick(rad))
    return out


def _encode_norm_to_radians(action_vec: Any, spec: Any) -> Float64MultiArray:
    arr = np.asarray(action_vec, dtype=np.float64).flatten()
    if len(arr) != len(spec.names):
        raise ValueError(
            f"Action vector length {len(arr)} != selector.names length "
            f"{len(spec.names)} (names: {list(spec.names)})."
        )
    out = np.empty(len(arr), dtype=np.float64)
    for i, selector in enumerate(spec.names):
        field, joint = (selector.split('.', 1) if '.' in selector else ('position', selector))
        if field != 'position':
            raise ValueError(f"lerobot-norm encoders only support 'position.*' selectors; got '{selector}'.")
        base = _base_joint(joint)
        if base not in _ALL_JOINTS:
            raise ValueError(f"Unknown joint '{joint}' in selector '{selector}'. Expected one of: {list(_ALL_JOINTS)}")
        out[i] = _tick_to_rad(_norm_to_tick(base, float(arr[i]))) + _JOINT_CORRECTION_RAD.get(base, 0.0)

    msg = Float64MultiArray()
    msg.data = out.tolist()
    return msg


def encode_arm_fwd_norm(action_vec: Any, spec: Any, stamp_ns: int | None = None) -> Float64MultiArray:
    _ = stamp_ns
    return _encode_norm_to_radians(action_vec, spec)


def encode_gripper_fwd_norm(action_vec: Any, spec: Any, stamp_ns: int | None = None) -> Float64MultiArray:
    _ = stamp_ns
    return _encode_norm_to_radians(action_vec, spec)
