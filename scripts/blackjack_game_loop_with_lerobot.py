#!/usr/bin/env python3
"""
Blackjack game loop WITH lerobot chip policy — self-contained.

Run just this one command (no separate bringup terminal needed):
    python3 scripts/blackjack_game_loop_with_lerobot.py

It manages everything:
  • launches the JTC bringup itself (cameras off) and waits for controllers active
  • deals/plays a hand exactly like blackjack_game_loop.py
  • on a decisive result, runs the matching trained lerobot policy directly over
    the right follower's serial bus:
        Player wins → right_pay_out_chip   (pay the player)
        Dealer wins → right_collect_chip   (collect the chip)
        Tie         → no chip motion
  • to free the serial bus for lerobot it HARD-kills the bringup (SIGKILL, so servo
    torque stays latched and the arm holds), runs the policy, then relaunches bringup
  • ENTER to play another hand, Ctrl-C to quit (bringup is torn down on exit)
"""

import os
import signal
import subprocess
import sys
import termios
import time
import tty
import urllib.error
import urllib.request
from pathlib import Path

from card_detector import CardDetector, hand_value
import dashboard

SCRIPTS_DIR   = Path(__file__).parent
WAYPOINTS_DIR = SCRIPTS_DIR.parent / 'waypoints'

RIGHT_DRAW_CARDS    = WAYPOINTS_DIR / 'right_draw_card.csv'
LEFT_DRAW_CARD      = WAYPOINTS_DIR / 'left_draw_card.csv'
LEFT_DRAW_CARD_HOLD = WAYPOINTS_DIR / 'left_draw_card_hold.csv'
LEFT_REVEAL_CARD    = WAYPOINTS_DIR / 'left_reveal_card.csv'

# Wrist cameras: right arm deals to player, left arm deals to dealer.
RIGHT_WRIST_CAM = '/dev/video2'   # right arm → player side
LEFT_WRIST_CAM  = '/dev/video0'   # left arm  → dealer side

REPO_ROOT = SCRIPTS_DIR.parent

# ── ROS 2 bringup (JTC) lifecycle ────────────────────────────────────────────
ROS_SETUP       = '/opt/ros/humble/setup.bash'
WORKSPACE_SETUP = REPO_ROOT / 'ros2_ws' / 'install' / 'setup.bash'
BRINGUP_CMD     = ('ros2 launch soa_bringup bi_soa_bringup.launch.py '
                   'controller:=jtc cameras:=false')

# Dashboard server owns the V4L2 cameras; we ask it to release them for the policy.
DASHBOARD_URL = os.environ.get('CARD_CAM_HUB', 'http://127.0.0.1:8000').rstrip('/')

# ── lerobot chip policy (direct serial, no ROS) ──────────────────────────────
LEROBOT_SRC    = REPO_ROOT / 'lerobot' / 'src'
LEROBOT_RECORD = REPO_ROOT / 'lerobot' / '.venv' / 'bin' / 'lerobot-record'
DATA_DIR       = REPO_ROOT / 'data'

# The chip arm is the RIGHT follower on its serial bus.
POLICY_ROBOT_TYPE = 'so101_follower'
POLICY_ROBOT_PORT = '/dev/ttyACM3'
POLICY_ROBOT_ID   = 'gix-follower4'
POLICY_EPISODE_S  = 13
# Camera observation for the policy — MUST match what the dataset was recorded
# with: wrist_right = right wrist (video0), overhead = video8.
POLICY_CAMERAS = (
    '{"wrist_right": {"type": "opencv", "index_or_path": "/dev/video2", '
    '"width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 10}, '
    '"overhead": {"type": "opencv", "index_or_path": "/dev/video8", '
    '"width": 640, "height": 480, "fps": 30, "warmup_s": 15}}'
)

# Decisive outcome → (checkpoint, task prompt, throwaway eval dataset repo_id).
# A 'Push' (tie) has no entry → no chip motion.
CHIP_POLICIES = {
    'Player wins': (
        DATA_DIR / 'right_pay_out_chip/policy_smolvla/checkpoints/100000/pretrained_model',
        'pay out chip to player',
        'aarony8881/eval_right_pay_out_chip',
    ),
    'Dealer wins': (
        DATA_DIR / 'right_collect_chip/policy_smolvla/checkpoints/100000/pretrained_model',
        'collect chip from player',
        'aarony8881/eval_right_collect_chip',
    ),
}


def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        # cbreak (not raw) keeps ISIG enabled so Ctrl-C still raises KeyboardInterrupt
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


ROS_PYTHON = 'python3.10'

def replay(csv_path: Path, arm: str | None = None, duration: float = 0.3):
    """Blocking replay — use for motions with no card detection (Phase 1)."""
    cmd = _replay_cmd(csv_path, arm, duration)
    print(f'  → {csv_path.name}' + (f' ({arm} arm)' if arm else ' (bimanual)'))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f'[ERROR] replay failed (exit {result.returncode}), stopping.')
        sys.exit(1)


def replay_bg(csv_path: Path, arm: str | None = None, duration: float = 0.3):
    """Non-blocking replay — returns Popen so we can scan cameras during motion."""
    cmd = _replay_cmd(csv_path, arm, duration)
    print(f'  → {csv_path.name}' + (f' ({arm} arm)' if arm else ' (bimanual)'))
    return subprocess.Popen(cmd)


def _replay_cmd(csv_path: Path, arm: str | None, duration: float) -> list:
    # Source ROS in-shell so replay_waypoints (rclpy) works even when this script
    # is launched from a plain, un-sourced terminal.
    inner = (f'{ROS_PYTHON} {SCRIPTS_DIR / "replay_waypoints.py"} '
             f'{csv_path} --duration {duration}')
    if arm:
        inner += f' --arm {arm}'
    full = f'source {ROS_SETUP} && source {WORKSPACE_SETUP} && exec {inner}'
    return ['bash', '-lc', full]


def wait_key(prompt: str) -> str:
    print(prompt, flush=True)
    while True:
        key = getch()
        if key in ('0', '1', '2', '3', '4', '5', '6', '7', '8', '9'):
            return key
        print(f'  (unknown key {repr(key)} — ignored)', flush=True)


def publish(player_detector: CardDetector, dealer_detector: CardDetector,
            phase: str, status: str, hand_num: int,
            dealer_hidden: int = 0, result: str | None = None) -> None:
    """Push the full table state to the dashboard (best-effort, never raises)."""
    dashboard.push_state(
        hand_num=hand_num, phase=phase, status=status, result=result,
        player={'cards': list(player_detector.ordered),
                'total': hand_value(player_detector.registry)},
        dealer={'cards': list(dealer_detector.ordered),
                'total': hand_value(dealer_detector.registry),
                'hidden': dealer_hidden},
    )


def replay_and_scan(side: str, player_detector: CardDetector, dealer_detector: CardDetector,
                    cameras: list, csv_path: Path, arm: str,
                    phase: str, hand_num: int, dealer_hidden: int = 0,
                    duration: float = 0.3, extra_frames: int = 60) -> None:
    """Run arm motion and scan cameras simultaneously — detects cards during the
    deal/flip, pushing each card to the dashboard the instant it's confirmed.

    duration     — seconds per waypoint (slower = more frames over the card).
    extra_frames — frames captured after the motion settles (more = surer confirm)."""
    label = side.capitalize()
    detector = player_detector if side == 'player' else dealer_detector

    # Live update: publish the full table state as soon as a new card is confirmed.
    def on_commit(registry, newly):
        publish(player_detector, dealer_detector, phase,
                f'{label} drew {", ".join(sorted(newly))}', hand_num,
                dealer_hidden=dealer_hidden)
    detector.on_commit = on_commit

    try:
        proc = replay_bg(csv_path, arm, duration)
        print(f'  [Vision] scanning during motion...', flush=True)
        newly = detector.scan_during(cameras, proc, extra_frames=extra_frames, dashboard_side=side)
        rc = proc.wait()
        if rc != 0:
            print(f'[ERROR] replay failed (exit {rc}), stopping.')
            sys.exit(1)
        if newly:
            print(f'  [Vision] new cards detected: {sorted(newly)}')
        else:
            print(f'  [Vision] no new cards detected')
        total = hand_value(detector.registry)
        cards_str = ', '.join(sorted(detector.registry)) if detector.registry else '(none)'
        print(f'  [{label}] cards: {cards_str}  →  total: {total}')
    finally:
        detector.on_commit = None


def play_hand(hand_num: int = 1):
    player_detector = CardDetector()
    dealer_detector = CardDetector()

    # Phase 1: initial deal — 4 separated single-arm steps (the other arm holds position).
    #   1. right deals player card 1
    #   2. left  deals dealer card 1
    #   3. right deals player card 2
    #   4. left  deals dealer card 2 (hold — hidden hole card)
    print('[Phase 1] Initial deal...')
    publish(player_detector, dealer_detector, 'phase1', 'Dealing opening hand…', hand_num)
    # Scan the player wrist during both opening deals so the starting hand is
    # registered even if the player stands without hitting. Scan the dealer's
    # first (up) card too — only the second dealer card (step 4) stays hidden.
    replay_and_scan('player', player_detector, dealer_detector, [RIGHT_WRIST_CAM],
                    RIGHT_DRAW_CARDS, 'right', 'phase1', hand_num, dealer_hidden=0)
    replay_and_scan('dealer', player_detector, dealer_detector, [LEFT_WRIST_CAM],
                    LEFT_DRAW_CARD, 'left', 'phase1', hand_num, dealer_hidden=0)
    replay_and_scan('player', player_detector, dealer_detector, [RIGHT_WRIST_CAM],
                    RIGHT_DRAW_CARDS, 'right', 'phase1', hand_num, dealer_hidden=0)
    replay(LEFT_DRAW_CARD_HOLD, arm='left')
    publish(player_detector, dealer_detector, 'phase1',
            'Opening hand dealt — dealer hole card face down', hand_num, dealer_hidden=1)

    # Player turn
    print('\n[Player Turn]  1-9 = HIT   0 = STAND')
    hits = 0
    player_bust = False
    while True:
        total = hand_value(player_detector.registry)
        if total > 21:
            print(f'  Player total is {total} — BUST.')
            player_bust = True
            publish(player_detector, dealer_detector, 'player_turn',
                    f'Player busts with {total}', hand_num, dealer_hidden=1)
            break
        if total == 21:
            print(f'  Player total is 21 — auto-stand.')
            publish(player_detector, dealer_detector, 'player_turn',
                    'Player has 21 — stands', hand_num, dealer_hidden=1)
            break
        d_total = hand_value(dealer_detector.registry)
        publish(player_detector, dealer_detector, 'player_turn',
                f'Player turn — {total}. Hit or stand?', hand_num, dealer_hidden=1)
        key = wait_key(f'  [Player {total} vs Dealer {d_total}]  HIT=1-9  STAND=0')
        if key in ('1', '2', '3', '4', '5', '6', '7', '8', '9'):
            hits += 1
            print(f'  HIT #{hits}')
            publish(player_detector, dealer_detector, 'player_turn',
                    f'Player hits…', hand_num, dealer_hidden=1)
            replay_and_scan('player', player_detector, dealer_detector, [RIGHT_WRIST_CAM],
                            RIGHT_DRAW_CARDS, 'right', 'player_turn', hand_num, dealer_hidden=1)
            publish(player_detector, dealer_detector, 'player_turn',
                    f'Player now has {hand_value(player_detector.registry)}', hand_num, dealer_hidden=1)
        else:  # '0'
            print('  STAND — player turn over.')
            publish(player_detector, dealer_detector, 'player_turn',
                    f'Player stands on {total}', hand_num, dealer_hidden=1)
            break

    # Player is done (stand, 21, or bust): reveal the held hole card first — scan left wrist during the reveal
    print('\n[Reveal] Revealing dealer hole card...')
    publish(player_detector, dealer_detector, 'reveal',
            'Revealing dealer hole card…', hand_num, dealer_hidden=1)
    # Slow the flip and dwell longer here so the hole card is reliably confirmed
    # BEFORE the dealer decides to hit (else a lone Ace looks like 11 and draws).
    replay_and_scan('dealer', player_detector, dealer_detector, [LEFT_WRIST_CAM],
                    LEFT_REVEAL_CARD, 'left', 'reveal', hand_num, dealer_hidden=0,
                    duration=0.6, extra_frames=60)
    publish(player_detector, dealer_detector, 'reveal',
            f'Dealer shows {hand_value(dealer_detector.registry)}', hand_num)

    # Player bust → dealer already wins; reveal only, skip the dealer turn.
    if not player_bust:
        # Dealer turn — fully automatic (house rules): hit on 16 or below, stand on 17+.
        print('\n[Dealer Turn]  (automatic: hit ≤16, stand ≥17)')
        deals = 0
        while True:
            total = hand_value(dealer_detector.registry)
            if total >= 17:
                verb = 'busts' if total > 21 else 'stands'
                print(f'  Dealer {verb} on {total}.')
                publish(player_detector, dealer_detector, 'dealer_turn',
                        f'Dealer {verb} on {total}', hand_num)
                break
            print(f'  Dealer must hit on {total} (≤16) — dealing...')
            publish(player_detector, dealer_detector, 'dealer_turn',
                    f'Dealer must hit on {total}…', hand_num)
            deals += 1
            replay_and_scan('dealer', player_detector, dealer_detector, [LEFT_WRIST_CAM],
                            LEFT_DRAW_CARD, 'left', 'dealer_turn', hand_num)
            publish(player_detector, dealer_detector, 'dealer_turn',
                    f'Dealer now has {hand_value(dealer_detector.registry)}', hand_num)

    # Final result
    p_total = hand_value(player_detector.registry)
    d_total = hand_value(dealer_detector.registry)
    print(f'\n=== Hand complete ===')
    print(f'  Player: {sorted(player_detector.registry)}  →  {p_total}')
    print(f'  Dealer: {sorted(dealer_detector.registry)}  →  {d_total}')
    if p_total > 21:
        result = 'Dealer wins'
        print('  Result: Player BUST — Dealer wins')
    elif d_total > 21:
        result = 'Player wins'
        print('  Result: Dealer BUST — Player wins')
    elif p_total > d_total:
        result = 'Player wins'
        print('  Result: Player wins')
    elif d_total > p_total:
        result = 'Dealer wins'
        print('  Result: Dealer wins')
    else:
        result = 'Push'
        print('  Result: Tie')

    publish(player_detector, dealer_detector, 'result',
            f'Player {p_total} · Dealer {d_total}', hand_num, result=result)
    return result


# ── Dashboard camera ownership ───────────────────────────────────────────────
def dashboard_cameras(action: str) -> None:
    """Best-effort: ask the dashboard to release/resume the V4L2 cameras so the
    lerobot policy can own them. Silent no-op if no dashboard is running."""
    try:
        urllib.request.urlopen(f'{DASHBOARD_URL}/cameras/{action}',
                               data=b'', timeout=5)
        print(f'[Cameras] dashboard {action}d the cameras.')
    except urllib.error.HTTPError as e:
        # Dashboard is up but lacks the pause/resume route — it still holds the
        # cameras, so the policy WILL fail to open them. Make this impossible to miss.
        print(f'[Cameras] WARNING: dashboard returned HTTP {e.code} for '
              f'/cameras/{action} — it is running OLD code and still owns the '
              f'cameras. Restart dashboard_server.py, then retry.', flush=True)
    except Exception:
        pass  # no dashboard listening → cameras are already free, nothing to do


# ── ROS 2 bringup lifecycle ──────────────────────────────────────────────────
_bringup_proc = None


def _sourced(cmd: str) -> list:
    return ['bash', '-lc', f'source {ROS_SETUP} && source {WORKSPACE_SETUP} && {cmd}']


def launch_bringup():
    """Start the JTC bringup as its own process group and wait until the arm
    controllers are active (not just present)."""
    global _bringup_proc
    print('[Bringup] launching JTC bringup (cameras off)…', flush=True)
    _bringup_proc = subprocess.Popen(_sourced(f'exec {BRINGUP_CMD}'),
                                     start_new_session=True)
    if not wait_bringup_ready():
        print('[ERROR] bringup did not reach active controllers in time.')
        kill_bringup()
        sys.exit(1)
    print('[Bringup] controllers active.', flush=True)


def wait_bringup_ready(timeout: float = 90.0) -> bool:
    deadline = time.time() + timeout
    check = 'ros2 control list_controllers -c /follower/controller_manager'
    while time.time() < deadline:
        if _bringup_proc and _bringup_proc.poll() is not None:
            return False  # launch process exited early
        out = subprocess.run(_sourced(check), capture_output=True, text=True)
        for line in out.stdout.splitlines():
            # 'inactive' contains 'active', so exclude it explicitly.
            if 'right_arm_controller' in line and 'inactive' not in line and 'active' in line:
                return True
        time.sleep(2.0)
    return False


def kill_bringup():
    """Hard-kill the bringup process group with SIGKILL. Because on_deactivate is
    skipped, the Feetech servos keep torque latched and the arm holds position
    while the serial bus is freed for lerobot."""
    global _bringup_proc
    if _bringup_proc is not None:
        print('[Bringup] hard-killing bringup (arm holds torque)…', flush=True)
        try:
            os.killpg(os.getpgid(_bringup_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        _bringup_proc.wait()
        _bringup_proc = None
    # Sweep any stragglers that escaped the group, then let the OS release the ports.
    subprocess.run(['pkill', '-9', '-f', 'bi_soa_bringup'])
    subprocess.run(['pkill', '-9', '-f', 'ros2_control_node'])
    time.sleep(2.0)


def run_chip_policy(result: str) -> bool:
    """Run the trained chip policy for the outcome over the right follower's serial
    bus. Returns True if the bringup was torn down (caller must relaunch it)."""
    entry = CHIP_POLICIES.get(result)
    if entry is None:
        print(f'[Policy] {result} — no chip motion.')
        return False
    ckpt, task, repo_id = entry
    if not ckpt.exists():
        print(f'[Policy] checkpoint missing, skipping: {ckpt}')
        return False

    print(f'\n[Policy] {result} → running "{task}"…', flush=True)
    kill_bringup()                      # free the serial bus (arm holds via latched torque)
    dashboard_cameras('release')        # free /dev/video0 + /dev/video8 for lerobot

    # lerobot refuses to overwrite an existing eval dataset — clear the cache.
    subprocess.run(['rm', '-rf', str(Path.home() / '.cache/huggingface/lerobot' / repo_id)])

    env = os.environ.copy()
    env['PYTHONPATH'] = f'{LEROBOT_SRC}:' + env.get('PYTHONPATH', '')
    cmd = [
        str(LEROBOT_RECORD),
        f'--robot.type={POLICY_ROBOT_TYPE}',
        f'--robot.port={POLICY_ROBOT_PORT}',
        f'--robot.id={POLICY_ROBOT_ID}',
        f'--robot.cameras={POLICY_CAMERAS}',
        f'--policy.path={ckpt}',
        f'--dataset.repo_id={repo_id}',
        f'--dataset.single_task={task}',
        '--dataset.num_episodes=1',
        f'--dataset.episode_time_s={POLICY_EPISODE_S}',
        '--dataset.push_to_hub=false',
        '--dataset.streaming_encoding=true',
        '--dataset.encoder_threads=2',
        '--play_sounds=false',
    ]
    try:
        rc = subprocess.run(cmd, env=env).returncode
    finally:
        dashboard_cameras('resume')     # give the cameras back to the dashboard
    if rc != 0:
        print(f'[Policy] WARNING: lerobot-record exited {rc}.')
    else:
        print('[Policy] chip motion complete.')
    return True


def main():
    print('=== Blackjack Game Loop (with lerobot chip policy) ===')

    kill_bringup()  # clear any bringup already running (e.g. a stale Terminal 1)
    launch_bringup()
    hand_num = 0
    try:
        while True:
            hand_num += 1
            print(f'\n========== HAND {hand_num} ==========')
            result = play_hand(hand_num)

            # Pay out / collect via the trained policy (skips on a tie).
            if run_chip_policy(result):
                launch_bringup()  # policy tore down the bringup → bring it back

            print('\nPress ENTER to play another hand, or Ctrl-C to end the game.', flush=True)
            input()
    finally:
        kill_bringup()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n[Interrupted] Exiting.')
        sys.exit(130)
