#!/usr/bin/env python3
"""
Blackjack game loop — bimanual JTC bringup must already be running.

Flow:
  1. Phase 1: initial deal — 4 separated single-arm steps (right, left, right, left-hold)
  2. Player turn loop:
       1-9  → hit: right_draw_cards.csv (right arm) → scan → show player total
       0    → stand: end player turn
  3. Player done (stand or 21+): reveal hole card via left_reveal_card.csv → scan → show dealer total
  4. Dealer turn loop:
       1-9  → deal: left_draw_card.csv (left arm) → scan → show dealer total
       0    → end dealer turn

Prerequisite (Terminal 1):
    ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false
"""

import subprocess
import sys
import termios
import tty
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
    cmd = [ROS_PYTHON, str(SCRIPTS_DIR / 'replay_waypoints.py'), str(csv_path),
           '--duration', str(duration)]
    if arm:
        cmd += ['--arm', arm]
    return cmd


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


def main():
    print('=== Blackjack Game Loop ===')
    print('Prerequisite: bi_soa_bringup.launch.py controller:=jtc must be running')

    hand_num = 0
    while True:
        hand_num += 1
        print(f'\n========== HAND {hand_num} ==========')
        play_hand(hand_num)
        print('\nPress ENTER to play another hand, or Ctrl-C to end the game.', flush=True)
        input()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n[Interrupted] Exiting.')
        sys.exit(130)
