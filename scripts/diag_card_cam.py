#!/usr/bin/env python3
"""
Diagnose card detection on ONE camera — answers "is the model actually seeing
cards on this side?".

Pulls a burst of frames and runs YOLO on each, printing per-frame detections
(label + confidence) and a summary. Use it with a real card under the wrist cam.

Usage:
    # via the dashboard hub (preferred — same path the game uses):
    python3 scripts/diag_card_cam.py player        # right wrist
    python3 scripts/diag_card_cam.py dealer        # left wrist
    python3 scripts/diag_card_cam.py overhead

    # straight off a device (bypasses the dashboard):
    python3 scripts/diag_card_cam.py /dev/video0

    # options
    python3 scripts/diag_card_cam.py player --frames 40 --conf 0.5

Run the dashboard server first if you want the hub path; otherwise the named
camera will say it can't reach the hub and you should pass a /dev/videoN path.
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import card_detector as cd  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description='Diagnose card detection on one camera')
    ap.add_argument('camera', help="hub name (player/dealer/overhead) or a /dev/videoN path")
    ap.add_argument('--frames', type=int, default=30, help='frames to grab (default 30)')
    ap.add_argument('--conf', type=float, default=0.5,
                    help='min confidence to report (default 0.5 — lower than the '
                         'game so you can see weak/false detections too)')
    args = ap.parse_args()

    is_device = args.camera.startswith('/dev/') or args.camera.isdigit()
    det = cd.CardDetector(conf=args.conf, min_detections=1)

    if is_device:
        dev = int(args.camera) if args.camera.isdigit() else args.camera
        print(f'Opening device {dev} directly...')
        src = cd._LocalSource(det._open_camera(dev))
    else:
        if not cd.CAM_HUB:
            print('CARD_CAM_HUB is disabled; pass a /dev/videoN path instead.')
            sys.exit(1)
        print(f'Reading "{args.camera}" from dashboard hub {cd.CAM_HUB}...')
        src = cd._HubSource(cd.CAM_HUB, args.camera)

    if not src.isOpened():
        print('ERROR: no frames available from that source. '
              'Is the camera connected / dashboard running / device number correct?')
        sys.exit(2)

    totals: dict[str, int] = defaultdict(int)
    best: dict[str, float] = defaultdict(float)
    got = 0
    for i in range(args.frames):
        ok, frame = src.read()
        if not ok:
            print(f'  frame {i+1}: <no frame>')
            continue
        got += 1
        dets = []
        for r in det.model(frame, verbose=False):
            for box in r.boxes:
                c = float(box.conf[0])
                if c >= args.conf:
                    label = r.names[int(box.cls[0])]
                    dets.append((label, c))
                    totals[label] += 1
                    best[label] = max(best[label], c)
        shown = ', '.join(f'{l}:{c:.2f}' for l, c in sorted(dets, key=lambda x: -x[1])) or '—'
        print(f'  frame {i+1:2d}: {shown}')
    src.release()

    print('\n=== summary ===')
    print(f'frames with image: {got}/{args.frames}')
    if not totals:
        print('No cards detected at all → the model ran but saw nothing. '
              'Check the camera is aimed at the card, lighting, and focus.')
    else:
        print(f'{"label":6s} {"frames":>6s} {"best conf":>9s}')
        for label, n in sorted(totals.items(), key=lambda x: -x[1]):
            print(f'{label:6s} {n:6d} {best[label]:9.2f}')
        print('\nA real card should dominate (high frame count, high conf). '
              'Sporadic low-count labels are the false reads to filter with '
              'CARD_MIN_DETECTIONS / CARD_CONF.')


if __name__ == '__main__':
    main()
