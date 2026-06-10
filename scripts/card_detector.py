import os
import time
import urllib.error
import urllib.request
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

WEIGHTS = Path(__file__).parent.parent / 'Playing-Cards-Detection' / 'yolov8s_playing_cards.pt'

# When the dashboard server is up it owns the wrist cameras and serves their
# frames, so the detector reads from it instead of opening the device (which
# would collide on V4L2). Set CARD_CAM_HUB='' to always open devices directly.
CAM_HUB = (os.environ.get('CARD_CAM_HUB', 'http://127.0.0.1:8000') or '').rstrip('/') or None


class _LocalSource:
    """Reads frames straight from a V4L2 device (original behaviour)."""

    def __init__(self, cap):
        self.cap = cap

    def isOpened(self):
        return self.cap.isOpened()

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


class _HubSource:
    """Reads frames from the dashboard server's /frame/<name> endpoint.

    Deduplicates by the server's X-Frame-Id so each read() returns a *new*
    frame, matching the pacing of a real camera read.
    """

    def __init__(self, hub, name):
        self.url = f'{hub}/frame/{name}'
        self.last_id = None
        self._ok = self._probe()

    def isOpened(self):
        return self._ok

    def _fetch(self):
        """Returns (status, data, frame_id):
           'ok'       — fresh frame returned
           'notready' — server up but no frame yet (HTTP 404); keep using the
                        hub, never open the device the server owns
           'down'     — server unreachable; caller should open the device."""
        try:
            r = urllib.request.urlopen(self.url, timeout=0.6)
            data = r.read()
            return ('ok', data, r.headers.get('X-Frame-Id')) if data else ('notready', None, None)
        except urllib.error.HTTPError:
            return 'notready', None, None
        except Exception:
            return 'down', None, None

    def _probe(self):
        # Wait briefly for the server's first frame. If the server is down OR it
        # has no frames for this camera after the window, fall back to opening
        # the device directly so detection is never starved by a dead hub cam.
        deadline = time.time() + 3.0
        while time.time() < deadline:
            status, _, _ = self._fetch()
            if status == 'ok':
                return True
            if status == 'down':
                return False
            time.sleep(0.1)
        return False  # server up but no frames for this cam — use the device

    def read(self):
        for _ in range(50):                 # wait up to ~1s for a fresh frame
            status, data, fid = self._fetch()
            if status == 'down':
                return False, None
            if status == 'ok' and fid != self.last_id:
                self.last_id = fid
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    return True, frame
            time.sleep(0.02)
        return False, None

    def release(self):
        pass


def card_value(label: str) -> int:
    """Blackjack value of a card label like 'AH', 'KS', '10D', '7C'."""
    rank = label[:-1]  # strip suit letter
    if rank == 'A':
        return 11
    if rank in ('K', 'Q', 'J'):
        return 10
    return int(rank)


def hand_value(cards: set) -> int:
    """Total blackjack value of a card set, with soft-ace reduction."""
    total = sum(card_value(c) for c in cards)
    aces = sum(1 for c in cards if c.startswith('A'))
    while total > 21 and aces:
        total -= 10
        aces -= 1
    return total


class CardDetector:
    """
    Persistent card registry from live camera feeds.

    Cards are added when seen in >= min_detections frames across all scans.
    They are never removed mid-hand — temporal disappearance (arm occlusion,
    frame edge) does not drop already-confirmed cards.
    Call reset() between hands.
    """

    def __init__(self, conf: float | None = None, min_detections: int | None = None):
        self.model = YOLO(str(WEIGHTS))
        # Higher = fewer false reads. Tune live via env without editing code:
        #   CARD_CONF (per-frame YOLO confidence, default 0.85)
        #   CARD_MIN_DETECTIONS (frames a card must be seen before confirming, default 8)
        self.conf = conf if conf is not None else float(os.environ.get('CARD_CONF', '0.85'))
        self.min_detections = (min_detections if min_detections is not None
                               else int(os.environ.get('CARD_MIN_DETECTIONS', '8')))
        self._counts: dict[str, int] = defaultdict(int)
        self.registry: set[str] = set()
        self.ordered: list[str] = []   # cards in the order they were confirmed (deal order)
        # Optional callback(registry, newly) fired the moment new cards are
        # confirmed — lets the UI show a card as soon as it's detected.
        self.on_commit = None

    def _open_camera(self, dev: str | int):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap

    def _source(self, dev: str | int, side: str | None):
        """Frame source for one scan: the dashboard hub if it's serving frames
        (so the wrist cam can also be streamed live), else the device directly."""
        if CAM_HUB and side:
            hub = _HubSource(CAM_HUB, side)
            if hub.isOpened():
                print(f'  [CardDetector] {side}: reading frames from dashboard hub')
                return hub
            print(f'  [CardDetector] {side}: hub has no frames — opening device {dev} directly')
        return _LocalSource(self._open_camera(dev))

    def _infer_frame(self, frame, counts: dict) -> None:
        for r in self.model(frame, verbose=False):
            for box in r.boxes:
                if float(box.conf[0]) >= self.conf:
                    label = r.names[int(box.cls[0])]
                    counts[label] += 1

    def _commit(self, counts: dict) -> set[str]:
        newly: set[str] = set()
        for label, cnt in counts.items():
            self._counts[label] += cnt
            if label not in self.registry and self._counts[label] >= self.min_detections:
                self.registry.add(label)
                self.ordered.append(label)
                newly.add(label)
        if newly and self.on_commit:
            try:
                self.on_commit(self.registry, newly)
            except Exception:
                pass
        return newly

    def scan(self, devices: list[str | int], n_frames: int = 10,
             dashboard_side: str | None = None) -> set[str]:
        """Scan cameras for n_frames each, update registry, return newly confirmed cards."""
        counts: dict[str, int] = defaultdict(int)
        for dev in devices:
            cap = self._source(dev, dashboard_side)
            if not cap.isOpened():
                print(f'  [CardDetector] warning: could not open {dev}')
                continue
            for _ in range(n_frames):
                ret, frame = cap.read()
                if not ret:
                    break
                self._infer_frame(frame, counts)
            cap.release()
        return self._commit(counts)

    def scan_during(self, devices: list[str | int], proc, extra_frames: int = 30,
                    dashboard_side: str | None = None) -> set[str]:
        """
        Scan cameras continuously while subprocess proc is running, then
        capture extra_frames after it finishes (arm still close to cards).
        This is the right time to detect — the wrist camera is over the card
        during the deal/flip motion.
        """
        # Commit every few frames so cards surface (and reach the UI via
        # on_commit) the instant they're confirmed, not at the end of the scan.
        commit_every = 4
        counts: dict[str, int] = defaultdict(int)
        newly_all: set[str] = set()
        for dev in devices:
            cap = self._source(dev, dashboard_side)
            if not cap.isOpened():
                print(f'  [CardDetector] warning: could not open {dev}')
                continue
            proc_done = False
            extra = 0
            since_commit = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                self._infer_frame(frame, counts)
                since_commit += 1
                if since_commit >= commit_every:
                    newly_all |= self._commit(counts)
                    counts.clear()
                    since_commit = 0
                if not proc_done and proc.poll() is not None:
                    proc_done = True
                    time.sleep(1.5)  # let arm settle over card
                if proc_done:
                    extra += 1
                    if extra >= extra_frames:
                        break
            cap.release()
        newly_all |= self._commit(counts)   # flush any remainder
        return newly_all

    def reset(self) -> None:
        self._counts.clear()
        self.registry.clear()
        self.ordered.clear()
