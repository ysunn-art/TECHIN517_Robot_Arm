#!/usr/bin/env python3
"""
Casino dashboard server for the blackjack game.

Decoupled from the game loop: it holds the latest game state and owns the
table cameras, serving a live web UI. The game loop pushes state to it over
HTTP (fire-and-forget), so if this server is down the game is unaffected.

Run (any time, before or after the game loop):
    python3 ~/techin517_final/scripts/dashboard_server.py
Then open  http://<this-host>:8000  in a browser.

Cameras — this server owns only the two wrist cams and streams them live.
The overhead RealSense (/dev/video8) is left free for the lerobot policies:
    player   -> PLAYER_CAM   (default /dev/video0, right wrist, MJPG)
    dealer   -> DEALER_CAM   (default /dev/video2, left wrist,  MJPG)

The card detector reads frames FROM this server (GET /frame/<name>) instead of
opening the wrist devices itself, so there's no V4L2 contention. If this server
isn't running, the detector falls back to opening the device directly.

Env overrides:
    DASHBOARD_PORT  (default 8000)
    OVERHEAD_CAM / PLAYER_CAM / DEALER_CAM
"""

import json
import os
import threading
import time
from pathlib import Path
from queue import Queue, Empty

import cv2
from flask import Flask, Response, request, jsonify

HERE = Path(__file__).parent
UI_FILE = HERE / 'dashboard_ui.html'

PORT = int(os.environ.get('DASHBOARD_PORT', '8000'))

# Overhead RealSense (/dev/video8) is intentionally NOT owned here — it's left
# free for the lerobot pay-out/collect policies. The dashboard only owns the two
# wrist cams used for card vision during dealing.
CAM_DEVICES = {
    'player':   (os.environ.get('PLAYER_CAM',   '/dev/video2'), 'MJPG'),   # right wrist
    'dealer':   (os.environ.get('DEALER_CAM',   '/dev/video0'), 'MJPG'),   # left wrist
}

app = Flask(__name__)

# When set, all capture threads release their V4L2 devices and stop reopening,
# so another process (e.g. a lerobot policy) can own the cameras. Cleared to resume.
_PAUSED = threading.Event()


# ----------------------------------------------------------------------------
# Cameras — each owned by a background capture thread
# ----------------------------------------------------------------------------
class Camera:
    # If a camera keeps returning the *same* frame for this many seconds it has
    # hung (USB/MJPG stall) — reopen the device to recover.
    STALE_TIMEOUT = 4.0

    def __init__(self, name, dev, fourcc=None, w=640, h=480, fps=30):
        self.name = name
        self.dev = dev
        self.fourcc = fourcc
        self.w, self.h, self.fps = w, h, fps
        self.jpeg = None          # latest encoded frame
        self.frame_id = 0         # increments on every *new* frame
        self.lock = threading.Lock()
        self.released = threading.Event()   # set while the V4L2 device is NOT held
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # serve the freshest frame, not a backlog
        return cap

    def _loop(self):
        while True:
            if _PAUSED.is_set():         # released for an external owner (e.g. lerobot)
                self.released.set()
                time.sleep(0.2)
                continue
            cap = self._open()
            if not cap.isOpened():
                print(f'[cam {self.name}] cannot open {self.dev} — retrying')
                time.sleep(2.0)          # device busy/absent — retry
                continue
            self.released.clear()        # we now hold the device
            fail = 0
            prev = None
            last_change = time.time()
            while True:
                if _PAUSED.is_set():     # release the device and wait to be resumed
                    break
                ok, frame = cap.read()
                now = time.time()
                if not ok:
                    fail += 1
                    if fail > 30:
                        print(f'[cam {self.name}] read failing — reopening')
                        break
                    time.sleep(0.03)
                    continue
                fail = 0
                ok, buf = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok:
                    continue
                data = buf.tobytes()
                if data != prev:                 # a genuinely new frame
                    prev = data
                    last_change = now
                    with self.lock:
                        self.jpeg = data
                        self.frame_id += 1
                elif now - last_change > self.STALE_TIMEOUT:
                    print(f'[cam {self.name}] frozen on one frame — reopening')
                    break                        # camera hung: force a reopen
                time.sleep(1 / self.fps)
            cap.release()
            self.released.set()          # device handed back
            time.sleep(0.5)

    def get(self):
        with self.lock:
            return self.jpeg, self.frame_id


CAMERAS = {name: Camera(name, dev, fourcc)
           for name, (dev, fourcc) in CAM_DEVICES.items()}


@app.route('/stream/<name>')
def stream(name):
    cam = CAMERAS.get(name)
    if cam is None:
        return ('unknown camera', 404)

    def gen():
        boundary = b'--frame'
        last = -1
        while True:
            buf, fid = cam.get()
            if buf is None or fid == last:
                time.sleep(0.02)
                continue
            last = fid
            yield (boundary + b'\r\nContent-Type: image/jpeg\r\n\r\n'
                   + buf + b'\r\n')
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/frame/<name>')
def frame(name):
    """Single latest JPEG (used by the card detector). 404 until a frame exists."""
    cam = CAMERAS.get(name)
    if cam is None:
        return ('unknown camera', 404)
    buf, fid = cam.get()
    if buf is None:
        return ('', 404)
    return Response(buf, mimetype='image/jpeg',
                    headers={'Cache-Control': 'no-store',
                             'X-Frame-Id': str(fid)})


@app.route('/cameras/<action>', methods=['POST'])
def cameras(action):
    """Release the V4L2 devices so another process can use them, or resume.
    POST /cameras/release  → capture threads drop the cameras (returns once freed)
    POST /cameras/resume   → capture threads reopen the cameras"""
    if action == 'release':
        _PAUSED.set()
        # Give the capture threads up to ~3s to release their devices, checking
        # twice (the 1.5s gaps double as kernel/RealSense settle time). Returns
        # early as soon as every device is free.
        for _ in range(2):
            if all(c.released.is_set() for c in CAMERAS.values()):
                break
            time.sleep(1.5)
        freed = all(c.released.is_set() for c in CAMERAS.values())
        return (jsonify({'released': freed}), 200)
    if action == 'resume':
        _PAUSED.clear()
        return ('', 204)
    return ('unknown action', 404)


# ----------------------------------------------------------------------------
# Shared game state + Server-Sent Events
# ----------------------------------------------------------------------------
_state_lock = threading.Lock()
_state = {
    'hand_num': None,
    'phase': 'idle',
    'status': 'Waiting for game to start…',
    'player': {'cards': [], 'total': 0},
    'dealer': {'cards': [], 'total': 0, 'hidden': 0},
    'result': None,
}
_subscribers: list[Queue] = []
_sub_lock = threading.Lock()


def _broadcast():
    with _state_lock:
        payload = json.dumps(_state)
    with _sub_lock:
        for q in list(_subscribers):
            try:
                q.put_nowait(payload)
            except Exception:
                pass


@app.route('/state', methods=['GET', 'POST'])
def state():
    if request.method == 'POST':
        upd = request.get_json(force=True, silent=True) or {}
        with _state_lock:
            _state.update(upd)
        _broadcast()
        return ('', 204)
    with _state_lock:
        return jsonify(_state)


@app.route('/events')
def events():
    q: Queue = Queue(maxsize=8)
    with _sub_lock:
        _subscribers.append(q)
    with _state_lock:
        first = json.dumps(_state)

    def gen():
        yield f'data: {first}\n\n'
        try:
            while True:
                try:
                    payload = q.get(timeout=15)
                    yield f'data: {payload}\n\n'
                except Empty:
                    yield ': keep-alive\n\n'
        finally:
            with _sub_lock:
                if q in _subscribers:
                    _subscribers.remove(q)
    return Response(gen(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/')
def index():
    return Response(UI_FILE.read_text(), mimetype='text/html')


if __name__ == '__main__':
    devs = '  '.join(f'{n}={d}' for n, (d, _) in CAM_DEVICES.items())
    print(f'[dashboard] http://0.0.0.0:{PORT}   {devs}')
    app.run(host='0.0.0.0', port=PORT, threaded=True)
