"""
Tiny best-effort client for pushing game state / camera frames to the
dashboard server. Every call swallows errors so the game never breaks when
the dashboard isn't running.

Set DASHBOARD_URL to point elsewhere (default http://127.0.0.1:8000).
Set DASHBOARD_OFF=1 to disable all pushes.
"""

import json
import os
import queue
import threading
import urllib.request

URL = os.environ.get('DASHBOARD_URL', 'http://127.0.0.1:8000').rstrip('/')
OFF = os.environ.get('DASHBOARD_OFF', '') not in ('', '0', 'false', 'False')

# Single ordered worker queue: posts are sent in submission order so a later
# state update can't be overtaken by an earlier one (e.g. the final result being
# clobbered by a just-prior result=None push). Still non-blocking for the game.
_q: 'queue.Queue' = queue.Queue()


def _sender():
    while True:
        path, data, content_type = _q.get()
        try:
            req = urllib.request.Request(
                URL + path, data=data, method='POST',
                headers={'Content-Type': content_type})
            urllib.request.urlopen(req, timeout=1.0).read()
        except Exception:
            pass  # dashboard down / slow — ignore, never block the game
        finally:
            _q.task_done()


threading.Thread(target=_sender, daemon=True).start()


def _post(path: str, data: bytes, content_type: str):
    if OFF:
        return
    _q.put((path, data, content_type))  # enqueue; the single sender preserves order


def push_state(**fields):
    """Merge-update the dashboard game state with the given fields."""
    try:
        body = json.dumps(fields).encode()
    except Exception:
        return
    _post('/state', body, 'application/json')


def push_snapshot(side: str, jpeg_bytes: bytes):
    """Show a wrist-cam frame for 'player' or 'dealer'."""
    if jpeg_bytes:
        _post('/snap/' + side, jpeg_bytes, 'image/jpeg')
