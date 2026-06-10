#!/usr/bin/env python3
"""System check for the bimanual blackjack rig.

Resolves the current /dev/videoX and /dev/ttyACMx numbers (which shuffle on every
reboot/replug) by *stable* hardware identity, and reports the mapping the
launch/record commands need.

Camera identity:
  - Wrist cams are two identical XWF-1080P units that share a USB serial, so the
    only stable discriminator is the USB bus path each cable is plugged into.
    They expose MJPG.
  - The overhead camera is an Intel RealSense; its color node is the YUYV-only
    capture node (the depth/IR node exposes GREY/UYVY/Y12I instead).

Arm identity:
  - The four SO101 motor controllers are indistinguishable except by which
    physical USB port each cable is in (sysfs port chain, e.g. '3-2').

Run:  python3 scripts/system_check.py
Exit: 0 = all expected hardware found, 1 = something missing/moved.
"""

from __future__ import annotations

import glob
import os
import re
import subprocess
import sys

# Wrist cams keyed by USB bus path (stable while the cable stays in that port).
# Update these if you re-cable to different physical USB ports.
WRIST_BUS = {
    "usb-0000:0d:00.0-1": "right_wrist",   # /dev/video2
    "usb-0000:0c:00.4-1": "left_wrist",    # /dev/video0
}
REALSENSE_CARD = "RealSense"

# Motor serial ports keyed by USB port chain (stable while the cable stays in
# that physical port). Update these if you re-cable to different USB ports.
ARM_USB_PORT = {
    "1-1.1.1": "left_leader",
    "1-1.1.2": "left_follower",
    "1-1.1.3": "right_leader",
    "1-1.1.4": "right_follower",
}

# Game-input keyboards, identified by stable /dev/input/by-id name substrings.
# The game loop reads stdin (any keyboard works), but both must be plugged in:
#   full keyboard -> ` (backtick) = HIT / DEAL
#   numpad        -> 1 = STAND / END
KEYBOARD_ID = {
    "Chicony_HP_Elite_USB_Keyboard": "main_keyboard (backtick = HIT/DEAL)",
    "magic_force": "numpad (1 = STAND/END)",
}

GREEN, RED, YELLOW, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[0m"


def _v4l2(dev: str, *args: str) -> str:
    try:
        return subprocess.run(["v4l2-ctl", "-d", dev, *args],
                              capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return ""


def _video_nodes() -> list[str]:
    return sorted(glob.glob("/dev/video*"),
                  key=lambda x: int(re.sub(r"\D", "", x) or 0))


def _probe(dev: str) -> dict | None:
    fmts = _v4l2(dev, "--list-formats")
    if "Video Capture" not in fmts:
        return None
    pix = re.findall(r"\]: '(\w+)'", fmts)
    if not pix:                      # metadata-only node
        return None
    info = _v4l2(dev, "--info")
    card = re.search(r"Card type\s*:\s*(.+)", info)
    bus = re.search(r"Bus info\s*:\s*(.+)", info)
    return {
        "dev": dev,
        "card": card.group(1).strip() if card else "?",
        "bus": bus.group(1).strip() if bus else "?",
        "fmts": pix,
    }


def check_cameras() -> tuple[dict[str, str], bool]:
    caps = [c for c in (_probe(d) for d in _video_nodes()) if c]
    resolved: dict[str, str] = {}

    # Wrist cams: first MJPG-capable capture node on each known bus.
    for bus, role in WRIST_BUS.items():
        m = next((c for c in caps if c["bus"] == bus and "MJPG" in c["fmts"]),
                 None)
        if m:
            resolved[role] = m["dev"]

    # Overhead = RealSense color node: YUYV present, MJPG absent (wrist cams have
    # MJPG), and not the depth/IR node (those expose GREY/UYVY/Y12I, not YUYV).
    rs = next((c for c in caps
               if REALSENSE_CARD in c["card"]
               and "YUYV" in c["fmts"] and "MJPG" not in c["fmts"]), None)
    if rs:
        resolved["overhead"] = rs["dev"]

    print(f"\n{'='*52}\n CAMERAS\n{'='*52}")
    ok = True
    for role in ("right_wrist", "left_wrist", "overhead"):
        dev = resolved.get(role)
        if dev:
            print(f"  {GREEN}[ OK ]{RESET} {role:12} -> {dev}")
        else:
            print(f"  {RED}[FAIL]{RESET} {role:12} -> NOT FOUND")
            ok = False
    return resolved, ok


def _acm_port(dev: str) -> str:
    """Return the stable USB port chain (e.g. '3-2') for a ttyACM device.

    Reads the sysfs symlink (no udevadm dependency) and pulls the USB port
    token, e.g. .../usb3/3-2/3-2:1.0/tty/ttyACM0 -> '3-2'.
    """
    name = os.path.basename(dev)
    path = os.path.realpath(f"/sys/class/tty/{name}")
    m = re.findall(r"/(\d+-[\d.]+)(?=/)", path)
    return m[-1] if m else "?"


def check_arms() -> tuple[dict[str, str], bool]:
    by_port = {_acm_port(d): d for d in sorted(glob.glob("/dev/ttyACM*"))}
    resolved = {role: by_port[port]
                for port, role in ARM_USB_PORT.items() if port in by_port}

    print(f"\n{'='*52}\n ARMS\n{'='*52}")
    ok = True
    for role in ("left_leader", "left_follower",
                 "right_leader", "right_follower"):
        dev = resolved.get(role)
        if dev:
            print(f"  {GREEN}[ OK ]{RESET} {role:14} -> {dev}")
        else:
            print(f"  {RED}[FAIL]{RESET} {role:14} -> NOT FOUND")
            ok = False
    return resolved, ok


def check_keyboards() -> tuple[dict[str, str], bool]:
    by_id = glob.glob("/dev/input/by-id/*-event-kbd")
    resolved: dict[str, str] = {}
    for substr, label in KEYBOARD_ID.items():
        m = next((p for p in by_id if substr in os.path.basename(p)), None)
        if m:
            resolved[label] = os.path.realpath(m)

    print(f"\n{'='*52}\n KEYBOARDS\n{'='*52}")
    ok = True
    for label in KEYBOARD_ID.values():
        dev = resolved.get(label)
        if dev:
            print(f"  {GREEN}[ OK ]{RESET} {label}")
        else:
            print(f"  {RED}[FAIL]{RESET} {label} -> NOT FOUND")
            ok = False
    return resolved, ok


def main() -> int:
    print("Bimanual blackjack rig - system check")
    cams, cams_ok = check_cameras()
    if cams_ok:
        print("\n  Camera args:")
        print(f"    wrist_right  {cams['right_wrist']}")
        print(f"    wrist_left   {cams['left_wrist']}")
        print(f"    overhead     {cams['overhead']}")

    arms, arms_ok = check_arms()
    if arms_ok:
        print("\n  Arm ports:")
        for role in ("left_leader", "left_follower",
                     "right_leader", "right_follower"):
            print(f"    {role:14} {arms[role]}")

    _kbds, kbds_ok = check_keyboards()

    print(f"\n{'='*52}")
    if cams_ok and arms_ok and kbds_ok:
        print(f" {GREEN}ALL OK{RESET}")
        return 0
    print(f" {RED}CHECK FAILED{RESET} - see above")
    return 1


if __name__ == "__main__":
    sys.exit(main())
