"""HEBI Mobile I/O input adapter for the combined base/xArm controller."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time


VENDOR_DIR = Path(__file__).resolve().parent / ".vendor" / "hebi_py"
LAYOUT_FILE = Path(__file__).resolve().parent / "mobile_io_layout.json"
INPUT_DEADZONE = 0.08


def _load_hebi():
    if str(VENDOR_DIR) not in sys.path:
        sys.path.insert(0, str(VENDOR_DIR))
    import hebi
    return hebi


def _deadzone(value):
    value = float(value)
    if not math.isfinite(value) or abs(value) <= INPUT_DEADZONE:
        return 0.0
    return math.copysign((abs(value) - INPUT_DEADZONE) / (1.0 - INPUT_DEADZONE), value)


def _stop(base_state, arm_state, seq):
    base_state.update({"seq": seq, "enabled": False, "x": 0, "y": 0, "rotation": 0})
    arm_state.update_tag_pose({
        "seq": seq, "enabled": False,
        "position": {"x": 0, "y": 0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    })


def mobile_io_loop(base_state, arm_state, family="HEBI", name="mobileIO"):
    hebi = _load_hebi()
    lookup = hebi.Lookup()
    mobile = None
    seq = 2_000_000_000
    print(f"Mobile I/O: searching for family={family!r}, name={name!r}", flush=True)
    while not base_state.stop_requested and mobile is None:
        mobile = hebi.util.create_mobile_io(lookup, family=family, name=name)
        if mobile is None:
            time.sleep(1.0)
    if mobile is None:
        return
    print("Mobile I/O: connected", flush=True)
    if not mobile.send_layout(layout_file=str(LAYOUT_FILE), timeout_ms=2000):
        print("Mobile I/O: layout acknowledgement timed out; continuing", flush=True)
    mobile.clear_text()
    mobile.add_text("Joystick: base\nHold ARM: ARKit xArm\nSTOP: stop all")
    mobile.set_led_color("green")
    led_color = "green"
    last_feedback = time.monotonic()
    try:
        while not base_state.stop_requested:
            if not mobile.update(timeout_ms=150):
                if time.monotonic() - last_feedback > 0.4:
                    seq += 1
                    _stop(base_state, arm_state, seq)
                continue
            last_feedback = time.monotonic()
            seq += 1
            stop = mobile.get_button_state(8)
            x = _deadzone(mobile.get_axis_state(1))
            y = _deadzone(mobile.get_axis_state(2))
            rotation = float(mobile.get_button_state(5)) - float(mobile.get_button_state(7))
            base_enabled = not stop and (x != 0.0 or y != 0.0 or rotation != 0.0)
            base_state.update({
                "seq": seq, "enabled": base_enabled,
                "x": x, "y": y, "rotation": rotation,
            })

            try:
                position = tuple(float(v) for v in mobile.position)
                orientation = tuple(float(v) for v in mobile.orientation)  # w, x, y, z
            except (TypeError, ValueError):
                position, orientation = (), ()
            pose_valid = (
                len(position) == 3 and len(orientation) == 4
                and all(math.isfinite(v) for v in position + orientation)
                and sum(v * v for v in orientation) > 0.5
            )
            arm_enabled = not stop and mobile.get_button_state(3) and pose_valid
            # ARKit/WebXR coordinates -> robot base-relative control axes.
            px, py, pz = position
            qw, qx, qy, qz = orientation
            arm_state.update_tag_pose({
                "seq": seq, "enabled": arm_enabled,
                "position": {"x": -pz, "y": -px, "z": py},
                "orientation": {"x": -qz, "y": -qx, "z": qy, "w": qw},
                "tracking": {"inliers": "ARKit", "points": "MobileIO"},
            })
            next_color = "red" if stop else "blue" if arm_enabled else "green"
            if next_color != led_color:
                mobile.set_led_color(next_color, blocking=False)
                led_color = next_color
    finally:
        seq += 1
        _stop(base_state, arm_state, seq)
        mobile.set_led_color("transparent", blocking=False)
