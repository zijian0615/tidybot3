"""Hover the closed gripper a set gap above the table to check the table height in the arm frame.

    python calib/table_hover.py --x 520 --y -160 --table-z 300 --gap 20         plan only
    python calib/table_hover.py --x 520 --y -160 --table-z 300 --gap 20 --go    move; Enter returns to stow

table-z is the table top in the arm base frame (table height above the floor minus the 430 mm
arm base height). The fingertips stop GAP mm above the predicted table; measure the real gap
with a ruler. A background run (no tty) holds until calib/.hover_release exists.
table_z_true = table_z + gap - measured_gap. Path: lift at the stow x, move over
at a safe height, descend vertically; return in reverse.
"""
import argparse
import math
import os
import sys
import time

from xarm.wrapper import XArmAPI

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from phone_xarm_teleop import GRIPPER_COM_MM, GRIPPER_LOAD_KG, SHOULDER_MM, STOW_JOINTS_DEG  # noqa: E402

IP = "192.168.1.204"
GRIPPER_LEN_MM = 172.0  # flange to closed fingertips
CLEAR_MM = 80.0         # travel height above the hover point
SPEED_MM_S = 40.0
REACH_MAX_MM = 650.0
RELEASE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".hover_release")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--table-z", type=float, default=292.0)
    parser.add_argument("--gap", type=float, default=20.0)
    parser.add_argument("--go", action="store_true")
    args = parser.parse_args()
    hover_z = args.table_z + GRIPPER_LEN_MM + args.gap
    travel_z = hover_z + CLEAR_MM
    arm = XArmAPI(IP, is_radian=False)
    _, start = arm.get_position(is_radian=False)
    down = [180.0, 0.0, 0.0]
    path = [("lift", [start[0], start[1], travel_z]), ("over", [args.x, args.y, travel_z]),
            ("descend", [args.x, args.y, hover_z])]
    for name, p in path:
        reach = math.dist(p, SHOULDER_MM)
        print(f"{name:8s} flange ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f}) reach {reach:.0f} mm")
        if reach > REACH_MAX_MM:
            sys.exit(f"{name}: beyond reach limit {REACH_MAX_MM:.0f} mm")
    print(f"fingertips will stop {args.gap:.0f} mm above the predicted table (flange z {hover_z:.0f})")
    if not args.go:
        print("plan only; add --go to move")
        arm.disconnect()
        return
    arm.clean_error()
    arm.clean_warn()
    for name, code in (("motion_enable", arm.motion_enable(True)), ("set_mode(0)", arm.set_mode(0)),
                       ("set_state(0)", arm.set_state(0))):
        if code != 0:
            sys.exit(f"{name} failed: {code}")
    time.sleep(0.5)
    arm.set_tcp_load(GRIPPER_LOAD_KG, GRIPPER_COM_MM)
    arm.set_gripper_enable(True)
    arm.set_gripper_mode(0)
    arm.set_gripper_position(0, wait=True)
    try:
        for name, p in path:
            print(f"-> {name}", flush=True)
            code = arm.set_position(*p, *down, speed=SPEED_MM_S, mvacc=200, is_radian=False, wait=True)
            if code != 0 or arm.error_code != 0:
                sys.exit(f"stopped at {name}: code {code}, arm error {arm.error_code}")
        _, pos = arm.get_position(is_radian=False)
        print(f"hovering: flange ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}); measure the fingertip-to-table gap", flush=True)
        if sys.stdin.isatty():
            input("press Enter to return to stow ")
        else:  # background run: hold until the release file appears (or HOVER_HOLD_S passes)
            end = time.monotonic() + float(os.environ.get("HOVER_HOLD_S", "900"))
            while time.monotonic() < end and not os.path.exists(RELEASE_FILE):
                time.sleep(0.5)
            if os.path.exists(RELEASE_FILE):
                os.remove(RELEASE_FILE)
        for name, p in reversed(path[:2]):
            arm.set_position(*p, *down, speed=SPEED_MM_S, mvacc=200, is_radian=False, wait=True)
        arm.set_servo_angle(angle=STOW_JOINTS_DEG, speed=15, mvacc=150, is_radian=False, wait=True)
        print("back at stow", flush=True)
    finally:
        arm.set_state(4)
        arm.disconnect()


if __name__ == "__main__":
    main()
