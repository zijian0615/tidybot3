"""Bring the xArm6 back from an awkward pose to the stow pose in safe joint steps.

    python xarm_recover.py            plan only: forward-kinematics check of every step, no motion
    python xarm_recover.py --go       clear errors, set the gripper payload, execute step by step

Order: lift first (shoulder J2, then elbow J3), then swing J1 to the front, then the wrist,
then fold into the stow pose. A joint move that starts low beside the chassis must never
swing J1 first: the arm would sweep across the chassis and camera.
Run it while phone_base_teleop.py is not controlling the arm (its arm loop has stopped).
"""
import argparse
import sys
import time

import numpy as np
from xarm.wrapper import XArmAPI

from phone_xarm_teleop import (GRIPPER_COM_MM, GRIPPER_LOAD_KG, MIN_FLANGE_Z_MM, plan_lowest_flange,
                               recovery_plan)

IP = "192.168.1.204"
SPEED_DEG_S = 10.0
ACC_DEG_S2 = 100.0
GRIPPER_LEN_MM = 172.0


def check(arm, start, plan):
    lowest = plan_lowest_flange(arm, start, plan)
    print(f"start joints {np.round(start, 1).tolist()}")
    for name, target in plan:
        code, end = arm.get_forward_kinematics(target, input_is_radian=False, return_is_radian=False)
        print(f"{name}: joints {np.round(target, 1).tolist()} -> flange ({end[0]:.0f},{end[1]:.0f},{end[2]:.0f}) mm")
    print(f"lowest flange z on the whole path: {lowest:.0f} mm "
          f"({'OK' if lowest >= MIN_FLANGE_Z_MM else 'TOO LOW'})")
    return lowest >= MIN_FLANGE_Z_MM


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--go", action="store_true", help="execute (default: plan only)")
    args = parser.parse_args()
    arm = XArmAPI(IP, is_radian=False)
    print(f"state {arm.state}, error {arm.error_code}, warn {arm.warn_code}")
    if args.go:
        arm.clean_error(); arm.clean_warn(); time.sleep(0.5)
    code, start = arm.get_servo_angle(is_radian=False)
    if code != 0:
        sys.exit(f"cannot read joints: {code}")
    start = start[:6]
    plan = recovery_plan(start)
    ok = check(arm, start, plan)
    if not args.go:
        print("plan only; run with --go to move" if ok else "plan has a step below the floor limit")
        arm.disconnect()
        return
    if not ok:
        sys.exit("refusing to move: a step goes below the floor limit")
    for name, code in (("motion_enable", arm.motion_enable(True)), ("set_mode(0)", arm.set_mode(0)),
                       ("set_state(0)", arm.set_state(0))):
        if code != 0:
            sys.exit(f"{name} failed: {code}")
    time.sleep(0.5)
    # the controller rejects set_tcp_load (code 9) until the arm is enabled
    print(f"set gripper payload {GRIPPER_LOAD_KG} kg ->", arm.set_tcp_load(GRIPPER_LOAD_KG, GRIPPER_COM_MM))
    for name, target in plan:
        print(f"-> {name}", flush=True)
        code = arm.set_servo_angle(angle=target, speed=SPEED_DEG_S, mvacc=ACC_DEG_S2, is_radian=False, wait=True)
        if code != 0 or arm.error_code != 0:
            print(f"   stopped: code {code}, arm error {arm.error_code}; joints now "
                  f"{np.round(arm.angles[:6], 1).tolist()}", flush=True)
            arm.set_state(4)
            arm.disconnect()
            sys.exit(1)
        print(f"   ok, flange {np.round(arm.position[:3], 0).tolist()}", flush=True)
    print("stowed", flush=True)
    arm.set_state(4)
    arm.disconnect()


if __name__ == "__main__":
    main()
