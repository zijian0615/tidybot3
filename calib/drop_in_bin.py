"""Drop the held object into a bin docked in front of the robot (after nav2/tools/bin_dock.py).

    python calib/drop_in_bin.py                       plan only, bin from ~/rtabmap_ws/data/bins/docked.json
    python calib/drop_in_bin.py --go                  carry pose -> over the bin opening -> open -> stow
    python calib/drop_in_bin.py --go --from-stow      dry rehearsal without an object (starts at stow)

The bin opening centre is taken BIN_DEPTH_MM / 2 behind the detected bin face. The gripper
reaches forward horizontally so the object's bottom passes RIM_CLEAR_MM above the rim
(RIM_Z_MM in the arm base frame: 720 mm bin height - 430 mm arm base height).
"""
import argparse
import json
import math
import os
import sys
import time
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from side_grasp import (CARRY_JOINTS_DEG, GRASP_DEPTH_MM, GRASP_H_MM, GRIPPER_OPEN, IP, JOINT_SPEED_DEG_S,  # noqa: E402
                        pose_rpy, tool_points)
from phone_xarm_teleop import GRIPPER_COM_MM, GRIPPER_LOAD_KG, SHOULDER_MM, STOW_JOINTS_DEG  # noqa: E402

warnings.filterwarnings("ignore", message="Gimbal lock")
DOCKED = os.path.expanduser("~/rtabmap_ws/data/bins/docked.json")
ARM_IN_BASE_LINK_MM = (130.0, -10.0)
BIN_DEPTH_MM = 340.0     # bin front face to the wall behind it was ~0.38 m in the depth image
RIM_Z_MM = 290.0
RIM_CLEAR_MM = 60.0
REACH_MAX_MM = 620.0
CHASSIS_CLEAR_Z_MM = 60.0  # tool points stay above this anywhere over the chassis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--go", action="store_true")
    parser.add_argument("--from-stow", action="store_true", help="rehearsal without an object")
    parser.add_argument("--centre", type=float, nargs=2, metavar=("X", "Y"),
                        help="bin opening centre in the arm base frame (mm), e.g. from the rear camera; "
                             "overrides docked.json (whose face centre is biased once the bin overflows the front view)")
    args = parser.parse_args()
    det = json.load(open(DOCKED))
    face_x = det["x"] * 1000 - ARM_IN_BASE_LINK_MM[0]
    centre = np.array([face_x + BIN_DEPTH_MM / 2, det["y"] * 1000 - ARM_IN_BASE_LINK_MM[1]])
    if args.centre:
        centre = np.array(args.centre)
    axis_z = RIM_Z_MM + RIM_CLEAR_MM + GRASP_H_MM
    a = np.array([centre[0], centre[1], 0.0]) / np.linalg.norm(centre)
    flange = np.array([centre[0], centre[1], axis_z]) - GRASP_DEPTH_MM * a
    x = np.array([0, 0, 1.0])
    rpy = pose_rpy(np.column_stack([x, np.cross(a, x), a]))
    reach = math.dist(flange, SHOULDER_MM)
    print(f"{det['kind']} bin: face {face_x:.0f} mm ahead of the arm base, opening centre ({centre[0]:.0f},{centre[1]:.0f}); "
          f"drop flange ({flange[0]:.0f},{flange[1]:.0f},{flange[2]:.0f}), reach {reach:.0f} mm")
    if reach > REACH_MAX_MM:
        sys.exit("bin too far for the arm; dock closer")

    from xarm.wrapper import XArmAPI
    arm = XArmAPI(IP, is_radian=False)
    code, q_drop = arm.get_inverse_kinematics(list(flange) + rpy, input_is_radian=False, return_is_radian=False)
    if code != 0:
        sys.exit(f"no IK for the drop pose ({code})")
    q_drop = list(q_drop[:6])
    if max(abs(q_drop[3]), abs(q_drop[5])) > 150:
        # IK seeds from the current joints and can return the flipped wrist (J4, J6 near +-180);
        # (J4 - 180, -J5, J6 - 180) reaches the same pose with the wrist unwound
        flip = q_drop[:3] + [q_drop[3] - math.copysign(180.0, q_drop[3]), -q_drop[4],
                             q_drop[5] - math.copysign(180.0, q_drop[5])]
        _, p0 = arm.get_forward_kinematics(q_drop, input_is_radian=False, return_is_radian=False)
        _, p1 = arm.get_forward_kinematics(flip, input_is_radian=False, return_is_radian=False)
        if np.allclose(p0[:3], p1[:3], atol=1.0):
            print(f"unwound the IK wrist flip: {np.round(q_drop, 1).tolist()} -> {np.round(flip, 1).tolist()}")
            q_drop = flip
    if max(abs(q_drop[3]), abs(q_drop[5])) > 150:
        sys.exit(f"drop pose winds the wrist: {np.round(q_drop, 1).tolist()}")
    start = STOW_JOINTS_DEG if args.from_stow else CARRY_JOINTS_DEG

    def worst(q0, q1, samples=30):
        """Lowest object bottom over the bin front and lowest tool point anywhere (mm)."""
        over_bin, anywhere = math.inf, math.inf
        for k in range(samples + 1):
            q = [p + k / samples * (r - p) for p, r in zip(q0, q1)]
            f, t = tool_points(arm, q)
            for p in (f, t):
                anywhere = min(anywhere, p[2])
                if p[0] > face_x - 50:
                    over_bin = min(over_bin, p[2] - GRASP_H_MM - RIM_Z_MM)
        return over_bin, anywhere

    legs = ([("stow -> carry", STOW_JOINTS_DEG, CARRY_JOINTS_DEG)] if args.from_stow else []) + \
        [("carry -> drop", CARRY_JOINTS_DEG, q_drop), ("drop -> carry", q_drop, CARRY_JOINTS_DEG),
         ("carry -> stow", CARRY_JOINTS_DEG, STOW_JOINTS_DEG)]
    ok = True
    for name, q0, q1 in legs:
        rim, low = worst(q0, q1)
        good = rim >= 20 and low >= CHASSIS_CLEAR_Z_MM
        ok &= good
        print(f"{name:14s} object bottom over the rim >= {rim:.0f} mm, lowest tool point z {low:.0f} mm "
              f"{'OK' if good else 'TOO LOW'}")
    print(f"drop joints {np.round(q_drop, 1).tolist()}")
    if not ok:
        sys.exit("path check failed")
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

    def step(name, code):
        print(f"-> {name}: code {code}", flush=True)
        if code != 0 or arm.error_code != 0:
            raise RuntimeError(f"{name} failed: code {code}, arm error {arm.error_code}")

    try:
        for name, q0, q1 in legs:
            step(name, arm.set_servo_angle(angle=q1, speed=JOINT_SPEED_DEG_S, mvacc=150, is_radian=False, wait=True))
            if name == "carry -> drop":
                step("open gripper (drop)", arm.set_gripper_position(GRIPPER_OPEN, wait=True))
                time.sleep(1.0)
        print("done", flush=True)
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f"stopped: {exc}", flush=True)
    finally:
        arm.set_state(4)
        arm.disconnect()


if __name__ == "__main__":
    main()
