"""Side grasp of an upright bottle on the table: approach horizontally from the arm base direction.

    python calib/side_grasp.py --x 540 --y -50             plan only (IK + path checks)
    python calib/side_grasp.py --x 540 --y -50 --go        grasp, lift LIFT_MM, put back, release, stow

The gripper's approach axis (tool z) points horizontally from the arm base toward the bottle;
the fingers close horizontally across the bottle at GRASP_H_MM above the table. Sequence:
open -> joint move to HIGH_MM above the pre-grasp (wrist reoriented up there) -> straight down
to the pre-grasp PRE_BACK_MM short of the grasp -> straight in -> close
-> lift -> hold -> lower -> open -> straight back -> up -> stow. Every joint move is checked by
forward kinematics: the flange and fingertips stay TABLE_CLEAR_MM above the table wherever
they are over it (x > TABLE_EDGE_X_MM).
"""
import argparse
import math
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from xarm.wrapper import XArmAPI

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from phone_xarm_teleop import GRIPPER_COM_MM, GRIPPER_LOAD_KG, SHOULDER_MM, STOW_JOINTS_DEG  # noqa: E402

IP = "192.168.1.204"
FINGERTIP_MM = 172.0     # flange to closed fingertips
GRASP_DEPTH_MM = 150.0   # flange to the middle of the finger pads
GRASP_H_MM = 90.0        # grasp height above the table
PRE_BACK_MM = 100.0
HIGH_MM = 150.0          # reorient the wrist this far above the pre-grasp, then descend
LIFT_MM = 30.0
TABLE_EDGE_X_MM = 330.0  # conservative: anything beyond this x may be over the table
TABLE_CLEAR_MM = 25.0
REACH_MAX_MM = 650.0
WRIST_MAX_DEG = 150.0    # J4/J6 cable limit margin (wound wrist caused C14/C16 on 2026-10-21)
LINEAR_SPEED_MM_S = 30.0
JOINT_SPEED_DEG_S = 15.0
GRIPPER_OPEN = 850
BOTTLE_KG = 0.5
SQUEEZE_MM = 10.0        # close this far below the object width instead of fully (0 crushed the bottle)
# Carry pose for driving: stow-like arm swung to the left, gripper horizontal pointing left, so the
# held bottle sits over the chassis (bottle centre ~0.29 m left of the chassis centre, ~0.84 m up).
CARRY_JOINTS_DEG = [90.0, -90.0, -5.0, 0.0, 5.0, 0.0]


def pose_rpy(rot):
    roll, pitch, yaw = Rot.from_matrix(rot).as_euler("xyz", degrees=True)  # R = Rz(yaw) Ry(pitch) Rx(roll)
    return [roll, pitch, yaw]


def tool_points(arm, q):
    """Flange and fingertip positions (mm) for joints q."""
    code, p = arm.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
    if code != 0:
        raise RuntimeError(f"FK failed: {code}")
    rot = Rot.from_euler("xyz", p[3:6], degrees=True).as_matrix()
    flange = np.array(p[:3])
    return flange, flange + FINGERTIP_MM * rot[:, 2]


def path_clear(arm, q0, q1, table_z, samples=30):
    worst = math.inf
    for k in range(samples + 1):
        q = [a + k / samples * (b - a) for a, b in zip(q0, q1)]
        for p in tool_points(arm, q):
            if p[0] > TABLE_EDGE_X_MM:
                worst = min(worst, p[2] - table_z)
    return worst


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True, help="bottle centre, arm base frame (mm)")
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--table-z", type=float, default=292.0)
    parser.add_argument("--width", type=float, default=65.0, help="object width across the fingers (mm)")
    parser.add_argument("--carry", type=float, default=0.0,
                        help="after lifting, hold the object in the carry pose this many seconds, then put it back")
    parser.add_argument("--keep", action="store_true",
                        help="after lifting, stay in the carry pose holding the object (for driving to a bin)")
    parser.add_argument("--go", action="store_true")
    args = parser.parse_args()

    a = np.array([args.x, args.y, 0.0])
    a /= np.linalg.norm(a)
    centre = np.array([args.x, args.y, args.table_z + GRASP_H_MM])
    grasp = centre - GRASP_DEPTH_MM * a
    pre = grasp - PRE_BACK_MM * a
    high = pre + [0, 0, HIGH_MM]
    lift = grasp + [0, 0, LIFT_MM]
    pre_lift = pre + [0, 0, LIFT_MM]
    close_to = int(max(0.0, args.width - SQUEEZE_MM) * 10)  # gripper units: 850 = ~85 mm open

    arm = XArmAPI(IP, is_radian=False)
    _, start = arm.get_servo_angle(is_radian=False)
    start = start[:6]
    if max(abs(a - b) for a, b in zip(start, STOW_JOINTS_DEG)) > 5.0:
        # IK seeds from the current joints; from other poses (e.g. the carry pose) it picked a
        # wound-wrist branch whose linear descent hit error 24 (2026-10-21)
        sys.exit(f"start from the stow pose (joints now {np.round(start, 1).tolist()})")
    best = None
    for tool_x in ([0, 0, -1], [0, 0, 1]):  # tool x down or up; fingers close horizontally either way
        x = np.array(tool_x, float)
        rot = np.column_stack([x, np.cross(a, x), a])
        rpy = pose_rpy(rot)
        code, q = arm.get_inverse_kinematics(list(high) + rpy, input_is_radian=False, return_is_radian=False)
        if code != 0:
            print(f"tool x {tool_x}: no IK ({code})")
            continue
        q = list(q[:6])
        if max(abs(q[3]), abs(q[5])) > WRIST_MAX_DEG:
            print(f"tool x {tool_x}: wrist J4/J6 {q[3]:.0f}/{q[5]:.0f} deg beyond {WRIST_MAX_DEG:.0f}; skipped")
            continue
        # two stages: up high with the gripper still pointing down, then turn it horizontal
        up = [high[0] - 40.0, high[1], high[2] + 30.0, 180.0, 0.0, 0.0]
        code, q_arm = arm.get_inverse_kinematics(up, input_is_radian=False, return_is_radian=False)
        if code != 0:
            print(f"tool x {tool_x}: no IK for the raised gripper-down pose ({code})")
            continue
        q_arm = list(q_arm[:6])
        clear = min(path_clear(arm, start, q_arm, args.table_z), path_clear(arm, q_arm, q, args.table_z))
        cost = max(abs(qi - si) for qi, si in zip(q[3:], start[3:]))
        print(f"tool x {tool_x}: high joints {np.round(q, 1).tolist()} (wrist change {cost:.0f} deg, "
              f"table clearance on the way {clear:.0f} mm)")
        if clear >= TABLE_CLEAR_MM and (best is None or cost < best[0]):
            best = (cost, rpy, q, q_arm)
    if best is None:
        sys.exit("no safe IK solution for the reorientation pose")
    _, rpy, q_high, q_arm = best
    for name, p in (("high", high), ("pre-grasp", pre), ("grasp", grasp), ("lift", lift)):
        reach = math.dist(p, SHOULDER_MM)
        print(f"{name:9s} flange ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f}) reach {reach:.0f} mm")
        if reach > REACH_MAX_MM:
            sys.exit(f"{name} beyond reach")
    clear_out = min(path_clear(arm, q_high, q_arm, args.table_z), path_clear(arm, q_arm, STOW_JOINTS_DEG, args.table_z))
    print(f"clearance over the table high->stow {clear_out:.0f} mm (need {TABLE_CLEAR_MM:.0f}); "
          f"tool axis {centre[2] - args.table_z:.0f} mm above the table during the grasp")
    if clear_out < TABLE_CLEAR_MM:
        sys.exit("joint path too close to the table")
    if args.carry or args.keep:
        # the held object hangs GRASP_H_MM below the tool axis
        clear_carry = path_clear(arm, q_high, CARRY_JOINTS_DEG, args.table_z + GRASP_H_MM)
        f, t = tool_points(arm, CARRY_JOINTS_DEG)
        print(f"carry: flange ({f[0]:.0f},{f[1]:.0f},{f[2]:.0f}); object bottom clearance over the table on the "
              f"way {clear_carry:.0f} mm")
        if clear_carry < TABLE_CLEAR_MM:
            sys.exit("carry swing too close to the table")
    print(f"gripper closes to {close_to} for a {args.width:.0f} mm object")
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
    arm.set_gripper_speed(2000)

    def step(name, code):
        print(f"-> {name}: code {code}", flush=True)
        if code != 0 or arm.error_code != 0:
            raise RuntimeError(f"{name} failed: code {code}, arm error {arm.error_code}")

    held = False
    try:
        step("open gripper", arm.set_gripper_position(GRIPPER_OPEN, wait=True))
        step("joint move up (gripper down)", arm.set_servo_angle(angle=q_arm, speed=JOINT_SPEED_DEG_S, mvacc=150,
                                                                    is_radian=False, wait=True))
        step("turn the wrist horizontal", arm.set_servo_angle(angle=q_high, speed=JOINT_SPEED_DEG_S, mvacc=150,
                                                                 is_radian=False, wait=True))
        step("descend to pre-grasp", arm.set_position(*pre, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                                      is_radian=False, wait=True))
        step("straight in", arm.set_position(*grasp, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                             is_radian=False, wait=True))
        step("close gripper", arm.set_gripper_position(close_to, wait=True))
        time.sleep(0.5)
        _, width = arm.get_gripper_position()
        print(f"   gripper position after closing: {width} (0 = fully closed, nothing held)", flush=True)
        if width is None or width < close_to - 30:
            print("   no bottle between the fingers; backing out", flush=True)
        else:
            held = True
            com_z = (GRIPPER_LOAD_KG * GRIPPER_COM_MM[2] + BOTTLE_KG * GRASP_DEPTH_MM) / (GRIPPER_LOAD_KG + BOTTLE_KG)
            arm.set_tcp_load(GRIPPER_LOAD_KG + BOTTLE_KG, [0, 0, com_z])
            step("lift", arm.set_position(*lift, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200, is_radian=False, wait=True))
            if args.carry or args.keep:
                step("back out holding", arm.set_position(*pre_lift, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                                          is_radian=False, wait=True))
                step("rise holding", arm.set_position(*high, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                                      is_radian=False, wait=True))
                step("swing to the carry pose", arm.set_servo_angle(angle=CARRY_JOINTS_DEG, speed=JOINT_SPEED_DEG_S,
                                                                    mvacc=150, is_radian=False, wait=True))
                if args.keep:
                    _, w = arm.get_gripper_position()
                    print(f"done: holding in the carry pose (gripper {w})", flush=True)
                    return
                time.sleep(args.carry)
                _, w = arm.get_gripper_position()
                print(f"   still holding after {args.carry:.0f} s: gripper {w}", flush=True)
                step("swing back", arm.set_servo_angle(angle=q_high, speed=JOINT_SPEED_DEG_S, mvacc=150,
                                                       is_radian=False, wait=True))
                step("descend holding", arm.set_position(*pre_lift, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                                         is_radian=False, wait=True))
                step("in again", arm.set_position(*lift, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                                  is_radian=False, wait=True))
            else:
                time.sleep(2.0)
            step("lower", arm.set_position(*grasp, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200, is_radian=False, wait=True))
        step("open gripper", arm.set_gripper_position(GRIPPER_OPEN, wait=True))
        arm.set_tcp_load(GRIPPER_LOAD_KG, GRIPPER_COM_MM)
        step("straight back", arm.set_position(*pre, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200,
                                               is_radian=False, wait=True))
        step("rise", arm.set_position(*high, *rpy, speed=LINEAR_SPEED_MM_S, mvacc=200, is_radian=False, wait=True))
        step("turn the wrist back down", arm.set_servo_angle(angle=q_arm, speed=JOINT_SPEED_DEG_S, mvacc=150,
                                                             is_radian=False, wait=True))
        step("stow", arm.set_servo_angle(angle=STOW_JOINTS_DEG, speed=JOINT_SPEED_DEG_S, mvacc=150,
                                         is_radian=False, wait=True))
        print("done: " + ("grasped, lifted and put back" if held else "missed"), flush=True)
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f"stopped: {exc}; arm halted where it is (use the stow button / xarm_recover.py)", flush=True)
    finally:
        arm.set_state(4)
        arm.disconnect()


if __name__ == "__main__":
    main()
