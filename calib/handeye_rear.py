"""Eye-to-hand calibration of the rear-right camera against the xArm6 base.

AprilTag tag36h11 ID1 (25 mm) is taped to the back of the gripper. The arm visits
joint poses around the stow pose; at each one the tag pose in the camera
(solvePnP on corners averaged over several frames) is paired with the flange pose
from the controller. cv2.calibrateHandEye then gives the camera pose in the arm
base frame.

    python calib/handeye_rear.py --poses probe            plan only: FK check, no motion
    python calib/handeye_rear.py --poses probe --go       move, detect, save samples
    python calib/handeye_rear.py --poses sample --go      ~20 varied poses for the solve
    python calib/handeye_rear.py --solve                  solve from the saved samples

Every target is checked by forward kinematics (workspace box, reach, front keep-out,
lowest flange height along the joint-space path) before the arm moves. The arm moves
in joint mode at HANDEYE_SPEED_DEG_S and returns to the stow pose at the end.
phone_base_teleop.py must not be controlling the arm while this runs.
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from phone_xarm_teleop import (FRONT_KEEPOUT_X_MM, FRONT_KEEPOUT_Z_MM, GRIPPER_COM_MM,  # noqa: E402
                               GRIPPER_LOAD_KG, REACH_SLOW_MM, SHOULDER_MM, STOW_JOINTS_DEG,
                               WORKSPACE_MM, plan_lowest_flange)

HERE = os.path.dirname(os.path.abspath(__file__))
IP = "192.168.1.204"
TAG_ID = 1
TAG_MM = 24.95
HANDEYE_SPEED_DEG_S = 15.0
HANDEYE_ACC_DEG_S2 = 150.0
SETTLE_S = 1.0
FRAMES = 10
MIN_FLANGE_Z_MM = 150.0  # stay well above the chassis and the front camera for calibration
SAMPLES = os.path.join(HERE, "handeye_samples.json")
RESULT = os.path.join(HERE, "rear_cam_extrinsics.json")
# Arm base in base_link (chassis centre on the floor, x forward, y left), metres.
ARM_BASE_IN_BASE_LINK = (0.13, -0.01, 0.43)


def probe_poses():
    s = STOW_JOINTS_DEG
    poses = [list(s)]
    for joint, delta in ((0, 10), (0, -10), (1, 10), (2, -10), (4, 10), (5, 20)):
        q = list(s)
        q[joint] += delta
        poses.append(q)
    return poses


def sample_poses(n, seed):
    """Random joint offsets around a raised stow pose where the tag faces the camera.

    Probing (2026-10-21) showed the tag is lost when J1 swings it sideways, when J6 turns it
    more than ~15 deg, or when the arm lowers (it leaves the bottom of the image); raising
    the elbow (J3 more negative) brings it toward the image centre.
    """
    rng = np.random.default_rng(seed)
    centre = np.array([4.0, -60.0, -30.0, 0.0, 80.0, 0.0])
    span = np.array([7.0, 6.0, 9.0, 12.0, 12.0, 12.0])
    return [(centre + rng.uniform(-1, 1, 6) * span).round(1).tolist() for _ in range(n)]


def check_target(arm, q):
    """None if the flange at joints q is inside the calibration limits, else the reason."""
    code, p = arm.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
    if code != 0:
        return f"FK failed ({code})"
    for axis, (low, high) in WORKSPACE_MM.items():
        if not low <= p[axis] <= high:
            return f"outside workspace axis {'xyz'[axis]} ({p[axis]:.0f} mm)"
    reach = math.dist(p[:3], SHOULDER_MM)
    if reach > REACH_SLOW_MM:
        return f"reach {reach:.0f} mm"
    if p[2] < MIN_FLANGE_Z_MM:
        return f"flange z {p[2]:.0f} mm below {MIN_FLANGE_Z_MM:.0f}"
    if p[0] < FRONT_KEEPOUT_X_MM and p[2] < FRONT_KEEPOUT_Z_MM:
        return "inside the front keep-out"
    return None


def rpy_matrix(roll, pitch, yaw):
    """xArm convention (degrees): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = np.radians([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


class TagCamera:
    def __init__(self, device="/dev/video0"):
        c = json.load(open(os.path.join(HERE, "rear_cam_intrinsics.json")))
        self.K, self.D = np.array(c["K"]), np.array(c["D"])
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
        h = TAG_MM / 2
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

    def measure(self, frames=FRAMES, save=None):
        """Tag pose in the camera (R, t mm) from corners averaged over `frames` detections, or None."""
        for _ in range(5):  # flush buffered frames from before the move
            self.cap.read()
        found, frame = [], None
        for _ in range(frames * 2):
            ok, frame = self.cap.read()
            if not ok:
                continue
            corners, ids, _ = self.det.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if ids is not None and TAG_ID in ids.ravel():
                found.append(corners[list(ids.ravel()).index(TAG_ID)].reshape(4, 2))
            if len(found) >= frames:
                break
        if save is not None and frame is not None:
            cv2.imwrite(save, frame)
        if len(found) < frames // 2:
            return None
        pts = np.array(found)
        corners = np.median(pts, axis=0)
        jitter = float(np.max(np.std(pts, axis=0)))
        ok, rv, tv = cv2.solvePnP(self.obj, corners, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        side = float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
        return {"R": cv2.Rodrigues(rv)[0].tolist(), "t": tv.ravel().tolist(), "pixel": corners.mean(axis=0).tolist(),
                "side_px": side, "jitter_px": jitter, "n": len(found)}


def collect(args):
    from xarm.wrapper import XArmAPI
    arm = XArmAPI(IP, is_radian=False)
    poses = probe_poses() if args.poses == "probe" else sample_poses(args.n, args.seed)
    code, start = arm.get_servo_angle(is_radian=False)
    start = start[:6]
    print(f"state {arm.state}, error {arm.error_code}; joints now {np.round(start, 1).tolist()}")
    plan, previous = [], start
    for i, q in enumerate(poses):
        reason = check_target(arm, q)
        if reason is None:
            lowest = plan_lowest_flange(arm, previous, [("", q)])
            if lowest < MIN_FLANGE_Z_MM:
                reason = f"path dips to z {lowest:.0f} mm"
        _, p = arm.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
        print(f"{i:2d} {np.round(q, 1).tolist()} -> flange ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f}) "
              f"{'OK' if reason is None else 'SKIP: ' + reason}")
        if reason is None:
            plan.append(q)
            previous = q
    if plan_lowest_flange(arm, previous, [("", STOW_JOINTS_DEG)]) < MIN_FLANGE_Z_MM:
        sys.exit("return to stow would dip too low; not moving")
    if not args.go:
        print(f"plan only: {len(plan)} poses; add --go to move")
        arm.disconnect()
        return
    cam = TagCamera()
    arm.clean_error()
    arm.clean_warn()
    time.sleep(0.5)
    for name, code in (("motion_enable", arm.motion_enable(True)), ("set_mode(0)", arm.set_mode(0)),
                       ("set_state(0)", arm.set_state(0))):
        if code != 0:
            sys.exit(f"{name} failed: {code}")
    time.sleep(0.5)
    print(f"gripper payload -> {arm.set_tcp_load(GRIPPER_LOAD_KG, GRIPPER_COM_MM)}")
    os.makedirs(os.path.join(HERE, "handeye_images"), exist_ok=True)
    samples = json.load(open(SAMPLES)) if os.path.exists(SAMPLES) and args.append else []
    try:
        for i, q in enumerate(plan + [list(STOW_JOINTS_DEG)]):
            last = i == len(plan)
            code = arm.set_servo_angle(angle=q, speed=HANDEYE_SPEED_DEG_S, mvacc=HANDEYE_ACC_DEG_S2,
                                       is_radian=False, wait=True)
            if code != 0 or arm.error_code != 0:
                print(f"stopped: code {code}, arm error {arm.error_code}")
                break
            if last:
                print("back at stow")
                break
            time.sleep(SETTLE_S)
            code, pose = arm.get_position(is_radian=False)
            tag = cam.measure(save=os.path.join(HERE, "handeye_images", f"{args.poses}_{i:02d}.jpg"))
            if tag is None:
                print(f"{i:2d}: tag not seen")
                continue
            samples.append({"joints": q, "flange": pose[:6], "tag": tag})
            print(f"{i:2d}: tag at pixel ({tag['pixel'][0]:.0f},{tag['pixel'][1]:.0f}), {tag['side_px']:.0f} px, "
                  f"jitter {tag['jitter_px']:.2f} px, {np.linalg.norm(tag['t']):.0f} mm away", flush=True)
    except KeyboardInterrupt:
        print("interrupted; stopping the arm")
    finally:
        arm.set_state(4)
        arm.disconnect()
        cam.cap.release()
        with open(SAMPLES, "w") as f:
            json.dump(samples, f, indent=1)
        print(f"{len(samples)} samples in {SAMPLES}")


def solve(args):
    samples = json.load(open(SAMPLES))
    print(f"{len(samples)} samples")
    if len(samples) < 6:
        sys.exit("need at least 6 samples with varied tag orientation")
    r_b2g, t_b2g, r_t2c, t_t2c = [], [], [], []
    for s in samples:
        x, y, z, roll, pitch, yaw = s["flange"]
        r_g2b = rpy_matrix(roll, pitch, yaw)
        t_g2b = np.array([x, y, z])
        # eye-to-hand: feed base->gripper in place of gripper->base; the result is camera->base
        r_b2g.append(r_g2b.T)
        t_b2g.append(-r_g2b.T @ t_g2b)
        r_t2c.append(np.array(s["tag"]["R"]))
        t_t2c.append(np.array(s["tag"]["t"]))
    results = {}
    for name, method in (("Tsai", cv2.CALIB_HAND_EYE_TSAI), ("Park", cv2.CALIB_HAND_EYE_PARK),
                         ("Horaud", cv2.CALIB_HAND_EYE_HORAUD), ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS)):
        r_c2b, t_c2b = cv2.calibrateHandEye(r_b2g, t_b2g, r_t2c, t_t2c, method=method)
        # consistency: the tag pose in the flange frame should be the same for every sample
        tags = []
        for rb, tb, rt, tt in zip(r_b2g, t_b2g, r_t2c, t_t2c):
            r_g2b, t_g2b = rb.T, -rb.T @ tb
            p_base = r_c2b @ tt + t_c2b.ravel()  # tag centre in the base frame
            tags.append(r_g2b.T @ (p_base - t_g2b))
        tags = np.array(tags)
        spread = float(np.sqrt(np.mean(np.sum((tags - tags.mean(axis=0)) ** 2, axis=1))))
        results[name] = (r_c2b, t_c2b.ravel(), spread, tags.mean(axis=0))
        print(f"{name:10s} camera in arm base: t ({t_c2b[0,0]:.0f},{t_c2b[1,0]:.0f},{t_c2b[2,0]:.0f}) mm, "
              f"tag-in-flange spread {spread:.1f} mm, tag at {np.round(tags.mean(axis=0), 0).tolist()} mm")
    best = min(results, key=lambda k: results[k][2])
    r, t, spread, tag = results[best]
    t_link = np.array(ARM_BASE_IN_BASE_LINK) + t / 1000.0
    axis = r[:, 2]
    print(f"chosen {best}: camera optical axis in arm base frame {np.round(axis, 2).tolist()}, "
          f"camera in base_link ({t_link[0]:.3f},{t_link[1]:.3f},{t_link[2]:.3f}) m")
    with open(RESULT, "w") as f:
        json.dump({"method": best, "samples": len(samples), "tag_in_flange_spread_mm": spread,
                   "R_armbase_cam": r.tolist(), "t_armbase_cam_mm": t.tolist(),
                   "t_baselink_cam_m": t_link.tolist(), "tag_in_flange_mm": tag.tolist(),
                   "arm_base_in_base_link_m": list(ARM_BASE_IN_BASE_LINK)}, f, indent=1)
    print(f"saved {RESULT}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--poses", choices=("probe", "sample"), default="probe")
    parser.add_argument("--n", type=int, default=24)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--go", action="store_true", help="move the arm (default: plan only)")
    parser.add_argument("--append", action="store_true", help="add to the saved samples instead of replacing")
    parser.add_argument("--solve", action="store_true")
    args = parser.parse_args()
    solve(args) if args.solve else collect(args)


if __name__ == "__main__":
    main()
