"""Pixel-level refinement of the rear camera extrinsics from calib/handeye_samples.json.

Jointly estimates the camera pose in the arm base frame and the AprilTag pose in the flange
frame by minimising the reprojection error of the tag corners over all samples (the tag is
only ~34 px wide, so per-sample PnP orientations are too noisy for cv2.calibrateHandEye).
Leave-one-out runs report the stability. Writes calib/rear_cam_extrinsics.json.
"""
import json
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from handeye_rear import ARM_BASE_IN_BASE_LINK, RESULT, SAMPLES, TAG_MM, rpy_matrix  # noqa: E402

C = json.load(open(os.path.join(HERE, "rear_cam_intrinsics.json")))
K, D = np.array(C["K"]), np.array(C["D"])
h = TAG_MM / 2
OBJ = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])


def load():
    flanges, obs = [], []
    for s in json.load(open(SAMPLES)):
        x, y, z, r, p, yw = s["flange"]
        flanges.append((rpy_matrix(r, p, yw), np.array([x, y, z])))
        rv = cv2.Rodrigues(np.array(s["tag"]["R"]))[0]
        obs.append(cv2.projectPoints(OBJ, rv, np.array(s["tag"]["t"]), K, D)[0].reshape(4, 2))
    return flanges, np.array(obs)


def unpack(x):
    return Rot.from_rotvec(x[:3]).as_matrix(), x[3:6], Rot.from_rotvec(x[6:9]).as_matrix(), x[9:12]


def residuals(x, flanges, obs):
    r_cb, t_cb, r_tf, t_tf = unpack(x)  # base -> camera, tag in flange
    out = []
    for (r_g, t_g), o in zip(flanges, obs):
        p_base = (r_g @ (r_tf @ OBJ.T + t_tf[:, None])).T + t_g
        p_cam = (r_cb @ p_base.T).T + t_cb
        out.append((cv2.projectPoints(p_cam, np.zeros(3), np.zeros(3), K, D)[0].reshape(4, 2) - o).ravel())
    return np.concatenate(out)


def fit(flanges, obs, starts=60, x0=None):
    tag0 = json.load(open(SAMPLES))[0]["tag"]
    rng = np.random.default_rng(0)
    best = None
    inits = [x0] if x0 is not None else []
    for trial in range(starts if x0 is None else 0):
        r_tf, t_tf = Rot.random(random_state=trial).as_matrix(), rng.uniform(-100, 150, 3)
        t_ct = np.eye(4); t_ct[:3, :3], t_ct[:3, 3] = tag0["R"], tag0["t"]
        t_ft = np.eye(4); t_ft[:3, :3], t_ft[:3, 3] = r_tf, t_tf
        t_bf = np.eye(4); t_bf[:3, :3], t_bf[:3, 3] = flanges[0]
        t_cb = t_ct @ np.linalg.inv(t_ft) @ np.linalg.inv(t_bf)
        inits.append(np.concatenate([Rot.from_matrix(t_cb[:3, :3]).as_rotvec(), t_cb[:3, 3],
                                     Rot.from_matrix(r_tf).as_rotvec(), t_tf]))
    for x in inits:
        r = least_squares(residuals, x, args=(flanges, obs), method="lm", max_nfev=4000)
        rms = float(np.sqrt(np.mean(r.fun ** 2)))
        if best is None or rms < best[0]:
            best = (rms, r.x)
    return best


def camera_in_base(x):
    r_cb, t_cb, _, _ = unpack(x)
    return r_cb.T, -r_cb.T @ t_cb


def main():
    flanges, obs = load()
    rms, x = fit(flanges, obs)
    r_bc, t_bc = camera_in_base(x)
    _, _, r_tf, t_tf = unpack(x)
    per = np.sqrt(np.mean(residuals(x, flanges, obs).reshape(len(obs), -1) ** 2, axis=1))
    print(f"{len(obs)} samples, reprojection RMS {rms:.2f} px (worst sample {per.max():.2f} px)")
    loo_t, loo_axis = [], []
    for i in range(len(obs)):
        keep = [k for k in range(len(obs)) if k != i]
        _, xi = fit([flanges[k] for k in keep], obs[keep], x0=x)
        ri, ti = camera_in_base(xi)
        loo_t.append(ti)
        loo_axis.append(np.degrees(np.arccos(np.clip(ri[:, 2] @ r_bc[:, 2], -1, 1))))
    spread = np.max(np.linalg.norm(np.array(loo_t) - t_bc, axis=1))
    print(f"camera in arm base: ({t_bc[0]:.0f},{t_bc[1]:.0f},{t_bc[2]:.0f}) mm, optical axis "
          f"{np.round(r_bc[:, 2], 3).tolist()}")
    print(f"leave-one-out: position moves up to {spread:.1f} mm, optical axis up to {max(loo_axis):.2f} deg")
    print(f"tag in flange: ({t_tf[0]:.0f},{t_tf[1]:.0f},{t_tf[2]:.0f}) mm, normal {np.round(r_tf[:, 2], 2).tolist()}")
    t_link = np.array(ARM_BASE_IN_BASE_LINK) + t_bc / 1000.0
    print(f"camera in base_link: ({t_link[0]:.3f},{t_link[1]:.3f},{t_link[2]:.3f}) m")
    with open(RESULT, "w") as f:
        json.dump({"method": "pixel bundle adjustment (calib/handeye_ba.py)", "samples": len(obs),
                   "reprojection_rms_px": rms, "loo_position_spread_mm": float(spread),
                   "loo_axis_spread_deg": float(max(loo_axis)),
                   "R_armbase_cam": r_bc.tolist(), "t_armbase_cam_mm": t_bc.tolist(),
                   "t_baselink_cam_m": t_link.tolist(), "R_flange_tag": r_tf.tolist(), "t_flange_tag_mm": t_tf.tolist(),
                   "arm_base_in_base_link_m": list(ARM_BASE_IN_BASE_LINK)}, f, indent=1)
    print(f"saved {RESULT}")


if __name__ == "__main__":
    main()
