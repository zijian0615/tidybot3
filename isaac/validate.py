"""Check the Isaac model against the real robot's hand-eye calibration samples.

    cd tidybot3 && PYTHONPATH=. ~/isaacsim-venv/bin/python -m isaac.validate --samples calib/handeye_samples.json

Each sample has the arm's joint angles, the flange pose the xArm controller reported
and the AprilTag (taped to the back of the gripper) as the rear camera saw it. Here the
arm is put at each sample's joints and
  1. the flange pose from Isaac's physics is compared with the controller's;
  2. the tag's centre (its pose on the flange: calib/rear_cam_extrinsics.json) is
     projected through the Isaac model's rear camera (its pose on the chassis and its
     lens) and compared with the pixel the real camera measured.
With --render, the rear camera's image at one sample is saved, the measured tag
centre marked on it.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from . import config as CFG


def rpy_matrix(r, p, y):
    r, p, y = np.radians([r, p, y])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", default=str(CFG.CALIB / "handeye_samples.json"))
    ap.add_argument("--extrinsics", default=str(CFG.CALIB / "rear_cam_extrinsics.json"))
    ap.add_argument("--render", type=int, default=-1, help="save the rear image at this sample")
    ap.add_argument("--out", default="isaac_validate")
    a = ap.parse_args()
    samples = json.loads(Path(a.samples).read_text())
    ext = json.loads(Path(a.extrinsics).read_text()) if Path(a.extrinsics).exists() else None
    if ext is None:
        raise SystemExit(f"need {a.extrinsics} for the tag's pose on the flange")
    R_ft, t_ft = np.array(ext["R_flange_tag"]), np.array(ext["t_flange_tag_mm"]) / 1000.0

    from .sim import TidybotXarm6
    sim = TidybotXarm6(headless=True, table=False, cube=False, overview=False)
    sim.finalize()
    rows = []
    for i, s in enumerate(samples):
        sim.reset(arm_deg=s["joints"])
        sim.step(240)                               # settle: the drives hold the joints against gravity
        q = np.degrees(sim.arm_joints())
        p, R = sim.flange_in_arm_base()
        x, y, z, r, pi, yw = s["flange"]
        dp = np.linalg.norm(p * 1000 - [x, y, z])
        Rc = rpy_matrix(r, pi, yw)
        dang = np.degrees(np.arccos(np.clip((np.trace(Rc.T @ R) - 1) / 2, -1, 1)))
        # the tag's centre in the world, from Isaac's flange
        f = sim.frames()["flange"]
        tag_w = f[0] + f[1] @ t_ft
        px = sim.project("rear", tag_w)[0]
        dpx = np.linalg.norm(px - s["tag"]["pixel"])
        rows.append((dp, dang, dpx, np.abs(q - s["joints"]).max()))
        print(f"sample {i:2d}: flange off by {dp:5.2f} mm, {dang:4.2f} deg; tag at {np.round(px, 1)} vs measured "
              f"{np.round(s['tag']['pixel'], 1)}: {dpx:5.2f} px (joints settled within {rows[-1][3]:.2f} deg)", flush=True)
        if i == a.render:
            import cv2
            for _ in range(3):
                sim.render(frames=4)
            img = sim.render(views=["rear"], frames=8)["rear"].copy()
            u, v = (int(round(c)) for c in s["tag"]["pixel"])
            cv2.circle(img, (u, v), 14, (255, 0, 0), 2)
            Path(a.out).mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(Path(a.out) / f"rear_sample{i}.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    r = np.array(rows)
    print(f"SUMMARY flange position: mean {r[:, 0].mean():.2f} mm, max {r[:, 0].max():.2f} mm | flange angle: max "
          f"{r[:, 1].max():.2f} deg | tag pixel: rms {np.sqrt((r[:, 2] ** 2).mean()):.2f} px, max {r[:, 2].max():.2f} px",
          flush=True)
    sim.close()


if __name__ == "__main__":
    main()
