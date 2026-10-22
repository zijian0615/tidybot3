"""3D position of an upright bottle on the table from the rear camera (arm base frame, mm).

    python calib/locate_bottle.py U_CENTRE V_BOTTOM U_LEFT U_RIGHT V_WIDTH [--table-z 292] [--radius 32.5]

Two estimates: (a) the bottom contact pixel on the table plane, pushed back by the radius;
(b) the silhouette width at row V_WIDTH (independent of the table height). Prints both and the mean.
"""
import argparse
import json
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
C = json.load(open(os.path.join(HERE, "rear_cam_intrinsics.json")))
E = json.load(open(os.path.join(HERE, "rear_cam_extrinsics.json")))
K, D = np.array(C["K"]), np.array(C["D"])
R, T = np.array(E["R_armbase_cam"]), np.array(E["t_armbase_cam_mm"])


def ray(u, v):
    n = cv2.undistortPoints(np.array([[[u, v]]], float), K, D).ravel()
    d = R @ np.array([n[0], n[1], 1.0])
    return d / np.linalg.norm(d)


def locate(uc, vb, ul, ur, vw, table_z=292.0, radius=32.5):
    d = ray(uc, vb)
    front = T + (table_z - T[2]) / d[2] * d
    h = np.array([d[0], d[1], 0.0])
    a = front[:2] + radius * (h / np.linalg.norm(h))[:2]
    dl, dr = ray(ul, vw), ray(ur, vw)
    dist = radius / np.sin(np.arccos(np.clip(dl @ dr, -1, 1)) / 2)
    dm = (dl + dr) / np.linalg.norm(dl + dr)
    b = (T + dist * dm)[:2]
    return a, b, (a + b) / 2


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for name in ("uc", "vb", "ul", "ur", "vw"):
        p.add_argument(name, type=float)
    p.add_argument("--table-z", type=float, default=292.0)
    p.add_argument("--radius", type=float, default=32.5)
    a = p.parse_args()
    ea, eb, m = locate(a.uc, a.vb, a.ul, a.ur, a.vw, a.table_z, a.radius)
    print(f"table-plane {np.round(ea)}  width {np.round(eb)}  mean {np.round(m)}  disagreement {np.linalg.norm(ea - eb):.0f} mm")
