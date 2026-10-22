"""Collect checkerboard views from a UVC camera for intrinsic/distortion calibration.

    python capture_intrinsics.py [--device /dev/video0] [--out rear_cam_images] [--target 40]

Saves a frame whenever the full 9x6 inner-corner board is found, sharp, and in a new place
(position, size or tilt) compared with the frames already kept. Prints which image regions
still lack coverage. Ctrl-C stops.
"""
import argparse
import os
import time

import cv2
import numpy as np

PATTERN = (9, 6)
GRID = (3, 3)  # coverage cells across the image


def describe(corners, shape):
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    w = np.ptp(pts[:, 0])
    h = np.ptp(pts[:, 1])
    grid = corners.reshape(PATTERN[1], PATTERN[0], 2)
    top = np.linalg.norm(grid[0, -1] - grid[0, 0])
    bottom = np.linalg.norm(grid[-1, -1] - grid[-1, 0])
    left = np.linalg.norm(grid[-1, 0] - grid[0, 0])
    right = np.linalg.norm(grid[-1, -1] - grid[0, -1])
    tilt = (top - bottom) / max(top, bottom), (left - right) / max(left, right)
    return np.array([c[0] / shape[1], c[1] / shape[0], w / shape[1], h / shape[0], tilt[0], tilt[1]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "rear_cam_images"))
    parser.add_argument("--target", type=int, default=40)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    kept = []
    coverage = np.zeros(GRID, int)
    last_report = 0.0
    print(f"capturing to {args.out}; move the board slowly, hold still 1-2 s per pose", flush=True)
    try:
        while len(kept) < args.target:
            ok, frame = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, None, fx=0.5, fy=0.5)
            found, corners = cv2.findChessboardCorners(small, PATTERN, cv2.CALIB_CB_FAST_CHECK)
            now = time.time()
            if found:
                corners = cv2.cornerSubPix(gray, corners * 2.0, (11, 11), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
                sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
                d = describe(corners, gray.shape)
                novel = all(np.linalg.norm(d - k) > 0.12 for k in kept)
                if sharp > 60 and novel:
                    kept.append(d)
                    cx, cy = min(int(d[0] * GRID[1]), GRID[1] - 1), min(int(d[1] * GRID[0]), GRID[0] - 1)
                    coverage[cy, cx] += 1
                    cv2.imwrite(os.path.join(args.out, f"{len(kept):03d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    print(f"[{len(kept):2d}/{args.target}] kept: centre ({d[0]:.2f},{d[1]:.2f}) size {d[2]:.2f} "
                          f"tilt ({d[4]:+.2f},{d[5]:+.2f})", flush=True)
            if now - last_report > 8:
                names = [["左上", "上", "右上"], ["左", "中", "右"], ["左下", "下", "右下"]]
                missing = [names[r][c] for r in range(GRID[0]) for c in range(GRID[1]) if coverage[r, c] == 0]
                print(f"   coverage {coverage.tolist()}  board seen now: {found}  still missing: {' '.join(missing) or '-'}",
                      flush=True)
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        print(f"done: {len(kept)} frames in {args.out}", flush=True)


if __name__ == "__main__":
    main()
