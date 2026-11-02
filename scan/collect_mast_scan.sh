#!/usr/bin/env bash
# Collect a higher-viewpoint scan of the lab: the mast (rear-right) camera's frames, each
# with the robot's pose in the RTAB-Map map at that instant, while the robot drives around.
#
# Run on the Jetson, with the Gemini 2 driver and RTAB-Map running in localization mode on
# the lab's database (lab-ems885.db), so map -> base_link is on /tf. Drive the robot with the
# phone teleop (or Nav2) while this runs; Ctrl-C to stop.
#
#     scan/collect_mast_scan.sh                      # defaults below
#     MAST_DEV=/dev/video2 RATE=4 scan/collect_mast_scan.sh
#
# Writes $OUT/:
#     images/<stamp_ns>.jpg   the mast camera's frames (as the camera sends them: MJPG)
#     poses.csv               stamp_ns, image, base_link in map (x y z qx qy qz qw), how the
#                             pose was found (exact | latest | none), sharpness (Laplacian var)
#     bag/                    a rosbag of /tf, /tf_static, the mast camera and whatever Gemini 2 /
#                             RTAB-Map topics are found (to redo the poses later if needed)
#     meta/                   calib/*.json, topic list, camera formats, clock sync, git revision
#
# The camera's pose in the map is then pose(base_link) x calib/rear_cam_extrinsics.json
# (arm_base_in_base_link_m, R_armbase_cam, t_armbase_cam_mm); its lens is
# calib/rear_cam_intrinsics.json. Do not move the camera: the calibration would no longer hold.
set -euo pipefail

# ---- settings (environment overrides) ---------------------------------------------------------
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${REPO:-$(dirname "$HERE")}"
OUT="${OUT:-$HOME/scans/lab-ems885-mast-$(date +%Y%m%d-%H%M%S)}"
MAST_DEV="${MAST_DEV:-auto}"          # the mast camera's device; auto: the first non-Orbbec V4L2 camera
MAST_TOPIC="${MAST_TOPIC:-/mast_cam/image_raw/compressed}"
WIDTH="${WIDTH:-1280}"; HEIGHT="${HEIGHT:-720}"   # as calibrated: do not change
RATE="${RATE:-5}"                     # frames saved per second
MAP_FRAME="${MAP_FRAME:-map}"
BASE_FRAME="${BASE_FRAME:-base_link}"
BAG="${BAG:-1}"                       # 1: also record a rosbag
RAW_DEPTH="${RAW_DEPTH:-0}"           # 1: bag the raw depth too when no compressed depth (~60 MB/s)
FORCE="${FORCE:-0}"                   # 1: start even if map -> base_link is not available yet

say() { printf '\033[1m[scan]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[scan] %s\033[0m\n' "$*" >&2; exit 1; }

# ---- ROS 2 -----------------------------------------------------------------------------------
set +u
if ! command -v ros2 >/dev/null; then
    for f in /opt/ros/*/setup.bash; do [ -f "$f" ] && source "$f" && break; done
fi
[ -f "$HOME/rtabmap_ws/install/setup.bash" ] && source "$HOME/rtabmap_ws/install/setup.bash"
set -u
command -v ros2 >/dev/null || die "ros2 not found: source your ROS 2 setup first"
python3 -c "import rclpy, tf2_ros, cv2, sensor_msgs" 2>/dev/null \
    || die "python3 needs rclpy, tf2_ros, sensor_msgs and cv2 (OpenCV)"

say "topics:"
TOPICS="$(timeout 15 ros2 topic list 2>/dev/null || true)"
[ -n "$TOPICS" ] || die "no ROS 2 topics: is the camera driver / RTAB-Map running (same ROS_DOMAIN_ID)?"
echo "$TOPICS" | sed 's/^/    /'
echo "$TOPICS" | grep -qx /tf || die "/tf is not published"

say "checking that $MAP_FRAME -> $BASE_FRAME is on /tf (RTAB-Map localized) ..."
TF_OUT="$(timeout 10 ros2 run tf2_ros tf2_echo "$MAP_FRAME" "$BASE_FRAME" 2>&1 || true)"
if echo "$TF_OUT" | grep -q "Translation"; then
    echo "$TF_OUT" | grep -m1 -A1 "Translation" | sed 's/^/    /'
elif [ "$FORCE" = 1 ]; then
    say "WARNING: no $MAP_FRAME -> $BASE_FRAME yet (FORCE=1: going on; frames get no pose until it appears)"
else
    die "no $MAP_FRAME -> $BASE_FRAME: start RTAB-Map in localization mode on lab-ems885.db and let it relocalize (drive a little), or FORCE=1"
fi

# ---- the mast camera ---------------------------------------------------------------------------
if [ "$MAST_DEV" = auto ]; then
    MAST_DEV=""
    for d in /dev/v4l/by-id/*-video-index0; do
        [ -e "$d" ] || continue
        case "${d,,}" in *orbbec*|*gemini*) continue ;; esac
        MAST_DEV="$d"; break
    done
    [ -n "$MAST_DEV" ] || { MAST_DEV=/dev/video0; say "WARNING: no non-Orbbec camera in /dev/v4l/by-id; trying $MAST_DEV"; }
fi
[ -e "$MAST_DEV" ] || die "mast camera $MAST_DEV not found (set MAST_DEV)"
say "mast camera: $MAST_DEV -> $(readlink -f "$MAST_DEV")"
if command -v fuser >/dev/null && fuser "$(readlink -f "$MAST_DEV")" >/dev/null 2>&1; then
    die "$MAST_DEV is in use by another process (stop it first): $(fuser "$(readlink -f "$MAST_DEV")" 2>&1)"
fi

# ---- the output and its metadata -------------------------------------------------------------------
mkdir -p "$OUT/images" "$OUT/meta"
say "output: $OUT"
cp "$REPO"/calib/rear_cam_*.json "$OUT/meta/" 2>/dev/null || say "WARNING: no calib/rear_cam_*.json in $REPO"
echo "$TOPICS" > "$OUT/meta/topics.txt"
echo "$TF_OUT" > "$OUT/meta/tf_check.txt"
{ echo "date: $(date -Iseconds)"; echo "host: $(hostname)"; echo "mast_dev: $MAST_DEV"
  echo "size: ${WIDTH}x${HEIGHT}  rate: $RATE  frames: $MAP_FRAME -> $BASE_FRAME"
  echo "repo: $REPO @ $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo '?')"; } > "$OUT/meta/info.txt"
command -v v4l2-ctl >/dev/null && v4l2-ctl -d "$MAST_DEV" --list-formats-ext > "$OUT/meta/v4l2_formats.txt" 2>&1 || true
{ command -v chronyc >/dev/null && chronyc tracking; command -v timedatectl >/dev/null && timedatectl; } \
    > "$OUT/meta/clock.txt" 2>&1 || true

# ---- the camera node: publish, save, and pose every frame ------------------------------------------
cat > "$OUT/meta/mast_scan_node.py" <<'PY'
import csv, math, os, sys, threading, time
import cv2, numpy as np, rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformListener

dev, topic, w, h, rate, map_f, base_f, out = sys.argv[1:9]
w, h, rate = int(w), int(h), float(rate)


class MastScan(Node):
    def __init__(self):
        super().__init__("mast_scan")
        self.pub = self.create_publisher(CompressedImage, topic, 5)
        self.tf = Buffer(cache_time=Duration(seconds=30))
        TransformListener(self.tf, self)


def main():
    rclpy.init()
    node = MastScan()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)          # keep the camera's JPEG: no re-encoding
    if not cap.isOpened():
        sys.exit(f"cannot open {dev}")
    f = open(os.path.join(out, "poses.csv"), "w", newline="")
    log = csv.writer(f)
    log.writerow(["stamp_ns", "image", "x", "y", "z", "qx", "qy", "qz", "qw", "pose", "sharpness"])
    n = n_pose = 0
    next_t = time.monotonic()
    t0 = time.monotonic()
    while rclpy.ok():
        if not cap.grab():
            time.sleep(0.01)
            continue
        stamp = node.get_clock().now()                # the frame's time, on the clock /tf uses
        if time.monotonic() < next_t:
            continue                                  # keep the driver's queue drained, save at `rate`
        next_t = max(next_t + 1.0 / rate, time.monotonic())      # behind: skip ahead, no burst
        ok, frame = cap.retrieve()
        if not ok:
            continue
        if frame.ndim == 3 or frame.dtype != np.uint8:             # the driver decoded it after all
            jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
            img = frame
        else:
            jpg = frame.tobytes()
            img = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if gray.shape[1] != w or gray.shape[0] != h:
            sys.exit(f"camera gives {gray.shape[1]}x{gray.shape[0]}, calibrated for {w}x{h}")
        sharp = float(cv2.Laplacian(cv2.resize(gray, (w // 2, h // 2)), cv2.CV_64F).var())
        msg = CompressedImage()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "mast_cam_optical"
        msg.format = "jpeg"
        msg.data = jpg
        node.pub.publish(msg)
        name = f"{stamp.nanoseconds}.jpg"
        with open(os.path.join(out, "images", name), "wb") as fh:
            fh.write(jpg)
        pose, how = [math.nan] * 7, "none"
        for when, label in ((stamp, "exact"), (Time(), "latest")):
            try:
                t = node.tf.lookup_transform(map_f, base_f, when, timeout=Duration(seconds=0.1))
                tr, q = t.transform.translation, t.transform.rotation
                pose, how = [tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w], label
                break
            except Exception:
                continue
        log.writerow([stamp.nanoseconds, name] + [f"{v:.5f}" for v in pose] + [how, f"{sharp:.1f}"])
        n += 1
        n_pose += how == "exact"
        if n % 25 == 0:
            f.flush()
            print(f"[mast] {n} frames in {time.monotonic() - t0:.0f} s, {n_pose} with an exact pose, "
                  f"last sharpness {sharp:.0f}, pose {how}", flush=True)
    f.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
PY

PIDS=()
cleanup() {
    say "stopping ..."
    for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null || true; done
    local n_img n_exact n_pose
    n_img=$(find "$OUT/images" -name '*.jpg' | wc -l)
    n_pose=$(($(wc -l < "$OUT/poses.csv" 2>/dev/null || echo 1) - 1))
    n_exact=$(grep -c ',exact,' "$OUT/poses.csv" 2>/dev/null || echo 0)
    say "done: $n_img images, $n_exact of $n_pose with an exact pose, in $OUT ($(du -sh "$OUT" | cut -f1))"
    [ "$BAG" = 1 ] && [ -d "$OUT/bag" ] && ros2 bag info "$OUT/bag" 2>/dev/null | sed 's/^/    /' || true
    say "send it over, e.g.: tar -C \"$(dirname "$OUT")\" -cf - \"$(basename "$OUT")\" | ssh csi_admin_r2@100.111.110.27 'tar -C ~/datasets -xf -'"
}
trap cleanup EXIT
trap 'exit 0' INT TERM

# (the process substitution keeps $! the node's own pid, so Ctrl-C / kill reach it)
python3 "$OUT/meta/mast_scan_node.py" "$MAST_DEV" "$MAST_TOPIC" "$WIDTH" "$HEIGHT" "$RATE" \
    "$MAP_FRAME" "$BASE_FRAME" "$OUT" > >(tee "$OUT/meta/node.log") 2>&1 &
PIDS+=($!)
for _ in $(seq 20); do [ -f "$OUT/poses.csv" ] && break; kill -0 "${PIDS[0]}" 2>/dev/null || break; sleep 1; done
[ -f "$OUT/poses.csv" ] || die "the camera node did not start: see $OUT/meta/node.log"

if [ "$BAG" = 1 ]; then
    REC=(/tf /tf_static "$MAST_TOPIC")
    pick() { echo "$TOPICS" | grep -E "$1" | head -1 || true; }
    for pat in 'color/image_raw/compressed$' 'color/camera_info$' 'depth/camera_info$' \
               'localization_pose$' '^/odom$' 'rtabmap/odom$'; do
        t="$(pick "$pat")"; [ -n "$t" ] && REC+=("$t")
    done
    t="$(pick 'depth/image_raw/compressedDepth$')"
    if [ -n "$t" ]; then REC+=("$t")
    elif [ "$RAW_DEPTH" = 1 ]; then t="$(pick 'depth/image_raw$')"; [ -n "$t" ] && REC+=("$t"); fi
    say "recording: ${REC[*]}"
    ros2 bag record -o "$OUT/bag" "${REC[@]}" > "$OUT/meta/bag.log" 2>&1 &
    PIDS+=($!)
fi

cat <<EOF

  Recording. Drive the robot (phone teleop or Nav2), slowly (<= 0.2 m/s, slow turns):
    - several passes along the long bench at 0.5, 1.0 and 1.5 m from it, stopping a few seconds each
    - a few slow turns in place in the middle of the lab, and through the door to the corridor
    - keep the arm stowed: it hides part of the camera's view
  Ctrl-C to stop.

EOF
wait "${PIDS[0]}"
