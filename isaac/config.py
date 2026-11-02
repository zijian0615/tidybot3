"""The real robot's geometry and cameras, for the Isaac Sim model.

Frames: base_link is the chassis centre on the floor (x forward, y left, z up).
The arm base frame is base_link shifted to MOUNT_IN_BASE (no rotation): the
frame the xArm controller and calib/*.json use.

Sources (branch ros-bridge of this repo):
  calib/rear_cam_extrinsics.json  arm_base_in_base_link_m, R_armbase_cam, t_armbase_cam_mm
                                  (hand-eye bundle adjustment, 20 samples, 0.55 px rms)
  calib/rear_cam_intrinsics.json  K, D (OpenCV k1 k2 p1 p2 k3), 1280 x 720
  calib/table_lab_right.json      the lab table's top, 292 mm above the arm base
  phone_xarm_teleop.py            STOW_JOINTS_DEG, GRIPPER_LOAD_KG
When calib/ is present the JSON files are read; otherwise the values copied below.
"""

import json
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
CALIB = REPO / "calib"

# ---- the chassis (estimated: no CAD of the real chassis in the repo) ----------------------------
# The TidyBot++ chassis meshes (models/assets/base), scaled in x and y so the front
# bumper is 0.317 m ahead of the centre: the stow pose's flange, 207 mm ahead of the
# arm base, is "~2 cm past the bumper" (phone_xarm_teleop.py). Heights unchanged.
CHASSIS_SCALE_XY = 1.315
CHASSIS_TOP_Z = 0.335              # the arm plate's top in the TidyBot meshes
# Caster steer axes (constants.py: h_x, h_y), for reference
CASTER_XY = [(0.19015, -0.17015), (0.19015, 0.17015), (-0.19015, 0.17015), (-0.19015, -0.17015)]

# ---- the arm -------------------------------------------------------------------------------
MOUNT_IN_BASE = (0.13, -0.01, 0.43)          # arm base in base_link (calib: arm_base_in_base_link_m)
STOW_JOINTS_DEG = [0.0, -60.0, -20.0, 0.0, 80.0, 0.0]
FINGERTIP_LEN = 0.172                        # flange to the closed fingertips (calib/table_lab_right.json)
GRIPPER_LOAD_KG = 0.82

# ---- the lab table (calib/table_lab_right.json) ---------------------------------------------------
TABLE_TOP_ABOVE_ARM_BASE = 0.292

# ---- the lab map (branch ros-bridge: maps/) -----------------------------------------------------
# RTAB-Map's 2D occupancy grid of EMS 885 (Gemini 2, 0.05 m cells), the map frame as Isaac's world.
# The base starts at the waypoint table_right (maps/waypoints_lab-ems885.yaml: "grasp parking at
# the right-hand table, recorded by teleop"). The grid has no heights: the bench ahead of it is
# cut out of the grid and built at the table's height; every other occupied cell is an obstacle
# OBSTACLE_HEIGHT tall.
MAPS = REPO / "maps"
LAB_MAP = "lab-ems885_map.yaml"
LAB_START = (2.13, 1.42, -48.0)              # x, y (m), yaw (deg) in the map frame
# Where to look for the bench, in the start pose's frame (ahead f, left s; metres). On the grid
# it is occupied from 0.475 to 1.05 m ahead (free behind), about 1.65 m to the right and 2.15 m
# to the left; its occupied cells in this window are fitted with a rectangle (it runs along the
# aisle, ~44 deg) and removed from the obstacles. The real hover test (flange 0.52 m ahead of the
# arm base) lands on it.
LAB_BENCH = dict(f=(0.30, 1.25), s=(-1.80, 2.30))
OBSTACLE_HEIGHT = 1.0

# The textured RGB-D scan of the same lab (Hugging Face: zijian2022/tidybot-lab-ems885-scan): the
# same map frame, but its z origin is the Gemini 2's height (the floor near table_right at z -0.405;
# the floor is uneven, -0.385 .. -0.435 across the lab). It is the scene's look; the grid's
# obstacles (hidden) and the bench stay its collision, as the scan has holes (tabletops, the tops
# of cabinets and the upper walls: the camera was low and faced forward).
SCAN_DIR = Path.home() / "datasets" / "tidybot-lab-ems885-scan"
SCAN_MESH = "mesh/lab-ems885_textured_mesh.obj"
SCAN_Z_OFFSET = 0.405                        # raise the scan so the floor at table_right is at 0

# ---- cameras ---------------------------------------------------------------------------------
_REAR_FALLBACK = dict(
    size=(1280, 720),
    K=[[1031.4764030185038, 0.0, 603.30114378351], [0.0, 1030.8882642543836, 384.31677667556716], [0.0, 0.0, 1.0]],
    D=[-0.47295697735854175, 0.2504783850233827, 0.0028832390574032147, 0.0016285110614038733, -0.06483282841307123],
    R_armbase_cam=[[0.16608374362398284, -0.05358297241290816, 0.9846547898483209],
                   [-0.9861057055716924, -0.012492193572106736, 0.16564867201010636],
                   [0.0034245500139348795, -0.9984852578617046, -0.05491322509239016]],
    t_armbase_cam_m=[-0.3322582700428782, -0.34382601864592937, 0.5506232983389298],
)

# The front camera: an Orbbec Gemini 2 on the chassis' front edge, 28 cm in from the
# right side, its lens 39 cm above the floor (as measured on the robot). Assumed: level,
# looking straight ahead, on the front edge (x 0.31 m), the right side at the (estimated)
# chassis half-width. Lens: the Gemini 2's colour stream at 1280 x 720, 86 x 55 deg field
# (Orbbec's datasheet), no distortion; depth rendered from the same viewpoint.
FRONT_CAM_IN_BASE = (0.31, -0.241 * CHASSIS_SCALE_XY + 0.28, 0.39)
FRONT_CAM_PITCH_DEG = 0.0                  # down-tilt, if it has one
_fx, _fy = 640.0 / math.tan(math.radians(86.0 / 2)), 360.0 / math.tan(math.radians(55.0 / 2))


def _front_camera():
    t = math.radians(FRONT_CAM_PITCH_DEG)
    fwd = np.array([math.cos(t), 0.0, -math.sin(t)])          # optical axis (z), in base_link
    right = np.array([0.0, -1.0, 0.0])                         # image x
    down = np.cross(fwd, right)                                # image y
    return dict(size=(1280, 720), K=[[_fx, 0.0, 640.0], [0.0, _fy, 360.0], [0.0, 0.0, 1.0]], D=[0.0] * 5,
                R_armbase_cam=np.stack([right, down, fwd], 1).tolist(),       # arm base and base_link: no rotation
                t_armbase_cam_m=(np.asarray(FRONT_CAM_IN_BASE) - np.asarray(MOUNT_IN_BASE)).tolist(),
                housing=(0.025, 0.09, 0.03), depth=True, source="measured on the robot + Orbbec datasheet")


FRONT_CAMERA = _front_camera()


def rear_camera():
    """The rear-right camera: intrinsics and its pose in the arm base frame."""
    cam = dict(_REAR_FALLBACK, name="rear", source="copied values (calib/ not present)")
    ext, intr = CALIB / "rear_cam_extrinsics.json", CALIB / "rear_cam_intrinsics.json"
    if ext.exists() and intr.exists():
        e, i = json.loads(ext.read_text()), json.loads(intr.read_text())
        cam.update(size=tuple(i["image_size"]), K=i["K"], D=i["D"], R_armbase_cam=e["R_armbase_cam"],
                   t_armbase_cam_m=[v / 1000.0 for v in e["t_armbase_cam_mm"]], source=str(ext.relative_to(REPO)))
    return cam


def cameras():
    cams = [rear_camera()]
    if FRONT_CAMERA is not None:
        cams.append(dict(FRONT_CAMERA, name="front"))
    for c in cams:
        c["K"], c["D"] = np.asarray(c["K"], float), np.asarray(c["D"], float)
        c["R_armbase_cam"], c["t_armbase_cam_m"] = np.asarray(c["R_armbase_cam"], float), np.asarray(c["t_armbase_cam_m"], float)
    return cams
