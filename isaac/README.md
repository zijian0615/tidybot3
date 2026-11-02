# TidyBot with the xArm6 in Isaac Sim

This is the real robot's geometry in Isaac Sim 5.1:
- the base: three planar DOFs (x, y, yaw), position-driven as in the MuJoCo model;
- the official Isaac asset `Ufactory/xarm6/xarm6.usd` (the arm with the xArm gripper), fixed at the calibrated mount;
- the cameras at their calibrated poses, each rendered through its real lens model.

Run from the repo root with Isaac Sim's Python:

```bash
PYTHONPATH=. ~/isaacsim-venv/bin/python -m isaac.demo --world lab --out isaac_demo --video   # the lab: dock at the bench, pick, back off
PYTHONPATH=. ~/isaacsim-venv/bin/python -m isaac.demo --world table --out isaac_demo         # an empty floor and a table
PYTHONPATH=. ~/isaacsim-venv/bin/python -m isaac.validate --render 5             # check against calib/handeye_samples.json
```

| File | Contents |
|---|---|
| `config.py` | the robot's geometry and the cameras' calibration (read from `calib/*.json` when present, else copied values) |
| `sim.py` | `TidybotXarm6`: builds the scene, steps the physics, sets base / arm / gripper targets, renders the cameras |
| `validate.py` | compares the model with the hand-eye calibration samples |
| `demo.py` | drives the base 10 cm forward, picks the cube off the lab table, backs off and stows (records the rear, front, front depth and an overview) |

## The lab (`--world lab`)

The scene is built from RTAB-Map's 2D occupancy grid of EMS 885 (`maps/lab-ems885_map.yaml`, 0.05 m cells), and the map frame is Isaac's world.

- **Start pose**: the base starts at the waypoint `table_right` (`maps/waypoints_lab-ems885.yaml`: the grasp parking spot at the right-hand table). The demo begins 0.5 m short of it, at `table_right_pre`, as Nav2 leaves it, then drives up to the bench.
- **The bench**: the grid has no heights, so the bench ahead of the waypoint is cut out of it.
  - Its direction is the rectangle fitted to its cells, 41.9°, along the aisle.
  - Its depth comes from the stretch in front of the waypoint: the near edge is 0.40 m ahead (the grid profile says 0.475), the far edge 1.05 m.
  - It is built at the table's measured height (`calib/table_lab_right.json`).
  - The real hover-test point lands on it, and the cube is placed there.
- **Every other occupied cell** is an obstacle `config.OBSTACLE_HEIGHT` (1.0 m) tall: one static mesh with collision.
- **The look** is the textured RGB-D scan of the same lab (Hugging Face `zijian2022/tidybot-lab-ems885-scan`: 68.6 k faces, two 8192 px textures).
  - Download it to `config.SCAN_DIR` (`~/datasets/tidybot-lab-ems885-scan`). Without it, the grid's boxes are shown instead.
  - It shares the map frame, but its z origin is the Gemini 2's height. The floor near `table_right` is at z −0.405 in the cloud (`config.SCAN_Z_OFFSET` raises it to 0). The floor is uneven, −0.385 .. −0.435 across the lab.
  - The scan has holes: tabletops, the tops of cabinets and the upper walls, because the camera was low and faced forward. So it is looks only. The grid's boxes (hidden) and the bench are the collision.
  - The cloud shows the bench's front face up to about 0.67 m but not its top, consistent with the 0.722 m tabletop.

## Where the numbers come from (branch `ros-bridge`)

| | Source |
|---|---|
| Arm base in `base_link`: (0.13, −0.01, 0.43) m | `calib/rear_cam_extrinsics.json`: `arm_base_in_base_link_m` |
| Rear-right camera pose | `calib/rear_cam_extrinsics.json`: `R_armbase_cam`, `t_armbase_cam_mm` (hand-eye bundle adjustment, 0.55 px rms) |
| Rear-right camera lens | `calib/rear_cam_intrinsics.json`: K, D (OpenCV), 1280×720 |
| Lab table top: 292 mm above the arm base | `calib/table_lab_right.json` |
| Stow pose, gripper payload | `phone_xarm_teleop.py` |

## Validation against the real robot

`isaac.validate` puts the arm at each of the 20 hand-eye samples' joint angles. It compares the flange with the controller's reported pose, and projects the AprilTag on the gripper's back through the Isaac rear camera:

- flange: 0.19 mm mean, 0.23 mm max, 0.05° max;
- tag: 0.67 px rms, 1.32 px max. The calibration's own residual is 0.55 px.

## Estimates and gaps

- **Chassis**: there's no CAD of the real chassis. The TidyBot++ chassis meshes are scaled ×1.315 in x and y, so the front bumper sits 0.317 m ahead of the centre (the stow pose's note: "~2 cm past the bumper"). A riser lifts the arm base to 0.43 m.
- **Camera mast**: the rear camera's housing and mast are placeholders. Only the lens pose is measured.
- **Front camera** (Orbbec Gemini 2): measured on the robot, 28 cm in from the chassis' right side and 39 cm above the floor.
  - Assumed: level, looking straight ahead, on the front edge (x 0.31 m), with the right side at the estimated chassis half-width.
  - The lens is the datasheet's colour stream (1280×720, 86°×55°, no distortion). Depth is rendered from the same viewpoint.
  - Adjust `config.FRONT_CAM_IN_BASE` and `config.FRONT_CAM_PITCH_DEG` if these assumptions are off.
- **Arm drives**: stiffer than the asset's (`sim.ARM_KP`), so the joints hold their targets under gravity as the real controller does. Self-collision is off, as in the asset (the gripper's linkage overlaps).
