# Maps

2D occupancy grids exported from RTAB-Map (Orbbec Gemini 2, 0.05 m/cell) for Nav2 `map_server`.
Each `*_map.yaml` points at the `.pgm` next to it.

| Map | Area | Waypoints |
|---|---|---|
| `lab-ems885_map` | EMS 885 lab | `waypoints_lab-ems885.yaml` |
| `rtabmap_run3_ray_map` | Corridor loop, ray-traced grid | `waypoints_rtabmap_run3_ray.yaml` |
| `rtabmap_run3_map` | Corridor loop, raw grid | |
| `rtabmap_run3_clean_map` | Corridor loop, cleaned grid | |

The RTAB-Map databases (`*.db`, 25-35 MB) and point clouds (`*.ply`, up to 87 MB) needed for
localization are not in git. They live on the Jetson in `~/rtabmap_ws/export/`.
