"""Drive the base and pick the cube off the lab table, recording the cameras.

    cd tidybot3 && PYTHONPATH=. ~/isaacsim-venv/bin/python -m isaac.demo --out isaac_demo [--video]

The arm's joint targets come from ik_solver.py's xArm6 model (the same kinematics,
checked against the controller by isaac.validate), for the fingertip point
(config.FINGERTIP_LEN past the flange) pointing down.
"""

import argparse
from pathlib import Path

import numpy as np

from . import config as CFG

DOWN = np.array([1.0, 0.0, 0.0, 0.0])        # (x, y, z, w): the IK site's z (the tool axis) pointing down
GRASP_ABOVE_CENTRE = 0.012                   # open, the xArm gripper's fingertips reach below the closed TCP:
                                             # grip the cube's upper part, the fingers clear of the table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="isaac_demo")
    ap.add_argument("--video", action="store_true", help="save frames every 0.1 s, not just the key moments")
    ap.add_argument("--path-tracing", action="store_true")
    ap.add_argument("--world", default="lab", choices=["lab", "table"],
                    help="lab: the RTAB-Map grid of EMS 885 at the right-hand table; table: an empty floor")
    ap.add_argument("--map", default=None, help="the lab map's yaml (default config.MAPS / config.LAB_MAP)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    from .sim import TidybotXarm6, PHYSICS_DT
    from ik_solver import IKSolver
    import cv2
    sim = TidybotXarm6(headless=True, path_tracing=a.path_tracing, world=a.world, map_yaml=a.map)
    sim.finalize()
    if a.world == "lab":                       # Nav2 brings it to table_right_pre, 0.5 m short of the bench
        print("lab:", sim.lab_info, flush=True)
        pre = sim.base_ahead(-0.5)
        sim.reset_base(pre)
    ik = IKSolver(ee_offset=CFG.FINGERTIP_LEN, arm="xarm6")
    for _ in range(3):
        sim.render(frames=4)
    st = {"f": 0, "log": []}

    def shot(tag=None):
        imgs = sim.render(frames=8 if tag else 4)
        if not all(k in imgs for k in ("rear", "front", "overview")):
            return
        depth = imgs.get("front_depth")
        if depth is not None:                  # 0.15 .. 3 m, near = warm
            d8 = (255 * (1 - np.clip((depth - 0.15) / 2.85, 0, 1))).astype(np.uint8)
            depth = cv2.cvtColor(cv2.applyColorMap(d8, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
        else:
            depth = np.zeros_like(imgs["front"])
        size = (960, 540)
        row = np.vstack([np.hstack([cv2.resize(imgs["rear"], size), cv2.resize(imgs["front"], size)]),
                         np.hstack([cv2.resize(imgs["overview"], size), cv2.resize(depth, size)])])
        if a.video:
            cv2.imwrite(str(out / f"f{st['f']:05d}.png"), cv2.cvtColor(row, cv2.COLOR_RGB2BGR))
            st["f"] += 1
        if tag:
            cv2.imwrite(str(out / f"{tag}.png"), cv2.cvtColor(row, cv2.COLOR_RGB2BGR))

    def run(seconds, every=0.1):
        n = int(round(every / PHYSICS_DT))
        for _ in range(int(round(seconds / every))):
            sim.step(n)
            if a.video:
                shot()

    def arm_to(q_end, seconds):
        """A straight joint-space move (the drives track the moving target)."""
        q0 = sim.target[sim.i_arm].copy()
        n = int(round(seconds / 0.05))
        for i in range(1, n + 1):
            sim.set_arm_target(q0 + (q_end - q0) * i / n)
            sim.step(int(round(0.05 / PHYSICS_DT)))
            if a.video and i % 2 == 0:
                shot()

    def solve(p_arm, seed):
        q = ik.solve(np.asarray(p_arm, float), DOWN, seed)
        return q

    sim.set_gripper(0.0)
    run(1.0)
    shot("1_stow")

    # dock: drive up to the bench (the lab) or 10 cm forward (the empty floor)
    sim.set_base_target(*(sim.start if a.world == "lab" else sim.base_ahead(0.10)))
    run(3.0)
    shot("2_base_moved")
    base = sim.base_pose()

    # the cube in the arm base frame
    p_cube = sim.world_to_arm(_cube_pos(sim))
    q = sim.arm_joints()
    q_above = solve(p_cube + [0, 0, 0.12], q)
    arm_to(q_above, 2.5)
    shot("3_above")
    q_at = solve(p_cube + [0, 0, GRASP_ABOVE_CENTRE], q_above)
    arm_to(q_at, 1.5)
    run(0.5)
    sim.set_gripper(1.0)
    run(1.5)
    shot("4_closed")
    z_before = _cube_pos(sim)[2]
    q_up = solve(p_cube + [0, 0, 0.15], q_at)
    arm_to(q_up, 2.0)
    run(0.5)
    z_after = _cube_pos(sim)[2]
    shot("5_lifted")
    print(f"cube z {z_before:.3f} -> {z_after:.3f} m: {'PICKED' if z_after - z_before > 0.10 else 'FAILED'}; "
          f"gripper {sim.gripper():.2f}; base {np.round(base, 3)}", flush=True)
    # back the base away from the table before folding: a joint-space move to the stow
    # pose dips the hand, and with the base forward it would sweep it through the table
    sim.set_base_target(*sim.base_ahead(-0.5 if a.world == "lab" else 0.0))
    run(3.0)
    shot("6_base_back")
    arm_to(np.radians(CFG.STOW_JOINTS_DEG), 3.0)
    run(0.5)
    shot("7_stow_holding")
    print(f"back in stow: cube z {_cube_pos(sim)[2]:.3f} m", flush=True)
    sim.close()


def _cube_pos(sim):
    from isaacsim.core.prims import SingleRigidPrim
    if not hasattr(sim, "_cube"):
        sim._cube = SingleRigidPrim(prim_path="/World/Cube", name="cube")
        sim._cube.initialize()
    p, _ = sim._cube.get_world_pose()
    return np.asarray(p, float)


if __name__ == "__main__":
    main()
