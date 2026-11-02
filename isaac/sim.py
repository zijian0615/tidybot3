"""The TidyBot with the xArm6 and the xArm gripper in Isaac Sim.

    from isaac.sim import TidybotXarm6
    sim = TidybotXarm6(headless=True)
    sim.finalize()
    sim.set_base_target(0.2, 0.0, 0.0); sim.set_arm_target(np.radians(config.STOW_JOINTS_DEG)); sim.set_gripper(0.0)
    sim.step(240)
    images = sim.render()            # {"rear": 1280x720 RGB, as the real camera sees it, ...}

The robot is one articulation: an anchor fixed to the world, prismatic x and y and a
revolute yaw (the base's three planar DOFs, position-driven as in the MuJoCo model),
the chassis, and on it the official Isaac asset Ufactory/xarm6/xarm6.usd (arm and
gripper; the gripper's linkage follows drive_joint through mimic joints), fixed at
config.MOUNT_IN_BASE. Cameras ride on the chassis at their calibrated poses; a
camera with lens distortion is rendered as an ideal pinhole with the same focal
length on a larger canvas and remapped through its OpenCV model (K, D).
"""

from __future__ import annotations

import math
import struct
from pathlib import Path

import numpy as np

from . import config as CFG

XARM_USD = "/Isaac/Robots/Ufactory/xarm6/xarm6.usd"
ROBOT = "/World/Robot"
ANCHOR, BASE_X, BASE_Y, CHASSIS = (f"{ROBOT}/{n}" for n in ("anchor", "base_x", "base_y", "chassis"))
ARM = f"{ROBOT}/xarm"
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
BASE_JOINTS = ["base_x", "base_y", "base_yaw"]
GRIPPER_JOINT = "drive_joint"                # 0 open .. GRIPPER_CLOSED closed (the left outer knuckle)
GRIPPER_CLOSED = math.radians(48.7)
PHYSICS_DT = 1.0 / 240.0
MESHES = CFG.REPO / "models" / "assets" / "base"

# drive gains per radian / metre (USD angular drives are per degree: converted when set)
ARM_KP = [1.0e5, 1.0e5, 5.0e4, 5.0e4, 2.0e4, 2.0e4]
ARM_KD = [4.0e3, 4.0e3, 2.0e3, 2.0e3, 8.0e2, 8.0e2]
ARM_TORQUE = [163.2, 163.2, 81.6, 81.6, 30.6, 30.6]      # rated joint torques (xarm_ros2 joint_limits.yaml)


def read_stl(path):
    """Binary STL -> (points N x 3, triangle indices)."""
    b = Path(path).read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"]
    return tri.reshape(-1, 3).astype(np.float64), np.arange(n * 3)


def cam_matrix_usd(R_cv, t):
    """A camera prim's local transform (Gf row-vector convention) from an OpenCV camera
    pose (columns of R_cv: the camera's x right, y down, z forward in the parent frame)."""
    from pxr import Gf
    R = np.asarray(R_cv) @ np.diag([1.0, -1.0, -1.0])           # USD cameras look down -z, +y up
    M = np.eye(4)
    M[:3, :3] = R.T
    M[3, :3] = t
    return Gf.Matrix4d(*M.ravel().tolist())


class Lens:
    """Render an OpenCV camera (K, D at size W x H) through an ideal pinhole on a canvas
    big enough for the distorted image's field, and remap."""

    def __init__(self, K, D, size, margin=8):
        import cv2
        self.K, self.D, (self.W, self.H) = np.asarray(K, float), np.asarray(D, float), size
        u, v = np.meshgrid(np.arange(self.W, dtype=np.float32), np.arange(self.H, dtype=np.float32))
        pts = np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2)
        n = cv2.undistortPoints(pts, self.K, self.D, P=None).reshape(-1, 2)          # normalized pinhole coords
        fx, fy = self.K[0, 0], self.K[1, 1]
        self.CW = 2 * int(math.ceil(np.abs(n[:, 0]).max() * fx)) + margin
        self.CH = 2 * int(math.ceil(np.abs(n[:, 1]).max() * fy)) + margin
        self.map_x = (n[:, 0] * fx + self.CW / 2.0).reshape(self.H, self.W).astype(np.float32)
        self.map_y = (n[:, 1] * fy + self.CH / 2.0).reshape(self.H, self.W).astype(np.float32)

    def apertures(self, focal=24.0):
        """(horizontal, vertical) aperture for a pinhole with the lens's fx, fy on the canvas."""
        return focal * self.CW / self.K[0, 0], focal * self.CH / self.K[1, 1]

    def distort(self, canvas, nearest=False):
        import cv2
        return cv2.remap(canvas, self.map_x, self.map_y, cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)


class TidybotXarm6:
    def __init__(self, headless=True, table=True, cube=True, overview=True, path_tracing=False, spp=16,
                 world="table", map_yaml=None, scan=True):
        """world: "table" (an empty floor, a table ahead) or "lab" (the RTAB-Map grid of EMS 885,
        the base at the right-hand table: config.LAB_*; map_yaml overrides the map's path)."""
        from isaacsim import SimulationApp
        self.app = SimulationApp({"headless": headless})
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.storage.native import get_assets_root_path
        self.world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
        self.stage = omni.usd.get_context().get_stage()
        self.assets = get_assets_root_path()
        self.cameras = CFG.cameras()
        self.spp = spp
        self.world_kind = world
        self.use_scan = scan
        self.start = (0.0, 0.0, 0.0)
        if world == "lab":
            self.start = (CFG.LAB_START[0], CFG.LAB_START[1], math.radians(CFG.LAB_START[2]))
            self._floor_and_lights()
            self._lab(map_yaml or str(CFG.MAPS / CFG.LAB_MAP), cube)
        else:
            self._scene(table, cube)
        self._robot()
        self._camera_prims(overview)
        if path_tracing:
            import carb
            s = carb.settings.get_settings()
            s.set("/rtx/rendermode", "PathTracing")
            s.set("/rtx/pathtracing/spp", spp)
            s.set("/rtx/pathtracing/totalSpp", spp)
        self.t = 0.0

    # ---- the scene ----------------------------------------------------------------------
    def _box(self, path, size, center, rgb, collide=True, dynamic_mass=None, yaw_deg=None):
        from pxr import Gf, UsdGeom, UsdPhysics
        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(c)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if yaw_deg is not None:
            x.AddRotateZOp().Set(float(yaw_deg))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if dynamic_mass is not None:
            UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
            UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(dynamic_mass)
        return c.GetPrim()

    def _floor_and_lights(self):
        from pxr import Gf, UsdGeom, UsdLux
        st = self.stage
        self._box("/World/Floor", (40.0, 40.0, 0.02), (0.0, 0.0, -0.01), (0.42, 0.42, 0.40))
        UsdLux.DomeLight.Define(st, "/World/Dome").CreateIntensityAttr(350.0)
        sun = UsdLux.DistantLight.Define(st, "/World/Sun")
        sun.CreateIntensityAttr(900.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, 0.0))
        self.table_z = CFG.MOUNT_IN_BASE[2] + CFG.TABLE_TOP_ABOVE_ARM_BASE

    def start_frame(self, f, s, z=0.0):
        """A point given ahead (f) and left (s) of the start pose -> world."""
        x0, y0, yaw = self.start
        c, n = math.cos(yaw), math.sin(yaw)
        return np.array([x0 + f * c - s * n, y0 + f * n + s * c, z])

    def _lab(self, map_yaml, cube):
        """The lab from its occupancy grid: the bench at the table's height, every other occupied
        cell an obstacle box (runs of cells along a row merged), one static mesh with collision."""
        import cv2, yaml
        from pxr import Gf, UsdGeom, UsdPhysics, Vt
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = cv2.imread(str(Path(map_yaml).parent / meta["image"]), cv2.IMREAD_UNCHANGED)
        H, W = img.shape
        res, (ox, oy) = meta["resolution"], meta["origin"][:2]
        occ = img < int(255 * (1 - meta["occupied_thresh"]))          # trinary: 0 occupied
        # the bench: cells inside its rectangle (in the start frame) are not obstacles
        v, u = np.mgrid[0:H, 0:W]
        x, y = ox + (u + 0.5) * res, oy + (H - 1 - v + 0.5) * res
        x0, y0, yaw = self.start
        f = (x - x0) * math.cos(yaw) + (y - y0) * math.sin(yaw)
        s_ = -(x - x0) * math.sin(yaw) + (y - y0) * math.cos(yaw)
        (f0, f1), (s0, s1) = CFG.LAB_BENCH["f"], CFG.LAB_BENCH["s"]
        window = (f >= f0) & (f <= f1) & (s_ >= s0) & (s_ <= s1)
        bench = occ & window
        # the bench's rectangle: fitted to its cells (cell corners), in the world
        bx, by = x[bench], y[bench]
        corners = np.concatenate([np.stack([bx + dx, by + dy], 1) for dx in (-res / 2, res / 2) for dy in (-res / 2, res / 2)])
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(corners.astype(np.float32))
        if rw < rh:                                 # its long side along the rectangle's x
            rw, rh, ang = rh, rw, ang + 90.0
        # the fit gives the direction; the edges are robust percentiles of the cells along and
        # across it (a few stray cells would otherwise pull an edge out)
        a_ = math.radians(ang)
        along = (bx - cx) * math.cos(a_) + (by - cy) * math.sin(a_)
        across = -(bx - cx) * math.sin(a_) + (by - cy) * math.cos(a_)
        l0, l1 = np.percentile(along, [1, 99]) + [-res / 2, res / 2]
        # its depth from the stretch in front of the start (elsewhere along the aisle things
        # stand in front of it): cells within 0.5 m of the start along the bench
        a_start = (x0 - cx) * math.cos(a_) + (y0 - cy) * math.sin(a_)
        local = np.abs(along - a_start) < 0.5
        d0, d1 = np.percentile(across[local], [5, 95]) + [-res / 2, res / 2]
        cx, cy = (cx + (l0 + l1) / 2 * math.cos(a_) - (d0 + d1) / 2 * math.sin(a_),
                  cy + (l0 + l1) / 2 * math.sin(a_) + (d0 + d1) / 2 * math.cos(a_))
        rw, rh = l1 - l0, d1 - d0
        self.bench_rect = dict(center=(float(cx), float(cy)), length=float(rw), depth=float(rh), yaw_deg=float(ang))
        obstacle = occ & ~window
        pts, idx, n_box = [], [], 0
        faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        for row in range(H):
            cols = np.nonzero(obstacle[row])[0]
            if not len(cols):
                continue
            runs = np.split(cols, np.nonzero(np.diff(cols) > 1)[0] + 1)
            for r in runs:
                xa, xb = ox + r[0] * res, ox + (r[-1] + 1) * res
                yb = oy + (H - row) * res
                ya = yb - res
                base = len(pts)
                pts += [(xx, yy, zz) for xx in (xa, xb) for yy in (ya, yb) for zz in (0.0, CFG.OBSTACLE_HEIGHT)]
                for q in faces:
                    idx += [base + q[0], base + q[1], base + q[2], base + q[0], base + q[2], base + q[3]]
                n_box += 1
        m = UsdGeom.Mesh.Define(self.stage, "/World/Lab/obstacles")
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))
        m.CreateFaceVertexCountsAttr([3] * (len(idx) // 3))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(idx, np.int32)))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.57, 0.62)])
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr().Set("none")
        scan = CFG.SCAN_DIR / CFG.SCAN_MESH
        if self.use_scan and scan.exists():         # the scan is the look; the grid's boxes stay its collision
            self.scan_info = self._scan(scan, CFG.SCAN_Z_OFFSET)
            UsdGeom.Imageable(m.GetPrim()).MakeInvisible()
            UsdGeom.Imageable(self.stage.GetPrimAtPath("/World/Floor")).MakeInvisible()
        else:
            self.scan_info = None
        self.lab_info = dict(occupied=int(occ.sum()), bench_cells=int(bench.sum()), obstacle_boxes=n_box,
                             bench={k: (np.round(v, 3).tolist() if isinstance(v, tuple) else round(v, 3))
                                    for k, v in self.bench_rect.items()})
        # the bench: a top at the table's height on the fitted rectangle, and four legs
        L, D, t, ang = rw, rh, 0.04, math.radians(self.bench_rect["yaw_deg"])
        self._box("/World/Lab/bench/top", (L, D, t), (cx, cy, self.table_z - t / 2), (0.62, 0.48, 0.32),
                  yaw_deg=self.bench_rect["yaw_deg"])
        h = self.table_z - t
        for i, (dl, dd) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            ll, ld = dl * (L / 2 - 0.04), dd * (D / 2 - 0.04)
            self._box(f"/World/Lab/bench/leg{i}", (0.04, 0.04, h),
                      (cx + ll * math.cos(ang) - ld * math.sin(ang), cy + ll * math.sin(ang) + ld * math.cos(ang), h / 2),
                      (0.3, 0.3, 0.3), yaw_deg=self.bench_rect["yaw_deg"])
        yaw_deg = math.degrees(yaw)
        # the cube where the real hover test went: 0.52 m ahead of the arm base, 0.16 m right
        if cube:
            p = self.start_frame(CFG.MOUNT_IN_BASE[0] + 0.52, CFG.MOUNT_IN_BASE[1] - 0.16, self.table_z + 0.02)
            self.cube = self._box("/World/Cube", (0.04, 0.04, 0.04), p, (0.35, 0.75, 0.40), dynamic_mass=0.05,
                                  yaw_deg=yaw_deg)
        else:
            self.cube = None

    def _scan(self, obj_path, z_offset):
        """The textured scan (an OBJ of triangles, v/vt/vn, one texture per material) as one
        USD mesh, looks only: per-corner UVs, a subset and a UsdPreviewSurface per material."""
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        obj = Path(obj_path)
        V, T, faces, fmat, mats, cur = [], [], [], [], {}, None
        textures = {}
        name = None
        for line in (obj.parent / obj.read_text().split("mtllib", 1)[1].split()[0]).read_text().splitlines():
            if line.startswith("newmtl"):
                name = line.split()[1]
            elif line.startswith("map_Kd") and name:
                textures[name] = str(obj.parent / line.split()[1])
        with obj.open() as f:
            for line in f:
                if line.startswith("v "):
                    V.append(line.split()[1:4])
                elif line.startswith("vt "):
                    T.append(line.split()[1:3])
                elif line.startswith("f "):
                    faces.append([c.split("/")[:2] for c in line.split()[1:4]])
                    fmat.append(cur)
                elif line.startswith("usemtl"):
                    cur = mats.setdefault(line.split()[1], len(mats))
        V = np.asarray(V, np.float32)
        V[:, 2] += z_offset
        T = np.asarray(T, np.float32)
        F = np.asarray(faces, np.int64) - 1                       # (n, 3, [v, vt])
        fmat = np.asarray(fmat)
        m = UsdGeom.Mesh.Define(self.stage, "/World/Lab/scan")
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V))
        m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F[:, :, 0].ravel().astype(np.int32)))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set(Vt.Vec2fArray.FromNumpy(T[F[:, :, 1].ravel()]))
        bind = UsdShade.MaterialBindingAPI.Apply(m.GetPrim())
        for mname, mi in mats.items():
            # a subset in the materialBind family (other subsets' bindings are ignored)
            sub = bind.CreateMaterialBindSubset(mname, Vt.IntArray.FromNumpy(np.nonzero(fmat == mi)[0].astype(np.int32)))
            mat = UsdShade.Material.Define(self.stage, f"/World/Lab/Looks/{mname}")
            sh = UsdShade.Shader.Define(self.stage, f"/World/Lab/Looks/{mname}/surface")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
            tex = UsdShade.Shader.Define(self.stage, f"/World/Lab/Looks/{mname}/texture")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(textures[mname]))
            tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            reader = UsdShade.Shader.Define(self.stage, f"/World/Lab/Looks/{mname}/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
        return dict(vertices=len(V), faces=len(F), materials=list(mats))

    def _scene(self, table, cube):
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
        st = self.stage
        self._floor_and_lights()
        if table:                                   # the lab table in front of the robot
            cx, w, d, t = 0.80, 1.20, 0.60, 0.04
            self._box("/World/Table/top", (d, w, t), (cx, 0.0, self.table_z - t / 2), (0.62, 0.48, 0.32))
            for i, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
                h = self.table_z - t
                self._box(f"/World/Table/leg{i}", (0.04, 0.04, h), (cx + dx * (d / 2 - 0.04), dy * (w / 2 - 0.04), h / 2),
                          (0.3, 0.3, 0.3))
        if cube and table:                          # within reach: 0.49 m ahead of the arm base
            self.cube = self._box("/World/Cube", (0.04, 0.04, 0.04), (0.62, -0.10, self.table_z + 0.02),
                                  (0.35, 0.75, 0.40), dynamic_mass=0.05)
        else:
            self.cube = None

    # ---- the robot ----------------------------------------------------------------------
    def _body(self, path, mass, diag=(0.01, 0.01, 0.01)):
        from pxr import Gf, UsdGeom, UsdPhysics
        x = UsdGeom.Xform.Define(self.stage, path)
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        m.CreateMassAttr(float(mass))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag))
        return x.GetPrim()

    def _joint(self, kind, path, body0, body1, axis, kp, kd, max_force):
        from pxr import UsdPhysics
        J = UsdPhysics.PrismaticJoint if kind == "prismatic" else UsdPhysics.RevoluteJoint
        j = J.Define(self.stage, path)
        if body0:
            j.CreateBody0Rel().SetTargets([body0])
        j.CreateBody1Rel().SetTargets([body1])
        j.CreateAxisAttr(axis)
        drive = "linear" if kind == "prismatic" else "angular"
        if kind == "revolute":                      # USD angular drives: per degree
            kp, kd = kp * math.pi / 180.0, kd * math.pi / 180.0
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(kp))
        d.CreateDampingAttr(float(kd))
        d.CreateMaxForceAttr(float(max_force))
        d.CreateTargetPositionAttr(0.0)
        return j.GetPrim()

    def _chassis_looks(self):
        from pxr import Gf, UsdGeom, Vt
        colours = {"bumper": (0.11, 0.11, 0.10), "bottom_plate": (0.35, 0.38, 0.37), "body": (0.63, 0.65, 0.65),
                   "top_plate": (0.35, 0.38, 0.37), "arm_plate": (0.63, 0.65, 0.65)}
        s = CFG.CHASSIS_SCALE_XY
        for name, rgb in colours.items():
            P, idx = read_stl(MESHES / f"{name}.stl")
            P = P * [s, s, 1.0]
            m = UsdGeom.Mesh.Define(self.stage, f"{CHASSIS}/looks/{name}")
            m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
            m.CreateFaceVertexCountsAttr([3] * (len(idx) // 3))
            m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx.astype(np.int32)))
            m.CreateSubdivisionSchemeAttr("none")
            m.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        # the riser from the arm plate to the arm's base (0.43 m), under the arm
        mx, my, mz = CFG.MOUNT_IN_BASE
        h = mz - CFG.CHASSIS_TOP_Z
        self._box(f"{CHASSIS}/riser", (0.16, 0.16, h), (mx, my, CFG.CHASSIS_TOP_Z + h / 2), (0.2, 0.2, 0.22))
        # one collider for the chassis body
        half = 0.241 * s
        self._box(f"{CHASSIS}/collider", (2 * half, 2 * half, CFG.CHASSIS_TOP_Z - 0.027),
                  (0.0, 0.0, (CFG.CHASSIS_TOP_Z + 0.027) / 2), (1, 1, 1))
        from pxr import UsdGeom as G
        G.Imageable(self.stage.GetPrimAtPath(f"{CHASSIS}/collider")).MakeInvisible()

    def _robot(self):
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
        st = self.stage
        UsdGeom.Xform.Define(st, ROBOT)
        anchor = self._body(ANCHOR, 1.0)
        UsdPhysics.ArticulationRootAPI.Apply(anchor)
        art = PhysxSchema.PhysxArticulationAPI.Apply(anchor)
        art.CreateSolverPositionIterationCountAttr(32)
        # as the asset has it: its gripper's links overlap (the linkage jams if they collide)
        art.CreateEnabledSelfCollisionsAttr(False)
        UsdPhysics.FixedJoint.Define(st, f"{ROBOT}/joints/anchor").CreateBody1Rel().SetTargets([ANCHOR])
        self._body(BASE_X, 1.0)
        self._body(BASE_Y, 1.0)
        self._body(CHASSIS, 60.0, diag=(2.0, 2.0, 3.0))
        # the joints apart from the links (a joint's prim name is its DOF name)
        self._joint("prismatic", f"{ROBOT}/joints/base_x", ANCHOR, BASE_X, "X", 2.0e5, 2.0e4, 5.0e3)
        self._joint("prismatic", f"{ROBOT}/joints/base_y", BASE_X, BASE_Y, "Y", 2.0e5, 2.0e4, 5.0e3)
        self._joint("revolute", f"{ROBOT}/joints/base_yaw", BASE_Y, CHASSIS, "Z", 2.0e4, 2.0e3, 2.0e3)
        self._chassis_looks()

        # the arm (with its gripper): the official asset, fixed to the chassis at the mount
        arm = st.DefinePrim(ARM, "Xform")
        arm.GetReferences().AddReference(self.assets + XARM_USD)
        xf = UsdGeom.Xformable(arm)
        op = next((o for o in xf.GetOrderedXformOps() if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
        (op or xf.AddTranslateOp()).Set(Gf.Vec3d(*CFG.MOUNT_IN_BASE))
        root = st.GetPrimAtPath(f"{ARM}/root_joint")
        j = UsdPhysics.FixedJoint(root)
        j.GetBody0Rel().SetTargets([CHASSIS])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*CFG.MOUNT_IN_BASE))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        for p in (root, st.GetPrimAtPath(f"{ARM}/gripper/root_joint")):   # one articulation: ours
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        for name, kp, kd, tq in zip(ARM_JOINTS, ARM_KP, ARM_KD, ARM_TORQUE):
            d = UsdPhysics.DriveAPI.Get(self._find(name), "angular")
            d.GetTypeAttr().Set("force") if d.GetTypeAttr() else d.CreateTypeAttr("force")
            d.GetStiffnessAttr().Set(kp * math.pi / 180.0)
            d.GetDampingAttr().Set(kd * math.pi / 180.0)
            d.GetMaxForceAttr().Set(tq)
        d = UsdPhysics.DriveAPI.Get(self._find(GRIPPER_JOINT), "angular")
        d.GetStiffnessAttr().Set(200.0 * math.pi / 180.0)
        d.GetDampingAttr().Set(10.0 * math.pi / 180.0)
        d.GetMaxForceAttr().Set(30.0)
        # the arm's base sits on the riser: don't let them collide
        fp = UsdPhysics.FilteredPairsAPI.Apply(st.GetPrimAtPath(CHASSIS))
        for link in ("world", "link1"):
            fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{ARM}/{link}"))

    def _find(self, name):
        from pxr import Usd
        for p in Usd.PrimRange(self.stage.GetPrimAtPath(ARM)):
            if p.GetName() == name:
                return p
        raise KeyError(name)

    # ---- cameras ------------------------------------------------------------------------
    def _camera_prims(self, overview):
        from pxr import UsdGeom
        self.lens, self.cam_paths = {}, {}
        for c in self.cameras:
            lens = Lens(c["K"], c["D"], c["size"])
            path = f"{CHASSIS}/{c['name']}_camera"
            cam = UsdGeom.Camera.Define(self.stage, path)
            t_base = c["t_armbase_cam_m"] + np.asarray(CFG.MOUNT_IN_BASE)
            UsdGeom.Xformable(cam).AddTransformOp().Set(cam_matrix_usd(c["R_armbase_cam"], t_base))
            ha, va = lens.apertures()
            cam.CreateFocalLengthAttr(24.0)
            cam.CreateHorizontalApertureAttr(float(ha))
            cam.CreateVerticalApertureAttr(float(va))
            cam.CreateClippingRangeAttr().Set((0.01, 100.0))
            # its housing, behind the lens (a box around the lens would blind it), on a mast
            # down to the chassis top (the real brackets' shapes are not known: approximations)
            hx, hy, hz = c.get("housing", (0.05, 0.05, 0.05))
            body = t_base - np.asarray(c["R_armbase_cam"])[:, 2] * (hx / 2 + 0.005)
            self._box(f"{path}_body", (hx, hy, hz), body, (0.08, 0.08, 0.08), collide=False)
            mast_h = body[2] - hz / 2 - CFG.CHASSIS_TOP_Z
            if mast_h > 0.005:
                self._box(f"{path}_mast", (0.025, 0.025, mast_h), (body[0], body[1], CFG.CHASSIS_TOP_Z + mast_h / 2),
                          (0.15, 0.15, 0.16), collide=False)
            self.lens[c["name"]], self.cam_paths[c["name"]] = lens, path
        if overview:
            from pxr import Gf
            path = "/World/OverviewCam"
            cam = UsdGeom.Camera.Define(self.stage, path)
            eye, at = self.start_frame(-0.9, 1.6, 1.6), self.start_frame(0.45, 0.0, 0.6)   # behind, to the left
            view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*at), Gf.Vec3d(0, 0, 1))
            UsdGeom.Xformable(cam).AddTransformOp().Set(view.GetInverse())
            cam.CreateFocalLengthAttr(18.0)
            cam.CreateHorizontalApertureAttr(20.955)
            cam.CreateVerticalApertureAttr(20.955 * 9 / 16)
            self.cam_paths["overview"] = path

    # ---- runtime ------------------------------------------------------------------------
    def finalize(self):
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.sensors.camera import Camera
        self.cams = {}
        for name, path in self.cam_paths.items():
            res = (self.lens[name].CW, self.lens[name].CH) if name in self.lens else (1280, 720)
            self.cams[name] = Camera(prim_path=path, resolution=res)
        self.world.reset()
        self.depth_views = [c["name"] for c in self.cameras if c.get("depth")]
        for name, c in self.cams.items():
            c.initialize()
            if name in self.depth_views:
                c.add_distance_to_image_plane_to_frame()
        self.art = SingleArticulation(prim_path=ANCHOR, name="tidybot_xarm6")
        self.art.initialize()
        names = list(self.art.dof_names)
        self.dof = {n: i for i, n in enumerate(names)}
        self.i_base = [self.dof[n] for n in BASE_JOINTS]
        self.i_arm = [self.dof[n] for n in ARM_JOINTS]
        self.i_grip = self.dof[GRIPPER_JOINT]
        self.links = RigidPrim(prim_paths_expr=[CHASSIS, self._find("link6").GetPath().pathString], name="frames")
        self.links.initialize()
        self.target = self.art.get_joint_positions().copy()
        self.reset()

    def reset(self, arm_deg=None):
        q = self.art.get_joint_positions().copy()
        q[:] = 0.0
        q[self.i_base] = self.start
        q[self.i_arm] = np.radians(CFG.STOW_JOINTS_DEG if arm_deg is None else arm_deg)
        self.art.set_joint_positions(q)
        self.art.set_joint_velocities(np.zeros_like(q))
        self.target = q.copy()
        self._apply()

    def reset_base(self, pose):
        """Put the base at pose (x, y, yaw) at once, its target there too."""
        q = self.art.get_joint_positions().copy()
        q[self.i_base] = pose
        self.art.set_joint_positions(q)
        self.target[self.i_base] = pose
        self._apply()

    def _apply(self):
        from isaacsim.core.utils.types import ArticulationAction
        idx = self.i_base + self.i_arm + [self.i_grip]
        self.art.apply_action(ArticulationAction(joint_positions=self.target[idx], joint_indices=np.array(idx)))

    def set_base_target(self, x, y, yaw):
        self.target[self.i_base] = (x, y, yaw)
        self._apply()

    def set_arm_target(self, q6):
        self.target[self.i_arm] = q6
        self._apply()

    def set_gripper(self, closed):
        """0 open .. 1 closed."""
        self.target[self.i_grip] = float(np.clip(closed, 0.0, 1.0)) * GRIPPER_CLOSED
        self._apply()

    def step(self, n=1):
        for _ in range(n):
            self.world.step(render=False)
            self.t += PHYSICS_DT

    def base_pose(self):
        q = self.art.get_joint_positions()
        return q[self.i_base].copy()

    def arm_joints(self):
        return self.art.get_joint_positions()[self.i_arm].copy()

    def gripper(self):
        return float(self.art.get_joint_positions()[self.i_grip] / GRIPPER_CLOSED)

    def frames(self):
        """World poses (position, 3x3) of the chassis (base_link) and the flange (link6), from the physics."""
        pos, quat = self.links.get_world_poses()
        out = []
        for p, (w, x, y, z) in zip(np.asarray(pos, float), np.asarray(quat, float)):
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            out.append((p, R))
        return {"base_link": out[0], "flange": out[1]}

    def flange_in_arm_base(self):
        """The flange's pose in the arm base frame (as the xArm controller reports it): (p, R)."""
        f = self.frames()
        (pb, Rb), (pf, Rf) = f["base_link"], f["flange"]
        mount = pb + Rb @ np.asarray(CFG.MOUNT_IN_BASE)
        return Rb.T @ (pf - mount), Rb.T @ Rf

    def world_to_arm(self, p_world):
        """A world point in the arm base frame (where the IK and the xArm controller work)."""
        x, y, yaw = self.base_pose()
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my, mz = CFG.MOUNT_IN_BASE
        d = np.asarray(p_world, float) - [x + c * mx - s * my, y + s * mx + c * my, mz]
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])

    def base_ahead(self, dist):
        """The base pose dist metres ahead of the start pose (same heading)."""
        x0, y0, yaw = self.start
        return x0 + dist * math.cos(yaw), y0 + dist * math.sin(yaw), yaw

    def camera_world(self, name):
        """A configured camera's world pose (OpenCV axes): (R_world_cam, t_world_cam)."""
        c = next(c for c in self.cameras if c["name"] == name)
        pb, Rb = self.frames()["base_link"]
        t = pb + Rb @ (c["t_armbase_cam_m"] + np.asarray(CFG.MOUNT_IN_BASE))
        return Rb @ c["R_armbase_cam"], t

    def project(self, name, p_world):
        """World points -> pixels in a configured camera (with its distortion)."""
        import cv2
        c = next(c for c in self.cameras if c["name"] == name)
        R, t = self.camera_world(name)
        p_cam = (np.asarray(p_world, float).reshape(-1, 3) - t) @ R
        px, _ = cv2.projectPoints(p_cam, np.zeros(3), np.zeros(3), c["K"], c["D"])
        return px.reshape(-1, 2)

    def render(self, views=None, frames=None):
        """Images (RGB uint8): a configured camera as its real lens sees it, the overview as is;
        and "<name>_depth" (float32 m, distance to the image plane) for a camera with depth."""
        import omni.replicator.core as rep
        views = list(self.cams) if views is None else views
        for _ in range(3):
            rep.orchestrator.step(rt_subframes=frames or 4, pause_timeline=False)
            out = {}
            for v in views:
                rgba = self.cams[v].get_rgba()
                if rgba is None or not rgba.size:
                    break
                img = np.asarray(rgba)[:, :, :3].astype(np.uint8)
                out[v] = self.lens[v].distort(img) if v in self.lens else img
                if v in self.depth_views:
                    d = self.cams[v].get_depth()
                    if d is not None and np.asarray(d).size:
                        out[f"{v}_depth"] = self.lens[v].distort(np.asarray(d, np.float32), nearest=True)
            else:
                return out
        return out

    def close(self):
        self.app.close()
