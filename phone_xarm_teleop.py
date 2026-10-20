"""Low-speed, deadman-controlled phone teleoperation for a UFACTORY xArm."""

import argparse
import json
import math
import secrets
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


CONTROL_PERIOD_S = 0.05
COMMAND_TIMEOUT_S = 0.30
CONTROLLER_COMMAND_DURATION_S = 0.08
TRANSLATION_SPEED_MM_S = 180.0
ROTATION_SPEED_DEG_S = 55.0
PHONE_FULL_TRANSLATION_M = 0.10
PHONE_FULL_ROTATION_DEG = 20.0
DEVICE_CAMERA_OFFSET = (0.0, 0.02, -0.04)
POSITION_DEADZONE_M = 0.003
ROTATION_DEADZONE_DEG = 0.8
VISION_LINEAR_DEADZONE_MM_S = 5.0
VISION_ANGULAR_DEADZONE_DEG_S = 0.8
VISION_MAX_FRAME_GAP_S = 0.20
PHONE_FULL_LINEAR_SPEED_MM_S = 100.0
PHONE_FULL_ANGULAR_SPEED_DEG_S = 45.0
VISION_MAX_MEASURED_LINEAR_MM_S = 200.0
VISION_MAX_MEASURED_ANGULAR_DEG_S = 90.0
VISION_VELOCITY_FILTER_ALPHA = 0.22
VISION_IDLE_DECAY = 0.35
VISION_ANGULAR_IDLE_DECAY = 0.10
TRANSLATION_ACCEL_MM_S2 = 350.0
ROTATION_ACCEL_DEG_S2 = 100.0
WRIST_SOFT_LIMIT_DEG = 330.0
WRIST_STOP_LIMIT_DEG = 350.0
WRIST_LIMIT_JOINT_INDICES = (3, 5)  # J4 and J6


def shaped_axis(delta, full_scale, deadzone):
    """Remove tracker noise around zero while retaining the full command range."""
    magnitude = abs(delta)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign(
        min(1.0, (magnitude - deadzone) / max(1e-9, full_scale - deadzone)),
        delta,
    )


def shaped_velocity(value, full_scale, deadzone):
    """Normalize a measured velocity after suppressing slow visual drift."""
    return shaped_axis(value, full_scale, deadzone)


def filter_visual_velocity(previous, measured, deadzone, maximum, idle_decay=VISION_IDLE_DECAY):
    """Reject pose spikes and quickly drain residual velocity when motion stops."""
    measured = max(-maximum, min(maximum, measured))
    if abs(measured) < deadzone:
        return previous * idle_decay
    return previous + VISION_VELOCITY_FILTER_ALPHA * (measured - previous)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, vector):
    rotated = quat_mul(quat_mul(q, (*vector, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return rotated[:3]


def quat_delta_rotvec_deg(current, reference):
    q = quat_mul(current, (-reference[0], -reference[1], -reference[2], reference[3]))
    norm = math.sqrt(sum(value * value for value in q))
    q = tuple(value / norm for value in q)
    if q[3] < 0.0:
        q = tuple(-value for value in q)
    vector_norm = math.sqrt(sum(value * value for value in q[:3]))
    if vector_norm < 1e-9:
        return (0.0, 0.0, 0.0)
    angle_deg = math.degrees(2.0 * math.atan2(vector_norm, q[3]))
    return tuple(value * angle_deg / vector_norm for value in q[:3])


PAGE = r"""<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>
body{font-family:system-ui;margin:0;background:#111;color:#eee;text-align:center;touch-action:none}
#status{padding:12px;background:#333;min-height:24px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;padding:14px;max-width:520px;margin:auto}
button{font-size:24px;padding:22px 6px;background:#555;color:white;border:0;border-radius:12px}.active{background:#d84315}
.label{grid-column:1/-1;color:#aaa;margin-top:8px}small{display:block;padding:12px;color:#bbb}
</style></head><body><div id="status">连接中；按住移动，松手即停</div>
<div class="grid"><div class="label">位置（底座坐标系）</div>
<button data-axis="0" data-sign="1">X+</button><button data-axis="1" data-sign="1">Y+</button><button data-axis="2" data-sign="1">Z+</button>
<button data-axis="0" data-sign="-1">X−</button><button data-axis="1" data-sign="-1">Y−</button><button data-axis="2" data-sign="-1">Z−</button>
<div class="label">姿态</div>
<button data-axis="3" data-sign="1">RX+</button><button data-axis="4" data-sign="1">RY+</button><button data-axis="5" data-sign="1">RZ+</button>
<button data-axis="3" data-sign="-1">RX−</button><button data-axis="4" data-sign="-1">RY−</button><button data-axis="5" data-sign="-1">RZ−</button></div>
<small>平移 20 mm/s　旋转 5°/s　可同时按两个方向</small>
<script>
const token='__SESSION_TOKEN__',status=document.querySelector('#status');
let seq=0,lastOK=0;const active=new Map();
function vector(){const v=[0,0,0,0,0,0];for(const x of active.values())v[x.axis]+=x.sign;return v.map(x=>Math.max(-1,Math.min(1,x)))}
async function send(){const v=vector();try{const r=await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token,seq:++seq,enabled:active.size>0,velocity:v})});if(r.ok){lastOK=Date.now();const d=await r.json();status.textContent=d.mode+' | '+d.state}}
catch(e){status.textContent='连接丢失；机械臂应已停止'}}
function release(e,b){active.delete(e.pointerId);b.classList.remove('active');send()}
for(const b of document.querySelectorAll('button')){b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);active.set(e.pointerId,{axis:+b.dataset.axis,sign:+b.dataset.sign});b.classList.add('active');send()};b.onpointerup=e=>release(e,b);b.onpointercancel=e=>release(e,b);b.onlostpointercapture=e=>release(e,b)}
function stopAll(){active.clear();for(const b of document.querySelectorAll('button'))b.classList.remove('active');send()}
setInterval(send,80);setInterval(()=>{if(Date.now()-lastOK>500)status.textContent='连接超时；机械臂应已停止'},200);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll()});window.addEventListener('blur',stopAll);
</script></body></html>"""


def load_xarm_api():
    try:
        from xarm.wrapper import XArmAPI
        return XArmAPI
    except ImportError:
        # Prefer the persistent vendored checkout.  Keep the /tmp path as a
        # compatibility fallback for older robot images.
        candidates = (
            Path(__file__).resolve().parent / ".vendor" / "xarm-python-sdk",
            Path("/tmp/xarm-python-sdk"),
        )
        for sdk_path in candidates:
            if not sdk_path.is_dir():
                continue
            path_text = str(sdk_path)
            if path_text not in sys.path:
                sys.path.insert(0, path_text)
            try:
                from xarm.wrapper import XArmAPI
                return XArmAPI
            except ImportError:
                continue
        raise RuntimeError(
            "xArm SDK missing; run configure_xarm_sdk.sh or install xarm-python-sdk"
        )


class CommandState:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = False
        self.velocity = [0.0] * 6
        self.last_rx = 0.0
        self.last_seq = -1
        self.pose_ref_pos = None
        self.pose_ref_rot = None
        self.pose_prev_pos = None
        self.pose_prev_rot = None
        self.pose_prev_time = None
        self.filtered_pos_velocity = [0.0] * 3
        self.filtered_rot_velocity = [0.0] * 3
        self.last_pose_log = 0.0
        self.last_motion_log = 0.0
        self.pose_hz_started = time.monotonic()
        self.pose_hz_count = 0
        self.status = "等待手机命令"
        self.stop_requested = False

    def update(self, data):
        seq = int(data.get("seq", -1))
        raw = data.get("velocity", [0.0] * 6)
        if not isinstance(raw, list) or len(raw) != 6:
            raise ValueError("velocity must contain six values")
        velocity = [max(-1.0, min(1.0, float(value))) for value in raw]
        if not all(math.isfinite(value) for value in velocity):
            raise ValueError("velocity must be finite")
        with self.lock:
            if seq <= self.last_seq:
                return False
            self.last_seq = seq
            self.enabled = bool(data.get("enabled", False))
            self.velocity = velocity
            self.last_rx = time.monotonic()
            return True

    def update_pose(self, data):
        seq = int(data.get("seq", -1))
        enabled = bool(data.get("enabled", False))
        pos_data, quat_data = data["position"], data["orientation"]
        rot = (-quat_data["z"], -quat_data["x"], quat_data["y"], quat_data["w"])
        rot_norm = math.sqrt(sum(value * value for value in rot))
        if rot_norm < 1e-9:
            raise ValueError("invalid phone orientation")
        rot = tuple(value / rot_norm for value in rot)
        offset = quat_rotate(rot, DEVICE_CAMERA_OFFSET)
        pos = (-pos_data["z"] + offset[0], -pos_data["x"] + offset[1], pos_data["y"] + offset[2])
        now = time.monotonic()
        with self.lock:
            if seq <= self.last_seq:
                return False
            self.last_seq = seq
            if not enabled:
                self.enabled = False
                self.velocity = [0.0] * 6
                self.pose_ref_pos = None
                self.pose_ref_rot = None
                self.pose_prev_pos = None
                self.pose_prev_rot = None
                self.pose_prev_time = None
                self.filtered_pos_velocity[:] = [0.0] * 3
                self.filtered_rot_velocity[:] = [0.0] * 3
            else:
                if self.pose_ref_pos is None:
                    self.pose_ref_pos = pos
                    self.pose_ref_rot = rot
                if self.pose_prev_pos is None or self.pose_prev_time is None:
                    pos_velocity = [0.0] * 3
                    rot_velocity = [0.0] * 3
                else:
                    dt = now - self.pose_prev_time
                    if dt <= 0.0 or dt > VISION_MAX_FRAME_GAP_S:
                        pos_velocity = [0.0] * 3
                        rot_velocity = [0.0] * 3
                    else:
                        pos_velocity = [(value - previous) * 1000.0 / dt
                                        for value, previous in zip(pos, self.pose_prev_pos)]
                        rot_step_deg = quat_delta_rotvec_deg(rot, self.pose_prev_rot)
                        rot_velocity = [value / dt for value in rot_step_deg]
                self.filtered_pos_velocity = [
                    filter_visual_velocity(previous, measured, VISION_LINEAR_DEADZONE_MM_S,
                                           VISION_MAX_MEASURED_LINEAR_MM_S)
                    for previous, measured in zip(self.filtered_pos_velocity, pos_velocity)
                ]
                self.filtered_rot_velocity = [
                    filter_visual_velocity(previous, measured, VISION_ANGULAR_DEADZONE_DEG_S,
                                           VISION_MAX_MEASURED_ANGULAR_DEG_S,
                                           VISION_ANGULAR_IDLE_DECAY)
                    for previous, measured in zip(self.filtered_rot_velocity, rot_velocity)
                ]
                self.pose_prev_pos = pos
                self.pose_prev_rot = rot
                self.pose_prev_time = now
                pos_command = [shaped_velocity(value, PHONE_FULL_LINEAR_SPEED_MM_S,
                                               VISION_LINEAR_DEADZONE_MM_S)
                               for value in self.filtered_pos_velocity]
                rot_command = [shaped_velocity(value, PHONE_FULL_ANGULAR_SPEED_DEG_S,
                                               VISION_ANGULAR_DEADZONE_DEG_S)
                               for value in self.filtered_rot_velocity]
                self.enabled = True
                self.velocity = pos_command + rot_command
            self.last_rx = time.monotonic()
            return True

    def update_tag_pose(self, data):
        """Accept an AprilTag camera pose already expressed in robot axes.

        The browser sends metres and an xyzw quaternion.  Unlike update_pose,
        this path does not apply WebXR's coordinate conversion because the tag
        page performs that conversion while decomposing the tag homography.
        """
        seq = int(data.get("seq", -1))
        enabled = bool(data.get("enabled", False))
        pos_data, quat_data = data["position"], data["orientation"]
        pos = tuple(float(pos_data[k]) for k in ("x", "y", "z"))
        rot = tuple(float(quat_data[k]) for k in ("x", "y", "z", "w"))
        if not all(math.isfinite(value) for value in pos + rot):
            raise ValueError("tag pose must be finite")
        rot_norm = math.sqrt(sum(value * value for value in rot))
        if rot_norm < 1e-9:
            raise ValueError("invalid tag orientation")
        rot = tuple(value / rot_norm for value in rot)
        now = time.monotonic()
        with self.lock:
            if seq <= self.last_seq:
                return False
            self.last_seq = seq
            if not enabled:
                self.enabled = False
                self.velocity = [0.0] * 6
                self.pose_ref_pos = None
                self.pose_ref_rot = None
                self.pose_prev_pos = None
                self.pose_prev_rot = None
                self.pose_prev_time = None
                self.filtered_pos_velocity[:] = [0.0] * 3
                self.filtered_rot_velocity[:] = [0.0] * 3
                self.pose_hz_started = now
                self.pose_hz_count = 0
            else:
                if self.pose_ref_pos is None:
                    self.pose_ref_pos = pos
                    self.pose_ref_rot = rot
                if self.pose_prev_pos is None or self.pose_prev_time is None:
                    pos_velocity = [0.0] * 3
                    rot_velocity = [0.0] * 3
                else:
                    dt = now - self.pose_prev_time
                    if dt <= 0.0 or dt > VISION_MAX_FRAME_GAP_S:
                        pos_velocity = [0.0] * 3
                        rot_velocity = [0.0] * 3
                    else:
                        pos_velocity = [
                            (value - previous) * 1000.0 / dt
                            for value, previous in zip(pos, self.pose_prev_pos)
                        ]
                        rot_step_deg = quat_delta_rotvec_deg(rot, self.pose_prev_rot)
                        rot_velocity = [value / dt for value in rot_step_deg]
                self.filtered_pos_velocity = [
                    filter_visual_velocity(previous, measured, VISION_LINEAR_DEADZONE_MM_S,
                                           VISION_MAX_MEASURED_LINEAR_MM_S)
                    for previous, measured in zip(self.filtered_pos_velocity, pos_velocity)
                ]
                self.filtered_rot_velocity = [
                    filter_visual_velocity(previous, measured, VISION_ANGULAR_DEADZONE_DEG_S,
                                           VISION_MAX_MEASURED_ANGULAR_DEG_S,
                                           VISION_ANGULAR_IDLE_DECAY)
                    for previous, measured in zip(self.filtered_rot_velocity, rot_velocity)
                ]
                self.pose_prev_pos = pos
                self.pose_prev_rot = rot
                self.pose_prev_time = now
                pos_command = [
                    shaped_velocity(value, PHONE_FULL_LINEAR_SPEED_MM_S, VISION_LINEAR_DEADZONE_MM_S)
                    for value in self.filtered_pos_velocity
                ]
                rot_delta_deg = quat_delta_rotvec_deg(rot, self.pose_ref_rot)
                rot_command = [
                    shaped_velocity(value, PHONE_FULL_ANGULAR_SPEED_DEG_S, VISION_ANGULAR_DEADZONE_DEG_S)
                    for value in self.filtered_rot_velocity
                ]
                self.enabled = True
                self.velocity = pos_command + rot_command
                self.pose_hz_count += 1
                if now - self.last_pose_log >= 0.5:
                    elapsed = max(1e-6, now - self.pose_hz_started)
                    rate = self.pose_hz_count / elapsed
                    delta_mm = tuple((value - reference) * 1000.0
                                     for value, reference in zip(pos, self.pose_ref_pos))
                    quality = data.get("tracking", {})
                    print(
                        "[arm-pose] "
                        f"rate={rate:4.1f}Hz "
                        f"raw_m=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
                        f"delta_mm=({delta_mm[0]:+.1f},{delta_mm[1]:+.1f},{delta_mm[2]:+.1f}) "
                        f"vel_mm_s=({pos_velocity[0]:+.1f},{pos_velocity[1]:+.1f},{pos_velocity[2]:+.1f}) "
                        f"filtered=({self.filtered_pos_velocity[0]:+.1f},{self.filtered_pos_velocity[1]:+.1f},"
                        f"{self.filtered_pos_velocity[2]:+.1f}) "
                        f"rot_deg=({rot_delta_deg[0]:+.1f},{rot_delta_deg[1]:+.1f},{rot_delta_deg[2]:+.1f}) "
                        f"rot_vel=({rot_velocity[0]:+.1f},{rot_velocity[1]:+.1f},{rot_velocity[2]:+.1f}) "
                        f"cmd=({self.velocity[0]:+.3f},{self.velocity[1]:+.3f},{self.velocity[2]:+.3f},"
                        f"{self.velocity[3]:+.3f},{self.velocity[4]:+.3f},{self.velocity[5]:+.3f}) "
                        f"track={quality.get('inliers', '?')}/{quality.get('points', '?')}",
                        flush=True,
                    )
                    self.last_pose_log = now
                    self.pose_hz_started = now
                    self.pose_hz_count = 0
            self.last_rx = time.monotonic()
            return True


def handler_factory(state, motion_enabled, session_token):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != "/":
                self.send_error(404)
                return
            payload = PAGE.replace("__SESSION_TOKEN__", session_token).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_POST(self):
            if self.path != "/cmd":
                self.send_error(404)
                return
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 4096)
                data = json.loads(self.rfile.read(length))
                if not secrets.compare_digest(str(data.get("token", "")), session_token):
                    self.send_error(403, "stale phone session; refresh the page")
                    return
                state.update(data)
                with state.lock:
                    status = state.status
                payload = json.dumps({
                    "mode": "MOTION ENABLED" if motion_enabled else "DRY RUN",
                    "state": status,
                }).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except Exception as exc:
                self.send_error(400, str(exc))

        def log_message(self, *_):
            pass

    return Handler


def scaled_velocity(command):
    velocity = [
        command[0] * TRANSLATION_SPEED_MM_S,
        command[1] * TRANSLATION_SPEED_MM_S,
        command[2] * TRANSLATION_SPEED_MM_S,
        command[3] * ROTATION_SPEED_DEG_S,
        command[4] * ROTATION_SPEED_DEG_S,
        command[5] * ROTATION_SPEED_DEG_S,
    ]
    for start, maximum in ((0, TRANSLATION_SPEED_MM_S), (3, ROTATION_SPEED_DEG_S)):
        magnitude = math.sqrt(sum(value * value for value in velocity[start:start + 3]))
        if magnitude > maximum:
            scale = maximum / magnitude
            velocity[start:start + 3] = [
                value * scale for value in velocity[start:start + 3]
            ]
    return velocity


def dot_product(a, b):
    return sum(left * right for left, right in zip(a, b))


def wrist_limit_scale(angles):
    """Slow Cartesian motion as either cable-limited wrist joint nears ±360°."""
    margin_scale = 1.0
    for index in WRIST_LIMIT_JOINT_INDICES:
        magnitude = abs(angles[index])
        scale = (WRIST_STOP_LIMIT_DEG - magnitude) / (
            WRIST_STOP_LIMIT_DEG - WRIST_SOFT_LIMIT_DEG
        )
        margin_scale = min(margin_scale, max(0.0, min(1.0, scale)))
    return margin_scale


def arm_loop(state, ip, motion_enabled):
    arm = None
    last_sent = None
    smoothed = [0.0] * 6
    previous_angles = None
    blocked_outward_command = None
    try:
        XArmAPI = load_xarm_api()
        arm = XArmAPI(ip, is_radian=False)
        if not arm.connected:
            raise RuntimeError(f"could not connect to xArm at {ip}")
        code, errors = arm.get_err_warn_code()
        if code != 0 or errors[0] != 0:
            raise RuntimeError(f"xArm is faulted: api={code}, error/warn={errors}")
        if motion_enabled:
            for name, code in (
                ("motion_enable", arm.motion_enable(enable=True)),
                ("set_mode(0)", arm.set_mode(0)),
                ("set_state(0)", arm.set_state(0)),
            ):
                if code != 0:
                    raise RuntimeError(f"{name} failed: {code}")
            time.sleep(0.5)
            for name, code in (
                ("set_mode(5)", arm.set_mode(5)),
                ("set_state(0)", arm.set_state(0)),
            ):
                if code != 0:
                    raise RuntimeError(f"{name} failed: {code}")
            time.sleep(0.5)
        with state.lock:
            state.status = "已连接；按住方向键移动" if motion_enabled else "预览模式；不会移动"

        while not state.stop_requested:
            started = time.monotonic()
            with state.lock:
                fresh = started - state.last_rx <= COMMAND_TIMEOUT_S
                active = motion_enabled and state.enabled and fresh
                command = state.velocity[:] if active else [0.0] * 6
            target = scaled_velocity(command)
            if active:
                limits = ([TRANSLATION_ACCEL_MM_S2 * CONTROL_PERIOD_S] * 3
                          + [ROTATION_ACCEL_DEG_S2 * CONTROL_PERIOD_S] * 3)
                for index, limit in enumerate(limits):
                    delta = max(-limit, min(limit, target[index] - smoothed[index]))
                    smoothed[index] += delta
                target = smoothed[:]

                # J4 and J6 are bounded at about ±360 degrees.  Use the
                # controller's live report to identify the Cartesian command
                # that is winding either joint outward.  Slow only that
                # direction near the boundary; a substantially opposite
                # command remains at normal speed so the wrist can unwind.
                angles = arm.angles
                if len(angles) >= 6:
                    outward_near_limit = False
                    inward_near_limit = False
                    if previous_angles is not None:
                        for index in WRIST_LIMIT_JOINT_INDICES:
                            angle = angles[index]
                            delta_angle = angle - previous_angles[index]
                            outward = angle * delta_angle > 0.0
                            if abs(angle) >= WRIST_SOFT_LIMIT_DEG and abs(delta_angle) > 0.002:
                                if outward:
                                    outward_near_limit = True
                                else:
                                    inward_near_limit = True
                    target_energy = dot_product(target, target)
                    learned_outward_direction = False
                    if blocked_outward_command is None and target_energy > 1e-6:
                        if outward_near_limit:
                            blocked_outward_command = target[:]
                            learned_outward_direction = True
                        elif inward_near_limit:
                            # The present command is already unwinding, so its
                            # opposite is the direction that must be limited.
                            blocked_outward_command = [-value for value in target]
                    if learned_outward_direction:
                        print(
                            "[arm-limit] limiting outward motion; "
                            f"J4={angles[3]:+.1f}deg J6={angles[5]:+.1f}deg",
                            flush=True,
                        )
                    reversing = False
                    if blocked_outward_command is not None:
                        safely_inside = all(
                            abs(angles[index]) < WRIST_SOFT_LIMIT_DEG
                            for index in WRIST_LIMIT_JOINT_INDICES
                        )
                        reversing = dot_product(target, blocked_outward_command) < 0.0
                        if safely_inside:
                            blocked_outward_command = None
                    beyond_stop = any(
                        abs(angles[index]) >= WRIST_STOP_LIMIT_DEG
                        for index in WRIST_LIMIT_JOINT_INDICES
                    )
                    if beyond_stop:
                        # Cartesian direction is not a reliable indicator of
                        # individual wrist-joint direction across changing IK
                        # solutions.  Past the hard guard, require explicit
                        # single-joint recovery rather than risk error 23.
                        target = [0.0] * 6
                        smoothed[:] = target
                    elif not reversing:
                        scale = wrist_limit_scale(angles)
                        target = [value * scale for value in target]
                        smoothed[:] = target
                    previous_angles = angles[:]
            else:
                # Deadman release and stale-command stops remain immediate.
                smoothed[:] = [0.0] * 6
                target = smoothed[:]
            if motion_enabled and (active or target != last_sent):
                code = arm.vc_set_cartesian_velocity(
                    target,
                    is_radian=False,
                    is_tool_coord=False,
                    duration=CONTROLLER_COMMAND_DURATION_S,
                )
                if code != 0:
                    raise RuntimeError(f"velocity command failed: {code}")
                last_sent = target
            if active and started - state.last_motion_log >= 0.5:
                print(
                    "[arm-send] base_xyz_mm_s="
                    f"({target[0]:+.1f},{target[1]:+.1f},{target[2]:+.1f}) "
                    "base_rpy_deg_s="
                    f"({target[3]:+.1f},{target[4]:+.1f},{target[5]:+.1f})",
                    flush=True,
                )
                state.last_motion_log = started
            with state.lock:
                if not fresh and state.enabled:
                    state.status = "命令超时，已停止"
                elif not state.enabled:
                    state.status = "待机；按住方向键移动" if motion_enabled else "预览模式；不会移动"
                else:
                    state.status = "运行中" if motion_enabled else "预览命令中"
            time.sleep(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - started)))
    except Exception as exc:
        with state.lock:
            state.enabled = False
            state.velocity = [0.0] * 6
            state.status = f"机械臂控制停止：{exc}"
        print(state.status, flush=True)
    finally:
        if arm is not None:
            if motion_enabled:
                try:
                    arm.vc_set_cartesian_velocity([0.0] * 6, is_radian=False, duration=0.1)
                    arm.set_state(4)
                    arm.motion_enable(enable=False)
                except Exception:
                    pass
            arm.disconnect()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5002)
    parser.add_argument("--xarm-ip", default="192.168.1.204")
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()

    state = CommandState()
    session_token = secrets.token_urlsafe(24)
    server = ThreadingHTTPServer(
        (args.host, args.port), handler_factory(state, args.enable_motion, session_token)
    )
    arm_thread = threading.Thread(
        target=arm_loop, args=(state, args.xarm_ip, args.enable_motion), daemon=True
    )
    arm_thread.start()
    print(
        f"Phone xArm teleop: http://0.0.0.0:{args.port} "
        f"({'MOTION ENABLED' if args.enable_motion else 'DRY RUN'})",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with state.lock:
            state.enabled = False
            state.velocity = [0.0] * 6
            state.stop_requested = True
        server.server_close()
        arm_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
