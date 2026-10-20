"""Low-speed, base-only phone teleoperation with a deadman control.

The default mode is dry-run: it serves the phone UI and logs commands without
opening CAN. Pass --enable-motors only after checking the UI and stop behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
from pathlib import Path
import secrets
import socket
import ssl
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from spark_can.protocol import (
    FAULT_NAMES,
    PARAM_HARD_LIMIT_FWD_EN,
    PARAM_HARD_LIMIT_REV_EN,
    PARAM_IDLE_MODE,
    PARAM_CONTROL_TYPE,
    PARAM_SENSOR_TYPE,
    PARAM_KD_0,
    PARAM_KI_0,
    PARAM_KP_0,
    PARAM_KV_0,
    PARAM_OUTPUT_MAX_0,
    PARAM_OUTPUT_MIN_0,
    PARAM_SMART_CURRENT_CONFIG,
    PARAM_SMART_CURRENT_FREE,
    PARAM_SMART_CURRENT_STALL,
    flag_names,
)
from spark_can.rack import SparkMaxRack
from phone_xarm_teleop import CommandState as XArmCommandState, arm_loop as xarm_loop
from mobile_io_input import mobile_io_loop
from spark_config_values import (
    DRIVE_IDS,
    DRIVE_MOTOR_SIGNS,
    ENCODER_MAGNET_OFFSETS,
    SPARK_CURRENT_DRIVE_A,
    SPARK_CURRENT_LIMIT_RPM,
    SPARK_CURRENT_STEER_A,
    SPARK_IDLE_BRAKE,
    SPARK_KD_DRIVE,
    SPARK_KD_STEER,
    SPARK_KI,
    SPARK_KP_DRIVE,
    SPARK_KP_STEER,
    SPARK_KP_STEER_BY_ID,
    SPARK_KS_DRIVE,
    SPARK_KS_STEER,
    SPARK_KS_STEER_FULL_RPM,
    SPARK_KV,
    SPARK_OUTPUT_MAX,
    STEER_IDS,
)


CONTROL_PERIOD_S = 0.05
# The UI normally sends every 80 ms. Allow several missed Wi-Fi packets; an
# explicit pointer release is still applied immediately, while this timeout is
# only the fallback when the command stream disappears.
COMMAND_TIMEOUT_S = 0.40
MAX_TRANSLATION_MPS = 0.15
MAX_ROTATION_RAD_S = 0.35
MAX_TRANSLATION_ACCEL = 0.20
MAX_ROTATION_ACCEL = 0.60
MAX_STEER_RATE_RAD_S = 0.60
STEER_ERROR_GAIN = 2.0
STEER_DEADBAND_RAD = math.radians(2.0)
MIN_SUPPLY_V = 11.5
SUPPLY_RECOVER_V = 11.8
LOW_VOLTAGE_CONFIRM_S = 0.25
VOLTAGE_RECOVER_CONFIRM_S = 0.50
DRIVE_FULL_STEER_ERROR_RAD = math.radians(7.0)
DRIVE_CUTOFF_STEER_ERROR_RAD = math.radians(12.0)
DRIVE_READY_ERROR_RAD = math.radians(9.0)
DRIVE_REVOKE_ERROR_RAD = math.radians(12.0)
ROTATE_READY_ERROR_RAD = math.radians(12.0)
ROTATE_REVOKE_ERROR_RAD = math.radians(15.0)
DRIVE_READY_STEER_RPM = 15.0
DRIVE_REVOKE_STEER_RPM = 35.0
DRIVE_SETTLE_S = 0.25
DRIVE_RAMP_UP_S = 0.40

OFFSETS = dict(zip(STEER_IDS, ENCODER_MAGNET_OFFSETS))
DRIVE_SIGN = DRIVE_MOTOR_SIGNS
# Initial ground-load tuning: motors 5/7 stalled at -39 RPM with the
# unloaded calibration. Apply only to phone teleop pending live validation.
GROUND_STEER_KS = {1: 0.30, 3: 0.24, 5: (0.46, 0.26), 7: 0.30}
# ID 4 trails the other drive motors in the positive-RPM direction under the
# current chassis load. Raise only that direction's phone-teleop feedforward.
GROUND_DRIVE_KS = {4: (0.090, 0.120)}
KS = {**SPARK_KS_STEER, **GROUND_STEER_KS, **SPARK_KS_DRIVE, **GROUND_DRIVE_KS}
# Motor 5 needs more high-speed assistance but overshoots at low speed, so
# ramp its compensation over a wider command range.
KS_FULL_RPM = {**SPARK_KS_STEER_FULL_RPM, 1: 25.0, 5: 40.0}
H_X = (0.190150, 0.190150, -0.190150, -0.190150)
H_Y = (-0.170150, 0.170150, 0.170150, -0.170150)
RADIUS_M = 0.0508
N_STEER = 32.0 / 15.0 * 60.0 / 10.0
N_R1 = 50.0 / 14.0
N_DRIVE = N_R1 * (19.0 / 25.0) * (45.0 / 15.0)


PAGE_BUTTON_CONTROL = r"""<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>
body{font-family:system-ui;margin:0;background:#111;color:#eee;text-align:center;touch-action:none}
#status{padding:10px;background:#333;min-height:24px}.tabs{display:flex;gap:8px;padding:10px}.tabs button{flex:1;padding:12px;font-size:20px}
#pad{width:min(78vw,390px);height:min(78vw,390px);margin:14px auto;border-radius:50%;background:#262626;border:3px solid #777;position:relative}
#knob{width:72px;height:72px;border-radius:50%;background:#42a5f5;position:absolute;left:50%;top:50%;transform:translate(-50%,-50%)}
button{font-size:25px;padding:16px 24px;margin:5px;background:#555;color:white;border:0;border-radius:12px}.selected{background:#1976d2}.active{background:#d84315}
.hidden{display:none}.arm-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;padding:10px;max-width:520px;margin:auto}.arm-grid button{margin:0;padding:21px 4px}
.label{grid-column:1/-1;color:#aaa;margin-top:6px}small{display:block;padding:10px;color:#bbb}
</style></head><body><div id="status">连接中；松手即停</div>
<div class="tabs"><button id="baseTab" class="selected">底盘</button><button id="armTab">机械臂</button></div>
<section id="basePanel"><div id="pad"><div id="knob"></div></div><button id="left">↺</button><button id="right">↻</button><small>摇杆：平移　按钮：原地旋转</small></section>
<section id="armPanel" class="hidden"><div class="arm-grid"><div class="label">位置（底座坐标系）</div>
<button data-axis="0" data-sign="1">X+</button><button data-axis="1" data-sign="1">Y+</button><button data-axis="2" data-sign="1">Z+</button>
<button data-axis="0" data-sign="-1">X−</button><button data-axis="1" data-sign="-1">Y−</button><button data-axis="2" data-sign="-1">Z−</button>
<div class="label">姿态</div><button data-axis="3" data-sign="1">RX+</button><button data-axis="4" data-sign="1">RY+</button><button data-axis="5" data-sign="1">RZ+</button>
<button data-axis="3" data-sign="-1">RX−</button><button data-axis="4" data-sign="-1">RY−</button><button data-axis="5" data-sign="-1">RZ−</button></div><small>平移 20 mm/s　旋转 5°/s</small></section>
<script>
const token='__SESSION_TOKEN__',status=document.querySelector('#status'),pad=document.querySelector('#pad'),knob=document.querySelector('#knob');
let mode='base',x=0,y=0,rot=0,held=false,lastOK=0,baseSeq=0,armSeq=0;const armActive=new Map();
function setPad(e){const r=pad.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;let dx=(e.clientX-cx)/(r.width*.4),dy=(e.clientY-cy)/(r.height*.4),m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m}x=dx;y=-dy;knob.style.left=(50+dx*40)+'%';knob.style.top=(50+dy*40)+'%'}
function stopBase(){held=false;x=0;y=0;rot=0;knob.style.left='50%';knob.style.top='50%';for(const id of ['left','right'])document.querySelector('#'+id).classList.remove('active');sendBase()}
pad.onpointerdown=e=>{held=true;pad.setPointerCapture(e.pointerId);setPad(e)};pad.onpointermove=e=>{if(held)setPad(e)};pad.onpointerup=stopBase;pad.onpointercancel=stopBase;
for(const [id,v] of [['left',1],['right',-1]]){const b=document.querySelector('#'+id);b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);rot=v;b.classList.add('active');sendBase()};b.onpointerup=b.onpointercancel=()=>{rot=0;b.classList.remove('active');sendBase()}}
async function post(path,data){try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});if(r.ok){lastOK=Date.now();const d=await r.json();status.textContent=d.mode+' | '+d.state}}catch(e){status.textContent='连接丢失；机器人应已停止'}}
function sendBase(){return post('/cmd',{token,seq:++baseSeq,enabled:mode==='base'&&(held||rot!==0),x,y,rotation:rot})}
function armVector(){const v=[0,0,0,0,0,0];for(const a of armActive.values())v[a.axis]+=a.sign;return v.map(n=>Math.max(-1,Math.min(1,n)))}
function sendArm(){return post('/arm_cmd',{token,seq:++armSeq,enabled:mode==='arm'&&armActive.size>0,velocity:armVector()})}
function releaseArm(e,b){armActive.delete(e.pointerId);b.classList.remove('active');sendArm()}
for(const b of document.querySelectorAll('#armPanel button')){b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);armActive.set(e.pointerId,{axis:+b.dataset.axis,sign:+b.dataset.sign});b.classList.add('active');sendArm()};b.onpointerup=e=>releaseArm(e,b);b.onpointercancel=e=>releaseArm(e,b);b.onlostpointercapture=e=>releaseArm(e,b)}
function stopArm(){armActive.clear();for(const b of document.querySelectorAll('#armPanel button'))b.classList.remove('active');sendArm()}
function select(next){stopBase();stopArm();mode=next;document.querySelector('#basePanel').classList.toggle('hidden',mode!=='base');document.querySelector('#armPanel').classList.toggle('hidden',mode!=='arm');document.querySelector('#baseTab').classList.toggle('selected',mode==='base');document.querySelector('#armTab').classList.toggle('selected',mode==='arm')}
document.querySelector('#baseTab').onclick=()=>select('base');document.querySelector('#armTab').onclick=()=>select('arm');
setInterval(()=>{if(mode==='base')sendBase();else sendArm()},80);setInterval(()=>{if(Date.now()-lastOK>500)status.textContent='连接超时；机器人应已停止'},200);
function stopAll(){stopBase();stopArm()}document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll()});window.addEventListener('blur',stopAll);
</script></body></html>"""

# Combined control: WebXR phone pose drives the xArm while the on-screen pad
# drives the base. Holding the pad (including at its center) is the shared
# deadman, so both devices can be controlled at the same time.
PAGE = r"""<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>body{font-family:system-ui;margin:0;background:#111;color:#eee;text-align:center;touch-action:none}#status,#xrStatus{padding:8px;background:#333}#xr,#armHold{font-size:18px;padding:12px 20px;margin:6px;color:white;border:0;border-radius:10px}#xr{background:#1976d2}#armHold{background:#8b1e1e}#pad{width:min(66vw,330px);height:min(66vw,330px);margin:7px auto;border-radius:50%;background:#262626;border:3px solid #777;position:relative}#knob{width:72px;height:72px;border-radius:50%;background:#42a5f5;position:absolute;left:50%;top:50%;transform:translate(-50%,-50%)}button.turn{font-size:28px;padding:14px 32px;margin:5px;background:#555;color:white;border:0;border-radius:12px}.active{background:#d84315!important}small{display:block;padding:8px;color:#bbb}</style></head>
<body><div id="status">XR Browser：底盘和机械臂可同时控制</div><button id="xr">启动 ARKit</button><button id="armHold" disabled>按住控制机械臂</button><div id="xrStatus">AR未启动</div><div id="pad"><div id="knob"></div></div><button class="turn" id="left">↺</button><button class="turn" id="right">↻</button><small>红色按钮：机械臂；摇杆：底盘；松手即停</small>
<script>
const token='__SESSION_TOKEN__',status=document.querySelector('#status'),xrStatus=document.querySelector('#xrStatus'),pad=document.querySelector('#pad'),knob=document.querySelector('#knob'),armHold=document.querySelector('#armHold');let x=0,y=0,rot=0,held=false,armHeld=false,lastOK=0,baseSeq=0,poseSeq=0,xrSession=null,xrRef=null,armSending=false,pendingArmPose=null;
function baseActive(){return held||rot!==0}function setPad(e){const r=pad.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;let dx=(e.clientX-cx)/(r.width*.4),dy=(e.clientY-cy)/(r.height*.4),m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m}x=dx;y=-dy;knob.style.left=(50+dx*40)+'%';knob.style.top=(50+dy*40)+'%'}
async function post(path,data){try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});if(r.ok){lastOK=Date.now();const d=await r.json();status.textContent=d.mode+' | '+d.state}}catch(e){status.textContent='连接丢失；机器人应已停止'}}
function sendBase(){post('/cmd',{token,seq:++baseSeq,enabled:baseActive(),x,y,rotation:rot})}function stopBase(){held=false;x=0;y=0;rot=0;knob.style.left='50%';knob.style.top='50%';for(const id of ['left','right'])document.querySelector('#'+id).classList.remove('active');sendBase()}
async function flushArmPose(){if(armSending||!pendingArmPose)return;const data=pendingArmPose;pendingArmPose=null;armSending=true;await post('/arm_pose',data);armSending=false;if(pendingArmPose)flushArmPose()}
function queueArmPose(data){pendingArmPose=data;flushArmPose()}
function stopArm(){queueArmPose({token,seq:++poseSeq,enabled:false,position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}})}
pad.onpointerdown=e=>{held=true;pad.setPointerCapture(e.pointerId);setPad(e);sendBase()};pad.onpointermove=e=>{if(held)setPad(e)};pad.onpointerup=stopBase;pad.onpointercancel=stopBase;
for(const [id,v] of [['left',1],['right',-1]]){const b=document.querySelector('#'+id);b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);rot=v;b.classList.add('active');sendBase()};b.onpointerup=b.onpointercancel=stopBase}
async function startXR(){if(!navigator.xr){xrStatus.textContent='请使用 XR Browser / XRViewer';return}xrSession=await navigator.xr.requestSession('immersive-ar',{requiredFeatures:['dom-overlay'],domOverlay:{root:document.body}});const c=document.createElement('canvas'),gl=c.getContext('webgl',{xrCompatible:true});xrSession.updateRenderState({baseLayer:new XRWebGLLayer(xrSession,gl)});xrRef=await xrSession.requestReferenceSpace('local');xrSession.addEventListener('end',()=>{xrSession=null;armHold.disabled=true;xrStatus.textContent='AR已停止';releaseArm()});armHold.disabled=false;xrStatus.textContent='ARKit 位姿已连接';xrSession.requestAnimationFrame(onXRFrame)}
function onXRFrame(t,frame){if(!xrSession)return;xrSession.requestAnimationFrame(onXRFrame);const pose=frame.getViewerPose(xrRef);if(!pose)return;const p=pose.transform.inverse.position,q=pose.transform.inverse.orientation;queueArmPose({token,seq:++poseSeq,enabled:armHeld,position:{x:p.x,y:p.y,z:p.z},orientation:{x:q.x,y:q.y,z:q.z,w:q.w}})}
armHold.onpointerdown=e=>{e.preventDefault();armHold.setPointerCapture(e.pointerId);armHeld=true;armHold.classList.add('active')};function releaseArm(){armHeld=false;armHold.classList.remove('active');stopArm()}armHold.onpointerup=releaseArm;armHold.onpointercancel=releaseArm;armHold.onlostpointercapture=releaseArm;
document.querySelector('#xr').onclick=()=>startXR().catch(e=>xrStatus.textContent='AR启动失败: '+e);setInterval(sendBase,80);setInterval(()=>{if(Date.now()-lastOK>500)status.textContent='连接超时；机器人应已停止'},200);function stopAll(){armHeld=false;stopBase();stopArm()}document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll()});window.addEventListener('blur',stopAll);
</script></body></html>"""

STATIC_DIR = Path(__file__).resolve().parent / "static"
APRILTAG_PAGE = (STATIC_DIR / "apriltag-teleop.html").read_text()
NATURAL_PAGE = (STATIC_DIR / "natural-teleop.html").read_text()


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


class CommandState:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = False
        self.command = (0.0, 0.0, 0.0)
        self.last_rx = 0.0
        self.last_seq = -1
        self.status = "等待手机命令"
        self.stop_requested = False

    def update(self, data):
        seq = int(data.get("seq", -1))
        enabled = bool(data.get("enabled", False))
        x = clamp(float(data.get("x", 0.0)), -1.0, 1.0)
        y = clamp(float(data.get("y", 0.0)), -1.0, 1.0)
        rotation = clamp(float(data.get("rotation", 0.0)), -1.0, 1.0)
        with self.lock:
            # HTTP requests may finish out of order because both the browser
            # and ThreadingHTTPServer allow concurrent requests. Never let an
            # older held=true packet override a newer release packet.
            if seq <= self.last_seq:
                return False
            self.last_seq = seq
            self.enabled = enabled
            # Screen x points right; chassis +Y points left (+X forward).
            self.command = (y * MAX_TRANSLATION_MPS, -x * MAX_TRANSLATION_MPS, rotation * MAX_ROTATION_RAD_S)
            self.last_rx = time.monotonic()
            return True


def handler_factory(state, arm_state, dry_run, arm_enabled, session_token, pose_source):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            if self.path == "/":
                if pose_source == "natural":
                    page = NATURAL_PAGE
                elif pose_source == "apriltag":
                    page = APRILTAG_PAGE
                else:
                    page = PAGE
                payload = page.replace("__SESSION_TOKEN__", session_token).encode()
                content_type = "text/html; charset=utf-8"
            elif pose_source == "apriltag" and self.path.startswith("/static/apriltag/"):
                relative = self.path.removeprefix("/static/apriltag/")
                path = (STATIC_DIR / "apriltag" / relative).resolve()
                root = (STATIC_DIR / "apriltag").resolve()
                if root not in path.parents or not path.is_file():
                    self.send_error(404)
                    return
                payload = path.read_bytes()
                content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            elif pose_source == "natural" and self.path.startswith("/static/natural/"):
                relative = self.path.removeprefix("/static/natural/")
                path = (STATIC_DIR / "natural" / relative).resolve()
                root = (STATIC_DIR / "natural").resolve()
                if root not in path.parents or not path.is_file():
                    self.send_error(404)
                    return
                payload = path.read_bytes()
                content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_POST(self):
            if self.path not in ("/cmd", "/arm_cmd", "/arm_pose", "/arm_tag_pose"):
                self.send_error(404)
                return
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 4096)
                data = json.loads(self.rfile.read(length))
                if not secrets.compare_digest(str(data.get("token", "")), session_token):
                    self.send_error(403, "stale phone session; refresh the page")
                    return
                if self.path == "/cmd":
                    state.update(data)
                    mode = "BASE DRY RUN" if dry_run else "BASE ENABLED"
                    current_status = state.status
                elif self.path == "/arm_cmd":
                    arm_state.update(data)
                    mode = "XARM ENABLED" if arm_enabled else "XARM DISABLED"
                    current_status = arm_state.status if arm_enabled else "启动时需要 --enable-xarm"
                elif self.path == "/arm_pose":
                    arm_state.update_pose(data)
                    mode = "XARM IMU"
                    current_status = arm_state.status if arm_enabled else "启动时需要 --enable-xarm"
                else:
                    arm_state.update_tag_pose(data)
                    mode = "XARM VISION"
                    current_status = arm_state.status if arm_enabled else "启动时需要 --enable-xarm"
                payload = json.dumps({"mode": mode, "state": current_status}).encode()
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


def angle_rad(device, voltage):
    return wrap((voltage / 3.3 - OFFSETS[device]) * 2.0 * math.pi)


def spark_params(device_id):
    is_steer = device_id in STEER_IDS
    kp = SPARK_KP_STEER_BY_ID[device_id] if is_steer else SPARK_KP_DRIVE
    kd = SPARK_KD_STEER if is_steer else SPARK_KD_DRIVE
    amps = SPARK_CURRENT_STEER_A if is_steer else SPARK_CURRENT_DRIVE_A
    return (
        (PARAM_CONTROL_TYPE, "uint", 1), (PARAM_SENSOR_TYPE, "uint", 1),
        (PARAM_KP_0, "float", kp), (PARAM_KI_0, "float", 0.0),
        (PARAM_KD_0, "float", kd), (PARAM_KV_0, "float", SPARK_KV),
        (PARAM_OUTPUT_MIN_0, "float", -SPARK_OUTPUT_MAX), (PARAM_OUTPUT_MAX_0, "float", SPARK_OUTPUT_MAX),
        (PARAM_SMART_CURRENT_STALL, "uint", amps), (PARAM_SMART_CURRENT_FREE, "uint", amps),
        (PARAM_SMART_CURRENT_CONFIG, "uint", SPARK_CURRENT_LIMIT_RPM), (PARAM_IDLE_MODE, "uint", SPARK_IDLE_BRAKE),
        (PARAM_HARD_LIMIT_FWD_EN, "uint", 0), (PARAM_HARD_LIMIT_REV_EN, "uint", 0),
    )


def print_diagnostics(rack, targets, enabled, status):
    """Compare caster alignment and commanded/measured shaft speeds."""
    now = time.monotonic()
    rpms = dict(zip(rack.device_ids, rack.get_rpms()))
    rows = [f"Base diagnostics enabled={enabled} state={status}"]
    for idx, (steer_id, drive_id) in enumerate(zip(STEER_IDS, DRIVE_IDS)):
        current = angle_rad(steer_id, rack.telemetry[steer_id].analog_voltage)
        target = targets[idx]
        error = math.nan if target is None else math.degrees(wrap(target - current))
        rows.append(f"  caster {idx + 1}: angle={math.degrees(current):.1f}deg error={error:.1f}deg")
        for device in (steer_id, drive_id):
            tel = rack.telemetry[device]
            rows.append(
                f"    motor {device}: target={rpms[device]:.1f}rpm actual={tel.velocity_rpm:.1f}rpm"
                f" duty={tel.applied:.3f} current={tel.current:.2f}A"
                f" speed_age={now - tel.status0_rx_s:.2f}s"
            )
    print("\n".join(rows), flush=True)


def motor_loop(state, port, configure, diagnostics=False):
    if diagnostics:
        print(f"Phone steering compensation (V): { {i: KS[i] for i in STEER_IDS} }", flush=True)
    rack = SparkMaxRack.open(prefer="slcan", port=port, period_s=CONTROL_PERIOD_S,
                            max_rpm=270.0, ks_volts=KS, ks_full_rpm=KS_FULL_RPM)
    armed = False
    velocity = [0.0, 0.0, 0.0]
    targets = [None] * 4
    drive_ready = False
    drive_settle_since = None
    drive_ramp = 0.0
    low_voltage_since = None
    voltage_recover_since = None
    voltage_fault_latched = False
    last_diagnostics = 0.0
    try:
        if configure:
            state.status = "正在配置 SPARK，保持松手"
            for device in range(1, 9):
                for param in spark_params(device):
                    rack.write_parameter(device, *param, timeout=0.20, attempts=12)
        else:
            state.status = "使用当前 SPARK 配置，保持松手"
        rack.start()
        time.sleep(1.5)
        # Ignore anything sent while the controllers were being configured.
        # The operator must release and press again after the UI reports ready.
        with state.lock:
            state.enabled = False
            state.command = (0.0, 0.0, 0.0)
            state.last_rx = 0.0
            state.status = "配置完成；请松手后重新操作"
        while not state.stop_requested:
            started = time.monotonic()
            rack.check_health()
            now = time.monotonic()
            with state.lock:
                fresh = now - state.last_rx <= COMMAND_TIMEOUT_S
                enabled = state.enabled and fresh
                requested = state.command if enabled else (0.0, 0.0, 0.0)
            healthy = all(math.isfinite(t.last_rx_s) and now - t.last_rx_s <= 0.25 for t in rack.telemetry.values())
            voltage = min((t.voltage for t in rack.telemetry.values() if math.isfinite(t.voltage)), default=0.0)
            faults = {i: t.faults & ~(1 << 3) for i, t in rack.telemetry.items() if t.faults & ~(1 << 3)}

            # Ignore brief motor-start voltage dips, then latch a real low
            # voltage event until the supply has recovered with margin.
            if voltage_fault_latched:
                low_voltage_since = None
                if voltage >= SUPPLY_RECOVER_V:
                    if voltage_recover_since is None:
                        voltage_recover_since = now
                    elif now - voltage_recover_since >= VOLTAGE_RECOVER_CONFIRM_S:
                        voltage_fault_latched = False
                        voltage_recover_since = None
                else:
                    voltage_recover_since = None
            else:
                voltage_recover_since = None
                if voltage < MIN_SUPPLY_V:
                    if low_voltage_since is None:
                        low_voltage_since = now
                    elif now - low_voltage_since >= LOW_VOLTAGE_CONFIRM_S:
                        voltage_fault_latched = True
                        low_voltage_since = None
                else:
                    low_voltage_since = None

            if not healthy or faults:
                enabled = False
                requested = (0.0, 0.0, 0.0)
                state.status = f"保护停机 feedback={healthy} voltage={voltage:.1f} faults={faults}"
            elif voltage_fault_latched:
                enabled = False
                requested = (0.0, 0.0, 0.0)
                state.status = f"低电压保护 voltage={voltage:.1f} V，等待恢复到 {SUPPLY_RECOVER_V:.1f} V"
            elif not fresh:
                state.status = "命令超时，已停机"
            elif not state.enabled:
                state.status = f"待机 {voltage:.1f} V"
            else:
                state.status = f"运行 {voltage:.1f} V"

            limits = (MAX_TRANSLATION_ACCEL * CONTROL_PERIOD_S,) * 2 + (MAX_ROTATION_ACCEL * CONTROL_PERIOD_S,)
            for i in range(3):
                velocity[i] += clamp(requested[i] - velocity[i], -limits[i], limits[i])
            if not enabled:
                velocity[:] = (0.0, 0.0, 0.0)
                drive_ready = False
                drive_settle_since = None
                drive_ramp = 0.0
                for device in range(1, 9):
                    rack.set_velocity(device, 0.0)
                if armed:
                    rack.disarm()
                    armed = False
            else:
                vx, vy, wz = velocity
                plans = []
                for idx, (steer_id, drive_id) in enumerate(zip(STEER_IDS, DRIVE_IDS)):
                    wheel_vx = vx - wz * H_Y[idx]
                    wheel_vy = vy + wz * H_X[idx]
                    speed = math.hypot(wheel_vx, wheel_vy)
                    current = angle_rad(steer_id, rack.telemetry[steer_id].analog_voltage)
                    if speed > 0.005:
                        target = math.atan2(wheel_vy, wheel_vx)
                        error = wrap(target - current)
                        if abs(error) > math.pi / 2:
                            target = wrap(target + math.pi)
                            speed = -speed
                        targets[idx] = target
                    target = current if targets[idx] is None else targets[idx]
                    error = wrap(target - current)
                    if abs(error) <= STEER_DEADBAND_RAD:
                        steer_rate = 0.0
                    else:
                        # Remove the deadband before applying gain so the
                        # command approaches zero smoothly near alignment.
                        active_error = error - math.copysign(STEER_DEADBAND_RAD, error)
                        steer_rate = clamp(
                            STEER_ERROR_GAIN * active_error,
                            -MAX_STEER_RATE_RAD_S,
                            MAX_STEER_RATE_RAD_S,
                        )
                    plans.append((steer_id, drive_id, steer_rate, speed, abs(error)))

                # Use the worst caster error for every drive wheel so aligned
                # wheels cannot push the chassis and back-drive a caster that
                # is still turning. Differential steering compensation remains.
                max_error = max(plan[4] for plan in plans)
                max_steer_rpm = max(abs(rack.telemetry[plan[0]].velocity_rpm) for plan in plans)
                pure_rotation = math.hypot(vx, vy) < 0.01 and abs(wz) > 0.01
                ready_error = ROTATE_READY_ERROR_RAD if pure_rotation else DRIVE_READY_ERROR_RAD
                revoke_error = ROTATE_REVOKE_ERROR_RAD if pure_rotation else DRIVE_REVOKE_ERROR_RAD
                full_error = math.radians(10.0) if pure_rotation else DRIVE_FULL_STEER_ERROR_RAD
                cutoff_error = ROTATE_REVOKE_ERROR_RAD if pure_rotation else DRIVE_CUTOFF_STEER_ERROR_RAD
                if drive_ready and (
                    max_error >= revoke_error
                    or max_steer_rpm >= DRIVE_REVOKE_STEER_RPM
                ):
                    drive_ready = False
                    drive_settle_since = None
                    drive_ramp = 0.0
                if not drive_ready:
                    settled = (
                        max_error <= ready_error
                        and max_steer_rpm <= DRIVE_READY_STEER_RPM
                    )
                    if settled:
                        if drive_settle_since is None:
                            drive_settle_since = now
                        elif now - drive_settle_since >= DRIVE_SETTLE_S:
                            drive_ready = True
                    else:
                        drive_settle_since = None

                alignment_scale = clamp(
                    (cutoff_error - max_error) / (cutoff_error - full_error),
                    0.0,
                    1.0,
                )
                if drive_ready:
                    drive_ramp = min(1.0, drive_ramp + CONTROL_PERIOD_S / DRIVE_RAMP_UP_S)
                else:
                    drive_ramp = 0.0
                drive_scale = alignment_scale * drive_ramp

                for steer_id, drive_id, steer_rate, speed, _ in plans:
                    steer_rpm = N_STEER * steer_rate * 60.0 / (2.0 * math.pi)
                    wheel_rate = drive_scale * speed / RADIUS_M
                    drive_rpm = DRIVE_SIGN[drive_id] * (N_R1 * steer_rate - N_DRIVE * wheel_rate) * 60.0 / (2.0 * math.pi)
                    rack.set_velocity(steer_id, steer_rpm)
                    rack.set_velocity(drive_id, drive_rpm)
                if not armed:
                    rack.arm()
                    armed = True
            if diagnostics and now - last_diagnostics >= 1.0:
                print_diagnostics(rack, targets, enabled, state.status)
                last_diagnostics = now
            time.sleep(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - started)))
    finally:
        rack.disarm()
        rack.stop()


def guarded_motor_loop(state, port, configure, diagnostics=False):
    try:
        motor_loop(state, port, configure, diagnostics)
    except Exception as exc:
        with state.lock:
            state.enabled = False
            state.command = (0.0, 0.0, 0.0)
            state.status = f"启动失败：{exc}"
        print(f"Motor control stopped: {exc!r}", flush=True)


def local_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 1))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--can-port", default="/dev/ttyACM0")
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--enable-xarm", action="store_true")
    parser.add_argument("--mobile-io", action="store_true", help="Use HEBI Mobile I/O as the base and xArm input")
    parser.add_argument("--mobile-io-family", default="HEBI")
    parser.add_argument("--mobile-io-name", default="mobileIO")
    parser.add_argument("--xarm-ip", default="192.168.1.204")
    parser.add_argument(
        "--pose-source", choices=("natural", "apriltag", "webxr"), default="natural"
    )
    parser.add_argument("--tls-cert", help="HTTPS certificate; required by Safari camera access")
    parser.add_argument("--tls-key", help="HTTPS private key")
    parser.add_argument("--configure-sparks", action="store_true")
    parser.add_argument("--diagnostics", action="store_true", help="Print caster angles and motor target/actual RPM once per second")
    args = parser.parse_args()
    state = CommandState()
    arm_state = XArmCommandState()
    session_token = secrets.token_urlsafe(24)
    # Bind the web port before opening CAN. If another instance is running,
    # fail without starting a second motor-control thread.
    server = ThreadingHTTPServer(
        (args.host, args.port),
        handler_factory(
            state, arm_state, not args.enable_motors, args.enable_xarm,
            session_token, args.pose_source,
        ),
    )
    if bool(args.tls_cert) != bool(args.tls_key):
        parser.error("--tls-cert and --tls-key must be supplied together")
    if args.tls_cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.tls_cert, args.tls_key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
    motor_thread = None
    arm_thread = None
    mobile_io_thread = None
    if args.enable_motors:
        motor_thread = threading.Thread(
            target=guarded_motor_loop,
            args=(state, args.can_port, args.configure_sparks, args.diagnostics),
            daemon=True,
        )
        motor_thread.start()
    if args.enable_xarm:
        arm_thread = threading.Thread(
            target=xarm_loop, args=(arm_state, args.xarm_ip, True), daemon=True
        )
        arm_thread.start()
    if args.mobile_io:
        mobile_io_thread = threading.Thread(
            target=mobile_io_loop,
            args=(state, arm_state, args.mobile_io_family, args.mobile_io_name),
            daemon=True,
        )
        mobile_io_thread.start()
    print(
        f"Phone robot teleop: {'https' if args.tls_cert else 'http'}://{local_ip()}:{args.port} "
        f"(base={'ENABLED' if args.enable_motors else 'DRY RUN'}, "
        f"xarm={'ENABLED' if args.enable_xarm else 'DISABLED'})"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with state.lock:
            state.enabled = False
            state.stop_requested = True
        with arm_state.lock:
            arm_state.enabled = False
            arm_state.velocity = [0.0] * 6
            arm_state.stop_requested = True
        server.server_close()
        if motor_thread is not None:
            motor_thread.join(timeout=2.0)
        if arm_thread is not None:
            arm_thread.join(timeout=3.0)
        if mobile_io_thread is not None:
            mobile_io_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
