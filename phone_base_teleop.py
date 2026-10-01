"""Low-speed, base-only phone teleoperation with a deadman control.

The default mode is dry-run: it serves the phone UI and logs commands without
opening CAN. Pass --enable-motors only after checking the UI and stop behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import socket
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
from spark_config_values import (
    DRIVE_IDS,
    DRIVE_MOTOR_SIGNS,
    ENCODER_MAGNET_OFFSETS,
    SPARK_KS_DRIVE,
    SPARK_KS_STEER,
    SPARK_KS_STEER_FULL_RPM,
    STEER_IDS,
)


CONTROL_PERIOD_S = 0.05
COMMAND_TIMEOUT_S = 0.25
MAX_TRANSLATION_MPS = 0.15
MAX_ROTATION_RAD_S = 0.35
MAX_TRANSLATION_ACCEL = 0.20
MAX_ROTATION_ACCEL = 0.60
MIN_SUPPLY_V = 11.5
DRIVE_FULL_STEER_ERROR_RAD = math.radians(5.0)
DRIVE_CUTOFF_STEER_ERROR_RAD = math.radians(10.0)

OFFSETS = dict(zip(STEER_IDS, ENCODER_MAGNET_OFFSETS))
DRIVE_SIGN = DRIVE_MOTOR_SIGNS
KS = {**SPARK_KS_STEER, **SPARK_KS_DRIVE}
KS_FULL_RPM = SPARK_KS_STEER_FULL_RPM
H_X = (0.190150, 0.190150, -0.190150, -0.190150)
H_Y = (-0.170150, 0.170150, 0.170150, -0.170150)
RADIUS_M = 0.0508
N_STEER = 32.0 / 15.0 * 60.0 / 10.0
N_R1 = 50.0 / 14.0
N_DRIVE = N_R1 * (19.0 / 25.0) * (45.0 / 15.0)


PAGE = r"""<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>
body{font-family:system-ui;margin:0;background:#111;color:#eee;text-align:center;touch-action:none}
#status{padding:12px;background:#333}#pad{width:min(82vw,420px);height:min(82vw,420px);margin:24px auto;
border-radius:50%;background:#262626;border:3px solid #777;position:relative}
#knob{width:72px;height:72px;border-radius:50%;background:#42a5f5;position:absolute;left:50%;top:50%;
transform:translate(-50%,-50%)}button{font-size:28px;padding:18px 34px;margin:8px;background:#555;color:white;border:0;border-radius:12px}
.active{background:#d84315}small{display:block;padding:12px;color:#bbb}
</style></head><body><div id="status">连接中；松手即停</div><div id="pad"><div id="knob"></div></div>
<button id="left">↺</button><button id="right">↻</button><small>摇杆：前后/左右平移　按钮：原地旋转</small>
<script>
const sessionToken='__SESSION_TOKEN__';
const pad=document.querySelector('#pad'),knob=document.querySelector('#knob'),status=document.querySelector('#status');
let x=0,y=0,rot=0,held=false,lastOK=0;
function setPad(e){const r=pad.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;
 let dx=(e.clientX-cx)/(r.width*.4),dy=(e.clientY-cy)/(r.height*.4),m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m}
 x=dx;y=-dy;knob.style.left=(50+dx*40)+'%';knob.style.top=(50+dy*40)+'%'}
function stopPad(){held=false;x=0;y=0;knob.style.left='50%';knob.style.top='50%';send()}
pad.onpointerdown=e=>{held=true;pad.setPointerCapture(e.pointerId);setPad(e)};pad.onpointermove=e=>{if(held)setPad(e)};
pad.onpointerup=stopPad;pad.onpointercancel=stopPad;
for(const [id,v] of [['left',1],['right',-1]]){const b=document.querySelector('#'+id);
 b.onpointerdown=e=>{rot=v;b.classList.add('active');send()};b.onpointerup=b.onpointercancel=()=>{rot=0;b.classList.remove('active');send()}}
async function send(){try{const r=await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({token:sessionToken,enabled:held||rot!==0,x:x,y:y,rotation:rot})});if(r.ok){lastOK=Date.now();const d=await r.json();status.textContent=d.mode+' | '+d.state}}
catch(e){status.textContent='连接丢失；机器人应已停止'}}
setInterval(send,80);setInterval(()=>{if(Date.now()-lastOK>400)status.textContent='连接超时；机器人应已停止'},200);
document.addEventListener('visibilitychange',()=>{if(document.hidden){stopPad();rot=0;send()}});
</script></body></html>"""


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
        self.status = "等待手机命令"
        self.stop_requested = False

    def update(self, data):
        enabled = bool(data.get("enabled", False))
        x = clamp(float(data.get("x", 0.0)), -1.0, 1.0)
        y = clamp(float(data.get("y", 0.0)), -1.0, 1.0)
        rotation = clamp(float(data.get("rotation", 0.0)), -1.0, 1.0)
        with self.lock:
            self.enabled = enabled
            self.command = (y * MAX_TRANSLATION_MPS, x * MAX_TRANSLATION_MPS, rotation * MAX_ROTATION_RAD_S)
            self.last_rx = time.monotonic()


def handler_factory(state, dry_run, session_token):
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
                payload = json.dumps({"mode": "DRY RUN" if dry_run else "MOTORS ENABLED", "state": state.status}).encode()
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


def spark_params(is_steer):
    kp = 0.0004 if is_steer else 0.0003
    kd = 3.0e-5 if is_steer else 0.0
    amps = 20 if is_steer else 10
    return (
        (PARAM_CONTROL_TYPE, "uint", 1), (PARAM_SENSOR_TYPE, "uint", 1),
        (PARAM_KP_0, "float", kp), (PARAM_KI_0, "float", 0.0),
        (PARAM_KD_0, "float", kd), (PARAM_KV_0, "float", 0.0022),
        (PARAM_OUTPUT_MIN_0, "float", -0.25), (PARAM_OUTPUT_MAX_0, "float", 0.25),
        (PARAM_SMART_CURRENT_STALL, "uint", amps), (PARAM_SMART_CURRENT_FREE, "uint", amps),
        (PARAM_SMART_CURRENT_CONFIG, "uint", 20000), (PARAM_IDLE_MODE, "uint", 1),
        (PARAM_HARD_LIMIT_FWD_EN, "uint", 0), (PARAM_HARD_LIMIT_REV_EN, "uint", 0),
    )


def motor_loop(state, port, configure):
    rack = SparkMaxRack.open(prefer="slcan", port=port, period_s=CONTROL_PERIOD_S,
                            max_rpm=270.0, ks_volts=KS, ks_full_rpm=KS_FULL_RPM)
    armed = False
    velocity = [0.0, 0.0, 0.0]
    targets = [None] * 4
    try:
        if configure:
            state.status = "正在配置 SPARK，保持松手"
            for device in range(1, 9):
                for param in spark_params(device in STEER_IDS):
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
            if not healthy or voltage < MIN_SUPPLY_V or faults:
                enabled = False
                requested = (0.0, 0.0, 0.0)
                state.status = f"保护停机 feedback={healthy} voltage={voltage:.1f} faults={faults}"
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
                for device in range(1, 9):
                    rack.set_velocity(device, 0.0)
                if armed:
                    rack.disarm()
                    armed = False
            else:
                vx, vy, wz = velocity
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
                    steer_rate = clamp(error, -0.32, 0.32)
                    steer_rpm = N_STEER * steer_rate * 60.0 / (2.0 * math.pi)
                    abs_error = abs(error)
                    drive_scale = clamp(
                        (DRIVE_CUTOFF_STEER_ERROR_RAD - abs_error)
                        / (DRIVE_CUTOFF_STEER_ERROR_RAD - DRIVE_FULL_STEER_ERROR_RAD),
                        0.0,
                        1.0,
                    )
                    wheel_rate = drive_scale * speed / RADIUS_M
                    drive_rpm = DRIVE_SIGN[drive_id] * (N_R1 * steer_rate - N_DRIVE * wheel_rate) * 60.0 / (2.0 * math.pi)
                    rack.set_velocity(steer_id, steer_rpm)
                    rack.set_velocity(drive_id, drive_rpm)
                if not armed:
                    rack.arm()
                    armed = True
            time.sleep(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - started)))
    finally:
        rack.disarm()
        rack.stop()


def guarded_motor_loop(state, port, configure):
    try:
        motor_loop(state, port, configure)
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
    parser.add_argument("--configure-sparks", action="store_true")
    args = parser.parse_args()
    state = CommandState()
    session_token = secrets.token_urlsafe(24)
    # Bind the web port before opening CAN. If another instance is running,
    # fail without starting a second motor-control thread.
    server = ThreadingHTTPServer(
        (args.host, args.port), handler_factory(state, not args.enable_motors, session_token)
    )
    motor_thread = None
    if args.enable_motors:
        motor_thread = threading.Thread(
            target=guarded_motor_loop,
            args=(state, args.can_port, args.configure_sparks),
            daemon=True,
        )
        motor_thread.start()
    print(f"Phone base teleop: http://{local_ip()}:{args.port} ({'MOTORS ENABLED' if args.enable_motors else 'DRY RUN'})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with state.lock:
            state.enabled = False
            state.stop_requested = True
        server.server_close()
        if motor_thread is not None:
            motor_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
