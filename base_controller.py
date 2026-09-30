# Author: Jimmy Wu
# Date: October 2024
#
# IMPORTANT: The mobile base has very strong motors and should be closely
# monitored during operation, especially if any changes are made to this code.
# When testing out new code, consider starting out with very low speeds and
# turning the mobile base on its side to elevate the casters off the ground.
# SPARK smart-current limits and the slot-0 output clamp limit motor torque.
#
# If you will be developing on this controller rather than just running it,
# additional safety measures are highly recommended:
# - Always use an enabling device when operating the robot
# - Install safety barriers around the robot operating area
# - Install a physical e-stop button on the robot
# - Install bump sensors on the robot to detect collisions
#
# References:
# - https://github.com/google/powered-caster-vehicle

import argparse
import math
import queue
import threading
import time
from enum import Enum
import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface
from threadpoolctl import threadpool_limits
from constants import h_x, h_y, ENCODER_MAGNET_OFFSETS, ANALOG_FULL_V, MIN_SUPPLY_VOLTAGE
from constants import SPARK_KV, SPARK_KS_STEER, SPARK_KS_DRIVE, SPARK_KP, SPARK_KI, SPARK_KD_STEER, SPARK_KD_DRIVE
from constants import SPARK_OUTPUT_MAX, SPARK_CURRENT_STEER_A, SPARK_CURRENT_DRIVE_A
from constants import SPARK_CURRENT_LIMIT_RPM, SPARK_IDLE_BRAKE, SPARK_MAX_RPM
from constants import POLICY_CONTROL_PERIOD
from spark_can.protocol import (
    PARAM_IDLE_MODE, PARAM_KP_0, PARAM_KI_0, PARAM_KD_0, PARAM_KV_0,
    PARAM_OUTPUT_MIN_0, PARAM_OUTPUT_MAX_0,
    PARAM_SMART_CURRENT_STALL, PARAM_SMART_CURRENT_FREE, PARAM_SMART_CURRENT_CONFIG,
    PARAM_HARD_LIMIT_FWD_EN, PARAM_HARD_LIMIT_REV_EN,
)
from spark_can.protocol import FAULT_NAMES, WARNING_NAMES, flag_names
from spark_can.rack import SparkMaxRack
from utils import create_pid_file

# Vehicle
# slcan on the SH-C31G cannot carry a 250 Hz command stream for 8 SPARKs.
# The loop is still one fixed-period thread with Ruckig(dt=CONTROL_PERIOD).
# Raise CONTROL_FREQ on SocketCAN; do not split planning and CAN rates.
CONTROL_FREQ = 50                    # 50 Hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQ  # 20 ms
NUM_CASTERS = 4
STALE_S = 0.25

# Caster
b_x = -0.014008                  # Caster offset (m)
b_y = -0.003753                  # Lateral caster offset (m)
r = 0.0508                       # Wheel radius (m)
N_s = 32.0 / 15.0 * 60.0 / 10.0  # Steer gear ratio
N_r1 = 50.0 / 14.0               # Drive gear ratio (1st stage)
N_r2 = 19.0 / 25.0               # Drive gear ratio (2nd stage)
N_w = 45.0 / 15.0                # Wheel gear ratio
N_r1_r2_w = N_r1 * N_r2 * N_w
N_s_r2_w = N_s * N_r2 * N_w
TWO_PI = 2 * math.pi

def _spark_params(is_steer):
    kd = SPARK_KD_STEER if is_steer else SPARK_KD_DRIVE
    amps = SPARK_CURRENT_STEER_A if is_steer else SPARK_CURRENT_DRIVE_A
    return [
        (PARAM_KP_0, 'float', SPARK_KP),
        (PARAM_KI_0, 'float', SPARK_KI),
        (PARAM_KD_0, 'float', kd),
        (PARAM_KV_0, 'float', SPARK_KV),
        (PARAM_OUTPUT_MIN_0, 'float', -SPARK_OUTPUT_MAX),
        (PARAM_OUTPUT_MAX_0, 'float', SPARK_OUTPUT_MAX),
        (PARAM_SMART_CURRENT_STALL, 'uint', amps),
        (PARAM_SMART_CURRENT_FREE, 'uint', amps),
        (PARAM_SMART_CURRENT_CONFIG, 'uint', SPARK_CURRENT_LIMIT_RPM),
        (PARAM_IDLE_MODE, 'uint', SPARK_IDLE_BRAKE),
        (PARAM_HARD_LIMIT_FWD_EN, 'uint', 0),
        (PARAM_HARD_LIMIT_REV_EN, 'uint', 0),
    ]

class Motor:
    def __init__(self, num, rack):
        self.num = num
        self.is_steer = num % 2 != 0  # Odd num motors are steer motors
        self.rack = rack

    def _telemetry(self):
        return self.rack.telemetry[self.num]

    def get_position(self):
        position_rot = self._telemetry().position_rot
        if not math.isfinite(position_rot):
            return 0.0
        return TWO_PI * position_rot

    def get_velocity(self):
        velocity_rpm = self._telemetry().velocity_rpm
        if not math.isfinite(velocity_rpm):
            return 0.0
        return velocity_rpm * TWO_PI / 60.0

    def set_velocity(self, velocity):
        # velocity is motor rad/s. SPARK velocity setpoint is RPM.
        self.rack.set_velocity(self.num, velocity * 60.0 / TWO_PI)

    def set_neutral(self):
        self.rack.set_velocity(self.num, 0.0)

class Caster:
    def __init__(self, num, rack):
        self.num = num
        self.steer_motor = Motor(2 * self.num - 1, rack)
        self.drive_motor = Motor(2 * self.num, rack)
        self.encoder_offset = ENCODER_MAGNET_OFFSETS[self.num - 1]

    def get_steer_position(self):
        # Thrifty analog replaces CANcoder. Offset is in turns.
        voltage = self.steer_motor._telemetry().analog_voltage
        if not math.isfinite(voltage):
            return 0.0
        return TWO_PI * (voltage / ANALOG_FULL_V - self.encoder_offset)

    def get_positions(self):
        steer_motor_pos = self.steer_motor.get_position()
        drive_motor_pos = self.drive_motor.get_position()
        steer_pos = self.get_steer_position()
        drive_pos = steer_motor_pos / N_s_r2_w - drive_motor_pos / N_r1_r2_w
        return steer_pos, drive_pos

    def get_velocities(self):
        steer_motor_vel = self.steer_motor.get_velocity()
        drive_motor_vel = self.drive_motor.get_velocity()
        steer_vel = steer_motor_vel / N_s
        drive_vel = steer_motor_vel / N_s_r2_w - drive_motor_vel / N_r1_r2_w
        return steer_vel, drive_vel

    def set_velocities(self, steer_vel, drive_vel):
        self.steer_motor.set_velocity(N_s * steer_vel)
        self.drive_motor.set_velocity(N_r1 * steer_vel - N_r1_r2_w * drive_vel)

    def set_neutral(self):
        self.steer_motor.set_neutral()
        self.drive_motor.set_neutral()

class CommandType(Enum):
    POSITION = 'position'
    VELOCITY = 'velocity'

# Currently only used for velocity commands
class FrameType(Enum):
    GLOBAL = 'global'
    LOCAL = 'local'

class Vehicle:
    def __init__(self, max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79)):
        self.max_vel = np.array(max_vel)
        self.max_accel = np.array(max_accel)

        # Use PID file to enforce single instance
        create_pid_file('tidybot2-base-controller')

        self.rack = SparkMaxRack.open(
            prefer='slcan',
            period_s=CONTROL_PERIOD,
            max_duty=SPARK_OUTPUT_MAX,
            max_rpm=SPARK_MAX_RPM,
            command='velocity',
            # Odd CAN IDs are steer motors
            ks_volts={num: SPARK_KS_STEER if num % 2 != 0 else SPARK_KS_DRIVE for num in range(1, 2 * NUM_CASTERS + 1)},
        )
        try:
            for num in range(1, 2 * NUM_CASTERS + 1):
                is_steer = num % 2 != 0
                for param_id, kind, value in _spark_params(is_steer):
                    self.rack.write_parameter(num, param_id, kind, value)
                role = 'steer' if is_steer else 'drive'
                print(f'Configured SPARK {num} ({role})')
            self.rack.start()
            self._check_supply_voltage()
        except Exception:
            self.rack.stop()
            self.rack = None
            raise

        # Initialize casters
        self.casters = [Caster(num, self.rack) for num in range(1, NUM_CASTERS + 1)]

        # Joint space
        num_motors = 2 * NUM_CASTERS
        self.q = np.zeros(num_motors)
        self.dq = np.zeros(num_motors)
        self.tau = np.zeros(num_motors)

        # Operational space (global frame)
        num_dofs = 3  # (x, y, theta)
        self.x = np.zeros(num_dofs)
        self.dx = np.zeros(num_dofs)

        # C matrix relating operational space velocities to joint velocities
        self.C = np.zeros((num_motors, num_dofs))
        self.C_steer = self.C[::2]
        self.C_drive = self.C[1::2]

        # C_p matrix relating operational space velocities to wheel velocities at the contact points
        self.C_p = np.zeros((num_motors, num_dofs))
        self.C_p_steer = self.C_p[::2]
        self.C_p_drive = self.C_p[1::2]
        self.C_p_steer[:, :2] = [1.0, 0.0]
        self.C_p_drive[:, :2] = [0.0, 1.0]

        # C_qp^# matrix relating joint velocities to operational space velocities
        self.C_pinv = np.zeros((num_motors, num_dofs))
        self.CpT_Cqinv = np.zeros((num_dofs, num_motors))
        self.CpT_Cqinv_steer = self.CpT_Cqinv[:, ::2]
        self.CpT_Cqinv_drive = self.CpT_Cqinv[:, 1::2]

        # OTG (online trajectory generation)
        # Note: It would be better to couple x and y using polar coordinates
        self.otg = Ruckig(num_dofs, CONTROL_PERIOD)
        self.otg_inp = InputParameter(num_dofs)
        self.otg_out = OutputParameter(num_dofs)
        self.otg_res = Result.Working
        self.otg_inp.max_velocity = self.max_vel
        self.otg_inp.max_acceleration = self.max_accel

        # Control loop
        self.command_queue = queue.Queue(1)
        self.control_loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_loop_running = False

    def _check_supply_voltage(self):
        deadline = time.time() + 1.0
        supply_voltage = float('nan')
        while time.time() < deadline:
            supply_voltage = self.rack.telemetry[1].voltage
            if math.isfinite(supply_voltage):
                break
            time.sleep(0.02)
        print(f'Motor supply voltage: {supply_voltage:.2f} V')
        if not math.isfinite(supply_voltage) or supply_voltage < MIN_SUPPLY_VOLTAGE:
            raise Exception('Motor supply voltage is too low. Please charge the battery.')

    def _sensors_ok(self):
        now = time.monotonic()
        for caster in self.casters:
            for motor in (caster.steer_motor, caster.drive_motor):
                tel = motor._telemetry()
                if not math.isfinite(tel.last_rx_s) or now - tel.last_rx_s > STALE_S:
                    return False
            if not math.isfinite(caster.steer_motor._telemetry().analog_voltage):
                return False
        return True

    def update_state(self):
        # Joint positions and velocities. Telemetry is filled by the CAN thread.
        for i, caster in enumerate(self.casters):
            self.q[2*i : 2*i + 2] = caster.get_positions()
            self.dq[2*i : 2*i + 2] = caster.get_velocities()

        q_steer = self.q[::2]
        s = np.sin(q_steer)
        c = np.cos(q_steer)

        # C matrix
        self.C_steer[:, 0] = s / b_x
        self.C_steer[:, 1] = -c / b_x
        self.C_steer[:, 2] = (-h_x*c - h_y*s) / b_x - 1.0
        self.C_drive[:, 0] = c/r - b_y*s / (b_x*r)
        self.C_drive[:, 1] = s/r + b_y*c / (b_x*r)
        self.C_drive[:, 2] = (h_x*s - h_y*c) / r + b_y * (h_x*c + h_y*s) / (b_x*r)

        # C_p matrix
        self.C_p_steer[:, 2] = -b_x*s - b_y*c - h_y
        self.C_p_drive[:, 2] = b_x*c - b_y*s + h_x

        # C_qp^# matrix
        self.CpT_Cqinv_steer[0] = b_x*s + b_y*c
        self.CpT_Cqinv_steer[1] = -b_x*c + b_y*s
        self.CpT_Cqinv_steer[2] = b_x * (-h_x*c - h_y*s - b_x) + b_y * (h_x*s - h_y*c - b_y)
        self.CpT_Cqinv_drive[0] = r * c
        self.CpT_Cqinv_drive[1] = r * s
        self.CpT_Cqinv_drive[2] = r * (h_x*s - h_y*c - b_y)
        with threadpool_limits(limits=1, user_api='blas'):  # Prevent excessive CPU usage
            self.C_pinv = np.linalg.solve(self.C_p.T @ self.C_p, self.CpT_Cqinv)

        # Odometry
        dx_local = self.C_pinv @ self.dq
        theta_avg = self.x[2] + 0.5 * dx_local[2] * CONTROL_PERIOD
        R = np.array([
            [math.cos(theta_avg), -math.sin(theta_avg), 0.0],
            [math.sin(theta_avg), math.cos(theta_avg), 0.0],
            [0.0, 0.0, 1.0]
        ])
        self.dx = R @ dx_local
        self.x += self.dx * CONTROL_PERIOD

    def start_control(self):
        if self.control_loop_thread is None:
            print('To initiate a new control loop, please create a new instance of Vehicle.')
            return
        self.control_loop_running = True
        self.control_loop_thread.start()

    def stop_control(self):
        self.control_loop_running = False
        if self.control_loop_thread is not None and self.control_loop_thread.is_alive():
            self.control_loop_thread.join()
        self.control_loop_thread = None
        if self.rack is not None:
            self.rack.stop()
            self.rack = None

    def control_loop(self):
        # Set real-time scheduling policy
        try:
            import os
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO)))
        except (PermissionError, AttributeError, OSError):
            print('Failed to set real-time scheduling policy, please edit /etc/security/limits.d/99-realtime.conf')

        disable_motors = True
        last_command_time = time.time()
        last_step_time = time.time()
        while self.control_loop_running:
            # Maintain the desired control frequency
            while time.time() - last_step_time < CONTROL_PERIOD:
                time.sleep(0.0001)
            curr_time = time.time()
            step_time = curr_time - last_step_time
            last_step_time = curr_time
            if step_time > 2 * CONTROL_PERIOD:
                print(f'Warning: Step time {1000 * step_time:.3f} ms in {self.__class__.__name__} control_loop')

            # Update state
            self.update_state()

            # Global to local frame conversion
            theta = self.x[2]
            R = np.array([
                [math.cos(theta), math.sin(theta), 0.0],
                [-math.sin(theta), math.cos(theta), 0.0],
                [0.0, 0.0, 1.0]
            ])

            # Check for new command
            if not self.command_queue.empty():
                command = self.command_queue.get()
                last_command_time = time.time()
                target = command['target']

                # Velocity command
                if command['type'] == CommandType.VELOCITY:
                    if command['frame'] == FrameType.LOCAL:
                        target = R.T @ target
                    self.otg_inp.control_interface = ControlInterface.Velocity
                    self.otg_inp.target_velocity = np.clip(target, -self.max_vel, self.max_vel)

                # Position command
                elif command['type'] == CommandType.POSITION:
                    self.otg_inp.control_interface = ControlInterface.Position
                    self.otg_inp.target_position = target
                    self.otg_inp.target_velocity = np.zeros_like(self.dx)

                self.otg_res = Result.Working
                disable_motors = False

            # Maintain current pose if command stream is disrupted
            if time.time() - last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
                self.otg_inp.target_position = self.otg_out.new_position
                self.otg_inp.target_velocity = np.zeros_like(self.dx)
                self.otg_inp.current_velocity = self.dx  # Set this to prevent lurch when command stream resumes
                self.otg_res = Result.Working
                disable_motors = True

            # Slow down base during caster flip
            # Note: At low speeds, this section can be disabled for smoother movement
            if np.max(np.abs(self.dq[::2])) > 12.56:  # Steer joint speed > 720 deg/s
                if self.otg_inp.control_interface == ControlInterface.Position:
                    self.otg_inp.target_position = self.otg_out.new_position
                elif self.otg_inp.control_interface == ControlInterface.Velocity:
                    self.otg_inp.target_velocity = np.zeros_like(self.dx)

            # Update OTG
            if self.otg_res == Result.Working:
                self.otg_inp.current_position = self.x
                self.otg_res = self.otg.update(self.otg_inp, self.otg_out)
                self.otg_out.pass_to_input(self.otg_inp)

            if not self._sensors_ok():
                disable_motors = True

            if disable_motors:
                for i in range(NUM_CASTERS):
                    self.casters[i].set_neutral()
                self.rack.disarm()

            else:
                # FRC heartbeat replaces phoenix6.unmanaged.feed_enable
                self.rack.arm()

                # Operational space velocity
                dx_d = self.otg_out.new_velocity
                dx_d_local = R @ dx_d

                # Joint velocities
                dq_d = self.C @ dx_d_local

                # Send motor velocity commands
                for i in range(NUM_CASTERS):
                    self.casters[i].set_velocities(dq_d[2*i], dq_d[2*i + 1])

    def _enqueue_command(self, command_type, target, frame=None):
        if self.command_queue.full():
            print('Warning: Command queue is full. Is control loop running?')
        else:
            command = {'type': command_type, 'target': target}
            if frame is not None:
                command['frame'] = FrameType(frame)
            self.command_queue.put(command, block=False)

    def set_target_velocity(self, velocity, frame='global'):
        self._enqueue_command(CommandType.VELOCITY, velocity, frame)

    def set_target_position(self, position):
        self._enqueue_command(CommandType.POSITION, position)

    def get_encoder_offsets(self):
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if all(math.isfinite(c.steer_motor._telemetry().analog_voltage) for c in self.casters):
                break
            time.sleep(0.02)
        offsets = []
        for caster in self.casters:
            voltage = caster.steer_motor._telemetry().analog_voltage
            if not math.isfinite(voltage):
                raise RuntimeError(f'SPARK {caster.steer_motor.num} analog voltage is missing')
            offsets.append(voltage / ANALOG_FULL_V)
        print(f'ENCODER_MAGNET_OFFSETS = [{", ".join(f"{offset:.4f}" for offset in offsets)}]')
        return offsets

    def spin_one(self, device_id, rpm, seconds):
        """Low-RPM velocity check on one SPARK. Does not start the chassis loop."""
        if device_id not in self.rack.device_ids:
            raise ValueError(f'CAN ID {device_id} is not in {self.rack.device_ids}')
        print(f'Velocity test SPARK {device_id} at {rpm:.1f} RPM for {seconds:.2f}s; other motors stay at 0')
        self._print_status1(device_id, 'before')
        self.rack.arm()
        self.rack.set_velocity(device_id, rpm)
        t0 = time.time()
        try:
            while time.time() - t0 < seconds:
                tel = self.rack.telemetry[device_id]
                now = time.monotonic()
                print(
                    f'V={tel.voltage:5.2f} I={tel.current:5.2f} applied={tel.applied:7.3f} '
                    f'RPM={tel.velocity_rpm:8.1f} pos={tel.position_rot:8.3f} temp={tel.temp_c:4.0f} '
                    f'hb={int(tel.heartbeat_locked)} '
                    f'faults={flag_names(tel.faults, FAULT_NAMES)} '
                    f'warn={flag_names(tel.warnings, WARNING_NAMES)} '
                    f's0_age={1000 * (now - tel.status0_rx_s):4.0f}ms '
                    f's1_age={1000 * (now - tel.status1_rx_s):4.0f}ms'
                )
                time.sleep(0.1)
        finally:
            self.rack.disarm()
            time.sleep(0.2)
            tel = self.rack.telemetry[device_id]
            print(f'stopped RPM={tel.velocity_rpm:8.1f} I={tel.current:5.2f}')
            self._print_status1(device_id, 'after')

    def _print_status1(self, device_id, label):
        # Telemetry faults default to 0, so a missing Status 1 would read as "no fault".
        deadline = time.monotonic() + 0.5
        tel = self.rack.telemetry[device_id]
        while not math.isfinite(tel.status1_rx_s) and time.monotonic() < deadline:
            time.sleep(0.02)
        if not math.isfinite(tel.status1_rx_s):
            print(f'Status 1 {label}: NOT RECEIVED, faults unknown')
            return
        print(
            f'Status 1 {label} ({1000 * (time.monotonic() - tel.status1_rx_s):.0f} ms old): '
            f'faults={flag_names(tel.faults, FAULT_NAMES)} '
            f'warn={flag_names(tel.warnings, WARNING_NAMES)} '
            f'sticky_faults={flag_names(tel.sticky_faults, FAULT_NAMES)} '
            f'sticky_warn={flag_names(tel.sticky_warnings, WARNING_NAMES)}'
        )

    def fit_kv(self, device_id, rpms, seconds, settle=0.7):
        """Hold several low setpoints and fit bus volts = kV * RPM + V0 from steady state.

        The fit uses applied * bus voltage against measured RPM, so it is valid even
        when the velocity loop misses its setpoint. SPARK kV has no static term; V0
        is the friction the SPARK_KV feedforward cannot cover.
        """
        if device_id not in self.rack.device_ids:
            raise ValueError(f'CAN ID {device_id} is not in {self.rack.device_ids}')
        if seconds <= settle + 0.2:
            raise ValueError('seconds per point must exceed settle time by at least 0.2 s')
        print(f'kV fit SPARK {device_id} at {rpms} RPM, {seconds:.1f}s each; other motors stay at 0')
        self._print_status1(device_id, 'before')
        points = []
        self.rack.arm()
        try:
            for rpm in rpms:
                self.rack.set_velocity(device_id, rpm)
                t0 = time.time()
                volts, speeds, amps, analog = [], [], [], []
                while time.time() - t0 < seconds:
                    tel = self.rack.telemetry[device_id]
                    analog.append(tel.analog_voltage)
                    if time.time() - t0 >= settle:
                        volts.append(tel.applied * tel.voltage)
                        speeds.append(tel.velocity_rpm)
                        amps.append(tel.current)
                    time.sleep(CONTROL_PERIOD)
                v, w = float(np.mean(volts)), float(np.mean(speeds))
                points.append((v, w))
                ana = [a for a in analog if math.isfinite(a)]
                ana_txt = f'ana {min(ana):.3f}..{max(ana):.3f} V' if ana else 'ana missing'
                print(
                    f'set {rpm:6.1f} RPM: measured {w:7.1f} (sd {np.std(speeds):5.1f}) RPM, '
                    f'{v:6.3f} V, I {np.mean(amps):5.2f} A, {ana_txt}'
                )
                if abs(w) < 5.0 and abs(v) > 0.5:
                    print('Stalled or not driving: stopping before higher setpoints')
                    break
        finally:
            self.rack.disarm()
            time.sleep(0.2)
            self._print_status1(device_id, 'after')
        volts = np.array([p[0] for p in points])
        speeds = np.array([p[1] for p in points])
        if len(points) < 2 or np.ptp(speeds) < 10.0:
            print('Not enough speed spread to fit kV')
            return None
        kv, v0 = np.polyfit(speeds, volts, 1)
        resid = volts - (kv * speeds + v0)
        print(f'fit: V = {kv:.5f} V/RPM * RPM + {v0:.3f} V  (max residual {np.max(np.abs(resid)):.3f} V)')
        print(f'current SPARK_KV = {SPARK_KV}; fitted slope suggests SPARK_KV = {kv:.4f}')
        return kv, v0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SPARK base controller')
    parser.add_argument('--offsets', action='store_true', help='print Thrifty offsets in turns and exit')
    parser.add_argument('--spin-one', action='store_true', help='velocity-test a single unloaded motor')
    parser.add_argument('--fit-kv', action='store_true', help='fit volts-per-RPM on one unloaded motor')
    parser.add_argument('--rpms', type=str, default='50,80,150', help='comma-separated setpoints for --fit-kv')
    parser.add_argument('--id', type=int, default=1)
    parser.add_argument('--rpm', type=float, default=80.0)
    parser.add_argument('--seconds', type=float, default=1.0)
    args = parser.parse_args()
    vehicle = Vehicle(max_vel=(0.25, 0.25, 0.79))
    try:
        if args.offsets:
            time.sleep(0.3)
            vehicle.get_encoder_offsets()
        elif args.spin_one:
            time.sleep(0.3)
            vehicle.spin_one(args.id, args.rpm, args.seconds)
        elif args.fit_kv:
            time.sleep(0.3)
            rpms = [float(x) for x in args.rpms.split(',')]
            vehicle.fit_kv(args.id, rpms, max(args.seconds, 1.5))
        else:
            # No chassis command. Motors stay disarmed while the 50 Hz loop runs.
            vehicle.start_control()
            for _ in range(10):
                print(f'Vehicle - x: {vehicle.x} dx: {vehicle.dx}')
                time.sleep(POLICY_CONTROL_PERIOD)
    finally:
        vehicle.stop_control()
