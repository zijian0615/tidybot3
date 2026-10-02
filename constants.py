import numpy as np
from spark_config_values import (
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
    SPARK_KS_DRIVE,
    SPARK_KS_STEER,
    SPARK_KS_STEER_FULL_RPM,
    SPARK_KV,
    SPARK_MAX_RPM,
    SPARK_OUTPUT_MAX,
)

################################################################################
# Mobile base

# Vehicle center to steer axis (m)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka
# h_x, h_y = 0.140150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.120150 * np.array([-1.0, 1.0, 1.0, -1.0])  # ARX5

# Thrifty analog on the steer SPARKs: turns, so 0 rad is wheel-forward.
# anaV / 3.3 (Thrifty full scale). Filled from a live Status 3 read; 0 V is a valid angle, not a disconnect.
ANALOG_FULL_V = 3.3
# Measured 2026-10-01 with all four wheels manually aligned forward,
# using a 3 s average.

# SPARK slot-0 velocity loop.
# kP is duty cycle per RPM. kV is volts per RPM: the SPARK divides it by bus voltage.
# 0.0022 V/RPM is the --fit-kv slope at 50-150 RPM on SPARKs 2 and 3 (unloaded NEO,
# ~460 RPM/V vs 473 RPM/V datasheet). SPARK 1 fits steeper (0.0027) from extra viscous
# friction; the velocity loop covers that. The intercept is friction kV does not cover.
# These are not TalonFX gains. Output clamp and smart-current limits stand in
# for TalonFX torque-current limits.
MIN_SUPPLY_VOLTAGE = 11.5
# V, per-motor static-friction compensation measured at +/-50 RPM with the
# caster unloaded. Steering friction varies enough that one shared value either
# stalls motor 1 or overspeeds motors 3/7.
# Ramp steering static-friction feedforward from zero to its calibrated value
# across this command range. This avoids a full +/-kS step whenever a caster's
# steering command crosses zero.
# (negative-RPM, positive-RPM) ground-load compensation in volts. Directional
# values account for drivetrain friction and the reversed physical ID 4 motor.
# Normalize drive motor shaft direction to the chassis convention. CAN IDs 4
# and 8 are physically installed/wired opposite to IDs 2 and 6; the global
# sign was verified from chassis motion on the ground.

################################################################################
# Teleop and imitation learning

# Base and arm RPC servers
BASE_RPC_HOST = 'localhost'
BASE_RPC_PORT = 50000
ARM_RPC_HOST = 'localhost'
ARM_RPC_PORT = 50001
RPC_AUTHKEY = b'secret password'

# Cameras
BASE_CAMERA_SERIAL = 'TODO'
# WRIST_CAMERA_SERIAL = 'TODO'  # Not used by Kinova wrist camera

# Policy
POLICY_SERVER_HOST = 'localhost'
POLICY_SERVER_PORT = 5555
POLICY_CONTROL_FREQ = 10
POLICY_CONTROL_PERIOD = 1.0 / POLICY_CONTROL_FREQ
POLICY_IMAGE_WIDTH = 84
POLICY_IMAGE_HEIGHT = 84
