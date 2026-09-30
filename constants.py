import numpy as np

################################################################################
# Mobile base

# Vehicle center to steer axis (m)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka
# h_x, h_y = 0.140150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.120150 * np.array([-1.0, 1.0, 1.0, -1.0])  # ARX5

# Thrifty analog on the steer SPARKs: turns, so 0 rad is wheel-forward.
# anaV / 3.3 (Thrifty full scale). Filled from a live Status 3 read; 0 V is a valid angle, not a disconnect.
ANALOG_FULL_V = 3.3
# Measured 2026-09-29 on the chassis, wheels aligned forward, Thrifty on 5 V (0-3.3 V out), 3 s average.
ENCODER_MAGNET_OFFSETS = [0.9185, 0.6174, 0.8253, 0.1603]

# SPARK slot-0 velocity loop.
# kP is duty cycle per RPM. kV is volts per RPM: the SPARK divides it by bus voltage.
# 0.0022 V/RPM is the --fit-kv slope at 50-150 RPM on SPARKs 2 and 3 (unloaded NEO,
# ~460 RPM/V vs 473 RPM/V datasheet). SPARK 1 fits steeper (0.0027) from extra viscous
# friction; the velocity loop covers that. The intercept is friction kV does not cover.
# These are not TalonFX gains. Output clamp and smart-current limits stand in
# for TalonFX torque-current limits.
MIN_SUPPLY_VOLTAGE = 11.5
SPARK_KV = 0.0022
# V, --fit-kv intercepts at 50-150 RPM (SPARKs 1/3 steer, SPARK 2 drive); sent as arbitrary FF
SPARK_KS_STEER = 0.11
SPARK_KS_DRIVE = 0.13
SPARK_KP = 0.0001
SPARK_KI = 0.0
SPARK_KD_STEER = 1.0e-5
SPARK_KD_DRIVE = 0.0
SPARK_OUTPUT_MAX = 0.25
SPARK_CURRENT_STEER_A = 20
SPARK_CURRENT_DRIVE_A = 10
SPARK_CURRENT_LIMIT_RPM = 20000
SPARK_IDLE_BRAKE = 1
SPARK_MAX_RPM = 500.0

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
