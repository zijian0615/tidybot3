"""Calibrated SPARK MAX and caster values shared by all control programs."""

STEER_IDS = (1, 3, 5, 7)
DRIVE_IDS = (2, 4, 6, 8)

# Forward wheel angles, measured 2026-09-30 after reseating ID 7's breakout.
ENCODER_MAGNET_OFFSETS = [0.884889, 0.602722, 0.822707, 0.182538]
DRIVE_MOTOR_SIGNS = {2: 1.0, 4: -1.0, 6: 1.0, 8: 1.0}

# Slot-0 velocity-loop values stored persistently in each SPARK MAX.
SPARK_KV = 0.0022
SPARK_KP_STEER = 0.0004
SPARK_KP_DRIVE = 0.0003
SPARK_KI = 0.0
SPARK_KD_STEER = 3.0e-5
SPARK_KD_DRIVE = 0.0
SPARK_OUTPUT_MAX = 0.25
SPARK_CURRENT_STEER_A = 20
SPARK_CURRENT_DRIVE_A = 10
SPARK_CURRENT_LIMIT_RPM = 20000
SPARK_IDLE_BRAKE = 1
SPARK_MAX_RPM = 500.0

# Host-side arbitrary feedforward; these values are sent with each setpoint.
SPARK_KS_STEER = {1: 0.48, 3: 0.12, 5: 0.21, 7: 0.15}
SPARK_KS_STEER_FULL_RPM = {1: 20.0, 3: 10.0, 5: 10.0, 7: 10.0}
SPARK_KS_DRIVE = {
    2: (0.238, 0.314),
    4: (0.082, 0.082),
    6: (0.103, 0.103),
    8: (0.168, 0.242),
}
