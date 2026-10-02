"""Calibrated SPARK MAX and caster values shared by all control programs."""

STEER_IDS = (1, 3, 5, 7)
DRIVE_IDS = (2, 4, 6, 8)

# Forward wheel angles, measured 2026-10-01 with all four wheels aligned.
ENCODER_MAGNET_OFFSETS = [0.407924, 0.127250, 0.299150, 0.164690]
# IDs 4 and 8 are installed opposite to IDs 2 and 6. The global sign follows
# the chassis convention verified from ground motion on 2026-10-01.
DRIVE_MOTOR_SIGNS = {2: -1.0, 4: 1.0, 6: -1.0, 8: 1.0}

# Slot-0 velocity-loop values stored persistently in each SPARK MAX.
SPARK_KV = 0.0022
SPARK_KP_STEER = 0.0006
SPARK_KP_STEER_BY_ID = {1: 0.0006, 3: 0.0006, 5: 0.0006, 7: 0.0006}
SPARK_KP_DRIVE = 0.0005
SPARK_KI = 0.0
SPARK_KD_STEER = 3.0e-5
SPARK_KD_DRIVE = 0.0
SPARK_OUTPUT_MAX = 0.35
SPARK_CURRENT_STEER_A = 20
SPARK_CURRENT_DRIVE_A = 15
SPARK_CURRENT_LIMIT_RPM = 20000
SPARK_IDLE_BRAKE = 1
SPARK_MAX_RPM = 500.0

# Host-side arbitrary feedforward; these values are sent with each setpoint.
SPARK_KS_STEER = {1: 0.48, 3: 0.12, 5: 0.21, 7: 0.15}
SPARK_KS_STEER_FULL_RPM = {1: 20.0, 3: 10.0, 5: 10.0, 7: 10.0}
SPARK_KS_DRIVE = {
    2: (0.162, 0.186),
    4: (0.090, 0.082),
    6: (0.120, 0.120),
    8: (0.105, 0.131),
}
