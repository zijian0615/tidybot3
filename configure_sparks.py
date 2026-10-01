"""Write, verify, and persist the calibrated configuration for all SPARK MAXes."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

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
    SPARK_OUTPUT_MAX,
)
from spark_can.protocol import (
    PARAM_CONTROL_TYPE,
    PARAM_HARD_LIMIT_FWD_EN,
    PARAM_HARD_LIMIT_REV_EN,
    PARAM_IDLE_MODE,
    PARAM_KD_0,
    PARAM_KI_0,
    PARAM_KP_0,
    PARAM_KV_0,
    PARAM_OUTPUT_MAX_0,
    PARAM_OUTPUT_MIN_0,
    PARAM_SENSOR_TYPE,
    PARAM_SMART_CURRENT_CONFIG,
    PARAM_SMART_CURRENT_FREE,
    PARAM_SMART_CURRENT_STALL,
)
from spark_can.rack import SparkMaxRack


STEER_IDS = (1, 3, 5, 7)
DRIVE_IDS = (2, 4, 6, 8)
PARAMETER_NAMES = {
    PARAM_CONTROL_TYPE: "control_type",
    PARAM_SENSOR_TYPE: "sensor_type",
    PARAM_KP_0: "slot0_kP",
    PARAM_KI_0: "slot0_kI",
    PARAM_KD_0: "slot0_kD",
    PARAM_KV_0: "slot0_kV",
    PARAM_OUTPUT_MIN_0: "slot0_output_min",
    PARAM_OUTPUT_MAX_0: "slot0_output_max",
    PARAM_SMART_CURRENT_STALL: "smart_current_stall_A",
    PARAM_SMART_CURRENT_FREE: "smart_current_free_A",
    PARAM_SMART_CURRENT_CONFIG: "smart_current_limit_rpm",
    PARAM_IDLE_MODE: "idle_mode",
    PARAM_HARD_LIMIT_FWD_EN: "forward_limit_enabled",
    PARAM_HARD_LIMIT_REV_EN: "reverse_limit_enabled",
}


def spark_parameters(device_id):
    steer = device_id in STEER_IDS
    current = SPARK_CURRENT_STEER_A if steer else SPARK_CURRENT_DRIVE_A
    return (
        (PARAM_CONTROL_TYPE, "uint", 1),
        (PARAM_SENSOR_TYPE, "uint", 1),
        (PARAM_KP_0, "float", SPARK_KP_STEER if steer else SPARK_KP_DRIVE),
        (PARAM_KI_0, "float", SPARK_KI),
        (PARAM_KD_0, "float", SPARK_KD_STEER if steer else SPARK_KD_DRIVE),
        (PARAM_KV_0, "float", SPARK_KV),
        (PARAM_OUTPUT_MIN_0, "float", -SPARK_OUTPUT_MAX),
        (PARAM_OUTPUT_MAX_0, "float", SPARK_OUTPUT_MAX),
        (PARAM_SMART_CURRENT_STALL, "uint", current),
        (PARAM_SMART_CURRENT_FREE, "uint", current),
        (PARAM_SMART_CURRENT_CONFIG, "uint", SPARK_CURRENT_LIMIT_RPM),
        (PARAM_IDLE_MODE, "uint", SPARK_IDLE_BRAKE),
        (PARAM_HARD_LIMIT_FWD_EN, "uint", 0),
        (PARAM_HARD_LIMIT_REV_EN, "uint", 0),
    )


def snapshot():
    devices = {}
    for device_id in range(1, 9):
        steer = device_id in STEER_IDS
        device = {
            "role": "steer" if steer else "drive",
            "spark_persistent_parameters": {
                PARAMETER_NAMES[param_id]: {"parameter_id": param_id, "kind": kind, "value": value}
                for param_id, kind, value in spark_parameters(device_id)
            },
            "host_side_parameters": {},
        }
        if steer:
            caster = STEER_IDS.index(device_id)
            device["host_side_parameters"] = {
                "absolute_encoder_forward_offset_turns": ENCODER_MAGNET_OFFSETS[caster],
                "static_friction_feedforward_V": SPARK_KS_STEER[device_id],
                "static_friction_full_rpm": SPARK_KS_STEER_FULL_RPM[device_id],
            }
        else:
            negative, positive = SPARK_KS_DRIVE[device_id]
            device["host_side_parameters"] = {
                "motor_direction_sign": DRIVE_MOTOR_SIGNS[device_id],
                "static_friction_negative_V": negative,
                "static_friction_positive_V": positive,
            }
        devices[str(device_id)] = device
    return {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "notes": "Persistent values live in each SPARK; host-side values are applied by the Raspberry Pi controller.",
        "devices": devices,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--snapshot", default="spark_max_config.json")
    parser.add_argument("--write", action="store_true", help="write, verify, and persist all devices")
    args = parser.parse_args()

    result = snapshot()
    if args.write:
        rack = SparkMaxRack.open(prefer="slcan", port=args.port)
        try:
            for device_id in range(1, 9):
                for param_id, kind, value in spark_parameters(device_id):
                    rack.write_parameter(device_id, param_id, kind, value, timeout=0.20, attempts=12)
                rack.persist_parameters(device_id, timeout=1.0, attempts=5)
                print(f"SPARK {device_id}: parameters verified and persisted")
        finally:
            rack.stop()
        result["hardware_persisted_utc"] = datetime.now(timezone.utc).isoformat()

    destination = Path(args.snapshot)
    destination.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Saved configuration snapshot to {destination}")


if __name__ == "__main__":
    main()
