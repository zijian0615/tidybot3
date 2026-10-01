"""SPARK MAX firmware 25+ CAN frames (REV-Specs spark-frames)."""

from __future__ import annotations

import struct
from dataclasses import dataclass

DEV_TYPE = 2  # motor controller
MFR_REV = 5


def arb_id(api_class: int, api_index: int, device_id: int = 0) -> int:
    return (
        (DEV_TYPE << 24)
        | (MFR_REV << 16)
        | ((api_class & 0x3F) << 10)
        | ((api_index & 0xF) << 6)
        | (device_id & 0x3F)
    )


def parse_arb(can_id: int) -> tuple[int, int, int, int, int]:
    return (
        (can_id >> 24) & 0x1F,
        (can_id >> 16) & 0xFF,
        (can_id >> 10) & 0x3F,
        (can_id >> 6) & 0xF,
        can_id & 0x3F,
    )


def is_spark(can_id: int) -> bool:
    dev_type, mfr, *_ = parse_arb(can_id)
    return dev_type == DEV_TYPE and mfr == MFR_REV


# Commands. Velocity setpoint API index 0 matches REV-Specs (device 0 arb id 33882112).
ID_VELOCITY_SETPOINT = lambda i: arb_id(0, 0, i)
ID_DUTY_CYCLE_SETPOINT = lambda i: arb_id(0, 2, i)
ID_SET_STATUSES_ENABLED = lambda i: arb_id(1, 0, i)
ID_SECONDARY_HEARTBEAT = lambda i: arb_id(11, 2, i)
ID_PARAMETER_WRITE = lambda i: arb_id(14, 0, i)
ID_PERSIST_PARAMETERS = lambda i: arb_id(63, 15, i)
ID_PERSIST_RESPONSE = lambda i: 0x02050500 | (i & 0x3F)
# WPILib Universal Heartbeat (FRC 2025+). SPARK 25+ enables on this, not device ID 0.
ID_FRC_HEARTBEAT = 0x01011840

# REVLib SparkParameters.h. Written to RAM only; nothing here burns EEPROM.
PARAM_IDLE_MODE = 6  # uint: 0 coast, 1 brake
PARAM_CONTROL_TYPE = 5  # uint: 1 velocity
PARAM_SENSOR_TYPE = 9  # uint: 1 primary motor encoder
PARAM_KP_0 = 13  # float, duty per RPM
PARAM_KI_0 = 14
PARAM_KD_0 = 15
PARAM_KV_0 = 16  # float, volts per RPM (SPARK divides by bus voltage)
PARAM_OUTPUT_MIN_0 = 19
PARAM_OUTPUT_MAX_0 = 20
PARAM_SMART_CURRENT_STALL = 59  # uint amps
PARAM_SMART_CURRENT_FREE = 60
PARAM_SMART_CURRENT_CONFIG = 61  # limit RPM; 20000 keeps one limit across the range
# bool. Data-port limit pins share the connector with the Thrifty; keep them off.
PARAM_HARD_LIMIT_FWD_EN = 52
PARAM_HARD_LIMIT_REV_EN = 53

# Telemetry
ID_STATUS_0 = lambda i: arb_id(46, 0, i)
ID_STATUS_1 = lambda i: arb_id(46, 1, i)  # faults / warnings
ID_STATUS_2 = lambda i: arb_id(46, 2, i)
ID_STATUS_3 = lambda i: arb_id(46, 3, i)  # getAnalog() voltage/pos/vel
ID_STATUS_5 = lambda i: arb_id(46, 5, i)  # getAbsoluteEncoder() pos/vel
ID_STATUS_6 = lambda i: arb_id(46, 6, i)  # raw duty cycle + no-signal

STATUS0_BIT = 1 << 0
STATUS1_BIT = 1 << 1
STATUS2_BIT = 1 << 2
STATUS3_BIT = 1 << 3
STATUS5_BIT = 1 << 5
STATUS6_BIT = 1 << 6
APPLIED_SCALE = 0.00003082369457075716
VOLTAGE_SCALE = 0.0073260073260073
CURRENT_SCALE = 0.0366300366300366
ABS_DUTY_SCALE = 0.00001541161211566339
ANALOG_VOLTAGE_SCALE = 0.0048973607038123
ANALOG_VELOCITY_SCALE = 0.007812026887906498

PARAM_RESULT_OK = 0

# Status 1 bit names. Faults and sticky faults share one table, as do warnings.
FAULT_NAMES = ("other", "motor_type", "sensor", "can", "temperature", "drv", "esc_eeprom", "firmware")
WARNING_NAMES = ("brownout", "overcurrent", "esc_eeprom", "ext_eeprom", "sensor", "stall", "has_reset", "other")


def flag_names(bits: int, names: tuple[str, ...]) -> str:
    active = [n for i, n in enumerate(names) if bits & (1 << i)]
    return ",".join(active) if active else "-"


def encode_enable_statuses(*bits: int) -> bytes:
    mask = enabled = 0
    for b in bits:
        mask |= b
        enabled |= b
    return mask.to_bytes(2, "little") + enabled.to_bytes(2, "little")


def encode_enable_status2() -> bytes:
    return encode_enable_statuses(
        STATUS0_BIT, STATUS1_BIT, STATUS2_BIT, STATUS3_BIT, STATUS5_BIT, STATUS6_BIT
    )


def encode_secondary_heartbeat(device_ids: list[int], enabled: bool) -> bytes:
    bits = 0
    if enabled:
        for i in device_ids:
            bits |= 1 << i
    return bits.to_bytes(8, "little")


ARB_FF_SCALE = 0.0009765923  # volts (or duty) per LSB of the int16 arbitrary feedforward


def encode_setpoint(setpoint: float, arb_ff: float = 0.0) -> bytes:
    """8-byte setpoint. API index selects the control type. Slot 0, FF units = voltage."""
    buf = bytearray(8)
    struct.pack_into("<f", buf, 0, float(setpoint))
    ff = max(-32767, min(32767, round(float(arb_ff) / ARB_FF_SCALE)))
    struct.pack_into("<h", buf, 4, ff)
    return bytes(buf)


def encode_velocity(rpm: float, arb_ff_volts: float = 0.0) -> bytes:
    return encode_setpoint(rpm, arb_ff_volts)


def encode_duty(setpoint: float) -> bytes:
    sp = max(-1.0, min(1.0, float(setpoint)))
    buf = bytearray(encode_setpoint(sp))
    # bit 50: arbitrary FF units = duty cycle (matches REV Hardware Client / CanControl)
    buf[6] = 1 << 2
    return bytes(buf)


def encode_parameter(param_id: int, kind: str, value: float) -> bytes:
    """5-byte parameter write. Does not persist across power cycle."""
    if kind == "float":
        return struct.pack("<Bf", param_id & 0xFF, float(value))
    if kind == "uint":
        return struct.pack("<BI", param_id & 0xFF, int(value) & 0xFFFFFFFF)
    raise ValueError(kind)


def encode_persist_parameters() -> bytes:
    """Firmware 25.x persist-parameters key (0x3AA3), little endian."""
    return (15011).to_bytes(2, "little")


def decode_parameter_response(data: bytes) -> tuple[int, int, int, int] | None:
    """Return (param_id, type, value_bits, result_code). result 0 is success."""
    if len(data) < 7:
        return None
    raw = int.from_bytes(data[:7], "little")
    param_id = raw & 0xFF
    param_type = (raw >> 8) & 0xFF
    value_bits = (raw >> 16) & 0xFFFFFFFF
    result = (raw >> 48) & 0xFF
    return param_id, param_type, value_bits, result


def encode_frc_heartbeat(enabled: bool) -> bytes:
    """Pack WPILib RobotState. enabled + systemWatchdog must both be set to drive."""
    raw = 0
    raw |= 120 & 0xFF
    raw |= (1 & 0x3FF) << 8
    if enabled:
        raw |= 1 << 25
        raw |= 1 << 28
    raw |= (25 & 0x3F) << 32
    raw |= (1 & 0xF) << 38
    raw |= (1 & 0x1F) << 42
    raw |= (12 & 0x1F) << 59
    return raw.to_bytes(8, "little")


def _bits(data: bytes, start: int, length: int, signed: bool = False) -> int:
    val = int.from_bytes(data, "little")
    v = (val >> start) & ((1 << length) - 1)
    if signed and v & (1 << (length - 1)):
        v -= 1 << length
    return v


@dataclass
class Telemetry:
    voltage: float = float("nan")
    current: float = float("nan")
    temp_c: float = float("nan")
    applied: float = float("nan")
    velocity_rpm: float = float("nan")
    position_rot: float = float("nan")
    heartbeat_locked: bool = False
    # SparkMax.getAbsoluteEncoder(): duty-cycle encoder on the data port
    abs_position: float = float("nan")
    abs_velocity_rpm: float = float("nan")
    abs_duty: float = float("nan")
    abs_period_us: float = float("nan")
    abs_no_signal: bool = True
    analog_voltage: float = float("nan")
    analog_position: float = float("nan")
    analog_velocity_rpm: float = float("nan")
    faults: int = 0
    warnings: int = 0
    sticky_faults: int = 0
    sticky_warnings: int = 0
    last_rx_s: float = float("nan")
    # 0 above means "not received" until these are finite
    status0_rx_s: float = float("nan")
    status1_rx_s: float = float("nan")


def decode_status0(data: bytes, tel: Telemetry) -> None:
    if len(data) < 8:
        return
    tel.applied = _bits(data, 0, 16, True) * APPLIED_SCALE
    tel.voltage = _bits(data, 16, 12) * VOLTAGE_SCALE
    tel.current = _bits(data, 28, 12) * CURRENT_SCALE
    tel.temp_c = float(_bits(data, 40, 8))
    tel.heartbeat_locked = bool(_bits(data, 53, 1))


def decode_status1(data: bytes, tel: Telemetry) -> None:
    if len(data) < 3:
        return
    raw = int.from_bytes(data, "little")
    tel.faults = raw & 0xFF
    tel.warnings = (raw >> 16) & 0xFF
    if len(data) >= 6:
        tel.sticky_faults = (raw >> 24) & 0xFF
        tel.sticky_warnings = (raw >> 40) & 0xFF


def decode_status2(data: bytes, tel: Telemetry) -> None:
    if len(data) < 8:
        return
    tel.velocity_rpm = struct.unpack_from("<f", data, 0)[0]
    tel.position_rot = struct.unpack_from("<f", data, 4)[0]


def decode_status3(data: bytes, tel: Telemetry) -> None:
    """REVLib SparkMax.getAnalog(): Thrifty / analog voltage encoder."""
    if len(data) < 8:
        return
    tel.analog_voltage = _bits(data, 0, 10) * ANALOG_VOLTAGE_SCALE
    tel.analog_velocity_rpm = _bits(data, 10, 22, True) * ANALOG_VELOCITY_SCALE
    tel.analog_position = struct.unpack_from("<f", data, 4)[0]


def decode_status5(data: bytes, tel: Telemetry) -> None:
    """REVLib SparkAbsoluteEncoder.getVelocity() / getPosition()."""
    if len(data) < 8:
        return
    tel.abs_velocity_rpm = struct.unpack_from("<f", data, 0)[0]
    tel.abs_position = struct.unpack_from("<f", data, 4)[0]


def decode_status6(data: bytes, tel: Telemetry) -> None:
    """Raw duty-cycle absolute encoder: duty, period, no-signal flag."""
    if len(data) < 5:
        return
    tel.abs_duty = _bits(data, 0, 16) * ABS_DUTY_SCALE
    tel.abs_period_us = float(_bits(data, 16, 16))
    tel.abs_no_signal = bool(_bits(data, 32, 1))
