"""SPARK MAX rack: heartbeat, velocity (or duty) setpoints, and telemetry.

Mac: SH-C31G slcan. The CAN thread is the replacement for Phoenix feed_enable
and BaseStatusSignal.refresh_all. Callers only read Telemetry and set RPM.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Dict, Iterable, Optional, Sequence

from .bus import CanBus, open_bus
from .protocol import (
    ID_DUTY_CYCLE_SETPOINT,
    ID_FRC_HEARTBEAT,
    ID_PARAMETER_WRITE,
    ID_SECONDARY_HEARTBEAT,
    ID_SET_STATUSES_ENABLED,
    ID_VELOCITY_SETPOINT,
    PARAM_RESULT_OK,
    Telemetry,
    decode_parameter_response,
    decode_status0,
    decode_status1,
    decode_status2,
    decode_status3,
    decode_status5,
    decode_status6,
    encode_duty,
    encode_enable_status2,
    encode_frc_heartbeat,
    encode_parameter,
    encode_secondary_heartbeat,
    encode_velocity,
    is_spark,
    parse_arb,
)


def _echo_value(kind: str, value_bits: int) -> float:
    raw = value_bits.to_bytes(4, "little")
    if kind == "float":
        return struct.unpack("<f", raw)[0]
    return float(struct.unpack("<I", raw)[0])


def _value_matches(kind: str, expected: float, echoed: float) -> bool:
    if kind == "float":
        return math.isfinite(echoed) and math.isclose(echoed, float(expected), rel_tol=1e-3, abs_tol=1e-6)
    return int(echoed) == int(expected)


class SparkMaxRack:
    def __init__(
        self,
        bus: CanBus,
        device_ids: Sequence[int] = range(1, 9),
        period_s: float = 0.02,
        max_duty: float = 0.25,
        max_rpm: float = 500.0,
        command: str = "velocity",
        ks_volts: float | Dict[int, float] = 0.0,
        ks_deadband_rpm: float = 1.0,
    ):
        if command not in ("velocity", "duty"):
            raise ValueError(command)
        self.bus = bus
        self.device_ids = list(device_ids)
        self.period_s = period_s
        self.max_duty = max_duty
        self.max_rpm = max_rpm
        self.command = command
        # Static friction the SPARK kV cannot cover, sent as arbitrary feedforward.
        if isinstance(ks_volts, dict):
            self.ks_volts = {i: float(ks_volts.get(i, 0.0)) for i in self.device_ids}
        else:
            self.ks_volts = {i: float(ks_volts) for i in self.device_ids}
        self.ks_deadband_rpm = ks_deadband_rpm
        self._duties = {i: 0.0 for i in self.device_ids}
        self._rpms = {i: 0.0 for i in self.device_ids}
        self.telemetry: Dict[int, Telemetry] = {i: Telemetry() for i in self.device_ids}
        self._enable_output = False
        self._run = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._bus_lock = threading.Lock()
        self._frc_heartbeat_active = False

    @classmethod
    def open(cls, **kwargs) -> "SparkMaxRack":
        bus_kw = {k: kwargs.pop(k) for k in ("prefer", "port", "channel") if k in kwargs}
        return cls(open_bus(**bus_kw), **kwargs)

    def write_parameter(
        self, device_id: int, param_id: int, kind: str, value: float, timeout: float = 0.1, attempts: int = 5
    ) -> float:
        """Write one RAM parameter and wait for result code 0. Call before start().

        Acks normally arrive in ~1 ms but are occasionally lost on slcan, and macOS can
        stall the reader for 100-250 ms; RAM writes are idempotent, so a missing ack is
        resent. Rejections are not retried.
        """
        if self._run:
            raise RuntimeError("write_parameter must run before the CAN thread starts")
        payload = encode_parameter(param_id, kind, value)
        with self._bus_lock:
            for attempt in range(attempts):
                self.bus.send(ID_PARAMETER_WRITE(device_id), payload)
                deadline = time.time() + timeout
                while time.time() < deadline:
                    pkt = self.bus.recv(max(0.0, deadline - time.time()))
                    if pkt is None:
                        break
                    can_id, data = pkt
                    self._handle_rx(can_id, data)
                    _, _, api_class, api_idx, dev_id = parse_arb(can_id)
                    if api_class != 14 or api_idx != 1 or dev_id != device_id:
                        continue
                    parsed = decode_parameter_response(data)
                    if parsed is None or parsed[0] != param_id:
                        continue
                    _, _, value_bits, result = parsed
                    echoed = _echo_value(kind, value_bits)
                    if result != PARAM_RESULT_OK:
                        raise RuntimeError(
                            f"SPARK {device_id} rejected parameter {param_id}: result {result}, echo {echoed}"
                        )
                    if not _value_matches(kind, value, echoed):
                        raise RuntimeError(
                            f"SPARK {device_id} parameter {param_id} echo {echoed} != {value}"
                        )
                    if attempt:
                        print(f"SPARK {device_id} parameter {param_id} acknowledged on attempt {attempt + 1}")
                    return echoed
        raise RuntimeError(f"SPARK {device_id} did not acknowledge parameter {param_id} after {attempts} attempts")

    def start(self) -> None:
        payload = encode_enable_status2()
        with self._bus_lock:
            for i in self.device_ids:
                self.bus.send(ID_SET_STATUSES_ENABLED(i), payload)
                time.sleep(0.01)
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def arm(self) -> None:
        """Enable output. Setpoints stay at the last set_velocity/set_duty values."""
        with self._lock:
            self._enable_output = True
            self._frc_heartbeat_active = True

    def disarm(self) -> None:
        with self._lock:
            self._enable_output = False
            for i in self.device_ids:
                self._duties[i] = 0.0
                self._rpms[i] = 0.0

    def set_duty(self, device_id: int, duty: float) -> None:
        duty = max(-self.max_duty, min(self.max_duty, float(duty)))
        with self._lock:
            self._duties[device_id] = duty

    def set_duties(self, duties: Iterable[float]) -> None:
        for i, d in zip(self.device_ids, duties):
            self.set_duty(i, d)

    def set_velocity(self, device_id: int, rpm: float) -> None:
        rpm = max(-self.max_rpm, min(self.max_rpm, float(rpm)))
        with self._lock:
            self._rpms[device_id] = rpm

    def get_rpms(self) -> list[float]:
        with self._lock:
            return [self._rpms[i] for i in self.device_ids]

    def stop(self) -> None:
        self.disarm()
        time.sleep(self.period_s * 3)
        self._run = False
        if self._thread:
            self._thread.join(timeout=1.0)
        with self._bus_lock:
            self.bus.close()

    def _loop(self) -> None:
        next_t = time.perf_counter()
        while self._run:
            with self._lock:
                enable = self._enable_output
                duties = dict(self._duties)
                rpms = dict(self._rpms)
                send_frc = enable or self._frc_heartbeat_active
                self._frc_heartbeat_active = enable
            with self._bus_lock:
                if send_frc:
                    self.bus.send(ID_FRC_HEARTBEAT, encode_frc_heartbeat(enable))
                hb = encode_secondary_heartbeat(self.device_ids, enable)
                for i in self.device_ids:
                    self.bus.send(ID_SECONDARY_HEARTBEAT(i), hb)
                    if self.command == "velocity":
                        rpm = rpms[i] if enable else 0.0
                        ff = math.copysign(self.ks_volts[i], rpm) if abs(rpm) >= self.ks_deadband_rpm else 0.0
                        self.bus.send(ID_VELOCITY_SETPOINT(i), encode_velocity(rpm, ff))
                    else:
                        self.bus.send(ID_DUTY_CYCLE_SETPOINT(i), encode_duty(duties[i] if enable else 0.0))
                deadline = time.perf_counter() + self.period_s * 0.7
                while time.perf_counter() < deadline:
                    pkt = self.bus.recv(max(0.0, deadline - time.perf_counter()))
                    if pkt is None:
                        break
                    self._handle_rx(*pkt)
            next_t += self.period_s
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    def _handle_rx(self, can_id: int, data: bytes) -> None:
        if not is_spark(can_id):
            return
        _, _, api_class, api_idx, dev_id = parse_arb(can_id)
        if dev_id not in self.telemetry:
            return
        now = time.monotonic()
        if api_class == 46 and api_idx == 0:
            decode_status0(data, self.telemetry[dev_id])
            self.telemetry[dev_id].status0_rx_s = now
        elif api_class == 46 and api_idx == 1:
            decode_status1(data, self.telemetry[dev_id])
            self.telemetry[dev_id].status1_rx_s = now
        elif api_class == 46 and api_idx == 2:
            decode_status2(data, self.telemetry[dev_id])
        elif api_class == 46 and api_idx == 3:
            decode_status3(data, self.telemetry[dev_id])
        elif api_class == 46 and api_idx == 5:
            decode_status5(data, self.telemetry[dev_id])
        elif api_class == 46 and api_idx == 6:
            decode_status6(data, self.telemetry[dev_id])
        else:
            return
        self.telemetry[dev_id].last_rx_s = now
