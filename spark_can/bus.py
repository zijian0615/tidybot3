"""CAN backends: Mac/Linux slcan (SH-C31G) and Linux SocketCAN (edge device)."""

from __future__ import annotations

import glob
import os
import select
import struct
import termios
import time
from typing import Optional


class CanBus:
    def send(self, can_id: int, data: bytes) -> None:
        raise NotImplementedError

    def recv(self, timeout: float) -> Optional[tuple[int, bytes]]:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class SlcanBus(CanBus):
    """SH-C31G candlelight-alternative: Elmue slcan 2.5 CDC ACM at 1 Mbps."""

    def __init__(self, port: Optional[str] = None):
        self.port = port or self.autodetect_port()
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        cc = attrs[6]
        raw = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0, termios.B115200, termios.B115200, cc]
        raw[6][termios.VMIN] = 0
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, raw)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._rx = b""
        self._cmd("C\r")
        self._cmd("S8\r")
        self._cmd("ON\r")

    @staticmethod
    def autodetect_port() -> str:
        ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/tty.usbmodem*"))
        if not ports:
            raise RuntimeError("找不到 SH-C31G 串口（/dev/cu.usbmodem* 或 /dev/ttyACM*）")
        return ports[0]

    def _cmd(self, s: str, wait: float = 0.08) -> bytes:
        os.write(self.fd, s.encode())
        return self._read_raw(wait)

    def _read_raw(self, dt: float) -> bytes:
        data = b""
        end = time.time() + dt
        while time.time() < end:
            r, _, _ = select.select([self.fd], [], [], max(0.0, end - time.time()))
            if r:
                chunk = os.read(self.fd, 4096)
                if chunk:
                    data += chunk
        return data

    def send(self, can_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("classic CAN max 8 bytes")
        line = f"T{can_id:08X}{len(data):X}{data.hex().upper()}\r"
        os.write(self.fd, line.encode())

    def recv(self, timeout: float) -> Optional[tuple[int, bytes]]:
        end = time.time() + timeout
        while True:
            if b"\r" in self._rx:
                line, self._rx = self._rx.split(b"\r", 1)
                parsed = self._parse(line)
                if parsed is not None:
                    return parsed
                continue
            remain = end - time.time()
            if remain <= 0:
                return None
            r, _, _ = select.select([self.fd], [], [], remain)
            if not r:
                return None
            self._rx += os.read(self.fd, 4096)

    @staticmethod
    def _parse(line: bytes) -> Optional[tuple[int, bytes]]:
        if not line.startswith(b"T") or len(line) < 10:
            return None
        try:
            s = line.decode("ascii")
            can_id = int(s[1:9], 16)
            dlc = int(s[9], 16)
            data = bytes.fromhex(s[10:10 + dlc * 2])
            return can_id, data
        except (ValueError, UnicodeDecodeError):
            return None

    def close(self) -> None:
        try:
            os.write(self.fd, b"C\r")
        except OSError:
            pass
        os.close(self.fd)


class SocketCanBus(CanBus):
    """Linux edge device: `ip link set can0 up type can bitrate 1000000`."""

    CAN_EFF_FLAG = 0x80000000

    def __init__(self, channel: str = "can0"):
        import socket

        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._socket.bind((channel,))
        self._socket.setblocking(False)

    def send(self, can_id: int, data: bytes) -> None:
        frame = struct.pack("=IB3x8s", can_id | self.CAN_EFF_FLAG, len(data), data.ljust(8, b"\x00"))
        self._socket.send(frame)

    def recv(self, timeout: float) -> Optional[tuple[int, bytes]]:
        r, _, _ = select.select([self._socket], [], [], timeout)
        if not r:
            return None
        raw = self._socket.recv(16)
        can_id, dlc, data = struct.unpack("=IB3x8s", raw)
        return can_id & 0x1FFFFFFF, data[:dlc]

    def close(self) -> None:
        self._socket.close()


def open_bus(prefer: str = "auto", port: Optional[str] = None, channel: str = "can0") -> CanBus:
    if prefer == "socketcan":
        return SocketCanBus(channel)
    if prefer == "slcan" or prefer == "auto":
        try:
            return SlcanBus(port)
        except Exception:
            if prefer == "slcan":
                raise
            return SocketCanBus(channel)
    raise ValueError(prefer)
