"""Serial transport for the upstream ESP32 KVM UART protocol."""

from __future__ import annotations

from typing import Optional

import serial
from serial.tools import list_ports


class SerialPacketSink:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._port = port
        self._baudrate = baudrate
        self._serial: Optional[serial.Serial] = None

    @staticmethod
    def detect_port() -> Optional[str]:
        candidates = []
        for entry in list_ports.comports():
            description = f"{entry.device} {entry.description or ''} {entry.hwid or ''}".lower()
            if any(token in description for token in ("cp210", "usbserial", "usbmodem", "espressif", "303a")):
                candidates.append(entry.device)
        return candidates[0] if len(candidates) == 1 else None

    def open(self) -> None:
        device = serial.Serial()
        device.port = self._port
        device.baudrate = self._baudrate
        device.timeout = 0.2
        device.write_timeout = 0.5
        device.dtr = False
        device.rts = False
        device.open()
        self._serial = device

    def send(self, packet: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("serial bridge is not open")
        self._serial.write(packet)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
