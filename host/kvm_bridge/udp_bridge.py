"""UDP transport for the ESP32 KVM packet protocol."""

from __future__ import annotations

import socket
from typing import Optional


class UdpPacketSink:
    def __init__(self, host: str, port: int) -> None:
        self._endpoint = (host, port)
        self._socket: Optional[socket.socket] = None

    def open(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, packet: bytes) -> None:
        if self._socket is None:
            raise RuntimeError("UDP bridge is not open")
        self._socket.sendto(packet, self._endpoint)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
