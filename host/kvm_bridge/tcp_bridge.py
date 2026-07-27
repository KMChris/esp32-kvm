"""TCP transport for the ESP32-compatible KVM packet stream."""

from __future__ import annotations

import socket
from typing import Optional


class TcpPacketSink:
    def __init__(self, host: str, port: int) -> None:
        self._endpoint = (host, port)
        self._socket: Optional[socket.socket] = None

    def open(self) -> None:
        self._socket = socket.create_connection(self._endpoint)

    def send(self, packet: bytes) -> None:
        if self._socket is None:
            raise RuntimeError("TCP bridge is not open")
        self._socket.sendall(packet)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
