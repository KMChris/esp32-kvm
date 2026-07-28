"""UART packet encoding shared by the ESP-IDF KVM firmware and host bridge."""

from __future__ import annotations

from collections.abc import Iterable

PROTOCOL_HEADER = 0xFD
KEYBOARD_TYPE = 0x00
MOUSE_TYPE = 0x03
KEYBOARD_KEY_COUNT = 6


def _checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def encode_keyboard_report(modifiers: int, keys: Iterable[int]) -> bytes:
    key_list = list(keys)[:KEYBOARD_KEY_COUNT]
    payload = bytes([modifiers & 0xFF, KEYBOARD_TYPE, *key_list])
    payload += bytes(KEYBOARD_KEY_COUNT - len(key_list))
    return bytes([PROTOCOL_HEADER, *payload, _checksum(payload)])


def encode_mouse_report(buttons: int, dx: int, dy: int, wheel: int) -> bytes:
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))
    wheel = max(-127, min(127, int(wheel)))
    payload = bytes([
        0,
        MOUSE_TYPE,
        buttons & 0x1F,
        dx & 0xFF,
        (dx >> 8) & 0xFF,
        dy & 0xFF,
        (dy >> 8) & 0xFF,
        wheel & 0xFF,
    ])
    return bytes([PROTOCOL_HEADER, *payload, _checksum(payload)])
