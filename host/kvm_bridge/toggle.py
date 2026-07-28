"""State tracking for a configurable simultaneous toggle chord."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ToggleChord:
    parts: frozenset[str]
    _pressed: set[str] = field(default_factory=set)

    @classmethod
    def parse(cls, value: str) -> "ToggleChord":
        raw_parts = value.lower().split("+")
        parts = [part.strip() for part in raw_parts]
        if not parts or any(not part for part in parts) or len(set(parts)) != len(parts):
            raise ValueError("toggle must be a non-empty '+'-separated chord with unique keys")
        return cls(frozenset(parts))

    def press(self, key_name: str) -> bool:
        self._pressed.add(key_name.lower())
        return self.parts.issubset(self._pressed)

    def release(self, key_name: str) -> None:
        self._pressed.discard(key_name.lower())

    def clear(self) -> None:
        self._pressed.clear()
