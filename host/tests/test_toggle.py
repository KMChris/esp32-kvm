import importlib.util
from pathlib import Path

from kvm_bridge.toggle import ToggleChord


spec = importlib.util.spec_from_file_location("macos_host_main", Path(__file__).parents[1] / "main.py")
assert spec is not None and spec.loader is not None
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_default_toggle_is_a_safe_modifier_chord():
    assert module.DEFAULT_TOGGLE == "ctrl+alt+cmd+f"
    assert ToggleChord.parse(module.DEFAULT_TOGGLE).parts == frozenset({"ctrl", "alt", "cmd", "f"})
