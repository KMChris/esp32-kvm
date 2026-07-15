from kvm_bridge.toggle import ToggleChord


def test_toggle_chord_requires_all_keys_to_be_held_simultaneously():
    chord = ToggleChord.parse("ctrl+alt+cmd+f")

    assert chord.press("ctrl") is False
    assert chord.press("alt") is False
    assert chord.press("cmd") is False
    assert chord.press("f") is True

    chord.release("f")
    assert chord.press("f") is True


def test_toggle_chord_rejects_empty_or_duplicate_parts():
    for value in ("", "ctrl+ctrl+f", "ctrl++f"):
        try:
            ToggleChord.parse(value)
        except ValueError:
            pass
        else:
            raise AssertionError(f"expected {value!r} to be rejected")
