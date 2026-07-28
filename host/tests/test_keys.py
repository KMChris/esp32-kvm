from kvm_bridge.keys import modifier_for_name, usage_for_char, usage_for_name


def test_key_translation_supports_mac_printable_and_special_keys():
    assert usage_for_char("a") == 0x04
    assert usage_for_char("A") == 0x04
    assert usage_for_char("!") == 0x1E
    assert usage_for_name("enter") == 0x28
    assert usage_for_name("f13") == 0x68


def test_key_translation_falls_back_to_macos_punctuation_virtual_key_codes():
    from kvm_bridge.keys import usage_for_vk

    assert usage_for_vk(39) == 0x34
    assert usage_for_vk(41) == 0x33
    assert usage_for_vk(50) == 0x35


def test_key_translation_supports_shifted_us_punctuation():
    assert usage_for_char(":") == 0x33
    assert usage_for_char('"') == 0x34
    assert usage_for_char("_") == 0x2D
    assert usage_for_char("?") == 0x38


def test_modifier_translation_is_explicit():
    assert modifier_for_name("ctrl") == 0x01
    assert modifier_for_name("shift_r") == 0x20
    assert modifier_for_name("space") == 0
