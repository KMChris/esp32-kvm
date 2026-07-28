from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_firmware_exposes_serial_and_wifi_udp_input_modes_with_led_status_configuration():
    kconfig = (ROOT / "main" / "Kconfig.projbuild").read_text()

    assert "config APP_INPUT_MODE_WIFI_UDP" in kconfig
    assert "config APP_WIFI_AP_SSID" in kconfig
    assert "config APP_WIFI_UDP_PORT" in kconfig
    assert "config APP_STATUS_LED_GPIO" in kconfig


def test_wifi_udp_requires_an_explicit_strong_softap_password():
    kconfig = (ROOT / "main" / "Kconfig.projbuild").read_text()
    wifi_udp = (ROOT / "main" / "src" / "wifi_udp.c").read_text()

    assert 'config APP_WIFI_AP_PASSWORD\n    string "Wi-Fi SoftAP WPA2 password"\n    default ""' in kconfig
    assert "strlen(CONFIG_APP_WIFI_AP_PASSWORD) < 12U" in wifi_udp
