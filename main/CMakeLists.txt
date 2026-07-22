idf_component_register(SRCS "src/app_main.c"
                            "src/ble_hid_api.c"
                            "src/hid_report_transport.c"
                            "src/ble_hid_gatt.c"
                            "src/status_led.c"
                            "src/uart_proto.c"
                            "src/wifi_udp.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_hid esp_netif esp_wifi
                    PRIV_REQUIRES nvs_flash esp_driver_ledc esp_driver_uart esp_driver_usb_serial_jtag)

target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Wshadow
        -Wformat=2
        -Wdouble-promotion
        -Wconversion
        -Wsign-conversion
        -Wvla
)
