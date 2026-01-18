idf_component_register(SRCS "src/app_main.c"
                            "src/ble_hid_api.c"
                            "src/hid_report_transport.c"
                            "src/ble_hid_gatt.c"
                            "src/uart_proto.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_hid
                    PRIV_REQUIRES nvs_flash esp_driver_uart)

target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Wunused-parameter
        -Wshadow
        -Wformat=2
        -Wdouble-promotion
        -Wconversion
        -Wsign-conversion
        -Wvla
)
