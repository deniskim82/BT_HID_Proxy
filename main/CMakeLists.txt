idf_component_register(
    SRCS
        "main.c"
        "ble_hid_host.c"
        "hid_parser.c"
        "key_state.c"
        "ch9329.c"
        "storage.c"
        "button.c"
        "led_status.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        bt
        nvs_flash
        driver
        esp_timer
)
