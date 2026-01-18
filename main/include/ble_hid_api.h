#ifndef BLE_HID_API_H_
#define BLE_HID_API_H_

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_err.h"

typedef enum {
    ESP_HIDD_EVENT_REG_FINISH = 0,
    ESP_BAT_EVENT_REG,
    ESP_HIDD_EVENT_DEINIT_FINISH,
    ESP_HIDD_EVENT_BLE_CONNECT,
    ESP_HIDD_EVENT_BLE_DISCONNECT,
    ESP_HIDD_EVENT_BLE_CONGEST,
} esp_hidd_cb_event_t;

typedef enum {
    ESP_HIDD_STA_CONN_SUCCESS = 0x00,
    ESP_HIDD_STA_CONN_FAIL = 0x01,
} esp_hidd_sta_conn_state_t;

typedef enum {
    ESP_HIDD_INIT_OK = 0,
    ESP_HIDD_INIT_FAILED = 1,
} esp_hidd_init_state_t;

typedef enum {
    ESP_HIDD_DEINIT_OK = 0,
    ESP_HIDD_DEINIT_FAILED = 0,
} esp_hidd_deinit_state_t;

#define LEFT_CONTROL_KEY_MASK   (1 << 0)
#define LEFT_SHIFT_KEY_MASK     (1 << 1)
#define LEFT_ALT_KEY_MASK       (1 << 2)
#define LEFT_GUI_KEY_MASK       (1 << 3)
#define RIGHT_CONTROL_KEY_MASK  (1 << 4)
#define RIGHT_SHIFT_KEY_MASK    (1 << 5)
#define RIGHT_ALT_KEY_MASK      (1 << 6)
#define RIGHT_GUI_KEY_MASK      (1 << 7)

typedef uint8_t key_mask_t;

typedef union {
    struct hidd_init_finish_evt_param {
        esp_hidd_init_state_t state;
        esp_gatt_if_t gatts_if;
    } init_finish;
    struct hidd_deinit_finish_evt_param {
        esp_hidd_deinit_state_t state;
    } deinit_finish;
    struct hidd_connect_evt_param {
        uint16_t conn_id;
        esp_bd_addr_t remote_bda;
    } connect;
    struct hidd_congest_evt_param {
        uint16_t conn_id;
        bool congested;
    } congest;
    struct hidd_disconnect_evt_param {
        esp_bd_addr_t remote_bda;
    } disconnect;
} esp_hidd_cb_param_t;

typedef void (*esp_hidd_event_cb_t)(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t cb, uint8_t gamepad);
esp_err_t esp_hidd_profile_init(void);
esp_err_t esp_hidd_profile_deinit(void);
uint16_t esp_hidd_get_version(void);
void esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t mods, uint8_t *keys, uint8_t num);
void esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t btn, int8_t x, int8_t y, int8_t w);

#endif

