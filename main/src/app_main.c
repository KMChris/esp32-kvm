#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "driver/uart.h"
#include "ble_hid_api.h"
#include "uart_proto.h"
#include "app_config.h"

#define MAX_CONN         CONFIG_BT_ACL_CONNECTIONS
#define IDLE_INTERVAL_US 200000
#define UART_BUF_SIZE    1024
#define EVT_ADV_READY    (1 << 1)

typedef struct {
    int16_t id;
    esp_bd_addr_t addr;
} conn_slot_t;

static struct {
    conn_slot_t slots[MAX_CONN];
    uint8_t mouse_btn;
    int64_t last_activity;
    EventGroupHandle_t events;
    proto_ctx_t proto;
} g;

static const uint8_t svc_uuid[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x000A,
    .max_interval = 0x0010,
    .appearance = 0x03c1,
    .service_uuid_len = sizeof(svc_uuid),
    .p_service_uuid = (uint8_t *)svc_uuid,
    .flag = 0x6,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static inline bool has_connection(void) {
    for (uint8_t i = 0; i < MAX_CONN; i++)
        if (g.slots[i].id >= 0) return true;
    return false;
}

static void broadcast_kbd(const kbd_report_t *rpt) {
    if (!has_connection()) return;
    for (uint8_t i = 0; i < MAX_CONN; i++) {
        if (g.slots[i].id >= 0)
            esp_hidd_send_keyboard_value((uint16_t)g.slots[i].id, rpt->mods, (uint8_t *)rpt->keys, 6);
    }
    g.last_activity = esp_timer_get_time();
}

static void broadcast_mouse(const mouse_report_t *rpt) {
    if (!has_connection()) return;
    for (uint8_t i = 0; i < MAX_CONN; i++) {
        if (g.slots[i].id >= 0)
            esp_hidd_send_mouse_value((uint16_t)g.slots[i].id, rpt->buttons, rpt->dx, rpt->dy, rpt->wheel);
    }
    g.last_activity = esp_timer_get_time();
    g.mouse_btn = rpt->buttons;
}

static void idle_keepalive(void *arg) {
    (void)arg;
    if ((esp_timer_get_time() - g.last_activity) > IDLE_INTERVAL_US) {
        for (uint8_t i = 0; i < MAX_CONN; i++) {
            if (g.slots[i].id >= 0)
                esp_hidd_send_mouse_value((uint16_t)g.slots[i].id, g.mouse_btn, 0, 0, 0);
        }
        g.last_activity = esp_timer_get_time();
    }
}

static void on_hidd_event(esp_hidd_cb_event_t ev, esp_hidd_cb_param_t *p) {
    switch (ev) {
    case ESP_HIDD_EVENT_REG_FINISH:
        if (p->init_finish.state == ESP_HIDD_INIT_OK) {
            esp_ble_gap_set_device_name(APP_BLE_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
        }
        break;

    case ESP_HIDD_EVENT_BLE_CONNECT:
        for (uint8_t i = 0; i < MAX_CONN; i++) {
            if (g.slots[i].id < 0) {
                memcpy(g.slots[i].addr, p->connect.remote_bda, sizeof(esp_bd_addr_t));
                g.slots[i].id = (int16_t)p->connect.conn_id;
                break;
            }
        }
        esp_ble_conn_update_params_t cp = {
            .min_int = 6, .max_int = 6, .latency = 0, .timeout = 500
        };
        memcpy(cp.bda, p->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&cp);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        for (uint8_t i = 0; i < MAX_CONN; i++) {
            if (memcmp(g.slots[i].addr, p->disconnect.remote_bda, sizeof(esp_bd_addr_t)) == 0) {
                memset(&g.slots[i], 0, sizeof(conn_slot_t));
                g.slots[i].id = -1;
                break;
            }
        }
        esp_ble_gap_start_advertising(&adv_params);
        xEventGroupSetBits(g.events, EVT_ADV_READY);
        break;

    default:
        break;
    }
}

static void on_gap_event(esp_gap_ble_cb_event_t ev, esp_ble_gap_cb_param_t *p) {
    switch (ev) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        xEventGroupSetBits(g.events, EVT_ADV_READY);
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(p->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (p->ble_security.auth_cmpl.success)
            xEventGroupClearBits(g.events, EVT_ADV_READY);
        break;

    default:
        break;
    }
}

static void uart_rx_task(void *arg) {
    (void)arg;
    uint8_t byte;
    while (1) {
        if (uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY) > 0)
            proto_feed(&g.proto, byte);
    }
}

static void uart_init(void) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &cfg);
}

static void conn_init(void) {
    for (uint8_t i = 0; i < MAX_CONN; i++) {
        g.slots[i].id = -1;
        memset(g.slots[i].addr, 0, sizeof(esp_bd_addr_t));
    }
}

static void ble_init(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_hidd_profile_init();
    conn_init();
    esp_ble_gap_register_callback(on_gap_event);
    esp_hidd_register_callbacks(on_hidd_event, 0);

    esp_ble_auth_req_t auth = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t io = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = init_key;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &io, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1);
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1);
}

void app_main(void) {
    uart_init();

    g.events = xEventGroupCreate();
    proto_init(&g.proto, broadcast_kbd, broadcast_mouse);
    ble_init();

    xTaskCreate(uart_rx_task, "uart", 4096, NULL, configMAX_PRIORITIES - 5, NULL);

    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {.callback = idle_keepalive, .name = "idle"};
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 100000);
}
