#ifndef BLE_HID_INTERNAL_H_
#define BLE_HID_INTERNAL_H_

#include <stdbool.h>
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "ble_hid_api.h"
#include "hid_reports.h"

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
#define HIDD_VERSION                0x0100
#define HID_MAX_APPS                1
#define HID_NUM_REPORTS             5
#define HID_RPT_ID_KEY_IN           1
#define HID_RPT_ID_MOUSE_IN         3
#define HID_RPT_ID_LED_OUT          1
#define HID_RPT_ID_FEATURE          0
#define HIDD_APP_ID                 0x1812
#define BATTERY_APP_ID              0x180f
#define ATT_SVC_HID                 0x1812
#define HIDD_LE_NB_REPORT_INST_MAX  6
#define HIDD_LE_REPORT_MAX_LEN      255
#define HIDD_LE_REPORT_MAP_MAX_LEN  512
#define HIDD_LE_BOOT_REPORT_MAX_LEN 8
#define HID_FLAGS_REMOTE_WAKE       0x01
#define HID_PROTOCOL_MODE_BOOT      0x00
#define HID_PROTOCOL_MODE_REPORT    0x01
#define HID_INFORMATION_LEN         4
#define HID_REPORT_REF_LEN          2
#define HID_KBD_FLAGS               HID_FLAGS_REMOTE_WAKE
#define HID_REPORT_TYPE_INPUT       1
#define HID_REPORT_TYPE_OUTPUT      2
#define HID_REPORT_TYPE_FEATURE     3

enum {
    HIDD_LE_IDX_SVC,
    HIDD_LE_IDX_INCL_SVC,
    HIDD_LE_IDX_HID_INFO_CHAR,
    HIDD_LE_IDX_HID_INFO_VAL,
    HIDD_LE_IDX_HID_CTNL_PT_CHAR,
    HIDD_LE_IDX_HID_CTNL_PT_VAL,
    HIDD_LE_IDX_REPORT_MAP_CHAR,
    HIDD_LE_IDX_REPORT_MAP_VAL,
    HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF,
    HIDD_LE_IDX_PROTO_MODE_CHAR,
    HIDD_LE_IDX_PROTO_MODE_VAL,
    HIDD_LE_IDX_REPORT_KEY_IN_CHAR,
    HIDD_LE_IDX_REPORT_KEY_IN_VAL,
    HIDD_LE_IDX_REPORT_KEY_IN_CCC,
    HIDD_LE_IDX_REPORT_KEY_IN_REP_REF,
    HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR,
    HIDD_LE_IDX_REPORT_MOUSE_IN_VAL,
    HIDD_LE_IDX_REPORT_MOUSE_IN_CCC,
    HIDD_LE_IDX_REPORT_MOUSE_REP_REF,
    HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR,
    HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL,
    HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG,
    HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR,
    HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL,
    HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR,
    HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL,
    HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG,
    HIDD_LE_IDX_REPORT_CHAR,
    HIDD_LE_IDX_REPORT_VAL,
    HIDD_LE_IDX_REPORT_REP_REF,
    HIDD_LE_IDX_NB,
};

typedef struct {
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t flags;
} __attribute__((packed)) hids_hid_info_t;

typedef struct {
    bool in_use;
    bool connected;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
} hidd_clcb_t;

typedef struct {
    uint16_t att_tbl[HIDD_LE_IDX_NB];
} hidd_inst_t;

typedef struct {
    hidd_clcb_t hidd_clcb[HID_MAX_APPS];
    esp_gatt_if_t gatt_if;
    bool enabled;
    hidd_inst_t hidd_inst;
    esp_hidd_event_cb_t hidd_cb;
} hidd_le_env_t;

extern hidd_le_env_t hidd_le_env;
extern uint8_t hidProtocolMode;

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda);
bool hidd_clcb_dealloc(uint16_t conn_id);
void hidd_le_create_service(esp_gatt_if_t gatts_if);
esp_err_t hidd_register_cb(uint8_t gamepad);

#endif
