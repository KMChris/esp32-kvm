#include "ble_hid_internal.h"
#include "esp_gap_ble_api.h"
#include <string.h>

struct prf_char_pres_fmt {
    uint16_t unit;
    uint16_t description;
    uint8_t format;
    uint8_t exponent;
    uint8_t name_space;
};

static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];

static const uint8_t hidReportMap[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
    0xC0,
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x03, 0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x03, 0x81, 0x02,
    0x75, 0x05, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0,
};

enum {
    BAS_IDX_SVC,
    BAS_IDX_BATT_LVL_CHAR,
    BAS_IDX_BATT_LVL_VAL,
    BAS_IDX_BATT_LVL_NTF_CFG,
    BAS_IDX_BATT_LVL_PRES_FMT,
    BAS_IDX_NB,
};

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
};

hidd_le_env_t hidd_le_env;
uint8_t hidReportMapLen = sizeof(hidReportMap);
uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

static const uint8_t hidInfo[HID_INFORMATION_LEN] = {
    LO_UINT16(0x0111), HI_UINT16(0x0111), 0x00, HID_KBD_FLAGS
};

static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;
static uint8_t hidReportRefMouseIn[HID_REPORT_REF_LEN] = {HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT};
static uint8_t hidReportRefKeyIn[HID_REPORT_REF_LEN] = {HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT};
static uint8_t hidReportRefLedOut[HID_REPORT_REF_LEN] = {HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT};
static uint8_t hidReportRefFeature[HID_REPORT_REF_LEN] = {HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE};

static uint16_t hid_le_svc = ATT_SVC_HID;
uint16_t hid_count = 0;
esp_gatts_incl_svc_desc_t incl_svc = {0};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t include_service_uuid = ESP_GATT_UUID_INCLUDE_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t hid_info_char_uuid = ESP_GATT_UUID_HID_INFORMATION;
static const uint16_t hid_report_map_uuid = ESP_GATT_UUID_HID_REPORT_MAP;
static const uint16_t hid_control_point_uuid = ESP_GATT_UUID_HID_CONTROL_POINT;
static const uint16_t hid_report_uuid = ESP_GATT_UUID_HID_REPORT;
static const uint16_t hid_proto_mode_uuid = ESP_GATT_UUID_HID_PROTO_MODE;
static const uint16_t hid_kb_input_uuid = ESP_GATT_UUID_HID_BT_KB_INPUT;
static const uint16_t hid_kb_output_uuid = ESP_GATT_UUID_HID_BT_KB_OUTPUT;
static const uint16_t hid_mouse_input_uuid = ESP_GATT_UUID_HID_BT_MOUSE_INPUT;
static const uint16_t hid_repot_map_ext_desc_uuid = ESP_GATT_UUID_EXT_RPT_REF_DESCR;
static const uint16_t hid_report_ref_descr_uuid = ESP_GATT_UUID_RPT_REF_DESCR;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

static const uint16_t battary_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static const uint16_t bat_lev_uuid = ESP_GATT_UUID_BATTERY_LEVEL;
static const uint8_t bat_lev_ccc[2] = {0x00, 0x00};
static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;
static uint8_t battary_lev = 50;

static const esp_gatts_attr_db_t bas_att_db[BAS_IDX_NB] = {
    [BAS_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(battary_svc), (uint8_t *)&battary_svc}},
    [BAS_IDX_BATT_LVL_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [BAS_IDX_BATT_LVL_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&bat_lev_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), &battary_lev}},
    [BAS_IDX_BATT_LVL_NTF_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(bat_lev_ccc), (uint8_t *)bat_lev_ccc}},
    [BAS_IDX_BATT_LVL_PRES_FMT] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ, sizeof(struct prf_char_pres_fmt), 0, NULL}},
};

static esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] = {
    [HIDD_LE_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t), sizeof(hid_le_svc), (uint8_t *)&hid_le_svc}},
    [HIDD_LE_IDX_INCL_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&include_service_uuid, ESP_GATT_PERM_READ, sizeof(esp_gatts_incl_svc_desc_t), sizeof(esp_gatts_incl_svc_desc_t), (uint8_t *)&incl_svc}},
    [HIDD_LE_IDX_HID_INFO_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    [HIDD_LE_IDX_HID_INFO_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_info_char_uuid, ESP_GATT_PERM_READ, sizeof(hids_hid_info_t), sizeof(hidInfo), (uint8_t *)&hidInfo}},
    [HIDD_LE_IDX_HID_CTNL_PT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_nr}},
    [HIDD_LE_IDX_HID_CTNL_PT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_control_point_uuid, ESP_GATT_PERM_WRITE, sizeof(uint8_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_MAP_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    [HIDD_LE_IDX_REPORT_MAP_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap), (uint8_t *)&hidReportMap}},
    [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_repot_map_ext_desc_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&hidExtReportRefDesc}},
    [HIDD_LE_IDX_PROTO_MODE_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
    [HIDD_LE_IDX_PROTO_MODE_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_proto_mode_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint8_t), sizeof(hidProtocolMode), (uint8_t *)&hidProtocolMode}},
    [HIDD_LE_IDX_REPORT_KEY_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_REPORT_KEY_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_KEY_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_KEY_IN_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefKeyIn), sizeof(hidReportRefKeyIn), hidReportRefKeyIn}},
    [HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_REPORT_MOUSE_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_MOUSE_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_MOUSE_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefMouseIn), sizeof(hidReportRefMouseIn), hidReportRefMouseIn}},
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_kb_input_uuid, ESP_GATT_PERM_READ, HIDD_LE_BOOT_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_write_nr}},
    [HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_kb_output_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), HIDD_LE_BOOT_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_mouse_input_uuid, ESP_GATT_PERM_READ, HIDD_LE_BOOT_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
    [HIDD_LE_IDX_REPORT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefFeature), sizeof(hidReportRefFeature), hidReportRefFeature}},
};

static void hid_add_id_tbl(void);

void esp_hidd_prf_cb_hdl(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_GENERIC_HID);
        esp_hidd_cb_param_t hp;
        hp.init_finish.state = param->reg.status;
        if (param->reg.app_id == HIDD_APP_ID) {
            hidd_le_env.gatt_if = gatts_if;
            if (hidd_le_env.hidd_cb) {
                (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_REG_FINISH, &hp);
                hidd_le_create_service(hidd_le_env.gatt_if);
            }
        }
        if (param->reg.app_id == BATTERY_APP_ID) {
            hp.init_finish.gatts_if = gatts_if;
            if (hidd_le_env.hidd_cb)
                (hidd_le_env.hidd_cb)(ESP_BAT_EVENT_REG, &hp);
        }
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_hidd_cb_param_t cb = {0};
        memcpy(cb.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        cb.connect.conn_id = param->connect.conn_id;
        hidd_clcb_alloc(param->connect.conn_id, param->connect.remote_bda);
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        if (hidd_le_env.hidd_cb)
            (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_CONNECT, &cb);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        esp_hidd_cb_param_t cb = {0};
        memcpy(cb.disconnect.remote_bda, param->disconnect.remote_bda, sizeof(esp_bd_addr_t));
        if (hidd_le_env.hidd_cb)
            (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_DISCONNECT, &cb);
        hidd_clcb_dealloc(param->disconnect.conn_id);
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.num_handle == BAS_IDX_NB && param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_BATTERY_SERVICE_SVC && param->add_attr_tab.status == ESP_GATT_OK) {
            incl_svc.start_hdl = param->add_attr_tab.handles[BAS_IDX_SVC];
            incl_svc.end_hdl = incl_svc.start_hdl + BAS_IDX_NB - 1;
            esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
        }
        if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB && param->add_attr_tab.status == ESP_GATT_OK) {
            memcpy(hidd_le_env.hidd_inst.att_tbl, param->add_attr_tab.handles, HIDD_LE_IDX_NB * sizeof(uint16_t));
            hid_add_id_tbl();
            esp_ble_gatts_start_service(hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
        } else {
            esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
        }
        break;
    case ESP_GATTS_CONGEST_EVT: {
        esp_hidd_cb_param_t cb = {0};
        cb.congest.congested = param->congest.congested;
        cb.congest.conn_id = param->congest.conn_id;
        if (hidd_le_env.hidd_cb)
            (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_CONGEST, &cb);
        break;
    }
    default:
        break;
    }
}

void hidd_le_create_service(esp_gatt_if_t gatts_if) {
    esp_ble_gatts_create_attr_tab(bas_att_db, gatts_if, BAS_IDX_NB, 0);
}

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda) {
    for (uint8_t i = 0; i < HID_MAX_APPS; i++) {
        hidd_clcb_t *p = &hidd_le_env.hidd_clcb[i];
        if (!p->in_use) {
            p->in_use = true;
            p->conn_id = conn_id;
            p->connected = true;
            memcpy(p->remote_bda, bda, ESP_BD_ADDR_LEN);
            break;
        }
    }
}

bool hidd_clcb_dealloc(uint16_t conn_id) {
    (void)conn_id;
    for (uint8_t i = 0; i < HID_MAX_APPS; i++) {
        memset(&hidd_le_env.hidd_clcb[i], 0, sizeof(hidd_clcb_t));
        return true;
    }
    return false;
}

static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {.gatts_cb = esp_hidd_prf_cb_hdl, .gatts_if = ESP_GATT_IF_NONE},
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK)
            profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        else
            return;
    }
    for (int i = 0; i < PROFILE_NUM; i++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile_tab[i].gatts_if) {
            if (profile_tab[i].gatts_cb)
                profile_tab[i].gatts_cb(event, gatts_if, param);
        }
    }
}

esp_err_t hidd_register_cb(uint8_t gamepad) {
    (void)gamepad;
    return esp_ble_gatts_register_callback(gatts_event_handler);
}

static void hid_add_id_tbl(void) {
    uint8_t i = 0;
    hid_rpt_map[i].id = hidReportRefKeyIn[0];
    hid_rpt_map[i].type = hidReportRefKeyIn[1];
    hid_rpt_map[i].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_VAL];
    hid_rpt_map[i].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_CCC];
    hid_rpt_map[i].mode = HID_PROTOCOL_MODE_REPORT;
    i++;
    hid_rpt_map[i].id = hidReportRefMouseIn[0];
    hid_rpt_map[i].type = hidReportRefMouseIn[1];
    hid_rpt_map[i].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
    hid_rpt_map[i].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_CCC];
    hid_rpt_map[i].mode = HID_PROTOCOL_MODE_REPORT;
    i++;
    hid_rpt_map[i].id = hidReportRefKeyIn[0];
    hid_rpt_map[i].type = hidReportRefKeyIn[1];
    hid_rpt_map[i].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL];
    hid_rpt_map[i].cccdHandle = 0;
    hid_rpt_map[i].mode = HID_PROTOCOL_MODE_BOOT;
    i++;
    hid_rpt_map[i].id = hidReportRefLedOut[0];
    hid_rpt_map[i].type = hidReportRefLedOut[1];
    hid_rpt_map[i].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL];
    hid_rpt_map[i].cccdHandle = 0;
    hid_rpt_map[i].mode = HID_PROTOCOL_MODE_BOOT;
    i++;
    hid_rpt_map[i].id = hidReportRefFeature[0];
    hid_rpt_map[i].type = hidReportRefFeature[1];
    hid_rpt_map[i].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VAL];
    hid_rpt_map[i].cccdHandle = 0;
    hid_rpt_map[i].mode = HID_PROTOCOL_MODE_REPORT;
    hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}
