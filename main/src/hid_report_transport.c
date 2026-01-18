#include "ble_hid_internal.h"
#include <stdint.h>

static hid_report_map_t *rpt_tbl;
static uint8_t rpt_tbl_len;

static hid_report_map_t *rpt_by_id(uint8_t id, uint8_t type) {
    hid_report_map_t *r = rpt_tbl;
    for (uint8_t i = rpt_tbl_len; i > 0; i--, r++) {
        if (r->id == id && r->type == type && r->mode == hidProtocolMode)
            return r;
    }
    return NULL;
}

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report) {
    rpt_tbl = p_report;
    rpt_tbl_len = num_reports;
}

void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length, uint8_t *data) {
    hid_report_map_t *r = rpt_by_id(id, type);
    if (r)
        esp_ble_gatts_send_indicate(gatts_if, conn_id, r->handle, length, data, false);
}

