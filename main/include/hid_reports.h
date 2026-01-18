#ifndef HID_REPORTS_H_
#define HID_REPORTS_H_

#include <stdint.h>
#include "esp_gatt_defs.h"

typedef struct {
    uint16_t handle;
    uint16_t cccdHandle;
    uint8_t id;
    uint8_t type;
    uint8_t mode;
} hid_report_map_t;

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report);
void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length, uint8_t *data);

#endif
