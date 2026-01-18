#ifndef UART_PROTO_H_
#define UART_PROTO_H_

#include <stdint.h>
#include <stdbool.h>

#define PROTO_HEADER     0xFD
#define PROTO_PKT_LEN    9
#define PROTO_DATA_LEN   8

#define PROTO_TYPE_KBD   0x00
#define PROTO_TYPE_MOUSE 0x03

typedef enum {
    PARSE_IDLE,
    PARSE_DATA,
} proto_state_t;

typedef struct {
    uint8_t mods;
    uint8_t keys[6];
} kbd_report_t;

typedef struct {
    uint8_t buttons;
    int8_t dx;
    int8_t dy;
    int8_t wheel;
} mouse_report_t;

typedef struct {
    uint8_t type;
    union {
        kbd_report_t kbd;
        mouse_report_t mouse;
    };
} hid_cmd_t;

typedef void (*proto_kbd_cb_t)(const kbd_report_t *rpt);
typedef void (*proto_mouse_cb_t)(const mouse_report_t *rpt);

typedef struct {
    proto_state_t state;
    uint8_t idx;
    uint8_t buf[PROTO_DATA_LEN];
    proto_kbd_cb_t on_kbd;
    proto_mouse_cb_t on_mouse;
} proto_ctx_t;

void proto_init(proto_ctx_t *ctx, proto_kbd_cb_t kbd_cb, proto_mouse_cb_t mouse_cb);
void proto_feed(proto_ctx_t *ctx, uint8_t byte);
void proto_reset(proto_ctx_t *ctx);

#endif
