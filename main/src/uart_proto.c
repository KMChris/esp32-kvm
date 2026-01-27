#include "uart_proto.h"
#include <string.h>

void proto_init(proto_ctx_t *ctx, proto_kbd_cb_t kbd_cb, proto_mouse_cb_t mouse_cb) {
    memset(ctx, 0, sizeof(proto_ctx_t));
    ctx->on_kbd = kbd_cb;
    ctx->on_mouse = mouse_cb;
}

void proto_reset(proto_ctx_t *ctx) {
    ctx->state = PARSE_IDLE;
    ctx->idx = 0;
    ctx->checksum = 0;
}

uint8_t proto_calc_checksum(const uint8_t *data, uint8_t len) {
    uint8_t xor_sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        xor_sum ^= data[i];
    }
    return xor_sum;
}

static void proto_dispatch(proto_ctx_t *ctx) {
    uint8_t type = ctx->buf[1];
    
    if (type == PROTO_TYPE_KBD && ctx->on_kbd) {
        kbd_report_t rpt;
        rpt.mods = ctx->buf[0];
        memcpy(rpt.keys, &ctx->buf[2], 6);
        ctx->on_kbd(&rpt);
    } 
    else if (type == PROTO_TYPE_MOUSE && ctx->on_mouse) {
        mouse_report_t rpt;
        rpt.buttons = ctx->buf[2];
        rpt.dx = (int16_t)((uint16_t)ctx->buf[3] | ((uint16_t)ctx->buf[4] << 8));
        rpt.dy = (int16_t)((uint16_t)ctx->buf[5] | ((uint16_t)ctx->buf[6] << 8));
        rpt.wheel = (int8_t)ctx->buf[7];
        ctx->on_mouse(&rpt);
    }
}

void proto_feed(proto_ctx_t *ctx, uint8_t byte) {
    switch (ctx->state) {
    case PARSE_IDLE:
        if (byte == PROTO_HEADER) {
            ctx->idx = 0;
            ctx->checksum = 0;
            ctx->state = PARSE_DATA;
        }
        break;
        
    case PARSE_DATA:
        ctx->buf[ctx->idx++] = byte;
        ctx->checksum ^= byte;
        
        if (ctx->idx >= PROTO_DATA_LEN) {
            ctx->state = PARSE_CHECKSUM;
        }
        break;
        
    case PARSE_CHECKSUM:
        if (byte == ctx->checksum) {
            proto_dispatch(ctx);
        }
        ctx->state = PARSE_IDLE;
        ctx->idx = 0;
        ctx->checksum = 0;
        break;
    }
}
