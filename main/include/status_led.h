#ifndef STATUS_LED_H_
#define STATUS_LED_H_

typedef enum {
    STATUS_LED_SERIAL,
    STATUS_LED_WIFI_UDP,
} status_led_mode_t;

void status_led_init(status_led_mode_t mode);

#endif
