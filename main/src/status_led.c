#include "status_led.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "app_config.h"

#define STATUS_LED_DUTY_MAX 8191U
#define STATUS_LED_TICK_US  40000U

static status_led_mode_t status_mode;
static uint32_t status_phase;

static uint32_t status_led_duty(void) {
    if (status_mode == STATUS_LED_SERIAL) {
        const uint32_t period = STATUS_LED_DUTY_MAX * 2U;
        uint32_t duty;

        status_phase = (status_phase + 256U) % period;
        duty = status_phase;
        if (duty > STATUS_LED_DUTY_MAX) duty = period - duty;
        return duty;
    }

    status_phase++;
    return (status_phase % 25U) < 12U ? STATUS_LED_DUTY_MAX : 0U;
}

static void status_led_tick(void *arg) {
    uint32_t duty = status_led_duty();
    (void)arg;
#if CONFIG_APP_STATUS_LED_INVERTED
    duty = STATUS_LED_DUTY_MAX - duty;
#endif
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void status_led_init(status_led_mode_t mode) {
    const ledc_timer_config_t led_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t channel = {
        .gpio_num = CONFIG_APP_STATUS_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    const esp_timer_create_args_t timer_args = {
        .callback = status_led_tick,
        .name = "status_led",
    };
    esp_timer_handle_t timer;

    status_mode = mode;
    status_phase = 0;
    ESP_ERROR_CHECK(ledc_timer_config(&led_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, STATUS_LED_TICK_US));
}
