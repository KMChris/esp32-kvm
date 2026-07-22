#include "wifi_udp.h"

#include <errno.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "app_config.h"

#if CONFIG_APP_INPUT_MODE_WIFI_UDP

#define UDP_BUFFER_SIZE 64

static const char *TAG = "wifi_udp";

static void udp_rx_task(void *arg) {
    proto_ctx_t *proto = arg;
    const struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_APP_WIFI_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    uint8_t buffer[UDP_BUFFER_SIZE];
    int sock;

    while (true) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (bind(sock, (const struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            ESP_LOGE(TAG, "bind failed: errno=%d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "UDP input listening on port %d", CONFIG_APP_WIFI_UDP_PORT);
        while (true) {
            const int received = recvfrom(sock, buffer, sizeof(buffer), 0, NULL, NULL);
            if (received < 0) {
                ESP_LOGW(TAG, "recvfrom failed: errno=%d", errno);
                break;
            }
            proto_reset(proto);
            for (int i = 0; i < received; i++) proto_feed(proto, buffer[i]);
            proto_reset(proto);
        }
        close(sock);
    }
}

void wifi_udp_start(proto_ctx_t *proto) {
    if (strlen(CONFIG_APP_WIFI_AP_PASSWORD) < 12U) {
        ESP_LOGE(TAG, "Wi-Fi UDP disabled: configure a unique SoftAP password of at least 12 characters");
        return;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t ap_cfg = {
        .ap = {
            .channel = CONFIG_APP_WIFI_AP_CHANNEL,
            .max_connection = CONFIG_APP_WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    strlcpy((char *)ap_cfg.ap.ssid, CONFIG_APP_WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = strlen(CONFIG_APP_WIFI_AP_SSID);
    strlcpy((char *)ap_cfg.ap.password, CONFIG_APP_WIFI_AP_PASSWORD, sizeof(ap_cfg.ap.password));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP '%s' started at 192.168.4.1", CONFIG_APP_WIFI_AP_SSID);

    xTaskCreate(udp_rx_task, "udp_rx", 4096, proto, configMAX_PRIORITIES - 5, NULL);
}

#endif
