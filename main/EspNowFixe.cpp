#include "EspNowFixe.hpp"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <cstring>

namespace rta::espnow {

static const char* TAG = "ESP_NOW_FIXE";

struct __attribute__((packed)) EspNowDistancePacket {
    uint8_t magic[4];
    uint16_t distance_mm;
};

void init() {
    ESP_LOGI(TAG, "Initializing ESP-NOW (Fixe)...");

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                         WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peerInfo = {};
    for (int i = 0; i < 6; i++) {
        peerInfo.peer_addr[i] = 0xFF;
    }
    peerInfo.channel = 0; // Current channel
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer");
    } else {
        ESP_LOGI(TAG, "ESP-NOW broadcast peer added");
    }
}

void send_distance(uint16_t distance_mm) {
    EspNowDistancePacket pkt = {
        .magic = {'R', 'T', 'A', '!'},
        .distance_mm = distance_mm,
    };

    constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t err = esp_now_send(
        kBroadcastMac, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send ESP-NOW packet: %s",
                 esp_err_to_name(err));
    }
}

} // namespace rta::espnow
