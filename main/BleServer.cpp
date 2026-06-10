#include "ble_server.hpp"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <algorithm>
#include <cstring>

namespace rta::ble {

static const char* TAG = "RTA_BLE";

Server::Server(std::string_view device_name) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    std::size_t name_len = std::min(device_name.length(), sizeof(ble_device_name) - 1);
    std::memcpy(ble_device_name, device_name.data(), name_len);
    ble_device_name[name_len] = '\0';

    nimble_port_init();

    // Set sync callback
    ble_hs_cfg.sync_cb = []() { Server::ble_on_sync(); };
    
    ble_svc_gap_init();
    ble_svc_gatt_init();

    [[maybe_unused]] int rc = ble_svc_gap_device_name_set(ble_device_name);
}

auto Server::register_services(const struct ble_gatt_svc_def* svcs) -> int {
    int rc = ble_gatts_count_cfg(svcs);
    if (rc != 0) return rc;
    return ble_gatts_add_svcs(svcs);
}

auto Server::start() -> int {
    nimble_port_freertos_init([](void* param) { Server::ble_host_task(param); });
    return 0;
}

auto Server::ble_on_sync() -> void {
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    advertise();
}

auto Server::advertise() -> void {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    
    std::memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<std::uint8_t*>(ble_device_name);
    fields.name_len = std::strlen(ble_device_name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    std::memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER, &adv_params,
                          [](struct ble_gap_event* event, void* arg) {
                              return Server::ble_gap_event(event, arg);
                          }, nullptr);
    
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE Advertising started");
}

auto Server::ble_gap_event(struct ble_gap_event* event, void* arg) -> int {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connection %s", event->connect.status == 0 ? "established" : "failed");
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnection; reason=%d", event->disconnect.reason);
        advertise(); // Restart advertising
        break;
    }
    return 0;
}

auto Server::ble_host_task(void* param) -> void {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

} // namespace rta::ble
