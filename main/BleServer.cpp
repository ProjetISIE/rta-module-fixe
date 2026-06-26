#include "BleServer.hpp"
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

BleServer* BleServer::instance_ = nullptr;

// --- C Callbacks Wrappers ---
extern "C" {
int gap_event_handler(struct ble_gap_event* event, void* arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnection; reason=%d", event->disconnect.reason);
        BleServer::advertise();
        break;
    }
    return 0;
}

void on_sync_handler(void) {
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    BleServer::advertise();
}

void host_task_handler(void* param) { BleServer::ble_host_task(param); }
}
// ----------------------------

BleServer::BleServer(std::string_view device_name) : device_name_(device_name) {
    assert(instance_ == nullptr);
    instance_ = this;

    nimble_port_init();

    ble_hs_cfg.sync_cb = on_sync_handler;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(device_name_.c_str());
    assert(rc == 0);
}

BleServer::~BleServer() { instance_ = nullptr; }

auto BleServer::register_services(const struct ble_gatt_svc_def* svcs) -> int {
    int rc = ble_gatts_count_cfg(svcs);
    if (rc != 0) return rc;
    return ble_gatts_add_svcs(svcs);
}

auto BleServer::start() -> int {
    nimble_port_freertos_init(host_task_handler);
    return 0;
}

auto BleServer::advertise() -> void {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    std::memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<uint8_t*>(
        const_cast<char*>(instance_->device_name_.c_str()));
    fields.name_len = instance_->device_name_.length();
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    std::memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, nullptr);

    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE Advertising started");
}

auto BleServer::ble_host_task(void*) -> void {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

} // namespace rta::ble
