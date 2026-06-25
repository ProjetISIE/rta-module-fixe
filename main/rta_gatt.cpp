#include "rta_gatt.hpp"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include <atomic>
#include <cstring>

namespace rta::ble {

[[maybe_unused]] static const char *TAG = "RTA_GATT";

// UUIDs
static constexpr ble_uuid128_t SERVICE_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78,
              0x56, 0x34, 0x12, 0x78, 0x56}};

static constexpr ble_uuid128_t CHAR_DIST_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0xF1, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78,
              0x56, 0x34, 0x12, 0x78, 0x56}};

static std::uint16_t char_dist_handle;
static std::atomic<std::uint16_t> current_dist{
    0xFFFF}; // Sentinel value for NO DATA

extern "C" int gatt_char_access(std::uint16_t conn_handle,
                                std::uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (attr_handle == char_dist_handle) {
    std::uint16_t dist_val = current_dist.load(std::memory_order_relaxed);
    return os_mbuf_append(ctxt->om, &dist_val, sizeof(dist_val));
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_chr_def gatt_chrs[] = {
    {
        .uuid = &CHAR_DIST_UUID.u,
        .access_cb = gatt_char_access,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &char_dist_handle,
        .cpfd = nullptr,
    },
    {}, // Sentinel
};

const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVICE_UUID.u,
        .includes = nullptr,
        .characteristics = gatt_chrs,
    },
    {} // Sentinel
};

auto update_distance(Millimeters dist) -> void {
  current_dist.store(dist.value, std::memory_order_relaxed);
  ble_gatts_chr_updated(char_dist_handle);
}

auto invalidate_distance() -> void { update_distance(Millimeters{0xFFFF}); }

} // namespace rta::ble
