#ifndef RTA_BLE_SERVER_HPP
#define RTA_BLE_SERVER_HPP

#include "host/ble_gatt.h"
#include <string_view>

namespace rta::ble {

class BleServer {
public:
    explicit BleServer(std::string_view device_name);
    ~BleServer() = default;

    // Deleted copy to enforce unique management
    BleServer(const BleServer&) = delete;
    BleServer& operator=(const BleServer&) = delete;

    auto register_services(const struct ble_gatt_svc_def* svcs) -> int;
    auto start() -> int;

private:
    static auto ble_gap_event(struct ble_gap_event* event, void* arg) -> int;
    static auto ble_on_sync() -> void;
    static auto ble_host_task(void* param) -> void;
    static auto advertise() -> void;

    inline static char ble_device_name[32] = "RTA_FIXE";
};

} // namespace rta::ble

#endif // RTA_BLE_SERVER_HPP
