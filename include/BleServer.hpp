#ifndef RTA_BLE_SERVER_HPP_
#define RTA_BLE_SERVER_HPP_

#include "host/ble_hs.h"
#include <string>
#include <string_view>

namespace rta::ble {

extern "C" {
int gap_event_handler(struct ble_gap_event* event, void* arg);
void on_sync_handler(void);
void host_task_handler(void* param);
}

class BleServer {
  public:
    explicit BleServer(std::string_view device_name);
    ~BleServer();

    // Deleted copy to enforce unique management
    BleServer(const BleServer&) = delete;
    BleServer& operator=(const BleServer&) = delete;

    auto register_services(const struct ble_gatt_svc_def* svcs) -> int;
    auto start() -> int;

  private:
    static auto advertise() -> void;
    static auto ble_host_task(void* param) -> void;

    std::string device_name_;
    static BleServer* instance_;

    friend int ::rta::ble::gap_event_handler(struct ble_gap_event* event,
                                             void* arg);
    friend void ::rta::ble::on_sync_handler(void);
    friend void ::rta::ble::host_task_handler(void* param);
};

} // namespace rta::ble

#endif // RTA_BLE_SERVER_HPP_
