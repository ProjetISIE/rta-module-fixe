#ifndef RTA_BLE_SERVER_HPP
#define RTA_BLE_SERVER_HPP

#include "host/ble_hs.h"
#include <cstdint>
#include <string_view>

namespace rta::ble {

class BleServer {
public:
  explicit BleServer(std::string_view device_name);
  ~BleServer() = default;

  // Deleted copy to enforce unique management
  BleServer(const BleServer &) = delete;
  BleServer &operator=(const BleServer &) = delete;

  auto register_services(const struct ble_gatt_svc_def *svcs) -> int;
  auto start() -> int;

  // Internal use only (callbacks)
  static auto advertise() -> void;
  static auto ble_host_task(void *param) -> void;

private:
  inline static char ble_device_name[32] = "RTA_FIXE";
};

} // namespace rta::ble

#endif // RTA_BLE_SERVER_HPP
