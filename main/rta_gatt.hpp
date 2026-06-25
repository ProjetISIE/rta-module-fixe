#ifndef RTA_GATT_HPP_
#define RTA_GATT_HPP_

#include "host/ble_gatt.h"
#include "units.hpp"
#include <cstdint>

namespace rta::ble {

extern const struct ble_gatt_svc_def gatt_svcs[];

/**
 * @brief Update the distance value in the GATT table.
 * @param dist Distance strongly typed.
 */
auto update_distance(Millimeters dist) -> void;

/**
 * @brief Mark the distance as invalid in the GATT table.
 */
auto invalidate_distance() -> void;

} // namespace rta::ble

#endif // RTA_GATT_HPP_
