#ifndef RTA_GATT_H
#define RTA_GATT_H

#include "host/ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

// UUIDs
#define RTA_SERVICE_UUID        0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56
#define RTA_CHAR_DIST_UUID      0xF1, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56

extern const struct ble_gatt_svc_def rta_gatt_svcs[];

/**
 * @brief Update the distance value in the GATT table.
 * @param dist_mm Distance in millimeters. Use 0xFFFF for NO DATA.
 */
void rta_gatt_update_distance(uint16_t dist_mm);

#ifdef __cplusplus
}
#endif

#endif // RTA_GATT_H
