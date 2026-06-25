#ifndef RTA_ESPNOW_FIXE_HPP_
#define RTA_ESPNOW_FIXE_HPP_
#include <cstdint>

namespace rta::espnow {

void init();
void send_distance(uint16_t distance_mm);

} // namespace rta::espnow

#endif // RTA_ESPNOW_FIXE_HPP_
