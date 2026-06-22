#pragma once
#include <cstdint>

namespace rta {
namespace espnow {

void init();
void send_distance(uint16_t distance_mm);

} // namespace espnow
} // namespace rta
