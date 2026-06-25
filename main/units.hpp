#ifndef RTA_UNITS_HPP_
#define RTA_UNITS_HPP_

#include <compare>
#include <cstdint>

namespace rta {

/**
 * @brief Strong type for distance in millimeters.
 */
struct Millimeters {
    std::uint16_t value;

    auto operator<=>(const Millimeters&) const = default;
};

/**
 * @brief Strong type for angle in degrees.
 */
struct Degrees {
    float value;

    auto operator<=>(const Degrees&) const = default;
};

/**
 * @brief Strong type for distance in meters (for internal calculations if needed).
 */
struct Meters {
    float value;

    constexpr explicit operator Millimeters() const {
        return Millimeters{static_cast<std::uint16_t>(value * 1000.0f)};
    }
};

} // namespace rta

#endif // RTA_UNITS_HPP_
