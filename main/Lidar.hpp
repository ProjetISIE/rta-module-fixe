#ifndef RTA_LIDAR_HPP
#define RTA_LIDAR_HPP

#include "driver/uart.h"
#include "units.hpp"
#include <cstdint>
#include <expected>

namespace rta::lidar {

enum class Error {
  Timeout,
  InvalidSync,
  InvalidFormat,
  UartError,
  HardwareFailure
};

struct Point {
  Degrees angle;
  Millimeters distance;
  std::uint8_t quality;
  bool start_flag;
};

class Lidar {
public:
  static constexpr std::size_t RxBufferSize = 1024;
  static constexpr uart_port_t UartPort = UART_NUM_2;
  static constexpr int TxPin = 26;
  static constexpr int RxPin = 25;
  static constexpr int BaudRate = 460800;

  explicit Lidar();
  ~Lidar();

  // Deleted copy to enforce RAII
  Lidar(const Lidar &) = delete;
  Lidar &operator=(const Lidar &) = delete;

  auto reset() -> void;
  auto stop() -> void;
  auto start_scan() -> std::expected<void, Error>;
  auto read_point() -> std::expected<Point, Error>;

private:
  auto read_descriptor(std::uint32_t timeout_ms) -> std::expected<void, Error>;
};

} // namespace rta::lidar

#endif // RTA_LIDAR_HPP
