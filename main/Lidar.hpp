#ifndef RTA_LIDAR_HPP_
#define RTA_LIDAR_HPP_

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
  static constexpr std::size_t kRxBufferSize = 1024;
  static constexpr uart_port_t kUartPort = UART_NUM_2;
  static constexpr int kTxPin = 26;
  static constexpr int kRxPin = 25;
  static constexpr int kBaudRate = 460800;

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

#endif // RTA_LIDAR_HPP_
