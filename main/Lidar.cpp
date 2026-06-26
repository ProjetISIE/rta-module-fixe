#include "Lidar.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

namespace rta::lidar {

static const char* TAG = "LIDAR";

// RPLiDAR Commands
namespace cmd {
constexpr std::uint8_t STOP = 0x25;
constexpr std::uint8_t RESET = 0x40;
constexpr std::uint8_t SCAN = 0x20;
constexpr std::uint8_t SYNC1 = 0xA5;
constexpr std::uint8_t SYNC2 = 0x5A;
} // namespace cmd

Lidar::Lidar() {
    uart_config_t uart_config = {
        .baud_rate = kBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(
        uart_driver_install(kUartPort, kRxBufferSize, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(kUartPort, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(kUartPort, kTxPin, kRxPin, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

Lidar::~Lidar() {
    stop();
    uart_driver_delete(kUartPort);
}

auto Lidar::reset() -> void {
    const std::uint8_t pkt[2] = {cmd::SYNC1, cmd::RESET};
    uart_write_bytes(kUartPort, reinterpret_cast<const char*>(pkt),
                     sizeof(pkt));
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(kUartPort);
}

auto Lidar::stop() -> void {
    const std::uint8_t pkt[2] = {cmd::SYNC1, cmd::STOP};
    uart_write_bytes(kUartPort, reinterpret_cast<const char*>(pkt),
                     sizeof(pkt));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush(kUartPort);
}

auto Lidar::start_scan() -> std::expected<void, Error> {
    uart_flush(kUartPort);
    const std::uint8_t pkt[2] = {cmd::SYNC1, cmd::SCAN};
    uart_write_bytes(kUartPort, reinterpret_cast<const char*>(pkt),
                     sizeof(pkt));

    auto result = read_descriptor(2000);
    if (!result) return result;

    ESP_LOGI(TAG, "Scan started successfully");
    return {};
}

auto Lidar::read_descriptor(std::uint32_t timeout_ms)
    -> std::expected<void, Error> {
    std::uint8_t d[7];
    int len =
        uart_read_bytes(kUartPort, d, sizeof(d), pdMS_TO_TICKS(timeout_ms));
    if (len < static_cast<int>(sizeof(d))) {
        ESP_LOGE(TAG, "Timeout descriptor");
        return std::unexpected(Error::Timeout);
    }
    if (d[0] != cmd::SYNC1 || d[1] != cmd::SYNC2) {
        ESP_LOGE(TAG, "Invalid Sync: 0x%02X 0x%02X", d[0], d[1]);
        return std::unexpected(Error::InvalidSync);
    }

    std::uint32_t raw = d[2] | (d[3] << 8) | (d[4] << 16) |
                        (static_cast<std::uint32_t>(d[5]) << 24);
    std::uint32_t d_len = raw & 0x3FFFFFFF;
    std::uint8_t s_mode = (raw >> 30) & 0x03;

    if (d_len != 5 || s_mode != 1) {
        ESP_LOGE(TAG, "Unexpected format dLen=%lu sMode=%d", d_len, s_mode);
        return std::unexpected(Error::InvalidFormat);
    }
    return {};
}

auto Lidar::read_point() -> std::expected<Point, Error> {
    std::uint8_t d[5];
    int len = uart_read_bytes(kUartPort, d, sizeof(d), pdMS_TO_TICKS(100));
    if (len < static_cast<int>(sizeof(d))) {
        return std::unexpected(Error::Timeout);
    }

    if ((d[1] & 0x01) != 1) {
        return std::unexpected(Error::InvalidSync);
    }

    std::uint8_t S = d[0] & 0x01;
    std::uint8_t Sb = (d[0] >> 1) & 0x01;
    if (S == Sb) {
        return std::unexpected(Error::InvalidFormat);
    }

    Point pt;
    pt.start_flag = (S == 1);
    pt.quality = (d[0] >> 2) & 0x3F;

    std::uint16_t a_raw =
        ((d[1] >> 1) & 0x7F) | (static_cast<std::uint16_t>(d[2]) << 7);
    pt.angle = Degrees{a_raw / 64.0f};

    std::uint16_t d_raw = d[3] | (static_cast<std::uint16_t>(d[4]) << 8);
    pt.distance = Millimeters{static_cast<std::uint16_t>(d_raw / 4.0f)};

    return pt;
}

} // namespace rta::lidar
