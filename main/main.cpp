#include "BleServer.hpp"
#include "Lidar.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rta_gatt.hpp"
#include "units.hpp"
#include <cmath>

using namespace rta;

static const char *TAG = "RTA_FIXE_MAIN";

// Angular Filter Configuration
static constexpr Degrees ANGLE_CENTER{0.0f};
static constexpr Degrees ANGLE_HALF_WIDTH{22.5f};
static constexpr Millimeters DIST_MIN{50};
static constexpr Millimeters DIST_MAX{14000};

[[nodiscard]] static constexpr auto normalize_angle(Degrees a) -> Degrees {
  while (a.value < 0.0f)
    a.value += 360.0f;
  while (a.value >= 360.0f)
    a.value -= 360.0f;
  return a;
}

[[nodiscard]] static constexpr auto is_in_sector(Degrees angle) -> bool {
  const float lo =
      normalize_angle(Degrees{ANGLE_CENTER.value - ANGLE_HALF_WIDTH.value})
          .value;
  const float hi =
      normalize_angle(Degrees{ANGLE_CENTER.value + ANGLE_HALF_WIDTH.value})
          .value;
  const float a = normalize_angle(angle).value;
  if (lo <= hi)
    return (a >= lo && a <= hi);
  return (a >= lo || a <= hi);
}

void lidar_task(void *) {
  lidar::Lidar sensor;
  ESP_LOGI(TAG, "LiDAR RAII Initialized");

  TickType_t last_valid_point_time = xTaskGetTickCount();
  std::uint32_t consecutive_errors = 0;
  static constexpr std::uint32_t MAX_CONSECUTIVE_ERRORS = 20;

  while (true) {
    ESP_LOGI(TAG, "Attempting to start LiDAR scan...");
    sensor.stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    sensor.reset();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (auto res = sensor.start_scan(); !res) {
      ESP_LOGE(TAG, "Failed to start LiDAR scan, retrying in 5s...");
      ble::invalidate_distance();
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    ESP_LOGI(TAG, "LiDAR Scan started, entering data loop");
    last_valid_point_time = xTaskGetTickCount();
    consecutive_errors = 0;

    Millimeters min_dist{static_cast<std::uint16_t>(DIST_MAX.value + 1)};
    std::uint32_t cnt_scans = 0;
    std::uint32_t cnt_total = 0;
    TickType_t t_stats = xTaskGetTickCount();

    while (true) {
      auto point_res = sensor.read_point();
      if (point_res) {
        const auto &pt = *point_res;
        last_valid_point_time = xTaskGetTickCount();
        consecutive_errors = 0;
        cnt_total++;

        if (pt.start_flag) {
          cnt_scans++;
          if (min_dist <= DIST_MAX) {
            ble::update_distance(min_dist);
          }
          min_dist =
              Millimeters{static_cast<std::uint16_t>(DIST_MAX.value + 1)};
        }

        if (is_in_sector(pt.angle)) {
          if (pt.distance >= DIST_MIN && pt.distance <= DIST_MAX) {
            if (pt.distance < min_dist) {
              min_dist = pt.distance;
            }
          }
        }
      } else {
        consecutive_errors++;
      }

      // Watchdog: Invalidate data if no valid point for > 1s
      if ((xTaskGetTickCount() - last_valid_point_time) > pdMS_TO_TICKS(1000)) {
        if (consecutive_errors % 10 == 0) {
          ESP_LOGW(TAG, "LiDAR data timeout (1s), marking as invalid");
        }
        ble::invalidate_distance();
        last_valid_point_time = xTaskGetTickCount() - pdMS_TO_TICKS(500);
      }

      if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
        ESP_LOGE(TAG, "Too many consecutive LiDAR errors (%lu), restarting...",
                 consecutive_errors);
        break;
      }

      if ((xTaskGetTickCount() - t_stats) > pdMS_TO_TICKS(5000)) {
        ESP_LOGI(TAG, "LiDAR Stats: Scans: %lu, Total Pts: %lu", cnt_scans,
                 cnt_total);
        cnt_scans = cnt_total = 0;
        t_stats = xTaskGetTickCount();
      }
    }
  }
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "Starting RTA Fixed Module (C++23)");

  static ble::BleServer ble_server("RTA_FIXE");

  if (ble_server.register_services(ble::gatt_svcs) != 0) {
    ESP_LOGE(TAG, "Failed to register RTA GATT services");
    return;
  }

  ble_server.start();

  xTaskCreate(lidar_task, "lidar_task", 4096, nullptr, 5, nullptr);
}
