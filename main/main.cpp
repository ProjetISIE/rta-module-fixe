#include "ble_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "rta_gatt.h"
#include <cmath>
#include <cstdio>
#include <cstring>

static const char *TAG = "RTA_FIXE_MAIN";

// Angular Filter Configuration
#define ANGLE_CENTER 0.0f
#define ANGLE_HALF_WIDTH 22.5f
#define DIST_MIN_MM 50.0f
#define DIST_MAX_MM 14000.0f

static inline float normalize_angle(float a) {
  while (a < 0.0f)
    a += 360.0f;
  while (a >= 360.0f)
    a -= 360.0f;
  return a;
}

static bool is_in_sector(float angle) {
  float lo = normalize_angle(ANGLE_CENTER - ANGLE_HALF_WIDTH);
  float hi = normalize_angle(ANGLE_CENTER + ANGLE_HALF_WIDTH);
  angle = normalize_angle(angle);
  if (lo <= hi)
    return (angle >= lo && angle <= hi);
  return (angle >= lo || angle <= hi);
}

void lidar_task(void *arg) {
  lidar_init();
  ESP_LOGI(TAG, "LiDAR UART Initialized");

  TickType_t last_valid_point_time = xTaskGetTickCount();
  uint32_t consecutive_errors = 0;
  const uint32_t max_consecutive_errors =
      20; // ~2 seconds of failure at 100ms timeout

  while (true) {
    // Step 1: Ensure LiDAR is in scanning state
    ESP_LOGI(TAG, "Attempting to start LiDAR scan...");
    lidar_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    lidar_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!lidar_start_scan()) {
      ESP_LOGE(TAG, "Failed to start LiDAR scan, retrying in 5s...");
      rta_gatt_update_distance(-1.0f);
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    ESP_LOGI(TAG, "LiDAR Scan started, entering data loop");
    last_valid_point_time = xTaskGetTickCount();
    consecutive_errors = 0;

    float min_dist_mm = DIST_MAX_MM + 1.0f;
    lidar_point_t pt;
    uint32_t cnt_scans = 0;
    uint32_t cnt_total = 0;
    TickType_t t_stats = xTaskGetTickCount();

    // Step 2: Data collection loop
    while (true) {
      if (lidar_read_point(&pt)) {
        last_valid_point_time = xTaskGetTickCount();
        consecutive_errors = 0;
        cnt_total++;

        if (pt.start_flag) {
          cnt_scans++;
          if (min_dist_mm <= DIST_MAX_MM) {
            rta_gatt_update_distance(min_dist_mm / 1000.0f);
          }
          min_dist_mm = DIST_MAX_MM + 1.0f;
        }

        if (is_in_sector(pt.angle)) {
          if (pt.distance >= DIST_MIN_MM && pt.distance <= DIST_MAX_MM) {
            if (pt.distance < min_dist_mm) {
              min_dist_mm = pt.distance;
            }
          }
        }
      } else {
        consecutive_errors++;
      }

      // Watchdog: Invalidate data if no valid point for > 1s
      if ((xTaskGetTickCount() - last_valid_point_time) > pdMS_TO_TICKS(1000)) {
        if (consecutive_errors % 10 == 0) { // Throttle log
          ESP_LOGW(TAG, "LiDAR data timeout (1s), marking as invalid");
        }
        rta_gatt_update_distance(-1.0f);
        // Reset timer to avoid spamming the log if still in failure
        last_valid_point_time = xTaskGetTickCount() - pdMS_TO_TICKS(500);
      }

      // Auto-recovery: If too many consecutive errors, break and restart scan
      if (consecutive_errors >= max_consecutive_errors) {
        ESP_LOGE(TAG, "Too many consecutive LiDAR errors (%lu), restarting...",
                 consecutive_errors);
        break;
      }

      // Periodic Stats
      if ((xTaskGetTickCount() - t_stats) > pdMS_TO_TICKS(5000)) {
        ESP_LOGI(TAG, "LiDAR Stats: Scans: %lu, Total Pts: %lu", cnt_scans,
                 cnt_total);
        cnt_scans = cnt_total = 0;
        t_stats = xTaskGetTickCount();
      }
    }
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting RTA Fixed Module (ESP-IDF)");

  // Initialize BLE stack
  if (ble_server_init("RTA_FIXE") != 0) {
    ESP_LOGE(TAG, "Failed to initialize BLE server");
    return;
  }

  // Register RTA GATT services
  if (ble_server_register_services(rta_gatt_svcs) != 0) {
    ESP_LOGE(TAG, "Failed to register RTA GATT services");
    return;
  }

  // Start BLE
  ble_server_start();

  // Start LiDAR task
  xTaskCreate(lidar_task, "lidar_task", 4096, nullptr, 5, nullptr);
}
