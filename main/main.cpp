#include "ble_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "rta_gatt.h"
#include <cstdio>
#include <cstring>
#include <cmath>

static const char *TAG = "RTA_FIXE_MAIN";

// Angular Filter Configuration
#define ANGLE_CENTER     0.0f
#define ANGLE_HALF_WIDTH 22.5f
#define DIST_MIN_MM      50.0f
#define DIST_MAX_MM      14000.0f

static inline float normalize_angle(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static bool is_in_sector(float angle) {
    float lo = normalize_angle(ANGLE_CENTER - ANGLE_HALF_WIDTH);
    float hi = normalize_angle(ANGLE_CENTER + ANGLE_HALF_WIDTH);
    angle = normalize_angle(angle);
    if (lo <= hi) return (angle >= lo && angle <= hi);
    return (angle >= lo || angle <= hi);
}

void lidar_task(void *arg) {
    lidar_init();
    ESP_LOGI(TAG, "LiDAR UART Initialized");

    lidar_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    lidar_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!lidar_start_scan()) {
        ESP_LOGE(TAG, "Failed to start LiDAR scan");
        vTaskDelete(NULL);
        return;
    }

    float min_dist_mm = DIST_MAX_MM + 1.0f;
    uint32_t cnt_scans = 0;
    uint32_t cnt_total = 0;
    uint32_t cnt_filtered = 0;
    TickType_t t_stats = xTaskGetTickCount();

    lidar_point_t pt;

    while (true) {
        if (lidar_read_point(&pt)) {
            cnt_total++;

            if (pt.start_flag) {
                cnt_scans++;
                if (min_dist_mm <= DIST_MAX_MM) {
                    float dist_m = min_dist_mm / 1000.0f;
                    rta_gatt_update_distance(dist_m);
                    
                    // Simple console update
                    std::printf("\r[SCAN %5lu] Dist: %7.3f m | Points: %lu (filt: %lu)          ",
                                cnt_scans, dist_m, cnt_total, cnt_filtered);
                    std::fflush(stdout);
                }
                min_dist_mm = DIST_MAX_MM + 1.0f;
            }

            if (is_in_sector(pt.angle)) {
                cnt_filtered++;
                if (pt.distance >= DIST_MIN_MM && pt.distance <= DIST_MAX_MM) {
                    if (pt.distance < min_dist_mm) {
                        min_dist_mm = pt.distance;
                    }
                }
            }
        }

        // Stats every 5 seconds
        if ((xTaskGetTickCount() - t_stats) > pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "Stats: Scans: %lu, Total Pts: %lu, Filtered Pts: %lu", 
                     cnt_scans, cnt_total, cnt_filtered);
            cnt_scans = cnt_total = cnt_filtered = 0;
            t_stats = xTaskGetTickCount();
        }
        
        // Yield to prevent watchdog issues if needed, though lidar_read_point blocks/delays
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
