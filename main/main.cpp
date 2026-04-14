#include "ble_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

static const char *TAG = "MAIN_APP";
// The GATT service definition is exported by the C definition file
// extern "C" const struct ble_gatt_svc_def fixe_gatt_svcs[];

void process_and_display_task(void *arg) {
  while (true) {
    std::printf("\r[WAITING] No fix data parsed yet...          ");
    std::fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting BLE Server");
  // Initialize BLE stack
  if (ble_server_init("RTA_FIXE") != 0) {
    ESP_LOGE(TAG, "Failed to initialize BLE server");
    return;
  }
  // Register and Start BLE
  // ble_server_register_services(fixe_gatt_svcs);
  ble_server_start();
  // Start local display task
  xTaskCreate(process_and_display_task, "display_task", 4096, nullptr, 5,
              nullptr);
}
