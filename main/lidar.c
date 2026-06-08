#include "lidar.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static const char *TAG = "LIDAR";

void lidar_init(void) {
  uart_config_t uart_config = {
      .baud_rate = LIDAR_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART_PORT, 1024, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_PORT, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART_PORT, LIDAR_TX_PIN, LIDAR_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void lidar_send_cmd(uint8_t cmd) {
  uint8_t pkt[2] = {RPLIDAR_ANS_SYNC1, cmd};
  uart_write_bytes(LIDAR_UART_PORT, (const char *)pkt, 2);
}

static bool lidar_read_descriptor(uint32_t *d_len, uint8_t *send_mode,
                                  uint8_t *d_type, uint32_t timeout_ms) {
  uint8_t d[7];
  int len = uart_read_bytes(LIDAR_UART_PORT, d, 7, pdMS_TO_TICKS(timeout_ms));
  if (len < 7) {
    ESP_LOGE(TAG, "Timeout descriptor");
    return false;
  }
  if (d[0] != RPLIDAR_ANS_SYNC1 || d[1] != RPLIDAR_ANS_SYNC2) {
    ESP_LOGE(TAG, "Invalid Sync: 0x%02X 0x%02X", d[0], d[1]);
    return false;
  }
  uint32_t raw = d[2] | (d[3] << 8) | (d[4] << 16) | ((uint32_t)d[5] << 24);
  *d_len = raw & 0x3FFFFFFF;
  *send_mode = (raw >> 30) & 0x03;
  *d_type = d[6];
  return true;
}

void lidar_reset(void) {
  lidar_send_cmd(RPLIDAR_CMD_RESET);
  vTaskDelay(pdMS_TO_TICKS(500));
  uart_flush(LIDAR_UART_PORT);
}

void lidar_stop(void) {
  lidar_send_cmd(RPLIDAR_CMD_STOP);
  vTaskDelay(pdMS_TO_TICKS(100));
  uart_flush(LIDAR_UART_PORT);
}

bool lidar_start_scan(void) {
  uart_flush(LIDAR_UART_PORT);
  lidar_send_cmd(RPLIDAR_CMD_SCAN);
  uint32_t d_len;
  uint8_t s_mode, d_type;
  if (!lidar_read_descriptor(&d_len, &s_mode, &d_type, 2000))
    return false;
  if (d_len != 5 || s_mode != 1) {
    ESP_LOGE(TAG, "Unexpected format dLen=%lu sMode=%d", d_len, s_mode);
    return false;
  }
  ESP_LOGI(TAG, "Scan started successfully");
  return true;
}

bool lidar_read_point(lidar_point_t *point) {
  uint8_t d[5];
  int len = uart_read_bytes(LIDAR_UART_PORT, d, 5, pdMS_TO_TICKS(100));
  if (len < 5)
    return false;

  // Check if it's a valid data point packet
  // Arduino: if ((d[1]&0x01)!=1) return false;
  if ((d[1] & 0x01) != 1) {
    // Not a valid point packet, might be sync issue.
    // In Arduino, it returns false and the loop continues.
    // We might need to resync by reading one byte if this happens often.
    return false;
  }

  uint8_t S = d[0] & 0x01;
  uint8_t Sb = (d[0] >> 1) & 0x01;
  if (S == Sb)
    return false; // Invalid start flags

  point->start_flag = (S == 1);
  point->quality = (d[0] >> 2) & 0x3F;

  uint16_t a_raw = ((d[1] >> 1) & 0x7F) | ((uint16_t)d[2] << 7);
  point->angle = a_raw / 64.0f;

  uint16_t d_raw = d[3] | ((uint16_t)d[4] << 8);
  point->distance = d_raw / 4.0f;

  return true;
}
