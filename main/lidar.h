#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_BAUD   460800
#define LIDAR_UART_PORT UART_NUM_2

// RPLiDAR Commands
#define RPLIDAR_CMD_STOP       0x25
#define RPLIDAR_CMD_RESET      0x40
#define RPLIDAR_CMD_SCAN       0x20
#define RPLIDAR_CMD_GET_INFO   0x50
#define RPLIDAR_CMD_GET_HEALTH 0x52
#define RPLIDAR_ANS_SYNC1      0xA5
#define RPLIDAR_ANS_SYNC2      0x5A

typedef struct {
    float angle;
    float distance;
    uint8_t quality;
    bool start_flag;
} lidar_point_t;

/**
 * @brief Initialize LiDAR UART.
 */
void lidar_init(void);

/**
 * @brief Send a command to the LiDAR.
 */
void lidar_send_cmd(uint8_t cmd);

/**
 * @brief Reset the LiDAR.
 */
void lidar_reset(void);

/**
 * @brief Stop the LiDAR.
 */
void lidar_stop(void);

/**
 * @brief Start a scan.
 * @return true if successful.
 */
bool lidar_start_scan(void);

/**
 * @brief Read a single point from the LiDAR stream.
 * @param point Pointer to store the point data.
 * @return true if a valid point was read.
 */
bool lidar_read_point(lidar_point_t *point);

#ifdef __cplusplus
}
#endif

#endif // LIDAR_H
