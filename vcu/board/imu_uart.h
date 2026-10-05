#ifndef IMU_UART_H
#define IMU_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART5 to the VectorNav VN-200: PC12 (TX), PD2 (RX), 115200 8N1. RX runs
// continuously by circular DMA into a ring buffer, TX by interrupt.
#define IMU_UART_BAUD 115200U

void imu_uart_init(void);

// Copies out the bytes received since the last call, up to max. Call every
// pass of the main loop; the ring holds about 100 ms of sensor output.
size_t imu_uart_read(uint8_t *dst, size_t max);

// Starts sending len bytes (copied, at most 128). False if the previous send
// hasn't finished or len is too long.
bool imu_uart_write(const uint8_t *data, size_t len);

// Bytes lost because the ring was full when the main loop got to it
uint32_t imu_uart_dropped(void);

#ifdef __cplusplus
}
#endif

#endif
