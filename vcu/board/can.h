#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

// CAN1 on PB8 (RX) / PB9 (TX), powertrain bus at 500 kbit/s
extern CAN_HandleTypeDef hcan1;

void can_init(void);

// Queues a standard ID data frame. False if all 3 TX mailboxes are full.
bool can_write(CAN_HandleTypeDef *hcan, uint32_t id, const uint8_t *data, uint8_t len);

// Transmit error counter (ESR.TEC), same as Mbed's CAN::tderror(). Only the
// low 8 bits, so it wraps back to ~0 when the controller goes bus-off.
uint8_t can_tx_error_count(const CAN_HandleTypeDef *hcan);

// True once the controller is bus-off. It stays that way until reset.
bool can_bus_off(const CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif
