#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// HAL, clocks and every peripheral the firmware uses
void board_init(void);

// Init failed. Interrupts off, spin forever, nothing more goes out on CAN.
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
