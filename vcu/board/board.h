#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// HAL, clocks and every peripheral the firmware uses
void board_init(void);

// What caused the last reset: "power on", "reset pin", "watchdog", ...
const char *board_reset_cause(void);

// Init failed. Interrupts off and spin, so nothing more goes out on CAN,
// until the watchdog resets the chip (~250 ms) and init runs again.
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
