#ifndef CONSOLE_H
#define CONSOLE_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

// UART4 on PC10 (TX) / PC11 (RX), 115200 8N1, wired to the onboard
// STLINK-V3MODS virtual COM port. printf() goes here.
extern UART_HandleTypeDef huart4;

void console_init(void);

// Polled write straight to UART4, works with interrupts off. Only for
// Error_Handler. Does nothing if the console isn't set up yet.
void console_write_blocking(const char *text);

#ifdef __cplusplus
}
#endif

#endif
