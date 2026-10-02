#ifndef CONSOLE_H
#define CONSOLE_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

// 115200 8N1 to the ST-LINK virtual COM port, printf() goes here. On the VCU
// that's UART4 on PC10/PC11, on a NUCLEO-F446RE USART2 on PA2/PA3.
extern UART_HandleTypeDef huart_console;

void console_init(void);

// Polled write straight to the UART, works with interrupts off. Only for
// Error_Handler. Does nothing if the console isn't set up yet.
void console_write_blocking(const char *text);

#ifdef __cplusplus
}
#endif

#endif
