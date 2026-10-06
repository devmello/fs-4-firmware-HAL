#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OUT_RTD_LIGHT,  // PC0, Q7: RTD LED and onboard D19
    OUT_RTD_BUZZER, // PA7, Q8
    OUT_SOLENOID,   // PB1, Q10: regen brake solenoid
    OUT_BRAKELIGHT, // PC4, Q9
    OUT_COUNT
} gpio_output_t;

typedef enum {
    IN_RTD_BUTTON,    // PC13, high when pressed
    IN_BSPD_FAULT,    // PA2
    IN_BSPD_SHUTDOWN, // PA3
    IN_COUNT
} gpio_input_t;

// Drives every output low, then sets up the inputs and the RTD button
// interrupt. Runs first in board_init(), before the clocks.
void gpio_init(void);

void gpio_write(gpio_output_t output, bool level);

bool gpio_read(gpio_input_t input);

// Rising edges on the RTD button since boot, counted by the EXTI interrupt.
// The main loop compares it with the last value it saw.
uint32_t gpio_rtd_button_rises(void);

#ifdef __cplusplus
}
#endif

#endif
