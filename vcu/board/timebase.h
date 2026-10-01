#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// TIM5 as a free running 32-bit counter at 1 MHz (wraps every ~71.6 min)
void timebase_init(void);

uint32_t timebase_micros(void);

#ifdef __cplusplus
}
#endif

#endif
