#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// TIM5 counts microseconds in 32 bits, so it wraps every ~71.6 min. Its update
// interrupt counts the wraps, which are the upper 32 bits. 64 bits of
// microseconds last ~584,000 years, so times compare and subtract directly.
void timebase_init(void);

// Microseconds since timebase_init(), 0 before it
uint64_t timebase_micros(void);

#ifdef __cplusplus
}
#endif

#endif
