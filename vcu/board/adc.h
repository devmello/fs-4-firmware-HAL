#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ADC1, 12-bit, single software-triggered conversions
void adc_init(void);

// One blocking conversion on an ADC1 channel (ADC_CHANNEL_x), in volts
float adc_read_voltage(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif
