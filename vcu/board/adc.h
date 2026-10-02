#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ADC1 channels on the VCU (ADC_CHANNEL_x is just x on the F4)
#define ADC_CH_REAR_BSE  0U  // PA0
#define ADC_CH_FRONT_BSE 1U  // PA1
#define ADC_CH_APPS1     11U // PC1
#define ADC_CH_APPS2     12U // PC2
#define ADC_CH_BPPS      13U // PC3
#define ADC_CH_STEERING  15U // PC5

// ADC1, 12-bit, single software-triggered conversions
void adc_init(void);

// One blocking conversion, as a fraction of full scale: raw * (1 / 4095.0f),
// the same math as Mbed's AnalogIn::read() on STM32
float adc_read(uint32_t channel);

// One blocking conversion, in volts (adc_read() * 3.3f)
float adc_read_voltage(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif
