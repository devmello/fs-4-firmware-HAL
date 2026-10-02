#include "adc.h"

#include "board.h"

#define ADC_VREF       3.3f    // VDDA = +3V3A
#define ADC_FULL_SCALE 4095.0f // 12-bit

ADC_HandleTypeDef hadc1;

static uint32_t configured_channel = UINT32_MAX;

void adc_init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 90 / 4 = 22.5 MHz, F446 max is 36
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) {
        return;
    }

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    // PA0 = rear brake pressure (IN0), PA1 = front brake pressure (IN1)
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PC1 = APPS_1 (IN11), PC2 = APPS_2 (IN12), PC3 = BPPS (IN13), PC5 = steering (IN15)
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);
}

float adc_read(uint32_t channel) {
    // Only touch the sequencer when the channel changes
    if (channel != configured_channel) {
        ADC_ChannelConfTypeDef config = {0};
        config.Channel = channel;
        config.Rank = 1;
        // APPS/BPPS/steering: 7.5k source (12k || 20k). Brake pressure has no
        // cap at the pin. Both need >= 28 cycles at 22.5 MHz, 56 for margin.
        config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) {
            Error_Handler();
        }
        configured_channel = channel;
    }

    // 0 V on failure reads as an out of range sensor to the ETC
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0.0f;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) {
        return 0.0f;
    }

    // Same as Mbed's analogin_read() on STM32
    return (float)HAL_ADC_GetValue(&hadc1) * (1.0f / ADC_FULL_SCALE);
}

float adc_read_voltage(uint32_t channel) {
    // Same order of operations as Mbed's AnalogIn::read_voltage()
    return adc_read(channel) * ADC_VREF;
}
