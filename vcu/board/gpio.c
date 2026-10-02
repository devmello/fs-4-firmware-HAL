#include "gpio.h"

#include "board.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} pin_t;

#if defined(BOARD_NUCLEO_F446RE)
// NUCLEO-F446RE: the RTD light is LD2 (PA5) so it can be seen on the bench,
// and the RTD button is B1 (PC13), which pulls low when pressed. PA2/PA3 are
// the console here, so the BSPD inputs aren't connected and read 0.
static const pin_t outputs[OUT_COUNT] = {
    [OUT_RTD_LIGHT] = {GPIOA, GPIO_PIN_5},
    [OUT_RTD_BUZZER] = {GPIOA, GPIO_PIN_7},
    [OUT_SOLENOID] = {GPIOB, GPIO_PIN_1},
    [OUT_BRAKELIGHT] = {GPIOC, GPIO_PIN_4},
};
#define RTD_BUTTON_EDGE   GPIO_MODE_IT_FALLING
#define RTD_BUTTON_ACTIVE GPIO_PIN_RESET
#define HAS_BSPD_INPUTS   0
#else
// fs-4 VCU, from the schematic. Every output drives a low side FET.
static const pin_t outputs[OUT_COUNT] = {
    [OUT_RTD_LIGHT] = {GPIOC, GPIO_PIN_0},
    [OUT_RTD_BUZZER] = {GPIOA, GPIO_PIN_7},
    [OUT_SOLENOID] = {GPIOB, GPIO_PIN_1},
    [OUT_BRAKELIGHT] = {GPIOC, GPIO_PIN_4},
};
#define RTD_BUTTON_EDGE   GPIO_MODE_IT_RISING
#define RTD_BUTTON_ACTIVE GPIO_PIN_SET
#define HAS_BSPD_INPUTS   1
#endif

static const pin_t inputs[IN_COUNT] = {
    [IN_RTD_BUTTON] = {GPIOC, GPIO_PIN_13},
    [IN_BSPD_FAULT] = {GPIOA, GPIO_PIN_2},
    [IN_BSPD_SHUTDOWN] = {GPIOA, GPIO_PIN_3},
};

static volatile uint32_t rtd_rises;

void gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Low in the output latch before the pin becomes an output, so it never
    // drives high. Q7 (RTD light) has no gate pull-down, so until here it can
    // float on.
    GPIO_InitTypeDef gpio = {0};
    for (int i = 0; i < OUT_COUNT; i++) {
        HAL_GPIO_WritePin(outputs[i].port, outputs[i].pin, GPIO_PIN_RESET);
        gpio.Pin = outputs[i].pin;
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(outputs[i].port, &gpio);
    }

#if HAS_BSPD_INPUTS
    // Both come through dividers on the board, no pull needed
    gpio.Pin = inputs[IN_BSPD_FAULT].pin | inputs[IN_BSPD_SHUTDOWN].pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
#endif

    // RC filtered on the board. No pull, like Mbed's InterruptIn.
    gpio.Pin = inputs[IN_RTD_BUTTON].pin;
    gpio.Mode = RTD_BUTTON_EDGE;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(inputs[IN_RTD_BUTTON].port, &gpio);

    // EXTI line 13 is shared with PB13 (BNO086 INT), which isn't used
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void gpio_write(gpio_output_t output, bool level) {
    HAL_GPIO_WritePin(outputs[output].port, outputs[output].pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool gpio_read(gpio_input_t input) {
    if (input == IN_RTD_BUTTON) {
        return HAL_GPIO_ReadPin(inputs[input].port, inputs[input].pin) == RTD_BUTTON_ACTIVE;
    }
#if HAS_BSPD_INPUTS
    return HAL_GPIO_ReadPin(inputs[input].port, inputs[input].pin) == GPIO_PIN_SET;
#else
    return false;
#endif
}

uint32_t gpio_rtd_button_rises(void) {
    return rtd_rises;
}

void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(inputs[IN_RTD_BUTTON].pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin == inputs[IN_RTD_BUTTON].pin) {
        rtd_rises++;
    }
}
