#include "timebase.h"

#include "board.h"

// TIM5, same timer Mbed uses for its microsecond ticker
static TIM_HandleTypeDef htim5;

void timebase_init(void) {
    __HAL_RCC_TIM5_CLK_ENABLE();

    // Stop counting while the debugger has the core halted, so implausibility
    // timers don't run out at a breakpoint
    __HAL_DBGMCU_FREEZE_TIM5();

    // APB1 timers run at 2x PCLK1 when the APB1 prescaler isn't 1 (90 MHz here)
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timer_clock *= 2U;
    }

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = timer_clock / 1000000U - 1U;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFFU;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start(&htim5) != HAL_OK) {
        Error_Handler();
    }
}

uint32_t timebase_micros(void) {
    return TIM5->CNT;
}
