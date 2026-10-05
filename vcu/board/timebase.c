#include "timebase.h"

#include "board.h"

// TIM5, same timer Mbed uses for its microsecond ticker
static TIM_HandleTypeDef htim5;

// Times TIM5 has wrapped, the upper 32 bits of timebase_micros()
static volatile uint32_t tim5_wraps;

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

    // HAL_TIM_Base_Init loads the prescaler with an update event, which also
    // sets UIF. Clear it, or it would count as a wrap.
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

    // Same priority as the other interrupts, so none of them can run between
    // the handler clearing UIF and counting the wrap
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK) {
        Error_Handler();
    }
}

// Once per wrap, every ~71.6 min. UIF is checked because the write that
// clears it can reach TIM5 after the handler returns, which runs it again.
void TIM5_IRQHandler(void) {
    if ((TIM5->SR & TIM_SR_UIF) != 0U) {
        TIM5->SR = ~TIM_SR_UIF;
        tim5_wraps++;
    }
}

uint64_t timebase_micros(void) {
    // Interrupts off, so the wrap count can't change between the reads
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t high = tim5_wraps;
    uint32_t low = TIM5->CNT;
    // UIF is still set if TIM5 wrapped since its interrupt last ran, which it
    // can't while interrupts are off. Count that wrap here, and read the
    // counter again in case the first read came before it.
    if ((TIM5->SR & TIM_SR_UIF) != 0U) {
        high++;
        low = TIM5->CNT;
    }
    __set_PRIMASK(primask);
    return ((uint64_t)high << 32) | low;
}
