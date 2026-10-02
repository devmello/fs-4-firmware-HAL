#include "watchdog.h"

#include "board.h"

static IWDG_HandleTypeDef hiwdg;

void watchdog_init(void) {
    __HAL_DBGMCU_FREEZE_IWDG();

    // 32 kHz / 64 = 500 Hz, 125 counts = 250 ms. The LSI is only 17-47 kHz
    // (datasheet), so the real timeout is somewhere in 170-470 ms.
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 125U - 1U;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

void watchdog_refresh(void) {
    HAL_IWDG_Refresh(&hiwdg);
}
