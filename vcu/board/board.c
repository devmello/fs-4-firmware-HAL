#include "board.h"

#include "adc.h"
#include "can.h"
#include "console.h"
#include "gpio.h"
#include "imu_uart.h"
#include "timebase.h"
#include "watchdog.h"

#if defined(BOARD_NUCLEO_F446RE)
// NUCLEO-F446RE: no crystal, the ST-LINK feeds 8 MHz into OSC_IN (bypass)
#if HSE_VALUE != 8000000U
#error "NUCLEO-F446RE HSE is 8 MHz from the ST-LINK, check HSE_VALUE"
#endif
#define HSE_MODE   RCC_HSE_BYPASS
#define HSE_PLLM   4  // 8 MHz / 4 = 2 MHz
#else
#if HSE_VALUE != 24000000U
#error "VCU has a 24 MHz crystal, check HSE_VALUE"
#endif
#define HSE_MODE   RCC_HSE_ON
#define HSE_PLLM   12 // 24 MHz / 12 = 2 MHz
#endif

// HSE -> PLL -> 180 MHz SYSCLK, APB1 45 MHz, APB2 90 MHz. On the VCU it's the
// same as SetSysClock_PLL_HSE() in the Mbed custom target.
static void clock_init(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = HSE_MODE;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = HSE_PLLM;      // 2 MHz VCO input
    osc.PLL.PLLN = 180;           // 2 MHz * 180 = 360 MHz VCO
    osc.PLL.PLLP = RCC_PLLP_DIV2; // 360 / 2 = 180 MHz SYSCLK
    osc.PLL.PLLQ = 8;             // 45 MHz, 48 MHz domain is unused but keep it in spec
    osc.PLL.PLLR = 2;             // unused (no I2S/SAI)
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    // Needed above 168 MHz
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1; // 180 MHz
    clk.APB1CLKDivider = RCC_HCLK_DIV4;  //  45 MHz (CAN, console UART, TIM5 at 90 MHz)
    clk.APB2CLKDivider = RCC_HCLK_DIV2;  //  90 MHz (ADC)
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

static uint32_t reset_flags;

void board_init(void) {
    reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    HAL_Init();

    // Outputs low before anything that can take a while (HSE startup), then
    // the watchdog, so a hang anywhere in init ends in a reset
    gpio_init();
    watchdog_init();
    clock_init();

    timebase_init();
    console_init();
    adc_init();
    can_init();
    imu_uart_init();
}

const char *board_reset_cause(void) {
    if ((reset_flags & RCC_CSR_IWDGRSTF) != 0U) {
        return "watchdog";
    }
    if ((reset_flags & RCC_CSR_WWDGRSTF) != 0U) {
        return "window watchdog";
    }
    if ((reset_flags & RCC_CSR_SFTRSTF) != 0U) {
        return "software";
    }
    if ((reset_flags & RCC_CSR_LPWRRSTF) != 0U) {
        return "low power";
    }
    if ((reset_flags & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) != 0U) {
        return "power on";
    }
    if ((reset_flags & RCC_CSR_PINRSTF) != 0U) {
        return "reset pin";
    }
    return "unknown";
}

void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void Error_Handler(void) {
    __disable_irq();

    // Say where it failed, like Mbed's error report, then stop
    char message[] = "\r\nError_Handler from 0x00000000\r\n";
    char *digits = message + sizeof("\r\nError_Handler from 0x") - 1;
    uintptr_t caller = (uintptr_t)__builtin_return_address(0);
    for (int i = 0; i < 8; i++) {
        digits[i] = "0123456789abcdef"[(caller >> (28 - 4 * i)) & 0xFU];
    }
    console_write_blocking(message);

    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
