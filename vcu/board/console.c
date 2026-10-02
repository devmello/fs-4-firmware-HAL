#include "console.h"

#include <stdio.h>

// Interrupt driven TX so printf() never stalls the control loop, like Mbed's
// BufferedSerial. If the buffer fills up the extra output is dropped.
#define TX_BUFFER_SIZE 1024U // power of 2

#if defined(BOARD_NUCLEO_F446RE)
// NUCLEO-F446RE: the ST-LINK's virtual COM port is USART2, PA2 (TX) / PA3 (RX)
#define CONSOLE_UART        USART2
#define CONSOLE_IRQn        USART2_IRQn
#define CONSOLE_IRQHandler  USART2_IRQHandler
#define CONSOLE_CLK_BIT     RCC_APB1ENR_USART2EN
#define CONSOLE_CLK_ENABLE  __HAL_RCC_USART2_CLK_ENABLE
#define CONSOLE_GPIO        GPIOA
#define CONSOLE_GPIO_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE
#define CONSOLE_PINS        (GPIO_PIN_2 | GPIO_PIN_3)
#define CONSOLE_AF          GPIO_AF7_USART2
#else
// VCU: UART4, PC10 (TX) / PC11 (RX), to the STLINK-V3MODS virtual COM port
#define CONSOLE_UART        UART4
#define CONSOLE_IRQn        UART4_IRQn
#define CONSOLE_IRQHandler  UART4_IRQHandler
#define CONSOLE_CLK_BIT     RCC_APB1ENR_UART4EN
#define CONSOLE_CLK_ENABLE  __HAL_RCC_UART4_CLK_ENABLE
#define CONSOLE_GPIO        GPIOC
#define CONSOLE_GPIO_ENABLE __HAL_RCC_GPIOC_CLK_ENABLE
#define CONSOLE_PINS        (GPIO_PIN_10 | GPIO_PIN_11)
#define CONSOLE_AF          GPIO_AF8_UART4
#endif

UART_HandleTypeDef huart_console;

static uint8_t tx_buffer[TX_BUFFER_SIZE];
static char stdout_buffer[128];
static volatile uint32_t tx_head;    // next free slot, only main writes
static volatile uint32_t tx_tail;    // oldest unsent byte, only the ISR writes
static volatile uint16_t tx_in_flight;

void console_init(void) {
    huart_console.Instance = CONSOLE_UART;
    huart_console.Init.BaudRate = 115200;
    huart_console.Init.WordLength = UART_WORDLENGTH_8B;
    huart_console.Init.StopBits = UART_STOPBITS_1;
    huart_console.Init.Parity = UART_PARITY_NONE;
    huart_console.Init.Mode = UART_MODE_TX_RX;
    huart_console.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_console.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart_console) != HAL_OK) {
        Error_Handler();
    }

    // Line buffered like Mbed, so _write gets whole lines
    setvbuf(stdout, stdout_buffer, _IOLBF, sizeof(stdout_buffer));
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance != CONSOLE_UART) {
        return;
    }

    CONSOLE_CLK_ENABLE();
    CONSOLE_GPIO_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = CONSOLE_PINS;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = CONSOLE_AF;
    HAL_GPIO_Init(CONSOLE_GPIO, &gpio);

    HAL_NVIC_SetPriority(CONSOLE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CONSOLE_IRQn);
}

// Sends the next contiguous chunk of the buffer. Called from the ISR, or from
// main with the console interrupt masked.
static void start_next_chunk(void) {
    if (tx_in_flight != 0U || tx_head == tx_tail) {
        return;
    }

    uint32_t start = tx_tail % TX_BUFFER_SIZE;
    uint32_t count = tx_head - tx_tail;
    if (count > TX_BUFFER_SIZE - start) {
        count = TX_BUFFER_SIZE - start;
    }

    tx_in_flight = (uint16_t)count;
    if (HAL_UART_Transmit_IT(&huart_console, &tx_buffer[start], (uint16_t)count) != HAL_OK) {
        tx_in_flight = 0;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != CONSOLE_UART) {
        return;
    }
    tx_tail += tx_in_flight;
    tx_in_flight = 0;
    start_next_chunk();
}

void CONSOLE_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart_console);
}

static void put_byte(uint8_t byte) {
    if (tx_head - tx_tail < TX_BUFFER_SIZE) {
        tx_buffer[tx_head % TX_BUFFER_SIZE] = byte;
        __COMPILER_BARRIER(); // byte must be in the buffer before the ISR can see it
        tx_head++;
    }
}

// newlib calls this for stdout/stderr. "\n" becomes "\r\n" for serial terminals.
// Not for use from interrupts.
int _write(int file, char *ptr, int len) {
    (void)file;

    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            put_byte('\r');
        }
        put_byte((uint8_t)ptr[i]);
    }

    HAL_NVIC_DisableIRQ(CONSOLE_IRQn);
    start_next_chunk();
    HAL_NVIC_EnableIRQ(CONSOLE_IRQn);

    return len;
}

void console_write_blocking(const char *text) {
    if ((RCC->APB1ENR & CONSOLE_CLK_BIT) == 0U || (CONSOLE_UART->CR1 & USART_CR1_UE) == 0U) {
        return;
    }

    // Take the UART back from an interrupt transfer that may be half done
    CONSOLE_UART->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

    for (const char *c = text; *c != '\0'; c++) {
        while ((CONSOLE_UART->SR & USART_SR_TXE) == 0U) {
        }
        CONSOLE_UART->DR = (uint8_t)*c;
    }
    while ((CONSOLE_UART->SR & USART_SR_TC) == 0U) {
    }
}
