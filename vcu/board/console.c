#include "console.h"

#include <stdio.h>

// Interrupt driven TX so printf() never stalls the control loop, like Mbed's
// BufferedSerial. If the buffer fills up the extra output is dropped.
#define TX_BUFFER_SIZE 1024U // power of 2

UART_HandleTypeDef huart4;

static uint8_t tx_buffer[TX_BUFFER_SIZE];
static char stdout_buffer[128];
static volatile uint32_t tx_head;    // next free slot, only main writes
static volatile uint32_t tx_tail;    // oldest unsent byte, only the ISR writes
static volatile uint16_t tx_in_flight;

void console_init(void) {
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }

    // Line buffered like Mbed, so _write gets whole lines
    setvbuf(stdout, stdout_buffer, _IOLBF, sizeof(stdout_buffer));
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance != UART4) {
        return;
    }

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // PC10 = UART4_TX, PC11 = UART4_RX
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

// Sends the next contiguous chunk of the buffer. Called from the ISR, or from
// main with the UART4 interrupt masked.
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
    if (HAL_UART_Transmit_IT(&huart4, &tx_buffer[start], (uint16_t)count) != HAL_OK) {
        tx_in_flight = 0;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != UART4) {
        return;
    }
    tx_tail += tx_in_flight;
    tx_in_flight = 0;
    start_next_chunk();
}

void UART4_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart4);
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

    HAL_NVIC_DisableIRQ(UART4_IRQn);
    start_next_chunk();
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    return len;
}

void console_write_blocking(const char *text) {
    if ((RCC->APB1ENR & RCC_APB1ENR_UART4EN) == 0U || (UART4->CR1 & USART_CR1_UE) == 0U) {
        return;
    }

    // Take the UART back from an interrupt transfer that may be half done
    UART4->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

    for (const char *c = text; *c != '\0'; c++) {
        while ((UART4->SR & USART_SR_TXE) == 0U) {
        }
        UART4->DR = (uint8_t)*c;
    }
    while ((UART4->SR & USART_SR_TC) == 0U) {
    }
}
