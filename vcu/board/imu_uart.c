#include "imu_uart.h"

#include <string.h>

#include "board.h"

// The VN-200 sends 8.2 KB/s (one 82-byte message at 100 Hz), so the ring
// holds about 125 ms. The main loop empties it every pass.
#define RX_RING_SIZE 1024U // power of 2
#define RX_HALF      (RX_RING_SIZE / 2U)
#define TX_SIZE      128U

static UART_HandleTypeDef huart5;
static DMA_HandleTypeDef hdma_rx; // DMA1 Stream 0, channel 4 = UART5_RX

static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_halves; // half rings the DMA has filled, from its HT/TC interrupts
static uint32_t rx_read;            // total bytes handed out
static uint32_t dropped;

static uint8_t tx_buffer[TX_SIZE];
static volatile uint32_t tx_len;
static volatile uint32_t tx_pos; // only the UART5 interrupt moves it

static void rx_half_done(DMA_HandleTypeDef *hdma) {
    (void)hdma;
    rx_halves++;
}

static void rx_start(void) {
    rx_halves = 0;
    rx_read = 0;
    CLEAR_BIT(UART5->CR3, USART_CR3_DMAR);
    hdma_rx.XferHalfCpltCallback = rx_half_done;
    hdma_rx.XferCpltCallback = rx_half_done;
    if (HAL_DMA_Start_IT(&hdma_rx, (uint32_t)&UART5->DR, (uint32_t)rx_ring, RX_RING_SIZE) != HAL_OK) {
        Error_Handler();
    }
    SET_BIT(UART5->CR3, USART_CR3_DMAR);
}

void imu_uart_init(void) {
    // Clocks and pins here instead of in HAL_UART_MspInit, which console.c owns
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; // RX idles high if the sensor isn't plugged in
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    gpio.Pin = GPIO_PIN_12; // TX
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_2; // RX
    HAL_GPIO_Init(GPIOD, &gpio);

    huart5.Instance = UART5;
    huart5.Init.BaudRate = IMU_UART_BAUD;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
    }

    // The HAL's UART receive functions stop the DMA on any framing or noise
    // error, so RX is set up here directly: the DMA just keeps copying, and a
    // bad byte only costs the packet it's in (the parser checks the CRC).
    hdma_rx.Instance = DMA1_Stream0;
    hdma_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_CIRCULAR;
    hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_rx) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0); // TX only
    HAL_NVIC_EnableIRQ(UART5_IRQn);

    rx_start();
}

// Total bytes the DMA has written since rx_start()
static uint32_t rx_written(void) {
    __disable_irq();
    uint32_t halves = rx_halves;
    uint32_t flags = DMA1->LISR; // flags first, then the counter
    uint32_t remaining = DMA1_Stream0->NDTR;
    __enable_irq();

    // Halves the DMA finished that its interrupt hasn't counted yet
    if ((flags & DMA_LISR_HTIF0) != 0U) {
        halves++;
    }
    if ((flags & DMA_LISR_TCIF0) != 0U) {
        halves++;
    }
    uint32_t pos = (RX_RING_SIZE - remaining) % RX_RING_SIZE;
    // The DMA crossed into the next half between reading the flags and the
    // counter: an odd count means the second half of the ring
    if ((pos >= RX_HALF) != ((halves & 1U) != 0U)) {
        halves++;
    }
    return halves * RX_HALF + pos % RX_HALF;
}

size_t imu_uart_read(uint8_t *dst, size_t max) {
    // A DMA transfer error stops the stream. Start over.
    if ((DMA1_Stream0->CR & DMA_SxCR_EN) == 0U) {
        HAL_DMA_Abort(&hdma_rx);
        rx_start();
        return 0;
    }

    uint32_t written = rx_written();
    uint32_t available = written - rx_read;
    if (available > RX_RING_SIZE - 16U) {
        // The main loop fell behind and the DMA is writing over (or about to
        // write over) what hasn't been read. Keep the newest half.
        dropped += available - RX_HALF;
        rx_read = written - RX_HALF;
        available = RX_HALF;
    }
    if (available > max) {
        available = (uint32_t)max;
    }

    uint32_t start = rx_read % RX_RING_SIZE;
    uint32_t first = available;
    if (first > RX_RING_SIZE - start) {
        first = RX_RING_SIZE - start;
    }
    memcpy(dst, &rx_ring[start], first);
    memcpy(dst + first, &rx_ring[0], available - first);
    rx_read += available;
    return available;
}

bool imu_uart_write(const uint8_t *data, size_t len) {
    if (len == 0U || len > TX_SIZE || tx_pos < tx_len) {
        return false;
    }
    memcpy(tx_buffer, data, len);
    tx_pos = 0;
    tx_len = (uint32_t)len;
    SET_BIT(UART5->CR1, USART_CR1_TXEIE);
    return true;
}

uint32_t imu_uart_dropped(void) {
    return dropped;
}

void DMA1_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_rx);
}

// Only TXE is enabled. Receive errors don't raise an interrupt.
void UART5_IRQHandler(void) {
    if ((UART5->CR1 & USART_CR1_TXEIE) != 0U && (UART5->SR & USART_SR_TXE) != 0U) {
        if (tx_pos < tx_len) {
            UART5->DR = tx_buffer[tx_pos];
            tx_pos++;
        }
        if (tx_pos >= tx_len) {
            CLEAR_BIT(UART5->CR1, USART_CR1_TXEIE);
        }
    }
}
