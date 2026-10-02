#include "can.h"

#include <string.h>

#include "board.h"

// Both power of 2. TX: the worst burst is about 10 frames on CAN_D when the
// 10, 40, 50 and 80 ms jobs line up, plus forwarded frames.
#define TX_QUEUE_SIZE 32U
#define RX_QUEUE_SIZE 32U

typedef struct {
    CAN_HandleTypeDef hcan;

    // Only main adds (tx_head). Frames come off (tx_tail) in the CAN
    // interrupts, or in main with interrupts off.
    can_frame_t tx[TX_QUEUE_SIZE];
    volatile uint32_t tx_head;
    volatile uint32_t tx_tail;
    volatile uint32_t tx_dropped;

    // The RX interrupt adds (rx_head), main takes (rx_tail)
    can_frame_t rx[RX_QUEUE_SIZE];
    volatile uint32_t rx_head;
    volatile uint32_t rx_tail;
    volatile uint32_t rx_dropped;
} bus_t;

static bus_t buses[CAN_BUS_COUNT];

static bus_t *bus_of(const CAN_HandleTypeDef *hcan) {
    return hcan->Instance == CAN1 ? &buses[CAN_P] : &buses[CAN_D];
}

// Same timing Mbed computes from PCLK1 = 45 MHz (can_speed() in its can_api.c),
// so the sample points match the other Mbed nodes on each bus.
//   500k: 45 MHz / 5 = 9 MHz, 1 + 12 + 5 = 18 tq, sample point 72.2%, BTR 0x014B0004
//   1M:   45 MHz / 3 = 15 MHz, 1 + 10 + 4 = 15 tq, sample point 73.3%, BTR 0x01390002
static void bus_init(bus_t *bus, CAN_TypeDef *instance, uint32_t prescaler, uint32_t bs1,
                     uint32_t bs2, uint32_t filter_bank, IRQn_Type tx_irq, IRQn_Type rx_irq) {
    CAN_HandleTypeDef *hcan = &bus->hcan;
    hcan->Instance = instance;
    hcan->Init.Prescaler = prescaler;
    hcan->Init.Mode = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan->Init.TimeSeg1 = bs1;
    hcan->Init.TimeSeg2 = bs2;
    hcan->Init.TimeTriggeredMode = DISABLE;
    hcan->Init.AutoBusOff = DISABLE; // like Mbed, bus-off stays off until reset
    hcan->Init.AutoWakeUp = DISABLE;
    hcan->Init.AutoRetransmission = ENABLE;
    hcan->Init.ReceiveFifoLocked = DISABLE;
    hcan->Init.TransmitFifoPriority = DISABLE; // like Mbed: lowest id first among the 3 mailboxes
    if (HAL_CAN_Init(hcan) != HAL_OK) {
        Error_Handler();
    }

    // Accept everything into FIFO0, like Mbed. CAN1 has banks 0-13, CAN2 14-27.
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                               CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        Error_Handler();
    }

    // Both at the same priority, so they never preempt each other
    HAL_NVIC_SetPriority(tx_irq, 5, 0);
    HAL_NVIC_EnableIRQ(tx_irq);
    HAL_NVIC_SetPriority(rx_irq, 5, 0);
    HAL_NVIC_EnableIRQ(rx_irq);

    if (HAL_CAN_Start(hcan) != HAL_OK) {
        Error_Handler();
    }
}

void can_init(void) {
    // CAN1 first: it owns the filter banks that CAN2 uses
    bus_init(&buses[CAN_P], CAN1, 5, CAN_BS1_12TQ, CAN_BS2_5TQ, 0, CAN1_TX_IRQn, CAN1_RX0_IRQn);
    bus_init(&buses[CAN_D], CAN2, 3, CAN_BS1_10TQ, CAN_BS2_4TQ, 14, CAN2_TX_IRQn, CAN2_RX0_IRQn);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; // like Mbed, so RX reads recessive if the transceiver isn't driving it
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1; // CAN2 is AF9 on these pins too

    __HAL_RCC_GPIOB_CLK_ENABLE();
    if (hcan->Instance == CAN1) {
        // PB8 = CAN1_RX, PB9 = CAN1_TX, to U21 (ISO1044BD)
        __HAL_RCC_CAN1_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    } else {
        // PB5 = CAN2_RX, PB6 = CAN2_TX, to U22 (ISO1044BD). CAN2 needs CAN1's
        // clock as well, for the shared filters.
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Alternate = GPIO_AF9_CAN2;
    }
    HAL_GPIO_Init(GPIOB, &gpio);
}

// Moves queued frames into free mailboxes. Runs in a CAN interrupt, or in main
// with interrupts off. HAL_CAN_IRQHandler checks every enabled source, so a TX
// completion can be handled from the RX vector too, not just the TX one.
static void tx_pump(bus_t *bus) {
    while (bus->tx_tail != bus->tx_head && HAL_CAN_GetTxMailboxesFreeLevel(&bus->hcan) > 0U) {
        const can_frame_t *frame = &bus->tx[bus->tx_tail % TX_QUEUE_SIZE];

        CAN_TxHeaderTypeDef header = {0};
        if (frame->ext) {
            header.ExtId = frame->id;
            header.IDE = CAN_ID_EXT;
        } else {
            header.StdId = frame->id;
            header.IDE = CAN_ID_STD;
        }
        header.RTR = frame->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
        header.DLC = frame->dlc;
        header.TransmitGlobalTime = DISABLE;

        uint32_t mailbox;
        if (HAL_CAN_AddTxMessage(&bus->hcan, &header, frame->data, &mailbox) != HAL_OK) {
            return;
        }
        bus->tx_tail++;
    }
}

bool can_send_frame(can_bus_t bus_id, const can_frame_t *frame) {
    bus_t *bus = &buses[bus_id];
    bool queued = false;

    // A few microseconds: one copy and at most 6 mailbox writes
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // A mailbox may have freed up before its TX interrupt ran, so move the
    // queue along first instead of dropping a frame that would fit
    tx_pump(bus);
    if (bus->tx_head - bus->tx_tail < TX_QUEUE_SIZE) {
        bus->tx[bus->tx_head % TX_QUEUE_SIZE] = *frame;
        bus->tx_head++;
        queued = true;
    } else {
        bus->tx_dropped++;
    }
    tx_pump(bus);
    __set_PRIMASK(primask);

    return queued;
}

bool can_send(can_bus_t bus, uint32_t id, const uint8_t *data, uint8_t len) {
    can_frame_t frame = {0};
    frame.id = id;
    frame.dlc = len > 8U ? 8U : len;
    if (data != NULL) {
        memcpy(frame.data, data, frame.dlc);
    }
    return can_send_frame(bus, &frame);
}

bool can_read(can_bus_t bus_id, can_frame_t *frame) {
    bus_t *bus = &buses[bus_id];
    if (bus->rx_tail == bus->rx_head) {
        return false;
    }
    __COMPILER_BARRIER(); // read the slot only after seeing it's been filled
    *frame = bus->rx[bus->rx_tail % RX_QUEUE_SIZE];
    __COMPILER_BARRIER();
    bus->rx_tail++;
    return true;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    bus_t *bus = bus_of(hcan);

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            return;
        }
        if (bus->rx_head - bus->rx_tail >= RX_QUEUE_SIZE) {
            bus->rx_dropped++;
            continue;
        }

        can_frame_t *frame = &bus->rx[bus->rx_head % RX_QUEUE_SIZE];
        frame->ext = header.IDE == CAN_ID_EXT;
        frame->id = frame->ext ? header.ExtId : header.StdId;
        frame->rtr = header.RTR == CAN_RTR_REMOTE;
        frame->dlc = header.DLC > 8U ? 8U : (uint8_t)header.DLC;
        memcpy(frame->data, data, sizeof(frame->data));
        __COMPILER_BARRIER(); // frame complete before main can see it
        bus->rx_head++;
    }
}

static void tx_done(CAN_HandleTypeDef *hcan) {
    tx_pump(bus_of(hcan));
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    tx_done(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    tx_done(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    tx_done(hcan);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    tx_done(hcan);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    tx_done(hcan);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    tx_done(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    // FIFO0 overrun: at least one frame lost in the hardware FIFO
    if ((HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        bus_of(hcan)->rx_dropped++;
    }
    HAL_CAN_ResetError(hcan);

    // A mailbox that ended in an error is free again
    tx_pump(bus_of(hcan));
}

void CAN1_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&buses[CAN_P].hcan);
}

void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&buses[CAN_P].hcan);
}

void CAN2_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&buses[CAN_D].hcan);
}

void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&buses[CAN_D].hcan);
}

uint8_t can_tx_error_count(can_bus_t bus) {
    return (uint8_t)((buses[bus].hcan.Instance->ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
}

bool can_bus_off(can_bus_t bus) {
    return (buses[bus].hcan.Instance->ESR & CAN_ESR_BOFF) != 0U;
}

uint32_t can_tx_dropped(can_bus_t bus) {
    return buses[bus].tx_dropped;
}

uint32_t can_rx_dropped(can_bus_t bus) {
    return buses[bus].rx_dropped;
}
