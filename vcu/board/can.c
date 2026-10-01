#include "can.h"

CAN_HandleTypeDef hcan1;

void can_init(void) {
    // Same timing Mbed computes for 500k at 45 MHz, so the sample point matches
    // the other nodes on the bus: 45 MHz / 5 = 9 MHz, 1 + 12 + 5 = 18 tq per
    // bit, sample point 72.2%, BTR = 0x014B0004
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 5;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE; // like Mbed, bus-off stays off until reset
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    // Accept everything into FIFO0. Nothing reads it yet, but CAN2 (data bus)
    // will need banks 14-27 so set the split now.
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance != CAN1) {
        return;
    }

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB8 = CAN1_RX, PB9 = CAN1_TX, to U21 (ISO1044BD). Pull-ups like Mbed,
    // so RX reads recessive if the transceiver isn't driving it.
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

bool can_write(CAN_HandleTypeDef *hcan, uint32_t id, const uint8_t *data, uint8_t len) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        return false;
    }

    CAN_TxHeaderTypeDef header = {0};
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK;
}

uint8_t can_tx_error_count(const CAN_HandleTypeDef *hcan) {
    return (uint8_t)((hcan->Instance->ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
}

bool can_bus_off(const CAN_HandleTypeDef *hcan) {
    return (hcan->Instance->ESR & CAN_ESR_BOFF) != 0U;
}
