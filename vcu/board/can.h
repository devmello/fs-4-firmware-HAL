#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAN_P, // CAN_Powertrain: CAN1 on PB8 (RX) / PB9 (TX), 500 kbit/s
    CAN_D, // CAN_Data: CAN2 on PB5 (RX) / PB6 (TX), 1 Mbit/s
    CAN_BUS_COUNT
} can_bus_t;

typedef struct {
    uint32_t id;
    bool ext; // 29-bit id
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

void can_init(void);

// Queues a standard id data frame. The queue feeds the 3 TX mailboxes from the
// TX interrupt. False if the queue was full (counted in can_tx_dropped()).
// Main loop only, not from interrupts.
bool can_send(can_bus_t bus, uint32_t id, const uint8_t *data, uint8_t len);

// Same, for any frame (forwarding keeps the id format, RTR and DLC)
bool can_send_frame(can_bus_t bus, const can_frame_t *frame);

// Takes the oldest received frame. False if there's none. Main loop only.
bool can_read(can_bus_t bus, can_frame_t *frame);

// Transmit error counter (ESR.TEC), same as Mbed's CAN::tderror(). Only the
// low 8 bits, so it wraps back to ~0 when the controller goes bus-off.
uint8_t can_tx_error_count(can_bus_t bus);

// True once the controller is bus-off. It stays that way until reset, like Mbed.
bool can_bus_off(can_bus_t bus);

// Frames not sent because the TX queue was full
uint32_t can_tx_dropped(can_bus_t bus);

// Frames lost because the RX queue or the hardware FIFO was full
uint32_t can_rx_dropped(can_bus_t bus);

#ifdef __cplusplus
}
#endif

#endif
