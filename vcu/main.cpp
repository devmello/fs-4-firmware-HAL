#include <cstdint>
#include <cstdio>

#include "board.h"
#include "can.h"
#include "etc_controller.h"
#include "timebase.h"

// APPS_1 (PC1, ADC1_IN11), APPS_2 (PC2, ADC1_IN12) from the fs-4 VCU schematic
ETCController etc{ADC_CHANNEL_11, ADC_CHANNEL_12};

constexpr uint32_t SME_RPDO_THROTTLE_DEMAND_ID = 390; // 0x186
constexpr uint32_t VCU_TPDO_STATUS_ID = 403;          // 0x193
constexpr int16_t SME_MAX_SPEED_RPM = 7500;

constexpr uint32_t CAN_SEND_PERIOD_US = 50'000;     // 20 Hz
constexpr uint32_t DEBUG_PRINT_PERIOD_US = 500'000; // 2 Hz

void send_CAN_messages();
void send_etc_CAN_messages();
void send_sme_CAN_messages();
void print_debug();

// Fixed-rate deadline check. Skips missed periods instead of bursting.
static bool period_elapsed(uint32_t now, uint32_t &deadline, uint32_t period) {
    if (static_cast<int32_t>(now - deadline) < 0) {
        return false;
    }
    deadline += period;
    if (static_cast<int32_t>(now - deadline) >= 0) {
        deadline = now + period;
    }
    return true;
}

int main() {
    board_init();

    printf("Hello World!!\n");

    uint32_t now = timebase_micros();
    uint32_t next_can_send = now + CAN_SEND_PERIOD_US;
    uint32_t next_debug_print = now + DEBUG_PRINT_PERIOD_US;

    bool motor_was_enabled = false;

    while (true) {
        etc.update_state();

        // Send zero torque right away (T.4.2.5)
        if (motor_was_enabled && !etc.motor_enabled) {
            send_CAN_messages();
        }
        motor_was_enabled = etc.motor_enabled;

        now = timebase_micros();
        if (period_elapsed(now, next_can_send, CAN_SEND_PERIOD_US)) {
            send_CAN_messages();
        }
        if (period_elapsed(now, next_debug_print, DEBUG_PRINT_PERIOD_US)) {
            print_debug();
        }
    }
}

void send_CAN_messages() {
    send_sme_CAN_messages();  // first, so it always gets a TX mailbox
    send_etc_CAN_messages();
}

// VCU_TPDO_STATUS: READY_TO_DRIVE, MOTOR_ENABLED and the two APPS implausibilities
void send_etc_CAN_messages() {
    uint8_t data[8] = {0};

    data[0] = (1 << 0)                                                   // READY_TO_DRIVE
            | (1 << 1)                                                   // MOTOR_ENABLED
            | (static_cast<uint8_t>(etc.implaus_apps_out_of_range) << 4) // IMPLAUS_APPS_OUT_OF_RANGE
            | (static_cast<uint8_t>(etc.implaus_apps_deviation) << 6);   // IMPLAUS_APPS_DEVIATION

    can_write(&hcan1, VCU_TPDO_STATUS_ID, data, 8);
}

// SME_RPDO_Throttle_Demand: torque demand from the ETC
void send_sme_CAN_messages() {
    static uint8_t mbb_alive = 0;  // only advances on a sent frame
    uint8_t next_mbb_alive = (mbb_alive + 1) & 0x0F;

    bool power_ready = etc.motor_enabled;
    uint16_t torque = static_cast<uint16_t>(power_ready ? etc.torque_demand : 0);
    uint16_t max_speed = static_cast<uint16_t>(SME_MAX_SPEED_RPM);

    uint8_t data[8] = {0};
    data[0] = torque & 0xFF;                                        // TorqueDemand
    data[1] = (torque >> 8) & 0xFF;
    data[2] = max_speed & 0xFF;                                     // MaxSpeed
    data[3] = (max_speed >> 8) & 0xFF;
    data[4] = (1 << 0) | (static_cast<uint8_t>(power_ready) << 3); // Forward, PowerReady
    data[5] = next_mbb_alive;                                       // MBB_Alive

    if (can_write(&hcan1, SME_RPDO_THROTTLE_DEMAND_ID, data, 8)) {
        mbb_alive = next_mbb_alive;
    }
}

void print_debug() {
    printf("APPS1 %.3f V %.3f | APPS2 %.3f V %.3f | pedal %.3f | torque %d | dev %d oor %d | CAN tx err %d%s\n",
           etc.apps1_voltage, etc.apps1_position,
           etc.apps2_voltage, etc.apps2_position,
           etc.pedal_position, etc.torque_demand,
           etc.implaus_apps_deviation, etc.implaus_apps_out_of_range,
           can_tx_error_count(&hcan1), can_bus_off(&hcan1) ? " bus-off" : "");
}
