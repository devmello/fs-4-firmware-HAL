// fs-4 VCU main loop. Ported from Mbed (fs-4-firmware vcu/main.cpp, c12c834d)
// to the STM32 HAL.
//
// One loop, no RTOS. The CAN jobs that ran on Mbed's etc_queue and imu_queue
// threads are deadlines checked every pass, the RTD button interrupt only
// counts edges, and the VectorNav is read from a DMA ring and parsed here.

#include <cstdint>
#include <cstdio>
#include <numbers>

#include "adc.h"
#include "board.h"
#include "can.h"
#include "etc_controller.h"
#include "gpio.h"
#include "imu_uart.h"
#include "pins.h"
#include "timebase.h"
#include "traction_control.h"
#include "vn200.h"
#include "watchdog.h"

AdcInput steering_position{ADC_CH_STEERING};
InputPin bspd_fault{IN_BSPD_FAULT};
InputPin bspd_shutdown_out{IN_BSPD_SHUTDOWN};

void print_imu_event(Vn200::Event event, uint8_t code);

Vn200 imu{imu_uart_write, print_imu_event};

ETCController etc{ADC_CH_APPS1, ADC_CH_APPS2, ADC_CH_BPPS, ADC_CH_FRONT_BSE, ADC_CH_REAR_BSE,
                  IN_RTD_BUTTON, OUT_RTD_LIGHT, OUT_RTD_BUZZER, OUT_SOLENOID, OUT_BRAKELIGHT};
ETCState& etc_state = etc.state;

bool wheel_fl_read = false;
bool wheel_fr_read = false;
bool wheel_bl_read = false;
bool wheel_br_read = false;

void handle_powertrain_frame(can_frame_t& rx);
void handle_data_frame(can_frame_t& rx);
void update_wheel_reads();
void send_etc_CAN_messages();
void send_sme_CAN_messages_powertrain();
void send_sme_CAN_messages_data();
void send_imu_CAN_messages();
void update_traction_control();
void print_debug();

namespace {
static constexpr float RAD_TO_DEG = 57.2957795f;

static constexpr float STEERING_MIN_VOLTAGE = 0.227;
static constexpr float STEERING_MAX_VOLTAGE = 1.069;
static constexpr float STEERING_AVG_VOLTAGE = (STEERING_MAX_VOLTAGE + STEERING_MIN_VOLTAGE) / 2.0f;
static constexpr float STEERING_VOLTAGE_RANGE = STEERING_MAX_VOLTAGE - STEERING_AVG_VOLTAGE;
static constexpr float STEERING_MAX_ANGLE = 75.82; // Degrees

constexpr uint32_t IMU_CAN_PERIOD_US = 10'000;
constexpr uint32_t POWERTRAIN_PERIOD_US = 40'000;
constexpr uint32_t ETC_CAN_PERIOD_US = 50'000;
constexpr uint32_t DATA_PERIOD_US = 80'000;
constexpr uint32_t DEBUG_PRINT_PERIOD_US = 500'000;

// Mbed slept 5 ms in the 40 ms job between the CAN_P sends and the CAN_D
// copies. Here the copies wait for their own deadline.
constexpr uint32_t DATA_COPY_DELAY_US = 5'000;

uint8_t data_copy_throttle[8];
uint8_t data_copy_currents[8];
bool data_copy_pending = false;
uint64_t data_copy_at = 0;

uint32_t loop_max_us = 0;
uint32_t loop_count = 0;
} // namespace

// Fixed-rate deadline check. Skips missed periods instead of bursting.
static bool period_elapsed(uint64_t now, uint64_t& deadline, uint32_t period) {
    if (now < deadline) {
        return false;
    }
    deadline += period;
    if (now >= deadline) {
        deadline = now + period;
    }
    return true;
}

int main() {
    board_init();

    printf("Hello World!!\n");
    printf("Reset cause: %s\n", board_reset_cause());

    uint64_t now = timebase_micros();
    imu.start(now);

    // Mbed's call_every() runs a job one period after it's posted
    uint64_t next_imu = now + IMU_CAN_PERIOD_US;
    uint64_t next_powertrain = now + POWERTRAIN_PERIOD_US;
    uint64_t next_etc = now + ETC_CAN_PERIOD_US;
    uint64_t next_data = now + DATA_PERIOD_US;
    uint64_t next_debug = now + DEBUG_PRINT_PERIOD_US;

    uint32_t rtd_rises_seen = gpio_rtd_button_rises();

    while (true) {
        watchdog_refresh();
        uint64_t pass_start = timebase_micros();

        // Mbed read one frame per bus per pass. Here the queues are drained.
        can_frame_t rx;
        while (can_read(CAN_P, &rx)) {
            handle_powertrain_frame(rx);
        }
        while (can_read(CAN_D, &rx)) {
            handle_data_frame(rx);
        }

        uint8_t imu_bytes[128];
        size_t imu_count;
        while ((imu_count = imu_uart_read(imu_bytes, sizeof(imu_bytes))) > 0) {
            imu.feed(imu_bytes, imu_count, pass_start);
        }
        imu.poll(pass_start);
        imu.update_state(etc_state.vectornav);

        // Mbed ran this in the button's interrupt. Edges within one pass
        // (~60-110 us) become one call. Bounce slower than that still toggles
        // RTD, like in Mbed; the RC filter on the board is what stops it.
        uint32_t rtd_rises = gpio_rtd_button_rises();
        if (rtd_rises != rtd_rises_seen) {
            rtd_rises_seen = rtd_rises;
            etc.rtd_button_irq();
        }

        etc.update_state();

        // When several are due at once, Mbed's event queue ran the 80 ms job,
        // then the 50 ms one, then the 40 ms one (same-time events go in the
        // order they were queued, so the longest period first). This keeps the
        // frame order on CAN_D and which MBB_Alive the 80 ms frames carry.
        now = timebase_micros();
        if (period_elapsed(now, next_data, DATA_PERIOD_US)) {
            send_sme_CAN_messages_data();
        }
        if (period_elapsed(now, next_etc, ETC_CAN_PERIOD_US)) {
            send_etc_CAN_messages();
        }
        if (period_elapsed(now, next_powertrain, POWERTRAIN_PERIOD_US)) {
            send_sme_CAN_messages_powertrain();
        }
        if (data_copy_pending && now >= data_copy_at) {
            data_copy_pending = false;
            can_send(CAN_D, 390, data_copy_throttle, 8);
            can_send(CAN_D, 646, data_copy_currents, 8);
        }
        if (period_elapsed(now, next_imu, IMU_CAN_PERIOD_US)) {
            send_imu_CAN_messages();
        }
        if (period_elapsed(now, next_debug, DEBUG_PRINT_PERIOD_US)) {
            print_debug();
        }

        // Under 250 ms, or the watchdog resets the chip
        uint32_t pass_us = static_cast<uint32_t>(timebase_micros() - pass_start);
        if (pass_us > loop_max_us) {
            loop_max_us = pass_us;
        }
        loop_count++;
    }
}

void handle_powertrain_frame(can_frame_t& rx) {
    switch (rx.id) {
    case 0x391: {
        etc.battery_precharged = rx.data[0] & 0b01000000;
        etc.shutdown_closed = rx.data[0] & 0b00000100;
        if (!etc.shutdown_closed || !etc.battery_precharged) {
            etc.turn_off_rtd();
        }
        break;
    }
    case 1154: { // SME_TPDO_Torque_speed
        uint16_t wheel_rpm = rx.data[0] | (rx.data[1] << 8);
        float wheel_radius = 0.190f;
        float ground_speed =
            (11 / 40.0f) * (2 * std::numbers::pi * wheel_radius); // gear ratio * circumference
        ground_speed *= wheel_rpm;                                 // meters / minute
        ground_speed *= 60.0f / 1000.0f;                           // km / hr

        etc.update_regen_state(ground_speed);
        can_send_frame(CAN_D, &rx);
        break;
    }
    case 0x4c0: { // BATT_TPDO_TRAY_TEMPS
        uint8_t tray_temp_x2 = rx.data[1];
        float tray_temp = tray_temp_x2 / 2.0;
        if (tray_temp > 40.0) {
            etc.turn_off_rtd();
        }
        break;
    }
    case 1666:
        can_send_frame(CAN_D, &rx);
        break;
    }
}

void handle_data_frame(can_frame_t& rx) {
    switch (rx.id) {
    case 421: {
        etc.state.wheel_rpm_fl = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
        wheel_fl_read = true;
        update_wheel_reads();
        break;
    }
    case 422: {
        etc.state.wheel_rpm_fr = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
        wheel_fr_read = true;
        update_wheel_reads();
        break;
    }
    case 423: {
        etc.state.wheel_rpm_bl = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
        wheel_bl_read = true;
        update_wheel_reads();
        break;
    }
    case 424: {
        etc.state.wheel_rpm_br = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
        wheel_br_read = true;
        update_wheel_reads();
        break;
    }
    case 432: {
        const uint8_t modes = rx.data[0];
        etc.state.drive_mode = modes & 0b00000011;
        etc.state.traction_mode = (modes >> 2) & 0b00000011;
        etc.state.regen_mode = (modes >> 4) & 0b00000011;
        break;
    }
    }
}

void update_wheel_reads() {
    if (wheel_fl_read && wheel_fr_read && wheel_bl_read && wheel_br_read) {
        update_traction_control();

        wheel_fl_read = false;
        wheel_fr_read = false;
        wheel_bl_read = false;
        wheel_br_read = false;
    }
}

void send_etc_CAN_messages() {
    uint8_t tpdo_pedal_travel[8] = {0};
    uint16_t APPS1_scaled_voltage = static_cast<uint16_t>(etc_state.APPS1_voltage * 1000);
    uint16_t APPS2_scaled_voltage = static_cast<uint16_t>(etc_state.APPS2_voltage * 1000);
    uint16_t BPPS_scaled_voltage = static_cast<uint16_t>(etc_state.BPPS_voltage * 1000);
    tpdo_pedal_travel[0] = APPS1_scaled_voltage & 0xFF;
    tpdo_pedal_travel[1] = APPS1_scaled_voltage >> 8;
    tpdo_pedal_travel[2] = APPS2_scaled_voltage & 0xFF;
    tpdo_pedal_travel[3] = APPS2_scaled_voltage >> 8;
    tpdo_pedal_travel[4] = BPPS_scaled_voltage & 0xFF;
    tpdo_pedal_travel[5] = BPPS_scaled_voltage >> 8;
    tpdo_pedal_travel[6] = static_cast<uint8_t>(etc_state.APPS_position_avg * 100);
    tpdo_pedal_travel[7] = static_cast<uint8_t>(etc_state.BPPS_position * 100);

    uint8_t tpdo_status[8] = {0};
    uint16_t front_BSE_pressure = static_cast<uint16_t>(etc_state.front_BSE_pressure);
    uint16_t read_BSE_pressure = static_cast<uint16_t>(etc_state.read_BSE_pressure);
    float steering_position_V = steering_position.read_voltage();
    float steering_angle = ((steering_position_V - STEERING_AVG_VOLTAGE) / STEERING_VOLTAGE_RANGE) * STEERING_MAX_ANGLE;
    int16_t steering_angle_x100 = static_cast<int16_t>(steering_angle * 100);
    tpdo_status[0] = etc_state.ready_to_drive
                     | (etc_state.motor_enabled << 1)
                     | (etc_state.rtd_button_pressed << 2)
                     | (etc.battery_precharged << 3)
                     | (etc_state.implaus_APPS_range << 4)
                     | (etc_state.implaus_BPPS_range << 5)
                     | (etc_state.implaus_APPS_deviation << 6)
                     | (etc_state.implaus_BSE_range << 7);
    tpdo_status[1] = etc_state.implaus_brake_and_accel
                     | (etc_state.reversing << 1)
                     | (etc_state.brakelight_enabled << 2)
                     | (etc_state.regen_allowed << 3)
                     | (etc_state.solenoid_open << 4)
                     | (bspd_fault.read() << 6)
                     | (bspd_shutdown_out.read() << 7);
    tpdo_status[2] = front_BSE_pressure & 0xFF;
    tpdo_status[3] = front_BSE_pressure >> 8;
    tpdo_status[4] = read_BSE_pressure & 0xFF;
    tpdo_status[5] = read_BSE_pressure >> 8;
    tpdo_status[6] = steering_angle_x100 & 0xFF;
    tpdo_status[7] = steering_angle_x100 >> 8;

    can_send(CAN_D, 402, tpdo_pedal_travel, 8);
    can_send(CAN_D, 403, tpdo_status, 8);
}

void send_sme_CAN_messages_powertrain() {
    // Mbed always sent the CAN_D copies before this job ran again. A pass
    // late enough to be due for both sends the waiting copy now, before it's
    // overwritten.
    if (data_copy_pending) {
        data_copy_pending = false;
        can_send(CAN_D, 390, data_copy_throttle, 8);
        can_send(CAN_D, 646, data_copy_currents, 8);
    }

    etc.update_mbb_alive();

    // Bytes 6-7 and 4-7 were left uninitialized in the Mbed build (stack
    // garbage on the bus). Zero here.
    uint8_t tpdo_throttle_demand[8] = {0};
    tpdo_throttle_demand[0] = etc_state.motor_torque.read() & 0xFF;
    tpdo_throttle_demand[1] = etc_state.motor_torque.read() >> 8;
    tpdo_throttle_demand[2] = etc_state.MAX_SPEED & 0xFF;
    tpdo_throttle_demand[3] = etc_state.MAX_SPEED >> 8;
    tpdo_throttle_demand[4] = (!etc_state.reversing)
                              | (etc_state.reversing << 1)
                              | (etc_state.motor_enabled << 3);
    tpdo_throttle_demand[5] = etc_state.mbb_alive;

    uint8_t tpdo_max_currents[8] = {0};
    tpdo_max_currents[0] = etc_state.CHARGE_CURRENT_LIMIT & 0xFF;
    tpdo_max_currents[1] = etc_state.CHARGE_CURRENT_LIMIT >> 8;
    tpdo_max_currents[2] = etc_state.DISCHARGE_CURRENT_LIMIT & 0xFF;
    tpdo_max_currents[3] = etc_state.DISCHARGE_CURRENT_LIMIT >> 8;

    can_send(CAN_P, 390, tpdo_throttle_demand, 8);
    can_send(CAN_P, 646, tpdo_max_currents, 8);

    // The same two frames go out on CAN_D 5 ms later (see main)
    for (int i = 0; i < 8; i++) {
        data_copy_throttle[i] = tpdo_throttle_demand[i];
        data_copy_currents[i] = tpdo_max_currents[i];
    }
    data_copy_at = timebase_micros() + DATA_COPY_DELAY_US;
    data_copy_pending = true;
}

void send_sme_CAN_messages_data() {
    uint8_t tpdo_throttle_demand[8] = {0};
    tpdo_throttle_demand[0] = etc_state.motor_torque.read() & 0xFF;
    tpdo_throttle_demand[1] = etc_state.motor_torque.read() >> 8;
    tpdo_throttle_demand[2] = etc_state.MAX_SPEED & 0xFF;
    tpdo_throttle_demand[3] = etc_state.MAX_SPEED >> 8;
    tpdo_throttle_demand[4] = (!etc_state.reversing)
                              | (etc_state.reversing << 1)
                              | (etc_state.motor_enabled << 3);
    tpdo_throttle_demand[5] = etc_state.mbb_alive;

    uint8_t tpdo_max_currents[8] = {0};
    tpdo_max_currents[0] = etc_state.CHARGE_CURRENT_LIMIT & 0xFF;
    tpdo_max_currents[1] = etc_state.CHARGE_CURRENT_LIMIT >> 8;
    tpdo_max_currents[2] = etc_state.DISCHARGE_CURRENT_LIMIT & 0xFF;
    tpdo_max_currents[3] = etc_state.DISCHARGE_CURRENT_LIMIT >> 8;

    uint8_t tpdo_traction_data[8];
    uint8_t tc_slip = static_cast<uint8_t>(etc.traction_controller.get_slip() * 100.0f);
    uint8_t tc_output = static_cast<uint8_t>(etc_state.tc_mult_factor * 100.0f);
    uint8_t tc_integral = static_cast<uint8_t>(etc.traction_controller.get_integral() * 100.0f);
    uint8_t tc_loop_time = static_cast<uint8_t>(etc.traction_controller.get_loop_time() * 1000.0f);
    int16_t tc_raw_derivative = static_cast<int16_t>(etc.traction_controller.get_raw_derivative() * 1000.0f);
    int16_t tc_smoothed_derivative = static_cast<int16_t>(etc.traction_controller.get_smoothed_derivative() * 1000.0f);
    tpdo_traction_data[0] = tc_slip;
    tpdo_traction_data[1] = tc_output;
    tpdo_traction_data[2] = tc_integral;
    tpdo_traction_data[3] = tc_loop_time;
    tpdo_traction_data[4] = tc_raw_derivative & 0xFF;
    tpdo_traction_data[5] = tc_raw_derivative >> 8;
    tpdo_traction_data[6] = tc_smoothed_derivative & 0xFF;
    tpdo_traction_data[7] = tc_smoothed_derivative >> 8;

    can_send(CAN_D, 390, tpdo_throttle_demand, 8);
    can_send(CAN_D, 660, tpdo_traction_data, 8);
    can_send(CAN_D, 646, tpdo_max_currents, 8);
}

void update_traction_control() {
    etc_state.tc_mult_factor = etc.traction_controller.update(etc.state.wheel_rpm_fl, etc.state.wheel_rpm_fr, etc.state.wheel_rpm_bl, etc.state.wheel_rpm_br);
}

/// 100Hz VectorNav Messages
void send_imu_CAN_messages() {
    uint8_t buf_accel[6];
    int16_t accel_f = static_cast<int16_t>(etc_state.vectornav.accel[0] * 100);
    int16_t accel_r = static_cast<int16_t>(etc_state.vectornav.accel[1] * 100);
    int16_t accel_d = static_cast<int16_t>(etc_state.vectornav.accel[2] * 100);
    buf_accel[0] = accel_f & 0xFF;
    buf_accel[1] = (accel_f >> 8) & 0xFF;
    buf_accel[2] = accel_r & 0xFF;
    buf_accel[3] = (accel_r >> 8) & 0xFF;
    buf_accel[4] = accel_d & 0xFF;
    buf_accel[5] = (accel_d >> 8) & 0xFF;

    uint8_t buf_ypr[6];
    int16_t yaw = static_cast<int16_t>(etc_state.vectornav.ypr.yaw * 100);
    int16_t pitch = static_cast<int16_t>(etc_state.vectornav.ypr.pitch * 100);
    int16_t roll = static_cast<int16_t>(etc_state.vectornav.ypr.roll * 100);
    buf_ypr[0] = yaw & 0xFF;
    buf_ypr[1] = (yaw >> 8) & 0xFF;
    buf_ypr[2] = pitch & 0xFF;
    buf_ypr[3] = (pitch >> 8) & 0xFF;
    buf_ypr[4] = roll & 0xFF;
    buf_ypr[5] = (roll >> 8) & 0xFF;

    uint8_t buf_latlon[8];
    int32_t lat = static_cast<int32_t>(etc_state.vectornav.pos.lat * 1e7);
    int32_t lon = static_cast<int32_t>(etc_state.vectornav.pos.lon * 1e7);
    buf_latlon[0] = lat & 0xFF;
    buf_latlon[1] = (lat >> 8) & 0xFF;
    buf_latlon[2] = (lat >> 16) & 0xFF;
    buf_latlon[3] = (lat >> 24) & 0xFF;
    buf_latlon[4] = lon & 0xFF;
    buf_latlon[5] = (lon >> 8) & 0xFF;
    buf_latlon[6] = (lon >> 16) & 0xFF;
    buf_latlon[7] = (lon >> 24) & 0xFF;

    uint8_t buf_gyro[6];
    int16_t gyro_y = static_cast<int16_t>(etc_state.vectornav.ang_rate[0] * RAD_TO_DEG * 10);
    int16_t gyro_p = static_cast<int16_t>(etc_state.vectornav.ang_rate[1] * RAD_TO_DEG * 10);
    int16_t gyro_r = static_cast<int16_t>(etc_state.vectornav.ang_rate[2] * RAD_TO_DEG * 10);
    buf_gyro[0] = gyro_y & 0xFF;
    buf_gyro[1] = (gyro_y >> 8) & 0xFF;
    buf_gyro[2] = gyro_p & 0xFF;
    buf_gyro[3] = (gyro_p >> 8) & 0xFF;
    buf_gyro[4] = gyro_r & 0xFF;
    buf_gyro[5] = (gyro_r >> 8) & 0xFF;

    uint8_t buf_vel[6];
    int16_t vel_x = static_cast<int16_t>(etc_state.vectornav.vel[0] * 100);
    int16_t vel_y = static_cast<int16_t>(etc_state.vectornav.vel[1] * 100);
    int16_t vel_z = static_cast<int16_t>(etc_state.vectornav.vel[2] * 100);
    buf_vel[0] = vel_x & 0xFF;
    buf_vel[1] = (vel_x >> 8) & 0xFF;
    buf_vel[2] = vel_y & 0xFF;
    buf_vel[3] = (vel_y >> 8) & 0xFF;
    buf_vel[4] = vel_z & 0xFF;
    buf_vel[5] = (vel_z >> 8) & 0xFF;

    can_send(CAN_D, 720, buf_accel, 6);
    can_send(CAN_D, 976, buf_ypr, 6);
    can_send(CAN_D, 721, buf_latlon, 8);
    can_send(CAN_D, 977, buf_gyro, 6);
    can_send(CAN_D, 722, buf_vel, 6);
}

void send_sync() {
    can_send(CAN_P, 0x80, nullptr, 0);
}

// The Mbed build's VectorNav wrapper printed its setup steps and each async
// error. Same lines here, as the driver gets there; the SDK's error numbers
// for a missing reply. The setup is retried, so they can come again.
void print_imu_event(Vn200::Event event, uint8_t code) {
    switch (event) {
    case Vn200::Event::MODEL:
        printf("Connected to sensor!\n");
        printf("Sensor Model Number: %s\n", imu.model());
        printf("baud: %lu\n", (unsigned long)IMU_UART_BAUD);
        break;
    case Vn200::Event::CONFIGURED:
        printf("Binary output messages configured.\n");
        break;
    case Vn200::Event::COMMAND_ERROR:
        printf("VN: Error %u (%s) in reply to a setup command\n", code, Vn200::error_name(code));
        break;
    case Vn200::Event::BACKOFF:
        if (code == 0) {
            printf("VN: Error 303 (ResponseTimeout), setup tried again in 1 s\n");
        } else {
            printf("VN: Error %u (%s), setup tried again in 1 s\n", code, Vn200::error_name(code));
        }
        break;
    case Vn200::Event::DATA_TIMEOUT:
        printf("VN: no data for 500 ms, setting the sensor up again\n");
        break;
    case Vn200::Event::SENSOR_ERROR:
        printf("Received async error: %s\n", Vn200::error_name(code));
        break;
    }
}

// Not in the Mbed build, which printed the lines above and "Hello World!!". Twice a second on
// the ST-LINK's serial port, two lines: the ETC, then CAN, IMU and loop health.
void print_debug() {
    printf("APPS %.3f %.3f V | BPPS %.3f V | BSE %.3f %.3f V | torque %d | RTD %d EN %d | "
           "implaus %d%d%d%d%d\n",
           etc_state.APPS1_voltage, etc_state.APPS2_voltage, etc_state.BPPS_voltage,
           etc_state.front_BSE_voltage, etc_state.rear_BSE_voltage,
           etc_state.motor_torque.read(), etc_state.ready_to_drive, etc_state.motor_enabled,
           etc_state.implaus_APPS_deviation, etc_state.implaus_APPS_range,
           etc_state.implaus_BPPS_range, etc_state.implaus_BSE_range,
           etc_state.implaus_brake_and_accel);

    const Vn200::Stats& vn = imu.stats();
    printf("CAN P err %u%s drop %lu | CAN D err %u%s drop %lu | IMU %s pkts %lu crc %lu "
           "lost %lu cfg %lu err %02X | loop max %lu us, %lu passes\n",
           can_tx_error_count(CAN_P), can_bus_off(CAN_P) ? " bus-off" : "",
           (unsigned long)(can_tx_dropped(CAN_P) + can_rx_dropped(CAN_P)),
           can_tx_error_count(CAN_D), can_bus_off(CAN_D) ? " bus-off" : "",
           (unsigned long)(can_tx_dropped(CAN_D) + can_rx_dropped(CAN_D)),
           imu.configured() ? "ok" : "setup", (unsigned long)vn.packets,
           (unsigned long)vn.crc_errors, (unsigned long)imu_uart_dropped(),
           (unsigned long)vn.configs_started, vn.last_error,
           (unsigned long)loop_max_us, (unsigned long)loop_count);
    loop_max_us = 0;
    loop_count = 0;
}
