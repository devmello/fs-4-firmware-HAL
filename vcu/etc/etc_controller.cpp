//
// Created by Jackson Pinsonneault on 3/24/26.
// Ported from Mbed to STM32 HAL.
//

#include "etc_controller.h"
#include "adc.h"
#include <cmath>

ETCController::ETCController(uint32_t apps1_channel, uint32_t apps2_channel)
    : apps1_channel(apps1_channel), apps2_channel(apps2_channel) {}

// Clamps to [0, 1] (-0.1 => 0, 10 => 1, 0.3 => 0.3)
float ETCController::clamp(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

// low <= value <= high
bool ETCController::in_range(float value, float low, float high) {
    return low <= value && value <= high;
}

float ETCController::read_average_voltage(uint32_t channel) {
    float total = 0.0f;
    for (int i = 0; i < SAMPLES_PER_READ; i++) {
        total += adc_read_voltage(channel);
    }
    return total / SAMPLES_PER_READ;
}

float ETCController::voltage_to_travel(float voltage, float min_voltage, float max_voltage) {
    return (voltage - min_voltage) / (max_voltage - min_voltage);
}

float ETCController::travel_to_position(float travel) {
    return clamp((travel - PEDAL_DEADZONE_PERCENTAGE) / (1.0f - 2.0f * PEDAL_DEADZONE_PERCENTAGE));
}

float ETCController::map_pedal(float position) {
    return position * (PEDAL_MAP_LINEARITY + (1.0f - PEDAL_MAP_LINEARITY) * position);
}

// Refreshes voltages, pedal positions, implausibilities and torque demand
void ETCController::update_state() {
    apps1_voltage = read_average_voltage(apps1_channel);
    apps2_voltage = read_average_voltage(apps2_channel);

    apps1_travel = voltage_to_travel(apps1_voltage, APPS1_MIN_VOLTAGE, APPS1_MAX_VOLTAGE);
    apps2_travel = voltage_to_travel(apps2_voltage, APPS2_MIN_VOLTAGE, APPS2_MAX_VOLTAGE);

    apps1_position = travel_to_position(apps1_travel);
    apps2_position = travel_to_position(apps2_travel);
    pedal_position = (apps1_position + apps2_position) / 2.0f;

    update_implausibilities();

    motor_enabled = !implaus_active;
    torque_demand = motor_enabled ? static_cast<int16_t>(map_pedal(pedal_position) * MAX_TORQUE) : 0;
}

// T.4.2.4 (deviation), T.4.2.5 (100 ms), T.4.2.10 (out of range)
void ETCController::update_implausibilities() {
    bool deviation_active = std::fabs(apps1_travel - apps2_travel) > MAX_APPS_DEVIATION;

    bool out_of_range_active =
        !in_range(apps1_voltage, APPS1_MIN_VOLTAGE - APPS_OUT_OF_RANGE_MARGIN,
                  APPS1_MAX_VOLTAGE + APPS_OUT_OF_RANGE_MARGIN) ||
        !in_range(apps2_voltage, APPS2_MIN_VOLTAGE - APPS_OUT_OF_RANGE_MARGIN,
                  APPS2_MAX_VOLTAGE + APPS_OUT_OF_RANGE_MARGIN);

    // One timer for both faults, kept running through brief clean gaps
    if (deviation_active || out_of_range_active) {
        clear_timer.stop();
        clear_timer.reset();

        deviation_seen = deviation_seen || deviation_active;
        out_of_range_seen = out_of_range_seen || out_of_range_active;

        fault_timer.start();
        if (fault_timer.elapsed_time() > IMPLAUS_TIME_LIMIT) {
            implaus_active = true;
        }
    } else {
        clear_timer.start();
        if (clear_timer.elapsed_time() > IMPLAUS_CLEAR_TIME) {
            implaus_active = false;
            deviation_seen = false;
            out_of_range_seen = false;
            fault_timer.stop();
            fault_timer.reset();
            clear_timer.stop();
            clear_timer.reset();
        }
    }

    implaus_apps_deviation = implaus_active && deviation_seen;
    implaus_apps_out_of_range = implaus_active && out_of_range_seen;
}
