//
// Created by Jackson Pinsonneault on 3/24/26.
// Ported from Mbed to STM32 HAL.
//

#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include <chrono>
#include <cstdint>

#include "stopwatch.h"

class ETCController {
public:
    float apps1_voltage = 0.0f;
    float apps2_voltage = 0.0f;

    float apps1_position = 0.0f;
    float apps2_position = 0.0f;
    float pedal_position = 0.0f;

    int16_t torque_demand = 0;
    bool implaus_apps_deviation = false;
    bool implaus_apps_out_of_range = false;
    bool motor_enabled = false;

    // ADC1 channels (ADC_CHANNEL_x) for each APPS sensor
    ETCController(uint32_t apps1_channel, uint32_t apps2_channel);

    void update_state();

private:
    uint32_t apps1_channel;
    uint32_t apps2_channel;

    float apps1_travel = 0.0f;  // unclamped, before deadzone
    float apps2_travel = 0.0f;

    static constexpr int SAMPLES_PER_READ = 8;

    Stopwatch fault_timer;
    Stopwatch clear_timer;
    bool implaus_active = true;  // start disabled
    bool deviation_seen = false;
    bool out_of_range_seen = false;

    static constexpr float MAX_APPS_DEVIATION = 0.10f;       // T.4.2.4
    static constexpr float APPS_OUT_OF_RANGE_MARGIN = 0.05f; // T.4.2.10
    static constexpr std::chrono::milliseconds IMPLAUS_TIME_LIMIT{100}; // T.4.2.5
    static constexpr std::chrono::milliseconds IMPLAUS_CLEAR_TIME{100};

    static constexpr float PEDAL_MAP_LINEARITY = 0.5f;       // 1.0 = linear

    /*
        !! IMPORTANT: All global voltages are based off     !!
        !! an older testing time. Still, voltages           !!
        !! should always be checked and updated if wrong.   !!
    */

    static constexpr float APPS1_MIN_VOLTAGE = 0.396;
    static constexpr float APPS1_MAX_VOLTAGE = 1.086f;

    static constexpr float APPS2_MIN_VOLTAGE = 0.439f;
    static constexpr float APPS2_MAX_VOLTAGE = 1.133f;

    static constexpr float PEDAL_DEADZONE_PERCENTAGE = 0.03;

    static constexpr int16_t MAX_TORQUE = 32767 * 0.1; // Multiplied by 0.1 for controlled
                                                       // motor torque during onboarding.

    float clamp(float value);

    bool in_range(float value, float low, float high);

    float read_average_voltage(uint32_t channel);

    float voltage_to_travel(float voltage, float min_voltage, float max_voltage);

    float travel_to_position(float travel);

    float map_pedal(float position);

    void update_implausibilities();
};

#endif
