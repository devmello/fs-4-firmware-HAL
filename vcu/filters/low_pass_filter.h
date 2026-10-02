//
// Created by Jackson Pinsonneault on 6/1/26.
// Ported from Mbed to STM32 HAL.
//

#ifndef MBED_OS_LOWPASSFILTER_H
#define MBED_OS_LOWPASSFILTER_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include "stopwatch.h"

template <typename T>
class LowPassFilter {

public:
    /** Create a smoothing filter using EWMA (exponential weighted moving average) which resembles a low pass filter
     *
     * @param cutoff_frequency Cutoff frequency at the -3db level (see ../README.md, 𝜏 = 1 / (2π * f_c))
    */
    explicit LowPassFilter(const float cutoff_frequency) {
        // ETCState::motor_torque is built before board_init() starts TIM5. The
        // first sample() seeds the filter and resets the timer, so that's fine.
        _timer.start();
        set_time_constant(cutoff_frequency);
    }

    /** Read the previously filtered sample as an EWMA value, represented as this object's template
     *
     * @returns A value based off this object's template representing the previously filtered sample
    */
    T read() const {
        return static_cast<T>(_smoothed_value);
    }

    /** Sample a new EWMA value, represented as this object's template
     *
     * @param value New EWMA value that will be used as a new sample
     *
     * @returns A value based off this object's template respresenting the new filtered sample
    */
    T sample(const T value) {
        const float value_f = static_cast<float>(value);
        const unsigned long time_dif = std::chrono::duration_cast<std::chrono::microseconds>(_timer.elapsed_time()).count();
        _timer.reset();

        if (!_is_initialized) {
            _smoothed_value = value_f;
            _is_initialized = true;
        } else {
            const float exponent = -1.0 * static_cast<float>(time_dif) / std::pow(10,6) / _time_constant;
            const float exponential_component = std::exp(exponent);
            _smoothed_value = (1.0 - exponential_component) * value_f + exponential_component * _smoothed_value;
        }

        return static_cast<T>(_smoothed_value);
    }

    /** Changes the time constant to reflect the cutoff frequency at the -3db level
     *
     * @param cutoff_frequency Cutoff frequency at the -3db level (see ../README.md, 𝜏 = 1 / (2π * f_c))
    */
    void set_time_constant(const float cutoff_frequency) {
        // std::numbers::pi is the same double as M_PI, which newlib only
        // defines with GNU extensions on
        _time_constant = 1.0 / (2.0 * std::numbers::pi * cutoff_frequency);
    }

private:
    float _time_constant;           //  Time constant for RC sampling (set using cutoff frequency at the -3db level)
    float _smoothed_value = 0.0f;          //  The current value that should be returned by read()
    bool _is_initialized = false;   //  States whether the EWMA has started
    Stopwatch _timer;               //  Find the difference in times between reads
};

#endif // MBED_OS_LOWPASSFILTER_H
