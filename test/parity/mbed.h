// Just enough of mbed.h to build the original Mbed etc_controller.cpp on a
// host. AnalogIn and Timer are driven by the fake clock and ADC in parity.h.

#pragma once

#include <chrono>
#include <cstdint>

using namespace std::chrono;
using namespace std::chrono_literals;

typedef int PinName;

float parity_adc_sample(int input);
int64_t parity_now_us();

class AnalogIn {
public:
    AnalogIn(PinName pin) : pin(pin) {}
    float read_voltage() { return parity_adc_sample(pin); }

private:
    PinName pin;
};

// Same logic as TimerBase in mbed-os/drivers/source/Timer.cpp
class Timer {
public:
    void start() {
        if (!running) {
            start_us = parity_now_us();
            running = true;
        }
    }
    void stop() {
        time_us += slice();
        running = false;
    }
    void reset() {
        start_us = parity_now_us();
        time_us = 0;
    }
    microseconds elapsed_time() const { return microseconds{time_us + slice()}; }

private:
    int64_t slice() const { return running ? parity_now_us() - start_us : 0; }

    int64_t start_us = 0;
    int64_t time_us = 0;
    bool running = false;
};
