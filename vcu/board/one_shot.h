#ifndef ONE_SHOT_H
#define ONE_SHOT_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <utility>

#include "timebase.h"

// Stand-in for mbed::Timeout without interrupts: attach() arms it and poll(),
// called every pass of the main loop, runs the callback once the delay is up.
// Fires up to one loop pass late. Same 32-bit microsecond counter as Stopwatch,
// so delays have to stay under ~71 minutes.
class OneShot {
public:
    void attach(std::function<void()> callback, std::chrono::microseconds delay) {
        fn = std::move(callback);
        start_us = timebase_micros();
        delay_us = static_cast<uint32_t>(delay.count());
        armed = true;
    }

    void detach() { armed = false; }

    void poll() {
        if (armed && timebase_micros() - start_us >= delay_us) {
            armed = false;
            fn();
        }
    }

private:
    std::function<void()> fn;
    uint32_t start_us = 0;
    uint32_t delay_us = 0;
    bool armed = false;
};

#endif
