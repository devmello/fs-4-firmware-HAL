#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>
#include <cstdint>

#include "timebase.h"

// Same behavior as mbed::Timer: start() does nothing if already running,
// reset() zeroes the count without stopping it. Backed by the 1 MHz TIM5
// counter, so a single run has to stay under ~71 minutes before it wraps.
class Stopwatch {
public:
    void start() {
        if (!running) {
            start_us = timebase_micros();
            running = true;
        }
    }

    void stop() {
        if (running) {
            accumulated_us += timebase_micros() - start_us;
            running = false;
        }
    }

    void reset() {
        start_us = timebase_micros();
        accumulated_us = 0;
    }

    std::chrono::microseconds elapsed_time() const {
        uint32_t total = accumulated_us;
        if (running) {
            total += timebase_micros() - start_us;
        }
        return std::chrono::microseconds{total};
    }

private:
    uint32_t start_us = 0;
    uint32_t accumulated_us = 0;
    bool running = false;
};

#endif
