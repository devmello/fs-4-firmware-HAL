#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>
#include <cstdint>

#include "timebase.h"

// Same behavior as mbed::Timer: start() does nothing if already running,
// reset() zeroes the count without stopping it. Counts timebase_micros()'s
// 64-bit microseconds, like Mbed's Timer, so it doesn't wrap.
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
        uint64_t total = accumulated_us;
        if (running) {
            total += timebase_micros() - start_us;
        }
        return std::chrono::microseconds{static_cast<std::chrono::microseconds::rep>(total)};
    }

private:
    uint64_t start_us = 0;
    uint64_t accumulated_us = 0;
    bool running = false;
};

#endif
