// Drives an ETCController with a pseudo-random but repeatable trace of APPS
// voltages, ADC noise and loop timing, and prints its outputs after every
// update. The Mbed and HAL builds get the exact same trace, so their output
// files should be byte for byte identical.

#pragma once

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace parity {

struct Rng {
    uint64_t state;
    uint32_t next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<uint32_t>(state >> 11);
    }
    float uniform() { return static_cast<float>(next() & 0xFFFFFF) / static_cast<float>(0x1000000); }
};

inline uint64_t now_us = 0;
inline float apps1_volts = 0.0f;
inline float apps2_volts = 0.0f;
inline float noise_volts = 0.0f;
inline Rng sample_rng{0x9E3779B97F4A7C15ull};

constexpr int STEPS = 400000;

inline float sample(bool apps1) {
    float v = apps1 ? apps1_volts : apps2_volts;
    return v + (sample_rng.uniform() - 0.5f) * 2.0f * noise_volts;
}

// Pick a new pedal state: mostly agreeing sensors, some near the 10%
// deviation limit, some past it, some near or past the voltage limits
inline void new_regime(Rng &rng) {
    float travel = rng.uniform() * 1.2f - 0.1f;
    uint32_t kind = rng.next() % 8;
    float offset;
    if (kind == 0) {
        offset = 0.10f + (rng.uniform() - 0.5f) * 0.01f;
    } else if (kind == 1) {
        offset = 0.2f + rng.uniform() * 0.5f;
    } else {
        offset = (rng.uniform() - 0.5f) * 0.05f;
    }

    apps1_volts = 0.396f + travel * (1.086f - 0.396f);
    apps2_volts = 0.439f + (travel + offset) * (1.133f - 0.439f);
    if (kind == 3) {
        apps1_volts = 1.136f + (rng.uniform() - 0.5f) * 0.002f;
    }
    if (kind == 4) {
        apps2_volts = 0.389f + (rng.uniform() - 0.5f) * 0.002f;
    }
    if (kind == 5) {
        apps1_volts = 0.0f;
    }
    noise_volts = (rng.next() % 3 == 0) ? 0.02f : 0.002f;
}

// argv[1] = start time in us, to test the 32-bit timer wrap
template <typename ETC>
int run(ETC &etc, int argc, char **argv) {
    now_us = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 0;

    Rng rng{12345};
    new_regime(rng);
    for (int i = 0; i < STEPS; i++) {
        if (rng.next() % 40 == 0) {
            new_regime(rng);
        }

        etc.update_state();
        std::printf("%" PRIu64 " %.6f %.6f %.6f %.6f %.6f %d %d %d %d\n", now_us,
                    etc.apps1_voltage, etc.apps2_voltage, etc.apps1_position,
                    etc.apps2_position, etc.pedal_position, static_cast<int>(etc.torque_demand),
                    static_cast<int>(etc.implaus_apps_deviation),
                    static_cast<int>(etc.implaus_apps_out_of_range),
                    static_cast<int>(etc.motor_enabled));

        // Mostly a fast loop, sometimes a slow one, sometimes a long stall
        uint32_t k = rng.next() % 100;
        if (k < 70) {
            now_us += 20 + rng.next() % 400;
        } else if (k < 95) {
            now_us += 1000 + rng.next() % 20000;
        } else {
            now_us += 30000 + rng.next() % 90000;
        }
    }
    return 0;
}

} // namespace parity
