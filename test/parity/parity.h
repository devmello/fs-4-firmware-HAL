// Drives an fs-4 ETCController through a pseudo-random but repeatable trace
// and prints its state after every update. mbed_main.cpp runs fs-4's own code
// (copied from FS4_DIR, with upstream.patch applied) on the fake mbed.h in
// mbed/, hal_main.cpp runs the port on fake board functions. Both get the
// exact same trace, so their outputs have to match byte for byte.
//
// Each step does what the port's main loop does in one pass: CAN frames (with
// fs-4 main.cpp's handlers), the RTD button, update_state(), the 40 ms
// mbb_alive job. The Mbed side fires due Timeouts right before update_state(),
// which is where the port polls its OneShot.

#pragma once

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <numbers>

namespace parity {

struct Rng {
    uint64_t state;
    uint32_t next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<uint32_t>(state >> 11);
    }
    // [0, 1)
    float uniform() { return static_cast<float>(next() & 0xFFFFFF) / static_cast<float>(0x1000000); }
    float between(float low, float high) { return low + uniform() * (high - low); }
    bool one_in(uint32_t n) { return next() % n == 0; }
};

// ADC1 channels (the port's ADC_CH_*). The Mbed side uses them as PinNames.
constexpr uint32_t CH_REAR_BSE = 0;
constexpr uint32_t CH_FRONT_BSE = 1;
constexpr uint32_t CH_APPS1 = 11;
constexpr uint32_t CH_APPS2 = 12;
constexpr uint32_t CH_BPPS = 13;

// The port's gpio_output_t values and IN_RTD_BUTTON, also PinNames on the Mbed side
constexpr int PIN_RTD_LIGHT = 0;
constexpr int PIN_RTD_BUZZER = 1;
constexpr int PIN_SOLENOID = 2;
constexpr int PIN_BRAKELIGHT = 3;
constexpr int PIN_RTD_BUTTON = 0;

constexpr int STEPS = 150000;

inline uint64_t now_us = 0;
inline float volts[16] = {};       // by ADC channel
inline float noise_volts[16] = {}; // peak noise on each conversion
inline Rng adc_rng{0x9E3779B97F4A7C15ull};
inline int output_level[4] = {-1, -1, -1, -1}; // last level written, -1 = never
inline bool rtd_button_level = false;

// One conversion: volts plus noise as a 12-bit code, scaled like Mbed's
// AnalogIn::read() and the port's adc_read(): raw * (1 / 4095.0f)
inline float adc_read(uint32_t channel) {
    float v = volts[channel] + (adc_rng.uniform() - 0.5f) * 2.0f * noise_volts[channel];
    float raw = std::floor(v / 3.3f * 4095.0f + 0.5f);
    if (raw < 0.0f) {
        raw = 0.0f;
    }
    if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return raw * (1.0f / 4095.0f);
}

// Calibration from etc_controller.h, to aim the trace at its limits
constexpr float APPS1_MIN = 0.396f;
constexpr float APPS1_MAX = 1.086f;
constexpr float APPS2_MIN = 0.439f;
constexpr float APPS2_MAX = 1.133f;
constexpr float APPS_BUFFER = 0.015f;
constexpr float BPPS_MIN = 0.460f;
constexpr float BPPS_MAX = 0.972f;
constexpr float BPPS_BUFFER = 0.010f;
constexpr float BSE_LOW = 0.320f;  // range check, min - buffer
constexpr float BSE_HIGH = 1.406f; // max + buffer
constexpr float BSE_30_PSI = 0.3696f;

inline float pick_noise(Rng &rng) {
    uint32_t k = rng.next() % 8;
    if (k == 0) {
        return 0.0f;
    }
    return k < 3 ? 0.02f : 0.002f;
}

// Mostly agreeing sensors over the whole travel, some in the deadzone, near
// the 5% and 25% brake + accel limits, near or past the 10% deviation, near
// or past the voltage limits, unplugged (0 V) or shorted (3.3 V)
inline void new_apps_regime(Rng &rng) {
    float travel = rng.between(0.0f, 1.0f); // of the calibrated range, before the deadzone
    float offset = rng.between(-0.02f, 0.02f);
    uint32_t kind = rng.next() % 16;
    if (kind < 4) {
        travel = rng.between(-0.04f, 0.08f);
    } else if (kind == 8) {
        travel = rng.between(0.255f, 0.275f);
    } else if (kind == 9) {
        travel = rng.between(0.067f, 0.087f);
    } else if (kind == 10) {
        travel = rng.between(0.95f, 1.05f);
    } else if (kind == 11) {
        offset = (rng.one_in(2) ? 1.0f : -1.0f) * rng.between(0.089f, 0.099f); // 0.10 after the deadzone
    } else if (kind == 12) {
        offset = (rng.one_in(2) ? 1.0f : -1.0f) * rng.between(0.15f, 0.6f);
    }
    volts[CH_APPS1] = APPS1_MIN + travel * (APPS1_MAX - APPS1_MIN);
    volts[CH_APPS2] = APPS2_MIN + (travel + offset) * (APPS2_MAX - APPS2_MIN);

    if (kind == 13) {
        float d = rng.between(-0.004f, 0.004f);
        switch (rng.next() % 4) {
        case 0: volts[CH_APPS1] = APPS1_MIN - APPS_BUFFER + d; break;
        case 1: volts[CH_APPS1] = APPS1_MAX + APPS_BUFFER + d; break;
        case 2: volts[CH_APPS2] = APPS2_MIN - APPS_BUFFER + d; break;
        default: volts[CH_APPS2] = APPS2_MAX + APPS_BUFFER + d; break;
        }
    } else if (kind == 14) {
        volts[rng.one_in(2) ? CH_APPS1 : CH_APPS2] = 0.0f;
    } else if (kind == 15) {
        volts[rng.one_in(2) ? CH_APPS1 : CH_APPS2] = 3.3f;
    }
    noise_volts[CH_APPS1] = pick_noise(rng);
    noise_volts[CH_APPS2] = noise_volts[CH_APPS1];
}

// Released, pressed, near the 0.09 RTD threshold, near the range limits, rails
inline void new_bpps_regime(Rng &rng) {
    uint32_t kind = rng.next() % 10;
    float v;
    if (kind < 4) {
        v = rng.between(0.455f, 0.51f);
    } else if (kind < 7) {
        v = rng.between(0.52f, 0.97f);
    } else if (kind == 7) {
        v = rng.between(0.513f, 0.519f); // position 0.09 is at 0.51608 V
    } else if (kind == 8) {
        v = (rng.one_in(2) ? BPPS_MIN - BPPS_BUFFER : BPPS_MAX + BPPS_BUFFER) + rng.between(-0.004f, 0.004f);
    } else {
        v = rng.one_in(2) ? 0.0f : 3.3f;
    }
    volts[CH_BPPS] = v;
    noise_volts[CH_BPPS] = pick_noise(rng);
}

// No pressure, near 30 psi (brake light, brake + accel), braking, near the
// range limits, rails
inline void new_bse_regime(Rng &rng, uint32_t channel) {
    uint32_t kind = rng.next() % 10;
    float v;
    if (kind < 5) {
        v = rng.between(0.33f, 0.367f);
    } else if (kind == 5) {
        v = BSE_30_PSI + rng.between(-0.002f, 0.002f);
    } else if (kind < 8) {
        v = rng.between(0.372f, 1.39f);
    } else if (kind == 8) {
        v = (rng.one_in(2) ? BSE_LOW : BSE_HIGH) + rng.between(-0.004f, 0.004f);
    } else {
        v = rng.one_in(2) ? 0.0f : 3.3f;
    }
    volts[channel] = v;
    noise_volts[channel] = pick_noise(rng);
}

// Wheel speeds in 0.1 rpm, as the 421-424 frames carry them (fl, fr, bl, br)
inline uint16_t wheel_raw[4] = {};

// Stopped, around the 100 rpm activation (and exactly on it), driving with
// and without slip, front faster than rear, and the odd "negative" speed read
// as unsigned
inline void new_wheel_regime(Rng &rng) {
    uint32_t kind = rng.next() % 11;
    float front;
    if (kind < 2) {
        front = rng.between(0.0f, 80.0f);
    } else if (kind < 4) {
        front = rng.between(80.0f, 130.0f);
    } else {
        front = rng.between(100.0f, 1500.0f);
    }
    float rear = front * (1.0f + rng.between(-0.15f, 0.3f));
    if (kind == 8) {
        rear = front * rng.between(1.5f, 4.0f);
    } else if (kind == 9) {
        front = 0.0f;
    }
    float rpm[4] = {front, front, rear, rear};
    for (int i = 0; i < 4; i++) {
        float raw = rpm[i] * 10.0f + rng.between(-20.0f, 20.0f);
        wheel_raw[i] = static_cast<uint16_t>(raw < 0.0f ? 0.0f : raw);
    }
    if (kind == 10) {
        // Rear average right at 100.0 rpm, where the controller switches on
        wheel_raw[2] = static_cast<uint16_t>(999 + rng.next() % 3);
        wheel_raw[3] = static_cast<uint16_t>(2000 - wheel_raw[2]);
    } else if (rng.one_in(10)) {
        wheel_raw[rng.next() % 4] = static_cast<uint16_t>(65536 - 1 - rng.next() % 30);
    }
}

// What fs-4's main.cpp does with each received frame (main.cpp:63-131,
// 140-149, 270-272), with the port's ETC or the original
template <typename ETC>
struct Vcu {
    ETC &etc;
    bool wheel_fl_read = false;
    bool wheel_fr_read = false;
    bool wheel_bl_read = false;
    bool wheel_br_read = false;
    int tc_updates = 0;

    void powertrain_frame(uint32_t id, const uint8_t *data) {
        switch (id) {
        case 0x391: {
            etc.battery_precharged = data[0] & 0b01000000;
            etc.shutdown_closed = data[0] & 0b00000100;
            if (!etc.shutdown_closed || !etc.battery_precharged) {
                etc.turn_off_rtd();
            }
            break;
        }
        case 1154: { // SME_TPDO_Torque_speed
            uint16_t wheel_rpm = data[0] | (data[1] << 8);
            float wheel_radius = 0.190f;
            float ground_speed =
                (11 / 40.0f) * (2 * std::numbers::pi * wheel_radius); // gear ratio * circumference
            ground_speed *= wheel_rpm;                    // meters / minute
            ground_speed *= 60.0f / 1000.0f;              // km / hr

            etc.update_regen_state(ground_speed);
            break;
        }
        case 0x4c0: { // BATT_TPDO_TRAY_TEMPS
            uint8_t tray_temp_x2 = data[1];
            float tray_temp = tray_temp_x2 / 2.0;
            if (tray_temp > 40.0) {
                etc.turn_off_rtd();
            }
            break;
        }
        }
    }

    void data_frame(uint32_t id, const uint8_t *data) {
        switch (id) {
        case 421: {
            etc.state.wheel_rpm_fl = (data[0] + (data[1] << 8)) * 0.1f;
            wheel_fl_read = true;
            update_wheel_reads();
            break;
        }
        case 422: {
            etc.state.wheel_rpm_fr = (data[0] + (data[1] << 8)) * 0.1f;
            wheel_fr_read = true;
            update_wheel_reads();
            break;
        }
        case 423: {
            etc.state.wheel_rpm_bl = (data[0] + (data[1] << 8)) * 0.1f;
            wheel_bl_read = true;
            update_wheel_reads();
            break;
        }
        case 424: {
            etc.state.wheel_rpm_br = (data[0] + (data[1] << 8)) * 0.1f;
            wheel_br_read = true;
            update_wheel_reads();
            break;
        }
        case 432: {
            const uint8_t modes = data[0];
            etc.state.drive_mode = modes & 0b00000011;
            etc.state.traction_mode = (modes >> 2) & 0b00000011;
            etc.state.regen_mode = (modes >> 4) & 0b00000011;
        }
        }
    }

    void update_wheel_reads() {
        if (wheel_fl_read && wheel_fr_read && wheel_bl_read && wheel_br_read) {
            etc.state.tc_mult_factor = etc.traction_controller.update(
                etc.state.wheel_rpm_fl, etc.state.wheel_rpm_fr, etc.state.wheel_rpm_bl, etc.state.wheel_rpm_br);
            tc_updates++;

            wheel_fl_read = false;
            wheel_fr_read = false;
            wheel_bl_read = false;
            wheel_br_read = false;
        }
    }
};

inline void print_bool(bool b) {
    std::putchar(b ? '1' : '0');
}

template <typename ETC>
void print_state(ETC &etc, float torque_probe, float current_limit) {
    const auto &s = etc.state;
    const auto &tc = etc.traction_controller;
    std::printf("%" PRIu64 " |", now_us);
    std::printf(" %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g |",
                s.APPS1_voltage, s.APPS2_voltage, s.APPS1_position, s.APPS2_position,
                s.APPS_position_avg, s.BPPS_voltage, s.BPPS_position, s.front_BSE_voltage,
                s.rear_BSE_voltage, s.front_BSE_pressure, s.read_BSE_pressure);
    std::printf(" %d %d %d %d %d %d %d %d %d %d %d %d | ",
                static_cast<int>(s.unfiltered_motor_torque), static_cast<int>(s.motor_torque.read()),
                static_cast<int>(s.MAX_SPEED), static_cast<int>(s.CHARGE_CURRENT_LIMIT),
                static_cast<int>(s.MAX_DISCHARGE_CURRENT_LIMIT), static_cast<int>(s.DISCHARGE_CURRENT_LIMIT),
                static_cast<int>(s.mbb_alive), static_cast<int>(s.drive_mode),
                static_cast<int>(s.traction_mode), static_cast<int>(s.regen_mode),
                static_cast<int>(s.min_battery_voltage), static_cast<int>(s.current_draw));
    print_bool(s.rtd_button_pressed);
    print_bool(s.ready_to_drive);
    print_bool(s.motor_enabled);
    print_bool(s.implaus_APPS_deviation);
    print_bool(s.implaus_APPS_range);
    print_bool(s.implaus_BPPS_range);
    print_bool(s.implaus_BSE_range);
    print_bool(s.implaus_brake_and_accel);
    print_bool(s.regen_allowed);
    print_bool(s.solenoid_open);
    print_bool(s.reversing);
    print_bool(s.brakelight_enabled);
    print_bool(etc.battery_precharged);
    print_bool(etc.shutdown_closed);
    std::printf(" | %d %d %d %d |", output_level[PIN_RTD_LIGHT], output_level[PIN_RTD_BUZZER],
                output_level[PIN_SOLENOID], output_level[PIN_BRAKELIGHT]);
    std::printf(" %.9g %.9g %.9g %.9g %.9g |", s.wheel_rpm_fl, s.wheel_rpm_fr, s.wheel_rpm_bl,
                s.wheel_rpm_br, s.tc_mult_factor);
    std::printf(" %.9g %.9g %.9g %.9g %.9g %.9g | %.9g %.9g\n", tc.get_slip(), tc.get_integral(),
                tc.get_raw_derivative(), tc.get_smoothed_derivative(), tc.get_loop_time(),
                tc.get_output(), torque_probe, current_limit);
}

// What the trace reached, printed to stderr so a reviewer can see it does
// exercise the ETC (stdout is what gets compared)
struct Coverage {
    int rtd_on_steps = 0;
    int motor_enabled_steps = 0;
    int buzzer_on_steps = 0;
    int buzzer_timeouts = 0;
    int rtd_turned_on = 0;
    int rtd_turned_off_by_press = 0;
    int implaus_steps[5] = {};
    int brakelight_steps = 0;
    int negative_pre_map = 0;
    int capped_map = 0;
    int slip_nonzero = 0;
    float longest_tc_loop = 0.0f; // s

    template <typename ETC>
    void count(ETC &etc, int prev_buzzer) {
        const auto &s = etc.state;
        rtd_on_steps += s.ready_to_drive;
        motor_enabled_steps += s.motor_enabled;
        buzzer_on_steps += output_level[PIN_RTD_BUZZER] == 1;
        buzzer_timeouts += prev_buzzer == 1 && output_level[PIN_RTD_BUZZER] == 0;
        implaus_steps[0] += s.implaus_APPS_deviation;
        implaus_steps[1] += s.implaus_APPS_range;
        implaus_steps[2] += s.implaus_BPPS_range;
        implaus_steps[3] += s.implaus_BSE_range;
        implaus_steps[4] += s.implaus_brake_and_accel;
        brakelight_steps += s.brakelight_enabled;
        negative_pre_map += (s.APPS1_position + s.APPS2_position) / 2.0f < 0.0f;
        capped_map += s.APPS_position_avg == 1.0f;
        slip_nonzero += etc.traction_controller.get_slip() > 0.0f;
        if (etc.traction_controller.get_loop_time() > longest_tc_loop) {
            longest_tc_loop = etc.traction_controller.get_loop_time();
        }
    }

    void print(int tc_updates) const {
        std::fprintf(stderr,
                     "steps %d: RTD %d, motor enabled %d, buzzer on %d (%d timeouts), RTD on %d, "
                     "off by press %d, implaus dev %d range %d BPPS %d BSE %d brake+accel %d, "
                     "brake light %d, pedal below 0 before the map %d, map capped %d, "
                     "TC updates %d (slip > 0 in %d steps, longest loop time %.0f s)\n",
                     STEPS, rtd_on_steps, motor_enabled_steps, buzzer_on_steps, buzzer_timeouts,
                     rtd_turned_on, rtd_turned_off_by_press, implaus_steps[0], implaus_steps[1],
                     implaus_steps[2], implaus_steps[3], implaus_steps[4], brakelight_steps,
                     negative_pre_map, capped_map, tc_updates, slip_nonzero, longest_tc_loop);
    }
};

// argv[1]: start time in us (the second run starts just under 2^32). Call it
// before building the ETC, whose constructors start timers.
inline void start(int argc, char **argv) {
    now_us = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 0;
}

// torque_probe: a LowPassFilter<float> (each side's own) fed the same torque
// as motor_torque, because motor_torque.read() truncates its float state to an
// int16 and would hide a difference in the filter math.
// rtd_rise(): the RTD button rose (Mbed: the rise() callback, port:
// rtd_button_irq()). before_update(): the Mbed side fires due Timeouts.
template <typename ETC, typename Probe, typename RtdRise, typename BeforeUpdate>
int run(ETC &etc, Probe &torque_probe, RtdRise rtd_rise, BeforeUpdate before_update) {
    Rng rng{12345};
    Vcu<ETC> vcu{etc};
    Coverage coverage;
    uint64_t next_mbb_alive = now_us + 40000;

    // Boot: pedal released, brake held, then precharge and an RTD press, so
    // the buzzer's 2 s and the filters run across 2^32 us in the second run
    volts[CH_APPS1] = APPS1_MIN;
    volts[CH_APPS2] = APPS2_MIN;
    volts[CH_BPPS] = 0.7f;
    volts[CH_FRONT_BSE] = 0.34f;
    volts[CH_REAR_BSE] = 0.34f;
    for (uint32_t ch : {CH_APPS1, CH_APPS2, CH_BPPS, CH_FRONT_BSE, CH_REAR_BSE}) {
        noise_volts[ch] = 0.002f;
    }
    new_wheel_regime(rng);

    for (int step = 0; step < STEPS; step++) {
        bool scripted = step < 60;
        if (!scripted) {
            if (rng.one_in(40)) {
                new_apps_regime(rng);
            }
            if (rng.one_in(60)) {
                new_bpps_regime(rng);
            }
            if (rng.one_in(80)) {
                new_bse_regime(rng, CH_FRONT_BSE);
            }
            if (rng.one_in(80)) {
                new_bse_regime(rng, CH_REAR_BSE);
            }
        }
        if (rng.one_in(200)) {
            new_wheel_regime(rng);
        }

        // Powertrain bus, then data bus, like main's two drain loops
        uint8_t data[8] = {};
        if (step == 0 || rng.one_in(24)) {
            data[0] = (step == 0 || !rng.one_in(6)) ? 0x44 | (rng.next() & 0xBB) : rng.next() & 0xFF;
            vcu.powertrain_frame(0x391, data);
        }
        if (rng.one_in(3)) {
            uint32_t rpm = rng.one_in(3) ? rng.next() % 300 : rng.next() % 8000; // 5 km/h is ~254 rpm
            data[0] = rpm & 0xFF;
            data[1] = rpm >> 8;
            vcu.powertrain_frame(1154, data);
        }
        if (rng.one_in(48)) {
            uint32_t k = rng.next() % 8;
            data[1] = k == 0 ? 80 : (k < 3 ? 81 + rng.next() % 40 : 40 + rng.next() % 40); // 40 C is 80
            vcu.powertrain_frame(0x4c0, data);
        }
        int wheel_frames = rng.one_in(64) ? 8 : static_cast<int>(rng.next() % 4);
        for (int i = 0; i < wheel_frames; i++) {
            // A burst of 8 is every wheel twice: two TC updates at the same time
            int wheel = wheel_frames == 8 ? i % 4 : static_cast<int>(rng.next() % 4);
            data[0] = wheel_raw[wheel] & 0xFF;
            data[1] = wheel_raw[wheel] >> 8;
            vcu.data_frame(421 + wheel, data);
        }
        if (rng.one_in(150)) {
            data[0] = rng.next() & 0xFF;
            vcu.data_frame(432, data);
        }

        bool was_rtd = etc.state.ready_to_drive;
        if (step == 30 || (!scripted && !rtd_button_level && rng.one_in(250))) {
            rtd_button_level = true;
            rtd_rise();
        } else if (rtd_button_level && rng.one_in(15)) {
            rtd_button_level = false;
        }
        coverage.rtd_turned_on += !was_rtd && etc.state.ready_to_drive;
        coverage.rtd_turned_off_by_press += was_rtd && !etc.state.ready_to_drive;

        int prev_buzzer = output_level[PIN_RTD_BUZZER];
        before_update();
        etc.update_state();
        torque_probe.sample(etc.state.unfiltered_motor_torque);

        if (now_us >= next_mbb_alive) {
            etc.update_mbb_alive();
            next_mbb_alive += 40000;
            if (now_us >= next_mbb_alive) {
                next_mbb_alive = now_us + 40000;
            }
        }

        // Not called by fs-4's main, but it's ported code too
        float limit = etc.current_limit(rng.between(2.5f, 4.3f), rng.between(0.0f, 650.0f));
        etc.set_regen_torque(rng.one_in(2), rng.one_in(2), static_cast<int16_t>(rng.next()));

        print_state(etc, torque_probe.read(), limit);
        coverage.count(etc, prev_buzzer);

        // Mostly a fast loop, sometimes a slow one, sometimes a long stall
        uint32_t k = rng.next() % 100;
        if (k < 70) {
            now_us += 20 + rng.next() % 400;
        } else if (k < 95) {
            now_us += 1000 + rng.next() % 20000;
        } else {
            now_us += 30000 + rng.next() % 90000;
        }
        // Once, 72 min without a step: longer than 2^32 us, so the timers
        // running across it (traction control's loop timer, the filters') need
        // more than 32 bits of microseconds, as Mbed's Timer has
        if (step == STEPS / 2) {
            now_us += 72ull * 60 * 1000000;
        }
    }

    coverage.print(vcu.tc_updates);
    return 0;
}

} // namespace parity
