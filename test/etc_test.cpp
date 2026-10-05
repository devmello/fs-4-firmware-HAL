// Host tests for the fs-4 ETC port (ETCController, TractionController, the
// filters), Stopwatch and OneShot. The ADC, GPIO and timebase are replaced
// with the fakes below, so this builds with any desktop compiler.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>

#include "adc.h"
#include "etc_controller.h"
#include "filtered_analog_in.h"
#include "gpio.h"
#include "low_pass_filter.h"
#include "one_shot.h"
#include "pins.h"
#include "stopwatch.h"
#include "timebase.h"
#include "traction_control.h"

static uint64_t fake_now_us = 0;
static float fake_adc[16] = {};         // fraction of full scale, by ADC channel
static int fake_output[OUT_COUNT] = {}; // last level written, -1 = never
static int fake_output_writes[OUT_COUNT] = {};
static bool fake_input[IN_COUNT] = {};
static uint32_t fake_rtd_rises = 0;

extern "C" uint64_t timebase_micros(void) {
    return fake_now_us;
}

extern "C" float adc_read(uint32_t channel) {
    return fake_adc[channel];
}

extern "C" float adc_read_voltage(uint32_t channel) {
    return adc_read(channel) * 3.3f;
}

extern "C" void gpio_write(gpio_output_t output, bool level) {
    fake_output[output] = level ? 1 : 0;
    fake_output_writes[output]++;
}

extern "C" bool gpio_read(gpio_input_t input) {
    return fake_input[input];
}

extern "C" uint32_t gpio_rtd_button_rises(void) {
    return fake_rtd_rises;
}

static int failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                  \
        }                                                                \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                            \
    do {                                                                 \
        double a_ = (a), b_ = (b);                                       \
        if (!(std::fabs(a_ - b_) <= (tol))) {                            \
            std::printf("%s:%d: CHECK_NEAR(%s, %s) failed: %.9g vs %.9g\n", \
                        __FILE__, __LINE__, #a, #b, a_, b_);             \
            failures++;                                                  \
        }                                                                \
    } while (0)

// Calibration from etc_controller.h
constexpr float APPS1_MIN = 0.396f;
constexpr float APPS1_MAX = 1.086f;
constexpr float APPS2_MIN = 0.439f;
constexpr float APPS2_MAX = 1.133f;
constexpr float BPPS_MIN = 0.460f;
constexpr float BPPS_MAX = 0.972f;
constexpr int16_t MAX_TORQUE = 21298; // (int16_t)(32767 * 0.65)

static void set_volts(uint32_t channel, float volts) {
    fake_adc[channel] = volts / 3.3f;
}

// Fraction of each APPS's calibrated range, before the 3% deadzone
static void set_apps(float travel1, float travel2) {
    set_volts(ADC_CH_APPS1, APPS1_MIN + travel1 * (APPS1_MAX - APPS1_MIN));
    set_volts(ADC_CH_APPS2, APPS2_MIN + travel2 * (APPS2_MAX - APPS2_MIN));
}

// Pedal position the ETC works with: (travel - 0.03) / 0.94
static void set_pedal(float position) {
    float travel = position * 0.94f + 0.03f;
    set_apps(travel, travel);
}

// BPPS_position as the ETC computes it, buffer included
static void set_bpps(float position) {
    set_volts(ADC_CH_BPPS, BPPS_MIN + 0.010f + position * (BPPS_MAX - BPPS_MIN));
}

static void set_pressure(uint32_t channel, float psi) {
    set_volts(channel, (psi / 2000.0f * 2640.0f + 330.0f) / 1000.0f);
}

// Pedals released, no brake pressure, everything in range
static void set_inputs_at_rest() {
    set_apps(0.0f, 0.0f);
    set_volts(ADC_CH_BPPS, BPPS_MIN);
    set_volts(ADC_CH_FRONT_BSE, 0.340f);
    set_volts(ADC_CH_REAR_BSE, 0.340f);
}

static uint32_t rtd_rises_seen = 0;

static void reset_fakes() {
    fake_now_us = 0;
    for (int i = 0; i < OUT_COUNT; i++) {
        fake_output[i] = -1;
        fake_output_writes[i] = 0;
    }
    for (int i = 0; i < IN_COUNT; i++) {
        fake_input[i] = false;
    }
    fake_rtd_rises = 0;
    rtd_rises_seen = 0;
    set_inputs_at_rest();
}

static ETCController make_etc() {
    return ETCController{ADC_CH_APPS1, ADC_CH_APPS2, ADC_CH_BPPS, ADC_CH_FRONT_BSE, ADC_CH_REAR_BSE,
                         IN_RTD_BUTTON, OUT_RTD_LIGHT, OUT_RTD_BUZZER, OUT_SOLENOID, OUT_BRAKELIGHT};
}

// One update after `step_us`, then `count - 1` more
static void run(ETCController &etc, int count, uint32_t step_us) {
    for (int i = 0; i < count; i++) {
        fake_now_us += step_us;
        etc.update_state();
    }
}

// Same as main: one rtd_button_irq() per pass if the EXTI counter moved
static void main_pass(ETCController &etc) {
    uint32_t rises = gpio_rtd_button_rises();
    if (rises != rtd_rises_seen) {
        rtd_rises_seen = rises;
        etc.rtd_button_irq();
    }
    etc.update_state();
}

static void press_rtd(ETCController &etc) {
    fake_rtd_rises++;
    main_pass(etc);
}

// Precharged, shutdown closed and brake pressed, then RTD on
static void turn_on_rtd(ETCController &etc) {
    etc.battery_precharged = true;
    etc.shutdown_closed = true;
    set_bpps(0.3f);
    run(etc, 1, 50000); // BPPS_position comes from the last update
    press_rtd(etc);
}

static void test_constructor_drives_outputs_low() {
    reset_fakes();
    ETCController etc = make_etc();
    for (int i = 0; i < OUT_COUNT; i++) {
        CHECK(fake_output[i] == 0);
        CHECK(fake_output_writes[i] == 1);
    }
    CHECK(!etc.state.ready_to_drive);
    CHECK(!etc.state.motor_enabled);
    CHECK(etc.state.solenoid_open);
}

// The clamp in accelerator_mapping(), not in upstream
static void test_torque_at_rest_is_zero() {
    reset_fakes();
    ETCController etc = make_etc();
    etc.update_state();
    // The deadzone math puts a released pedal below 0 before the map
    CHECK(etc.state.APPS1_position < -0.03f);
    CHECK(etc.state.APPS2_position < -0.03f);
    CHECK(etc.state.APPS_position_avg == 0.0f);
    CHECK(etc.state.unfiltered_motor_torque == 0);
    CHECK(etc.state.motor_torque.read() == 0);

    // Inside the deadzone, where upstream extrapolated to negative torque
    for (float travel = 0.0f; travel < 0.029f; travel += 0.002f) {
        set_apps(travel, travel);
        run(etc, 1, 50000);
        CHECK(etc.state.APPS_position_avg == 0.0f);
        CHECK(etc.state.unfiltered_motor_torque == 0);
    }

    // Below the calibrated minimum but still in range
    set_volts(ADC_CH_APPS1, APPS1_MIN - 0.010f);
    set_volts(ADC_CH_APPS2, APPS2_MIN - 0.010f);
    run(etc, 1, 50000);
    CHECK(!etc.state.implaus_APPS_range);
    CHECK(etc.state.unfiltered_motor_torque == 0);
}

static float cubic_map(float x) {
    return -0.2f * x * x * x + 0.9f * x * x + 0.3f * x;
}

static void test_pedal_map() {
    reset_fakes();
    ETCController etc = make_etc();

    struct Point {
        float position;
        float torque_fraction;
    };
    // Exact table entries (i / 40)
    const Point points[] = {
        {0.1f, 0.0388f}, {0.25f, 0.128125f}, {0.5f, 0.35f}, {0.75f, 0.646875f}, {0.9f, 0.8532f},
    };
    for (const Point &p : points) {
        set_pedal(p.position);
        run(etc, 1, 50000);
        CHECK_NEAR((etc.state.APPS1_position + etc.state.APPS2_position) / 2.0f, p.position, 1e-5);
        CHECK_NEAR(etc.state.APPS_position_avg, p.torque_fraction, 2e-5);
    }

    // Between table points it's a linear interpolation of the cubic
    for (float position = 0.0f; position < 0.99f; position += 0.0123f) {
        set_pedal(position);
        run(etc, 1, 50000);
        float before_map = (etc.state.APPS1_position + etc.state.APPS2_position) / 2.0f;
        CHECK_NEAR(etc.state.APPS_position_avg, cubic_map(before_map), 1.6e-4);
        CHECK(etc.state.APPS_position_avg < 1.0f);
    }

    // index >= 40 returns 1.0, up to the top of the deadzone (1.0319)
    for (float travel : {0.98f, 0.99f, 1.0f, 1.02f}) {
        set_apps(travel, travel);
        run(etc, 1, 50000);
        CHECK(etc.state.APPS_position_avg == 1.0f);
        CHECK(etc.state.unfiltered_motor_torque == MAX_TORQUE);
    }
}

static void test_torque_scaling() {
    reset_fakes();
    ETCController etc = make_etc();
    for (float position = 0.0f; position <= 1.0f; position += 0.05f) {
        set_pedal(position);
        run(etc, 1, 50000);
        CHECK(etc.state.unfiltered_motor_torque ==
              static_cast<int16_t>(etc.state.APPS_position_avg * MAX_TORQUE));
    }
    set_pedal(0.5f);
    run(etc, 1, 50000);
    CHECK(std::abs(etc.state.unfiltered_motor_torque - 7454) <= 1); // 0.35 * 21298

    // Torque isn't zeroed while the motor is disabled (no RTD here)
    CHECK(!etc.state.motor_enabled);
    CHECK(etc.state.unfiltered_motor_torque > 0);
}

static void test_low_pass_filter() {
    const double tau = 1.0 / (2.0 * std::numbers::pi * 40.0);

    fake_now_us = 5000;
    LowPassFilter<int16_t> lpf{40};
    fake_now_us += 123456; // time before the first sample doesn't matter
    CHECK(lpf.sample(1000) == 1000);
    CHECK(lpf.read() == 1000);

    fake_now_us += 1000;
    double e = std::exp(-0.001 / tau);
    int16_t expected = static_cast<int16_t>(2000.0 * (1.0 - e) + 1000.0 * e); // 1222
    CHECK(lpf.sample(2000) == expected);
    CHECK(lpf.read() == expected);

    // No time passed: no change
    CHECK(lpf.sample(30000) == expected);

    // read() truncates toward zero
    LowPassFilter<int16_t> neg{40};
    neg.sample(-1000);
    fake_now_us += 1000;
    CHECK(neg.sample(-2000) == -expected);

    // Settles on a constant input
    for (int i = 0; i < 100; i++) {
        fake_now_us += 1000;
        lpf.sample(21298);
    }
    CHECK(lpf.read() >= 21297);
}

static void test_motor_torque_filter() {
    reset_fakes();
    ETCController etc = make_etc();
    set_pedal(0.5f);
    etc.update_state(); // first sample seeds the input filters and the torque filter
    int16_t seeded = etc.state.unfiltered_motor_torque;
    float apps1_seeded = etc.state.APPS1_voltage;
    CHECK(seeded > 7000);
    CHECK(etc.state.motor_torque.read() == seeded);

    // Pedal to the floor: the APPS inputs (60 Hz) and then the torque (40 Hz) lag
    set_apps(1.0f, 1.0f);
    run(etc, 1, 1000);
    double e60 = std::exp(-0.001 * 2.0 * std::numbers::pi * 60.0);
    double apps1_floored = APPS1_MAX;
    CHECK_NEAR(etc.state.APPS1_voltage, apps1_floored - (apps1_floored - apps1_seeded) * e60, 1e-5);
    int16_t unfiltered = etc.state.unfiltered_motor_torque;
    CHECK(unfiltered < MAX_TORQUE);
    double e40 = std::exp(-0.001 * 2.0 * std::numbers::pi * 40.0);
    CHECK(std::abs(etc.state.motor_torque.read() - static_cast<int>((1.0 - e40) * unfiltered + e40 * seeded)) <= 1);
    run(etc, 100, 1000);
    CHECK(etc.state.unfiltered_motor_torque == MAX_TORQUE);
    CHECK(etc.state.motor_torque.read() >= MAX_TORQUE - 1);
}

static void test_filtered_analog_in() {
    fake_now_us = 0;
    fake_adc[3] = 0.25f;
    AdcInput input{3};
    FilteredAnalogIn filtered{input, 60};
    fake_now_us = 777;
    CHECK(filtered.read() == 0.25f); // first read seeds

    fake_adc[3] = 0.75f;
    fake_now_us += 1000;
    double e = std::exp(-0.001 * 2.0 * std::numbers::pi * 60.0);
    CHECK_NEAR(filtered.read(), 0.75 - 0.5 * e, 1e-6);

    fake_now_us += 100000;
    CHECK_NEAR(filtered.read_voltage(), 0.75 * 3.3, 1e-6);
    CHECK(filtered.get_reference_voltage() == 3.3f);
    filtered.set_reference_voltage(5.0f);
    CHECK_NEAR(filtered.read_voltage(), 0.75 * 5.0, 1e-6);
    CHECK(filtered.read_u16() == static_cast<unsigned short>(0xFFFF * 0.75f));
}

struct ImplausCase {
    const char *name;
    void (*make_fault)();
    bool ETCState::*flag;
};

static void apps_deviation_fault() { set_apps(0.5f, 0.7f); }
static void apps_range_fault() { set_volts(ADC_CH_APPS1, 0.0f); } // unplugged
static void bpps_range_fault() { set_volts(ADC_CH_BPPS, 0.0f); }
static void bse_range_fault() { set_volts(ADC_CH_REAR_BSE, 3.3f); } // shorted

static const ImplausCase implaus_cases[] = {
    {"APPS deviation", apps_deviation_fault, &ETCState::implaus_APPS_deviation},
    {"APPS range", apps_range_fault, &ETCState::implaus_APPS_range},
    {"BPPS range", bpps_range_fault, &ETCState::implaus_BPPS_range},
    {"BSE range", bse_range_fault, &ETCState::implaus_BSE_range},
};

static int implaus_count(const ETCState &s) {
    return s.implaus_APPS_deviation + s.implaus_APPS_range + s.implaus_BPPS_range +
           s.implaus_BSE_range + s.implaus_brake_and_accel;
}

// Each flag sets once its fault has lasted more than 100 ms (whole ms) and
// clears on the first update without it
static void test_implaus_timers() {
    for (const ImplausCase &c : implaus_cases) {
        int failures_before = failures;
        reset_fakes();
        ETCController etc = make_etc();
        turn_on_rtd(etc);
        CHECK(etc.state.motor_enabled);

        c.make_fault();
        run(etc, 1, 50000); // filters settle within one 50 ms pass; timer starts
        CHECK(!(etc.state.*c.flag));
        CHECK(etc.state.motor_enabled);
        run(etc, 1, 100999); // 100 ms in whole ms
        CHECK(!(etc.state.*c.flag));
        CHECK(etc.state.motor_enabled);
        run(etc, 1, 1);
        CHECK(etc.state.*c.flag);
        CHECK(implaus_count(etc.state) == 1);
        CHECK(!etc.state.motor_enabled);
        run(etc, 10, 50000); // stays set while the fault lasts
        CHECK(etc.state.*c.flag);

        set_inputs_at_rest();
        run(etc, 1, 50000);
        CHECK(!(etc.state.*c.flag));
        CHECK(etc.state.motor_enabled);

        // A gap restarts the timer
        c.make_fault();
        run(etc, 2, 50000);
        set_inputs_at_rest();
        run(etc, 1, 50000);
        c.make_fault();
        run(etc, 1, 50000);
        run(etc, 1, 100999);
        CHECK(!(etc.state.*c.flag));
        run(etc, 1, 1);
        CHECK(etc.state.*c.flag);

        if (failures != failures_before) {
            std::printf("  (implausibility timer: %s)\n", c.name);
        }
    }
}

// Each check has its own timer
static void test_implaus_timers_independent() {
    reset_fakes();
    ETCController etc = make_etc();
    apps_deviation_fault();
    run(etc, 1, 0); // deviation timer starts at 0
    fake_now_us = 50000;
    bpps_range_fault();
    run(etc, 1, 0); // BPPS timer starts at 50 ms
    fake_now_us = 101000;
    run(etc, 1, 0);
    CHECK(etc.state.implaus_APPS_deviation);
    CHECK(!etc.state.implaus_BPPS_range);
    fake_now_us = 151000;
    run(etc, 1, 0);
    CHECK(etc.state.implaus_BPPS_range);
}

static void test_brake_and_accel() {
    reset_fakes();
    ETCController etc = make_etc();
    turn_on_rtd(etc);
    set_bpps(0.0f);
    set_pedal(0.5f);
    set_pressure(ADC_CH_FRONT_BSE, 29.0f);
    run(etc, 1, 50000);
    CHECK(!etc.state.implaus_brake_and_accel);
    CHECK(etc.state.motor_enabled);

    // Front pressure over 30 with the pedal over 25%: latched at once
    set_pressure(ADC_CH_FRONT_BSE, 31.0f);
    run(etc, 1, 50000);
    CHECK(etc.state.implaus_brake_and_accel);
    CHECK(!etc.state.motor_enabled);

    // Brake off, pedal still down: stays latched
    set_pressure(ADC_CH_FRONT_BSE, 0.0f);
    run(etc, 1, 50000);
    CHECK(etc.state.implaus_brake_and_accel);
    set_pedal(0.06f);
    run(etc, 1, 50000);
    CHECK(etc.state.implaus_brake_and_accel);

    // Clears under 5%
    set_pedal(0.04f);
    run(etc, 1, 50000);
    CHECK(!etc.state.implaus_brake_and_accel);
    CHECK(etc.state.motor_enabled);

    // Rear pressure doesn't count, and the pedal has to be over 25%
    set_pressure(ADC_CH_REAR_BSE, 500.0f);
    set_pedal(0.5f);
    run(etc, 1, 50000);
    CHECK(!etc.state.implaus_brake_and_accel);
    set_pressure(ADC_CH_FRONT_BSE, 500.0f);
    set_pedal(0.24f);
    run(etc, 1, 50000);
    CHECK(!etc.state.implaus_brake_and_accel);
    set_pedal(0.26f);
    run(etc, 1, 50000);
    CHECK(etc.state.implaus_brake_and_accel);
}

static void test_motor_enabled_needs_rtd() {
    reset_fakes();
    ETCController etc = make_etc();
    run(etc, 10, 50000);
    CHECK(implaus_count(etc.state) == 0);
    CHECK(!etc.state.ready_to_drive);
    CHECK(!etc.state.motor_enabled);

    turn_on_rtd(etc);
    CHECK(etc.state.ready_to_drive);
    CHECK(etc.state.motor_enabled);

    etc.turn_off_rtd();
    CHECK(!etc.state.ready_to_drive);
    CHECK(fake_output[OUT_RTD_LIGHT] == 0);
    run(etc, 1, 1000);
    CHECK(!etc.state.motor_enabled);
}

static void test_rtd_conditions() {
    for (int mask = 0; mask < 8; mask++) {
        bool precharged = mask & 1;
        bool shutdown_closed = mask & 2;
        bool brake = mask & 4;

        reset_fakes();
        ETCController etc = make_etc();
        etc.battery_precharged = precharged;
        etc.shutdown_closed = shutdown_closed;
        set_bpps(brake ? 0.10f : 0.08f); // has to be over 0.09
        run(etc, 1, 50000);
        press_rtd(etc);

        bool expected = precharged && shutdown_closed && brake;
        CHECK(etc.state.ready_to_drive == expected);
        CHECK(fake_output[OUT_RTD_LIGHT] == (expected ? 1 : 0));
        CHECK(fake_output[OUT_RTD_BUZZER] == (expected ? 1 : 0));
    }
}

static void test_press_while_rtd_turns_it_off() {
    reset_fakes();
    ETCController etc = make_etc();
    turn_on_rtd(etc);
    CHECK(etc.state.ready_to_drive);

    run(etc, 1, 500000);
    press_rtd(etc);
    CHECK(!etc.state.ready_to_drive);
    CHECK(!etc.state.motor_enabled);
    CHECK(fake_output[OUT_RTD_LIGHT] == 0);
    CHECK(fake_output[OUT_RTD_BUZZER] == 1); // turn_off_rtd() leaves the buzzer alone
    CHECK(etc.state.solenoid_open);          // SOLENOID_FORCE_OPEN: no solenoid toggle

    // Bounces within one main loop pass are one call, so RTD comes on and stays on
    run(etc, 1, 500000);
    fake_rtd_rises += 3;
    main_pass(etc);
    CHECK(etc.state.ready_to_drive);
}

static void test_buzzer() {
    reset_fakes();
    ETCController etc = make_etc();
    turn_on_rtd(etc);
    uint64_t on_at = fake_now_us;
    CHECK(fake_output[OUT_RTD_BUZZER] == 1);

    fake_now_us = on_at + 1999999;
    etc.update_state();
    CHECK(fake_output[OUT_RTD_BUZZER] == 1);
    fake_now_us = on_at + 2000000;
    etc.update_state(); // update_state() polls the timeout first
    CHECK(fake_output[OUT_RTD_BUZZER] == 0);
    CHECK(etc.state.ready_to_drive);

    // Off then on again within 2 s restarts the 2 s
    etc.turn_off_rtd();
    fake_now_us += 1000000;
    press_rtd(etc); // on
    fake_now_us += 1000000;
    press_rtd(etc); // off
    fake_now_us += 500000;
    press_rtd(etc); // on, 1.5 s after the first
    CHECK(etc.state.ready_to_drive);
    on_at = fake_now_us;
    fake_now_us = on_at + 1999999;
    etc.update_state();
    CHECK(fake_output[OUT_RTD_BUZZER] == 1);
    fake_now_us = on_at + 2000000;
    etc.update_state();
    CHECK(fake_output[OUT_RTD_BUZZER] == 0);
}

static void test_brakelight() {
    reset_fakes();
    ETCController etc = make_etc();
    int writes = fake_output_writes[OUT_BRAKELIGHT];
    set_pressure(ADC_CH_FRONT_BSE, 29.0f);
    run(etc, 1, 50000);
    CHECK(!etc.state.brakelight_enabled);
    CHECK(fake_output[OUT_BRAKELIGHT] == 0);
    set_pressure(ADC_CH_FRONT_BSE, 31.0f);
    run(etc, 1, 50000);
    CHECK(etc.state.brakelight_enabled);
    CHECK(fake_output[OUT_BRAKELIGHT] == 1);
    CHECK(fake_output_writes[OUT_BRAKELIGHT] == writes + 2); // written every pass

    // Front only
    set_pressure(ADC_CH_FRONT_BSE, 0.0f);
    set_pressure(ADC_CH_REAR_BSE, 500.0f);
    run(etc, 1, 50000);
    CHECK(!etc.state.brakelight_enabled);
    CHECK(fake_output[OUT_BRAKELIGHT] == 0);
}

static void test_solenoid_stays_open() {
    reset_fakes();
    ETCController etc = make_etc();
    turn_on_rtd(etc);
    int writes = fake_output_writes[OUT_SOLENOID];
    for (int i = 0; i < 5; i++) {
        run(etc, 1, 50000);
        press_rtd(etc);
        CHECK(etc.state.solenoid_open);
        CHECK(fake_output[OUT_SOLENOID] == 0); // pin = !solenoid_open
    }
    CHECK(fake_output_writes[OUT_SOLENOID] == writes + 10);
}

static void test_regen_disabled() {
    reset_fakes();
    ETCController etc = make_etc();
    set_bpps(0.95f); // over BPPS_MAX_NON_REGEN_BRAKING
    run(etc, 1, 50000);
    for (float speed : {0.0f, 3.0f, 5.0f, 5.1f, 30.0f, 120.0f}) {
        etc.update_regen_state(speed);
        CHECK(!etc.state.regen_allowed);
    }
    etc.state.regen_mode = 3;
    set_pedal(0.5f);
    run(etc, 1, 50000);
    CHECK(etc.state.unfiltered_motor_torque ==
          static_cast<int16_t>(etc.state.APPS_position_avg * MAX_TORQUE));
}

static void test_rtd_button_pressed_reads_pin() {
    reset_fakes();
    ETCController etc = make_etc();
    fake_input[IN_RTD_BUTTON] = true;
    run(etc, 1, 1000);
    CHECK(etc.state.rtd_button_pressed);
    fake_input[IN_RTD_BUTTON] = false;
    run(etc, 1, 1000);
    CHECK(!etc.state.rtd_button_pressed);
}

static void test_mbb_alive_and_current_limit() {
    reset_fakes();
    ETCController etc = make_etc();
    for (int i = 1; i <= 40; i++) {
        etc.update_mbb_alive();
        CHECK(etc.state.mbb_alive == i % 16);
    }

    // 2.5 - (V - 0.5) = 0 at 3.0 V, so the limit is the current itself
    CHECK_NEAR(etc.current_limit(3.0f, 100.0f), 100.0, 0.01);
    CHECK(etc.current_limit(3.5f, 100.0f) == 600.0f); // capped at MAX_DISCHARGE_CURRENT_LIMIT
}

static void test_traction_control() {
    fake_now_us = 1000;
    TractionController tc;
    CHECK(tc.get_output() == 1.0f);
    CHECK(tc.get_slip() == 0.0f);

    // Rear at or below 100 rpm: no reduction, controller reset
    fake_now_us += 5000;
    CHECK(tc.update(0.0f, 0.0f, 100.0f, 100.0f) == 1.0f);
    CHECK(tc.get_slip() == 0.0f);
    CHECK(tc.get_loop_time() == 0.0f);

    // Loop time from the Stopwatch, here 5 ms since the reset above
    fake_now_us += 5000;
    CHECK(tc.update(200.0f, 200.0f, 300.0f, 300.0f) == 1.0f); // zero gains
    CHECK_NEAR(tc.get_slip(), 1.0 / 3.0, 1e-6);
    CHECK(tc.get_loop_time() == 5000 / 1000000.0f);
    CHECK(tc.get_raw_derivative() == 0.0f); // no previous slip yet
    CHECK(tc.get_output() == 1.0f);

    fake_now_us += 10000;
    CHECK(tc.update(150.0f, 150.0f, 300.0f, 300.0f) == 1.0f);
    CHECK_NEAR(tc.get_slip(), 0.5, 1e-6);
    CHECK(tc.get_loop_time() == 10000 / 1000000.0f);
    double raw = (0.5 - 1.0 / 3.0) / 0.01;
    CHECK_NEAR(tc.get_raw_derivative(), raw, 1e-3);
    CHECK_NEAR(tc.get_smoothed_derivative(), raw, 1e-3); // first one seeds the filter
    CHECK(tc.get_integral() == 0.0f);                   // KI = 0 clamps it to 0

    fake_now_us += 20000;
    CHECK(tc.update(180.0f, 180.0f, 300.0f, 300.0f) == 1.0f);
    double raw2 = (0.4 - 0.5) / 0.02;
    double k = 0.02 / (0.2 + 0.02);
    CHECK_NEAR(tc.get_raw_derivative(), raw2, 1e-3);
    CHECK_NEAR(tc.get_smoothed_derivative(), (1 - k) * raw + k * raw2, 1e-3);

    // Slip is clamped to [0, 1]
    fake_now_us += 10000;
    tc.update(400.0f, 400.0f, 300.0f, 300.0f);
    CHECK(tc.get_slip() == 0.0f);
    fake_now_us += 10000;
    tc.update(0.0f, 0.0f, 300.0f, 300.0f);
    CHECK(tc.get_slip() == 1.0f);
    CHECK(tc.get_output() == 1.0f);

    // Slowing down resets it
    fake_now_us += 10000;
    CHECK(tc.update(0.0f, 0.0f, 50.0f, 50.0f) == 1.0f);
    CHECK(tc.get_slip() == 0.0f);
    CHECK(tc.get_raw_derivative() == 0.0f);
    CHECK(tc.get_smoothed_derivative() == 0.0f);
    CHECK(tc.get_integral() == 0.0f);

    // 72 min until the next update, more than 2^32 us: the loop time is all
    // of it, as with Mbed's 64-bit Timer
    fake_now_us += 72ull * 60 * 1000000;
    tc.update(200.0f, 200.0f, 300.0f, 300.0f);
    CHECK_NEAR(tc.get_loop_time(), 72 * 60.0, 1e-3);
}

static void test_stopwatch() {
    using std::chrono::microseconds;

    fake_now_us = 1000;
    Stopwatch sw;
    CHECK(sw.elapsed_time() == microseconds{0});

    sw.start();
    fake_now_us += 250;
    CHECK(sw.elapsed_time() == microseconds{250});

    sw.start(); // no-op while running
    fake_now_us += 250;
    CHECK(sw.elapsed_time() == microseconds{500});

    sw.stop();
    fake_now_us += 1000;
    CHECK(sw.elapsed_time() == microseconds{500});

    sw.start(); // resumes, keeps the 500
    fake_now_us += 100;
    CHECK(sw.elapsed_time() == microseconds{600});

    sw.reset(); // zeroes but keeps running
    fake_now_us += 40;
    CHECK(sw.elapsed_time() == microseconds{40});

    // Across 2^32 us and past it, where a 32-bit microsecond count wraps
    Stopwatch long_run;
    fake_now_us = 0xFFFFFFF0u;
    long_run.start();
    fake_now_us += 0x20;
    CHECK(long_run.elapsed_time() == microseconds{0x20});
    fake_now_us += 0x100000000u;
    CHECK(long_run.elapsed_time() == microseconds{0x100000020});
}

static void test_one_shot() {
    int fired = 0;
    OneShot shot;

    // Fires once, on the first poll() at or after the delay
    fake_now_us = 1000;
    shot.attach([&fired] { fired++; }, std::chrono::milliseconds{2});
    fake_now_us += 1999;
    shot.poll();
    CHECK(fired == 0);
    fake_now_us += 1;
    shot.poll();
    CHECK(fired == 1);
    fake_now_us += 5000;
    shot.poll();
    CHECK(fired == 1);

    // detach() before the delay: never fires
    shot.attach([&fired] { fired++; }, std::chrono::milliseconds{1});
    shot.detach();
    fake_now_us += 2000;
    shot.poll();
    CHECK(fired == 1);

    // A delay longer than 2^32 us, started just before 2^32. Cut to 32 bits
    // it would be ~25 s.
    const uint64_t attached_at = 0xFFFFFF00u;
    fake_now_us = attached_at;
    shot.attach([&fired] { fired++; }, std::chrono::minutes{72});
    fake_now_us += 60'000'000;
    shot.poll();
    CHECK(fired == 1);
    fake_now_us = attached_at + 72ull * 60 * 1000000 - 1;
    shot.poll();
    CHECK(fired == 1);
    fake_now_us += 1;
    shot.poll();
    CHECK(fired == 2);
}

int main() {
    test_constructor_drives_outputs_low();
    test_torque_at_rest_is_zero();
    test_pedal_map();
    test_torque_scaling();
    test_low_pass_filter();
    test_motor_torque_filter();
    test_filtered_analog_in();
    test_implaus_timers();
    test_implaus_timers_independent();
    test_brake_and_accel();
    test_motor_enabled_needs_rtd();
    test_rtd_conditions();
    test_press_while_rtd_turns_it_off();
    test_buzzer();
    test_brakelight();
    test_solenoid_stays_open();
    test_regen_disabled();
    test_rtd_button_pressed_reads_pin();
    test_mbb_alive_and_current_limit();
    test_traction_control();
    test_stopwatch();
    test_one_shot();

    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
