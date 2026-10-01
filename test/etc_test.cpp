// Host tests for ETCController and Stopwatch. The ADC and TIM5 counter are
// replaced with the fakes below, so this builds with any desktop compiler.

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "adc.h"
#include "etc_controller.h"
#include "stopwatch.h"
#include "timebase.h"

static uint32_t fake_now_us = 0;
static float fake_voltage[16] = {};

extern "C" uint32_t timebase_micros(void) {
    return fake_now_us;
}

extern "C" float adc_read_voltage(uint32_t channel) {
    return fake_voltage[channel];
}

static int failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                  \
        }                                                                \
    } while (0)

constexpr uint32_t APPS1 = 11;
constexpr uint32_t APPS2 = 12;
constexpr int16_t MAX_TORQUE = 3276; // 32767 * 0.1

// Calibration from etc_controller.h
static void set_travel(float apps1, float apps2) {
    fake_voltage[APPS1] = 0.396f + apps1 * (1.086f - 0.396f);
    fake_voltage[APPS2] = 0.439f + apps2 * (1.133f - 0.439f);
}

// One update per millisecond, like a (slow) main loop
static void run_ms(ETCController &etc, int ms) {
    for (int i = 0; i < ms; i++) {
        etc.update_state();
        fake_now_us += 1000;
    }
}

static void reset_fakes() {
    fake_now_us = 0;
    set_travel(0.0f, 0.0f);
}

// The ETC starts with the motor disabled and needs >100 ms of clean readings
static void settle(ETCController &etc) {
    run_ms(etc, 102);
}

static void test_starts_disabled() {
    reset_fakes();
    set_travel(0.5f, 0.5f);
    ETCController etc{APPS1, APPS2};

    run_ms(etc, 101); // t = 0..100 ms, clean but not for more than 100 ms yet
    CHECK(!etc.motor_enabled);
    CHECK(etc.torque_demand == 0);
    CHECK(!etc.implaus_apps_deviation);
    CHECK(!etc.implaus_apps_out_of_range);

    run_ms(etc, 1); // t = 101 ms
    CHECK(etc.motor_enabled);
    CHECK(etc.torque_demand > 0);
}

static void test_pedal_released() {
    reset_fakes();
    ETCController etc{APPS1, APPS2};
    settle(etc);
    CHECK(etc.motor_enabled);
    CHECK(etc.torque_demand == 0);
    CHECK(etc.pedal_position == 0.0f);
    CHECK(!etc.implaus_apps_deviation);
    CHECK(!etc.implaus_apps_out_of_range);
}

static void test_pedal_full() {
    reset_fakes();
    set_travel(1.0f, 1.0f);
    ETCController etc{APPS1, APPS2};
    settle(etc);
    CHECK(etc.motor_enabled);
    CHECK(etc.pedal_position == 1.0f);
    CHECK(etc.torque_demand == MAX_TORQUE);
}

static void test_pedal_half() {
    reset_fakes();
    set_travel(0.5f, 0.5f);
    ETCController etc{APPS1, APPS2};
    settle(etc);
    // position 0.5 -> map 0.5 * (0.5 + 0.5 * 0.5) = 0.375
    CHECK(std::fabs(etc.pedal_position - 0.5f) < 1e-4f);
    CHECK(std::abs(etc.torque_demand - 1228) <= 1);
}

static void test_deviation_trips_after_100ms() {
    reset_fakes();
    set_travel(0.5f, 0.5f);
    ETCController etc{APPS1, APPS2};
    settle(etc);

    set_travel(0.5f, 0.62f); // 12% apart

    run_ms(etc, 101); // 0..100 ms into the fault, exactly 100 ms is still allowed
    CHECK(etc.motor_enabled);
    CHECK(etc.torque_demand > 0);

    run_ms(etc, 1);
    CHECK(!etc.motor_enabled);
    CHECK(etc.torque_demand == 0);
    CHECK(etc.implaus_apps_deviation);
    CHECK(!etc.implaus_apps_out_of_range);
}

static void test_out_of_range_trips() {
    reset_fakes();
    // Both sensors shorted high, past max + 0.05 V but still agreeing
    ETCController etc{APPS1, APPS2};
    settle(etc);

    fake_voltage[APPS1] = 1.20f;
    fake_voltage[APPS2] = 1.247f;
    run_ms(etc, 102);
    CHECK(!etc.motor_enabled);
    CHECK(etc.implaus_apps_out_of_range);
    CHECK(!etc.implaus_apps_deviation);
}

static void test_brief_fault_ignored() {
    reset_fakes();
    ETCController etc{APPS1, APPS2};
    settle(etc);

    set_travel(0.5f, 0.7f);
    run_ms(etc, 50);
    set_travel(0.5f, 0.5f);
    run_ms(etc, 200);
    CHECK(etc.motor_enabled);
    CHECK(!etc.implaus_apps_deviation);
}

// The fault timer keeps running through clean gaps shorter than the clear time
static void test_fault_timer_spans_short_gaps() {
    reset_fakes();
    ETCController etc{APPS1, APPS2};
    settle(etc);

    set_travel(0.5f, 0.7f);
    run_ms(etc, 60);  // 0..59 ms
    set_travel(0.5f, 0.5f);
    run_ms(etc, 50);  // 60..109 ms
    CHECK(etc.motor_enabled);

    set_travel(0.5f, 0.7f);
    run_ms(etc, 1);   // 110 ms since the first fault
    CHECK(!etc.motor_enabled);
    CHECK(etc.implaus_apps_deviation);
}

static void test_recovers_after_100ms_clean() {
    reset_fakes();
    ETCController etc{APPS1, APPS2};
    settle(etc);

    set_travel(0.5f, 0.7f);
    run_ms(etc, 150);
    CHECK(!etc.motor_enabled);

    set_travel(0.3f, 0.3f);
    run_ms(etc, 101);
    CHECK(!etc.motor_enabled);

    run_ms(etc, 1);
    CHECK(etc.motor_enabled);
    CHECK(!etc.implaus_apps_deviation);
    CHECK(etc.torque_demand > 0);
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

    // 32-bit counter wrap
    Stopwatch wrap;
    fake_now_us = 0xFFFFFFF0u;
    wrap.start();
    fake_now_us += 0x20;
    CHECK(wrap.elapsed_time() == microseconds{0x20});
}

int main() {
    test_starts_disabled();
    test_pedal_released();
    test_pedal_full();
    test_pedal_half();
    test_deviation_trips_after_100ms();
    test_out_of_range_trips();
    test_brief_fault_ignored();
    test_fault_timer_spans_short_gaps();
    test_recovers_after_100ms_clean();
    test_stopwatch();

    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
