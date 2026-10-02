// Just enough of mbed.h to build fs-4's etc_controller, traction_control and
// FilterUtils on a host. Time, analog reads, pin reads and writes and the RTD
// button's rise() callback go to the parity_* hooks in mbed_main.cpp.

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <math.h>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // as in newlib's math.h
#endif

typedef int PinName;

float parity_analog_read(PinName pin);
void parity_digital_write(PinName pin, int value);
int parity_digital_read(PinName pin);
void parity_attach_rise(PinName pin, std::function<void()> func);
int64_t parity_now_us();

class AnalogIn {
public:
    // vref defaults to target.default-adc-vref (3.3 in fs-4's mbed_app.json5)
    AnalogIn(PinName pin, float vref = 3.3) : pin(pin), vref(vref) {}

    float read() { return parity_analog_read(pin); }
    float read_voltage() { return read() * vref; }
    void set_reference_voltage(float v) { vref = v; }
    float get_reference_voltage() const { return vref; }

private:
    PinName pin;
    float vref;
};

class DigitalOut {
public:
    DigitalOut(PinName pin) : pin(pin) {}

    void write(int value) { parity_digital_write(pin, value); }

private:
    PinName pin;
};

class DigitalIn {
public:
    DigitalIn(PinName pin) : pin(pin) {}

    int read() { return parity_digital_read(pin); }
    int is_connected() { return 1; }

private:
    PinName pin;
};

class InterruptIn {
public:
    InterruptIn(PinName pin) : pin(pin) {}

    int read() { return parity_digital_read(pin); }
    void rise(std::function<void()> func) { parity_attach_rise(pin, std::move(func)); }

private:
    PinName pin;
};

template <typename T, typename U>
std::function<void()> callback(U *obj, void (T::*method)()) {
    return [obj, method] { (obj->*method)(); };
}

// Same logic as TimerBase in mbed-os drivers/source/Timer.cpp, on the fake
// 64-bit microsecond clock (Mbed's ticker is 64-bit too, so it never wraps)
class Timer {
public:
    void start() {
        if (!running) {
            start_us = parity_now_us();
            running = true;
        }
    }

    void stop() {
        time_us += slicetime();
        running = false;
    }

    void reset() {
        start_us = parity_now_us();
        time_us = 0;
    }

    std::chrono::microseconds elapsed_time() const {
        return std::chrono::microseconds{time_us + slicetime()};
    }

private:
    int64_t slicetime() const { return running ? parity_now_us() - start_us : 0; }

    int64_t start_us = 0;
    int64_t time_us = 0;
    bool running = false;
};

class Timeout;

inline std::vector<Timeout *> &parity_timeouts() {
    static std::vector<Timeout *> list;
    return list;
}

// Fires from parity_fire_timeouts(), at the first call at or after its time,
// like the ticker interrupt would
class Timeout {
public:
    Timeout() { parity_timeouts().push_back(this); }
    ~Timeout() { std::erase(parity_timeouts(), this); }
    Timeout(const Timeout &) = delete;
    Timeout &operator=(const Timeout &) = delete;

    void attach(std::function<void()> func, std::chrono::microseconds t) {
        fn = std::move(func);
        due_us = parity_now_us() + t.count();
        armed = true;
    }

    void detach() { armed = false; }

    void fire_if_due() {
        if (armed && parity_now_us() >= due_us) {
            armed = false; // detached before the call, like TimeoutBase::handler()
            fn();
        }
    }

private:
    std::function<void()> fn;
    int64_t due_us = 0;
    bool armed = false;
};

inline void parity_fire_timeouts() {
    for (Timeout *t : parity_timeouts()) {
        t->fire_if_due();
    }
}

// debounced_digital_in.h builds one at static init, but no DebouncedDigitalIn
// is ever made, so attach() never runs
class Ticker {
public:
    template <typename F, typename D>
    void attach(F &&, D) {
        std::fprintf(stderr, "fake Ticker::attach() called\n");
        std::abort();
    }
};

using namespace std;
