#ifndef PINS_H
#define PINS_H

// Small C++ wrappers over the board's C API, so code ported from Mbed keeps
// its shape: AnalogIn becomes AdcInput, DigitalOut becomes OutputPin, and
// DigitalIn or InterruptIn becomes InputPin.

#include <cstdint>

#include "adc.h"
#include "gpio.h"

class AdcInput {
public:
    explicit AdcInput(uint32_t channel) : channel(channel) {}

    // raw * (1 / 4095.0f), like AnalogIn::read()
    float read() { return adc_read(channel); }

    // read() * vref, like AnalogIn::read_voltage(). Kept out of line like
    // Mbed's: inlined, GCC fuses it with the caller's next subtraction into
    // one vfma, which moves the 403 steering angle by a count at some codes.
    __attribute__((noinline)) float read_voltage() { return read() * vref; }

    float get_reference_voltage() const { return vref; }
    void set_reference_voltage(float volts) { vref = volts; }

private:
    uint32_t channel;
    float vref = 3.3f; // target.default-adc-vref in the Mbed build
};

class OutputPin {
public:
    explicit OutputPin(gpio_output_t output) : output(output) {}

    // Any non-zero value drives the pin high, like DigitalOut::write()
    void write(int value) { gpio_write(output, value != 0); }

private:
    gpio_output_t output;
};

class InputPin {
public:
    explicit InputPin(gpio_input_t input) : input(input) {}

    int read() { return gpio_read(input) ? 1 : 0; }

private:
    gpio_input_t input;
};

#endif
