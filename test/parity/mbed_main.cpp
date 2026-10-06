// fs-4's original ETCController (copied from FS4_DIR, see CMakeLists.txt) on
// the parity trace, with the fake mbed.h

#include "parity.h"

#include <functional>
#include <utility>

#include "etc_controller.h"

static std::function<void()> rtd_rise;

float parity_analog_read(PinName pin) {
    return parity::adc_read(static_cast<uint32_t>(pin));
}

void parity_digital_write(PinName pin, int value) {
    parity::output_level[pin] = value != 0 ? 1 : 0;
}

int parity_digital_read(PinName pin) {
    return pin == parity::PIN_RTD_BUTTON && parity::rtd_button_level ? 1 : 0;
}

void parity_attach_rise(PinName, std::function<void()> func) {
    rtd_rise = std::move(func);
}

int64_t parity_now_us() {
    return static_cast<int64_t>(parity::now_us);
}

int main(int argc, char **argv) {
    using namespace parity;
    start(argc, argv);
    ETCController etc{CH_APPS1, CH_APPS2, CH_BPPS, CH_FRONT_BSE, CH_REAR_BSE,
                      PIN_RTD_BUTTON, PIN_RTD_LIGHT, PIN_RTD_BUZZER, PIN_SOLENOID, PIN_BRAKELIGHT};
    LowPassFilter<float> torque_probe{40};
    return run(etc, torque_probe, [] { rtd_rise(); }, [] { parity_fire_timeouts(); });
}
