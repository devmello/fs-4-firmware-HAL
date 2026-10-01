// HAL port of ETCController on the parity trace

#include "parity.h"

#include "adc.h"
#include "etc_controller.h"
#include "timebase.h"

// TIM5 is 32 bits, so this wraps like the real counter
extern "C" uint32_t timebase_micros(void) {
    return static_cast<uint32_t>(parity::now_us);
}

extern "C" float adc_read_voltage(uint32_t channel) {
    return parity::sample(channel == 11);
}

int main(int argc, char **argv) {
    ETCController etc{11, 12};
    return parity::run(etc, argc, argv);
}
