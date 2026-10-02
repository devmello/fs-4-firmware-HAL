// HAL port of the fs-4 ETCController on the parity trace, with fake board
// functions

#include "parity.h"

#include "adc.h"
#include "etc_controller.h"
#include "gpio.h"
#include "timebase.h"

static_assert(parity::CH_APPS1 == ADC_CH_APPS1 && parity::CH_APPS2 == ADC_CH_APPS2 &&
              parity::CH_BPPS == ADC_CH_BPPS && parity::CH_FRONT_BSE == ADC_CH_FRONT_BSE &&
              parity::CH_REAR_BSE == ADC_CH_REAR_BSE);
static_assert(parity::PIN_RTD_LIGHT == OUT_RTD_LIGHT && parity::PIN_RTD_BUZZER == OUT_RTD_BUZZER &&
              parity::PIN_SOLENOID == OUT_SOLENOID && parity::PIN_BRAKELIGHT == OUT_BRAKELIGHT &&
              parity::PIN_RTD_BUTTON == IN_RTD_BUTTON);

// TIM5 is 32 bits, so this wraps like the real counter
extern "C" uint32_t timebase_micros(void) {
    return static_cast<uint32_t>(parity::now_us);
}

extern "C" float adc_read(uint32_t channel) {
    return parity::adc_read(channel);
}

extern "C" float adc_read_voltage(uint32_t channel) {
    return adc_read(channel) * 3.3f;
}

extern "C" void gpio_write(gpio_output_t output, bool level) {
    parity::output_level[output] = level ? 1 : 0;
}

extern "C" bool gpio_read(gpio_input_t input) {
    return input == IN_RTD_BUTTON && parity::rtd_button_level;
}

// The trace calls rtd_button_irq() itself, like main does when this moves
extern "C" uint32_t gpio_rtd_button_rises(void) {
    return 0;
}

int main(int argc, char **argv) {
    parity::start(argc, argv);
    ETCController etc{ADC_CH_APPS1, ADC_CH_APPS2, ADC_CH_BPPS, ADC_CH_FRONT_BSE, ADC_CH_REAR_BSE,
                      IN_RTD_BUTTON, OUT_RTD_LIGHT, OUT_RTD_BUZZER, OUT_SOLENOID, OUT_BRAKELIGHT};
    LowPassFilter<float> torque_probe{40};
    return parity::run(etc, torque_probe, [&etc] { etc.rtd_button_irq(); }, [] {});
}
