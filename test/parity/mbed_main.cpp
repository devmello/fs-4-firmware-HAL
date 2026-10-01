// Original Mbed ETCController (from MBED_VCU_DIR) on the parity trace

#include "parity.h"

#include "etc_controller.h"

float parity_adc_sample(int input) {
    return parity::sample(input == 1);
}

int64_t parity_now_us() {
    return static_cast<int64_t>(parity::now_us);
}

int main(int argc, char **argv) {
    ETCController etc{1, 2};
    return parity::run(etc, argc, argv);
}
