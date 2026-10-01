# STM32F4 HAL + CMSIS from the drivers/ submodules.
#
# stm32f4_add_hal(<name> DEVICE <STM32F446xx> HSE_VALUE <hz> CONFIG_DIR <dir> MODULES <adc can ...>)
#
# Compiles the HAL for one board. CONFIG_DIR must contain that board's
# stm32f4xx_hal_conf.h. Only the listed modules (plus the ones every HAL
# project needs) are built. Link <name> into the firmware to get the objects,
# include paths, defines and Cortex-M4F flags.
#
# This is an OBJECT library on purpose. Some F446 functions (HAL_RCC_OscConfig
# and friends) are weak in stm32f4xx_hal_rcc.c and overridden in _rcc_ex.c. In
# a static archive the linker keeps the weak generic version and never pulls
# in the override.

set(STM32F4_DRIVERS_DIR ${CMAKE_SOURCE_DIR}/drivers)
set(STM32F4_HAL_DIR     ${STM32F4_DRIVERS_DIR}/stm32f4xx-hal-driver)
set(STM32F4_CMSIS_DIR   ${STM32F4_DRIVERS_DIR}/cmsis-device-f4)
set(CMSIS_CORE_DIR      ${STM32F4_DRIVERS_DIR}/cmsis-core)

if(NOT EXISTS ${STM32F4_HAL_DIR}/Src/stm32f4xx_hal.c)
    message(FATAL_ERROR "HAL sources missing. Run: git submodule update --init")
endif()

# The linker scripts use (READONLY), which needs GNU ld 2.38
# (Arm GNU Toolchain 11.3 or newer)
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-prog-name=ld
    OUTPUT_VARIABLE ARM_LD OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${ARM_LD} --version OUTPUT_VARIABLE ARM_LD_VERSION)
string(REGEX MATCH "^[^\n]*" ARM_LD_VERSION "${ARM_LD_VERSION}")
string(REGEX MATCH "[0-9]+\\.[0-9]+[0-9.]*$" ARM_LD_VERSION "${ARM_LD_VERSION}")
if(ARM_LD_VERSION AND ARM_LD_VERSION VERSION_LESS 2.38)
    message(FATAL_ERROR "GNU ld ${ARM_LD_VERSION} is too old, need 2.38+ (Arm GNU Toolchain 11.3 or newer)")
endif()

# Cortex-M4 with single precision FPU, newlib-nano. nano.specs goes on the
# compile line too so the libc headers match the library that gets linked.
add_library(cortex_m4f INTERFACE)
set(CORTEX_M4F_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs)
target_compile_options(cortex_m4f INTERFACE ${CORTEX_M4F_FLAGS})
target_link_options(cortex_m4f INTERFACE ${CORTEX_M4F_FLAGS})

function(stm32f4_add_hal name)
    cmake_parse_arguments(ARG "" "DEVICE;HSE_VALUE;CONFIG_DIR" "MODULES" ${ARGN})

    set(modules cortex rcc rcc_ex pwr pwr_ex flash flash_ex gpio dma ${ARG_MODULES})
    list(REMOVE_DUPLICATES modules)

    set(sources ${STM32F4_HAL_DIR}/Src/stm32f4xx_hal.c)
    foreach(module ${modules})
        list(APPEND sources ${STM32F4_HAL_DIR}/Src/stm32f4xx_hal_${module}.c)
    endforeach()

    add_library(${name} OBJECT ${sources})

    # HSE_VALUE on the command line too, system_stm32f4xx.c defaults to 25 MHz
    target_compile_definitions(${name} PUBLIC
        ${ARG_DEVICE}
        USE_HAL_DRIVER
        HSE_VALUE=${ARG_HSE_VALUE}U
    )

    # SYSTEM so vendor headers don't trip our warnings
    target_include_directories(${name} SYSTEM PUBLIC
        ${STM32F4_HAL_DIR}/Inc
        ${STM32F4_CMSIS_DIR}/Include
        ${CMSIS_CORE_DIR}/Include
    )
    target_include_directories(${name} PUBLIC ${ARG_CONFIG_DIR})

    target_compile_options(${name} PRIVATE -ffunction-sections -fdata-sections)
    target_link_libraries(${name} PUBLIC cortex_m4f)
endfunction()
