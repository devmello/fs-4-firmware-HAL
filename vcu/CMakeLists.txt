stm32f4_add_hal(vcu_hal
    DEVICE STM32F446xx
    HSE_VALUE 24000000
    CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/board
    MODULES adc can tim tim_ex uart
)

add_executable(vcu
    main.cpp
    etc/etc_controller.cpp
    board/adc.c
    board/board.c
    board/can.c
    board/console.c
    board/syscalls.c
    board/timebase.c
    ${STM32F4_CMSIS_DIR}/Source/Templates/system_stm32f4xx.c
    ${STM32F4_CMSIS_DIR}/Source/Templates/gcc/startup_stm32f446xx.s
)

target_include_directories(vcu PRIVATE board etc)
target_link_libraries(vcu PRIVATE vcu_hal)

target_compile_options(vcu PRIVATE
    -ffunction-sections -fdata-sections
    $<$<COMPILE_LANGUAGE:C,CXX>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics>
)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F446RETX_FLASH.ld)
target_link_options(vcu PRIVATE
    -T${LINKER_SCRIPT}
    -u _printf_float
    -Wl,--gc-sections
    -Wl,-Map=$<TARGET_FILE_DIR:vcu>/vcu.map
    -Wl,--print-memory-usage
)
set_target_properties(vcu PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

add_custom_command(TARGET vcu POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:vcu> $<TARGET_FILE_DIR:vcu>/vcu.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:vcu> $<TARGET_FILE_DIR:vcu>/vcu.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:vcu>
    VERBATIM
)

# cmake --build --preset debug --target flash-vcu
find_program(OPENOCD openocd)
add_custom_target(flash-vcu
    COMMAND ${OPENOCD} -f ${CMAKE_SOURCE_DIR}/tools/openocd/vcu.cfg
            -c "program {$<TARGET_FILE:vcu>} verify reset exit"
    DEPENDS vcu
    USES_TERMINAL
    VERBATIM
)
