# vcu: the real board. nucleo: a NUCLEO-F446RE for bench tests, same pins for
# the APPS and CAN, console on the ST-LINK's USART2, clock from the ST-LINK.
set(VCU_BOARD vcu CACHE STRING "Board to build for: vcu or nucleo")
set_property(CACHE VCU_BOARD PROPERTY STRINGS vcu nucleo)
if(VCU_BOARD STREQUAL "vcu")
    set(BOARD_HSE_VALUE 24000000) # Y1 crystal
    set(BOARD_DEFINE BOARD_VCU)
elseif(VCU_BOARD STREQUAL "nucleo")
    set(BOARD_HSE_VALUE 8000000)  # MCO from the ST-LINK
    set(BOARD_DEFINE BOARD_NUCLEO_F446RE)
else()
    message(FATAL_ERROR "VCU_BOARD must be vcu or nucleo, not '${VCU_BOARD}'")
endif()

stm32f4_add_hal(vcu_hal
    DEVICE STM32F446xx
    HSE_VALUE ${BOARD_HSE_VALUE}
    CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/board
    MODULES adc can iwdg tim tim_ex uart
)

add_executable(vcu
    main.cpp
    etc/etc_controller.cpp
    board/adc.c
    board/board.c
    board/can.c
    board/console.c
    board/gpio.c
    board/imu_uart.c
    board/syscalls.c
    board/timebase.c
    board/watchdog.c
    ${STM32F4_CMSIS_DIR}/Source/Templates/system_stm32f4xx.c
    ${STM32F4_CMSIS_DIR}/Source/Templates/gcc/startup_stm32f446xx.s
)

target_include_directories(vcu PRIVATE board etc)
target_compile_definitions(vcu PRIVATE ${BOARD_DEFINE})
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

# Full flash backup to backups/<board>-<date>-<time>.bin, and restore of a .bin.
# Both work on the nucleo preset too (same cfg as flash-vcu).
#   cmake --build --preset debug --target backup-vcu
#   cmake --preset debug -DRESTORE_BIN=backups/vcu-....bin
#   cmake --build --preset debug --target restore-vcu
# DRY_RUN=1 in the environment prints the OpenOCD command only.
set(RESTORE_BIN "" CACHE FILEPATH "Binary for restore-vcu to flash at 0x08000000")
set(FLASH_BACKUP_ARGS
    -DOPENOCD=${OPENOCD}
    -DCFG=${CMAKE_SOURCE_DIR}/tools/openocd/vcu.cfg
    -DBACKUP_CFG=${CMAKE_SOURCE_DIR}/tools/openocd/vcu-noreset.cfg
    -DBOARD=${VCU_BOARD}
)
add_custom_target(backup-vcu
    COMMAND ${CMAKE_COMMAND} ${FLASH_BACKUP_ARGS} -DMODE=backup
            -DOUT_DIR=${CMAKE_SOURCE_DIR}/backups
            -P ${CMAKE_SOURCE_DIR}/cmake/flash-backup.cmake
    USES_TERMINAL
    VERBATIM
)
add_custom_target(restore-vcu
    COMMAND ${CMAKE_COMMAND} ${FLASH_BACKUP_ARGS} -DMODE=restore
            -DRESTORE_BIN=${RESTORE_BIN}
            -P ${CMAKE_SOURCE_DIR}/cmake/flash-backup.cmake
    USES_TERMINAL
    VERBATIM
)
