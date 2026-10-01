# Builds the original Mbed ETC and the HAL port against the same fake
# hardware, runs both on the same trace and checks the outputs match.

add_executable(etc_parity_mbed
    mbed_main.cpp
    ${MBED_VCU_DIR}/etc/etc_controller.cpp
)
target_include_directories(etc_parity_mbed PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MBED_VCU_DIR}/etc
)

add_executable(etc_parity_hal
    hal_main.cpp
    ${CMAKE_SOURCE_DIR}/vcu/etc/etc_controller.cpp
)
target_include_directories(etc_parity_hal PRIVATE
    ${CMAKE_SOURCE_DIR}/vcu/etc
    ${CMAKE_SOURCE_DIR}/vcu/board
)

# Second run starts just before the 32-bit microsecond counter wraps
foreach(start_us 0 4294000000)
    add_test(NAME etc_parity_${start_us}
        COMMAND ${CMAKE_COMMAND}
            -DMBED=$<TARGET_FILE:etc_parity_mbed>
            -DHAL=$<TARGET_FILE:etc_parity_hal>
            -DSTART_US=${start_us}
            -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/compare.cmake
    )
endforeach()
