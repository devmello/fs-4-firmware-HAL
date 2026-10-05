# Builds fs-4's original ETC (from FS4_DIR) and the HAL port against the same
# fake hardware, runs both on the same trace and checks the outputs match.

# Copied into the build dir with fs-4's layout, so upstream.patch applies and
# etc_controller.h's "../imu/vectornav_imu.h" lands on the stub
set(FS4_FILES
    vcu/etc/etc_controller.h
    vcu/etc/etc_controller.cpp
    vcu/etc/traction_control.h
    vcu/etc/traction_control.cpp
    libraries/FilterUtils/include/debounced_digital_in.h
    libraries/FilterUtils/include/filtered_analog_in.h
    libraries/FilterUtils/include/low_pass_filter.h
    libraries/FilterUtils/filtered_analog_in.cpp
    libraries/FilterUtils/low_pass_filter.cpp
)
set(FS4_STAGE ${CMAKE_CURRENT_BINARY_DIR}/fs4-stage)
set(FS4_COPY ${CMAKE_CURRENT_BINARY_DIR}/fs4)
set(UPSTREAM_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/upstream.patch)
set(IMU_STUB ${CMAKE_CURRENT_SOURCE_DIR}/mbed/vectornav_imu.h)

file(REMOVE_RECURSE ${FS4_STAGE})
foreach(f IN LISTS FS4_FILES)
    if(NOT EXISTS ${FS4_DIR}/${f})
        message(FATAL_ERROR "Parity test: no ${f} in FS4_DIR=${FS4_DIR}")
    endif()
    get_filename_component(dir ${f} DIRECTORY)
    file(COPY ${FS4_DIR}/${f} DESTINATION ${FS4_STAGE}/${dir})
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${FS4_DIR}/${f})
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${UPSTREAM_PATCH} ${IMU_STUB})

# The port's changes in behavior, both in accelerator_mapping(): the deadzone
# clamp, and the cubic computed in place of the 41-point table. The test then
# checks the rest of the ETC matches bit for bit; etc_test checks how far the
# map is from upstream's table. If this stops applying, fs-4's ETC changed:
# port the change to vcu/, then redo the patch.
find_program(PATCH_EXECUTABLE patch REQUIRED)
execute_process(
    COMMAND ${PATCH_EXECUTABLE} -p1 -N -F0 -f -s -i ${UPSTREAM_PATCH}
    WORKING_DIRECTORY ${FS4_STAGE}
    RESULT_VARIABLE patch_result
    OUTPUT_VARIABLE patch_output
    ERROR_VARIABLE patch_output
)
if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR
        "test/parity/upstream.patch doesn't apply to ${FS4_DIR}, so fs-4's ETC "
        "changed. Port the change to vcu/, then update the patch.\n${patch_output}")
endif()

# Only rewrite the copies that changed, so a reconfigure doesn't rebuild them
foreach(f IN LISTS FS4_FILES)
    get_filename_component(dir ${f} DIRECTORY)
    file(MAKE_DIRECTORY ${FS4_COPY}/${dir})
    file(COPY_FILE ${FS4_STAGE}/${f} ${FS4_COPY}/${f} ONLY_IF_DIFFERENT)
endforeach()
file(MAKE_DIRECTORY ${FS4_COPY}/vcu/imu)
file(COPY_FILE ${IMU_STUB} ${FS4_COPY}/vcu/imu/vectornav_imu.h ONLY_IF_DIFFERENT)

add_executable(etc_parity_mbed
    mbed_main.cpp
    ${FS4_COPY}/vcu/etc/etc_controller.cpp
    ${FS4_COPY}/vcu/etc/traction_control.cpp
    ${FS4_COPY}/libraries/FilterUtils/filtered_analog_in.cpp
    ${FS4_COPY}/libraries/FilterUtils/low_pass_filter.cpp
)
target_include_directories(etc_parity_mbed PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mbed
    ${FS4_COPY}/vcu/etc
    ${FS4_COPY}/libraries/FilterUtils/include
)

add_executable(etc_parity_hal
    hal_main.cpp
    ${CMAKE_SOURCE_DIR}/vcu/etc/etc_controller.cpp
    ${CMAKE_SOURCE_DIR}/vcu/etc/traction_control.cpp
    ${CMAKE_SOURCE_DIR}/vcu/filters/filtered_analog_in.cpp
)
target_include_directories(etc_parity_hal PRIVATE
    ${CMAKE_SOURCE_DIR}/vcu/etc
    ${CMAKE_SOURCE_DIR}/vcu/filters
    ${CMAKE_SOURCE_DIR}/vcu/imu
    ${CMAKE_SOURCE_DIR}/vcu/board
)
if(NOT MSVC)
    target_compile_options(etc_parity_hal PRIVATE -Wall -Wextra)
endif()

# Off by default, it makes the runs several times slower
option(PARITY_SANITIZE "Build the parity executables with ASan and UBSan" OFF)
if(PARITY_SANITIZE)
    foreach(target etc_parity_mbed etc_parity_hal)
        target_compile_options(${target} PRIVATE
            -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endforeach()
endif()

# Second run starts just under 2^32 us, where a 32-bit microsecond count wraps
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
