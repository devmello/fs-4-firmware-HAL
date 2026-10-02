# Runs the ETC logic on the host with a fake ADC and clock.
#   cmake --preset host && cmake --build --preset host && ctest --preset host

add_executable(etc_test
    etc_test.cpp
    ${CMAKE_SOURCE_DIR}/vcu/etc/etc_controller.cpp
)
target_include_directories(etc_test PRIVATE
    ${CMAKE_SOURCE_DIR}/vcu/etc
    ${CMAKE_SOURCE_DIR}/vcu/board
)
if(MSVC)
    target_compile_options(etc_test PRIVATE /W4)
else()
    target_compile_options(etc_test PRIVATE -Wall -Wextra)
endif()

add_test(NAME etc_test COMMAND etc_test)

# Optional: check the port against the original Mbed code, e.g.
#   cmake --preset host -DMBED_VCU_DIR=../fs-electrical-onboarding/fw/vcu
set(MBED_VCU_DIR "" CACHE PATH "fw/vcu of the Mbed onboarding repo, enables the parity test")
if(MBED_VCU_DIR)
    if(NOT EXISTS ${MBED_VCU_DIR}/etc/etc_controller.cpp)
        message(FATAL_ERROR "No etc/etc_controller.cpp in MBED_VCU_DIR=${MBED_VCU_DIR}")
    endif()
    add_subdirectory(parity)
endif()

# VectorNav VN-200 driver tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/imu/CMakeLists.txt)
    add_subdirectory(imu)
endif()
