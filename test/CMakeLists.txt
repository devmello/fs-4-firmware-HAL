# Runs the ETC logic on the host with a fake ADC, GPIO and clock.
#   cmake --preset host && cmake --build --preset host && ctest --preset host

add_executable(etc_test
    etc_test.cpp
    ${CMAKE_SOURCE_DIR}/vcu/etc/etc_controller.cpp
    ${CMAKE_SOURCE_DIR}/vcu/etc/traction_control.cpp
    ${CMAKE_SOURCE_DIR}/vcu/filters/filtered_analog_in.cpp
)
target_include_directories(etc_test PRIVATE
    ${CMAKE_SOURCE_DIR}/vcu/etc
    ${CMAKE_SOURCE_DIR}/vcu/filters
    ${CMAKE_SOURCE_DIR}/vcu/imu
    ${CMAKE_SOURCE_DIR}/vcu/board
)
if(MSVC)
    target_compile_options(etc_test PRIVATE /W4)
else()
    target_compile_options(etc_test PRIVATE -Wall -Wextra)
endif()

add_test(NAME etc_test COMMAND etc_test)

# Optional: check the port against the original fs-4 code, e.g.
#   cmake --preset host -DFS4_DIR=$HOME/Projects/fs-4-ref
set(FS4_DIR "" CACHE PATH "fs-4-firmware checkout, enables the parity test")
if(FS4_DIR)
    if(NOT EXISTS ${FS4_DIR}/vcu/etc/etc_controller.cpp)
        message(FATAL_ERROR "No vcu/etc/etc_controller.cpp in FS4_DIR=${FS4_DIR}")
    endif()
    add_subdirectory(parity)
endif()

# VectorNav VN-200 driver tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/imu/CMakeLists.txt)
    add_subdirectory(imu)
endif()
