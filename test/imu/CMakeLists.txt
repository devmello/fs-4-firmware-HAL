# VectorNav VN-200 driver on the host, against packets built from the ICD
# layout and a fake sensor for the configuration commands.

add_executable(vn200_test
    vn200_test.cpp
    ${CMAKE_SOURCE_DIR}/vcu/imu/vn200.cpp
)
target_include_directories(vn200_test PRIVATE
    ${CMAKE_SOURCE_DIR}/vcu/imu
)
if(MSVC)
    target_compile_options(vn200_test PRIVATE /W4)
else()
    target_compile_options(vn200_test PRIVATE -Wall -Wextra)
endif()

add_test(NAME vn200_test COMMAND vn200_test)
