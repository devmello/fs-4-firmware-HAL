cmake_minimum_required(VERSION 3.22)

project(fs-hal LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if(CMAKE_CROSSCOMPILING)
    include(stm32f4)
    add_subdirectory(vcu)
else()
    # Host build: unit tests for the hardware independent code
    enable_testing()
    add_subdirectory(test)
endif()
