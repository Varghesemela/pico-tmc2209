cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "/home/$ENV{USER}/pico/pico-sdk")

include(pico_sdk_import.cmake)

project(pico-tmc2209 C CXX ASM)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )


include_directories(${CMAKE_BINARY_DIR}/generated)
include_directories(${CMAKE_SOURCE_DIR}/inc)

add_subdirectory(lib)
add_subdirectory(src)
