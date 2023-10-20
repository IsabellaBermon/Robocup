cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(Encoder)
# pull in common dependencies
pico_sdk_init()

add_executable(encoder
        main.c
        encoder/encoder.c
        encoder/encoder.h
)
pico_enable_stdio_uart(encoder 0)
pico_enable_stdio_usb(encoder 1)
target_link_libraries(encoder 
        pico_stdlib
        hardware_i2c
        hardware_pwm
        pico_time)
# create map/bin/hex file etc.
pico_add_extra_outputs(encoder)


