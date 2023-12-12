# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")


# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(dribbler C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(dribbler main.cpp
                    )

pico_set_program_name(dribbler "dribbler")
pico_set_program_version(dribbler "0.1")

pico_enable_stdio_uart(dribbler 0)
pico_enable_stdio_usb(dribbler 1)

target_include_directories(dribbler PUBLIC inc/)

# Add the standard library to the build
target_link_libraries(dribbler pico_stdlib hardware_i2c hardware_gpio hardware_pwm)

pico_add_extra_outputs(dribbler)