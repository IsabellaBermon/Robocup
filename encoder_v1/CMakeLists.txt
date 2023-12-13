# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD "pico_w")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")
set(FREERTOS_KERNEL_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(encoder_v1 C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# Agregar esta línea para compilar archivos .cpp como código C++


add_executable(encoder_v1 
    src/main.c
    src/encoder.c
    src/motor_config.c
    src/control_functions.c
    src/robot_movement.c
    src/bt_functions.c
    src/MPU6050_i2c.c
    src/dribbler.c
                        )

pico_set_program_name(encoder_v1 "encoder_v1")
pico_set_program_version(encoder_v1 "0.1")

pico_enable_stdio_uart(encoder_v1 0)
pico_enable_stdio_usb(encoder_v1 1)

target_include_directories(encoder_v1 PUBLIC inc/)

# Add the standard library to the build
target_link_libraries(encoder_v1 
    pico_stdlib 
    hardware_i2c 
    hardware_gpio 
    hardware_pwm
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    pico_multicore
    )
    pico_btstack_make_gatt_header(encoder_v1 PRIVATE "${CMAKE_CURRENT_LIST_DIR}/mygatt.gatt")

pico_add_extra_outputs(encoder_v1)