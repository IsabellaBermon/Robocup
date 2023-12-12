# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK for the RP2040 microcontroller
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(pathfinder C CXX) 
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.0/pico-sdk")

# Initialize the Pico SDK
pico_sdk_init()

# Set the name of your source code file
add_executable(${PROJECT_NAME}
    src/main.c
    src/pathfinder.c
)

# Link the necessary libraries to the executable
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib

)
target_include_directories(${PROJECT_NAME} PUBLIC inc/)

# Enable USB support if needed
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create the UF2 file needed to load on the Raspberry Pi Pico
pico_add_extra_outputs(${PROJECT_NAME})

