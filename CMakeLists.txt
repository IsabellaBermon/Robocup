
cmake_minimum_required(VERSION 3.19)
include(pico_sdk_import.cmake)
set(PICO_BOARD "pico_w")
set(PROGRAM_NAME car)


project(car)

pico_sdk_init()
add_executable(${PROGRAM_NAME}
    blink.c
	)
target_link_libraries(${PROGRAM_NAME}
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_clocks
    hardware_pwm
    )
pico_enable_stdio_usb(car 1)
#pico_enable_stdio_uart(blink 1)
target_include_directories(${PROGRAM_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    )
pico_btstack_make_gatt_header(${PROGRAM_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/mygatt.gatt")
pico_add_extra_outputs(${PROGRAM_NAME})
