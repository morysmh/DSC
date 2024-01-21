cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(serverdsc C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
	src/main.cpp
	src/sensor.cpp
	src/mcp2515.c
	src/can.cpp
	src/i2c-display-lib.c
	)

pico_add_extra_outputs(${PROJECT_NAME})


pico_generate_pio_header(${PROJECT_NAME}
		${CMAKE_CURRENT_LIST_DIR}/pio/pio_rotary_encoder.pio)
pico_generate_pio_header(${PROJECT_NAME}
		${CMAKE_CURRENT_LIST_DIR}/pio/quadrature.pio)

target_include_directories(${PROJECT_NAME} PUBLIC inc)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
	hardware_uart
	hardware_adc
	hardware_spi
	hardware_pio
	hardware_clocks
	hardware_irq
	hardware_i2c
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

