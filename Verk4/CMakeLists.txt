cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

#set(PICO_BOARD "adafruit_feather_rp2040") # Uncomment if using adafruit feather rp2040 board

project(Verk4 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(main main.cpp)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

pico_add_extra_outputs(main)

add_subdirectory(lib)

target_link_libraries(main pico_stdlib hardware_i2c pico_multicore pico_util_headers hardware_irq SCD30 BME280) # Insert libraries used in here