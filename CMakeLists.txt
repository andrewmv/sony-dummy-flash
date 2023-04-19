cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(sony_dummy_flash C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_executable(sony_dummy_flash
	dummy_flash.c
)

# Generate PIO headers in build dir
pico_generate_pio_header(sony_dummy_flash ${CMAKE_CURRENT_LIST_DIR}/tx-miso.pio)
pico_generate_pio_header(sony_dummy_flash ${CMAKE_CURRENT_LIST_DIR}/rx-mosi.pio)

# Enable serial console over USB
pico_enable_stdio_usb(sony_dummy_flash 1)
pico_enable_stdio_uart(sony_dummy_flash 0)

pico_add_extra_outputs(sony_dummy_flash)

target_sources(sony_dummy_flash PRIVATE dummy_flash.c)

target_link_libraries(sony_dummy_flash PRIVATE
        pico_stdlib
        hardware_pio
        hardware_dma
        )
