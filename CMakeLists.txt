cmake_minimum_required(VERSION 3.13)

# BW made the Import directory and dropped the file there
include(/home/wilford/pico/pico-sdk/Import/pico_sdk_import.cmake)

project(pl_tones C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pl_tones
        pl_tones.c
        )

pico_generate_pio_header(pl_tones ${CMAKE_CURRENT_LIST_DIR}/out7seg.pio)

pico_enable_stdio_usb(pl_tones 1)
pico_enable_stdio_uart(pl_tones 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pl_tones)

target_link_libraries(pl_tones pico_stdlib hardware_adc hardware_pio hardware_dma)

