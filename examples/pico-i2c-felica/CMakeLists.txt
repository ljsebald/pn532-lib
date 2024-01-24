cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico)
set(PICO_PLATFORM rp2040)

include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(pico-i2c-felica C CXX ASM)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_executable(pico-i2c-felica)

target_sources(pico-i2c-felica PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/pico-i2c-felica.c
	${CMAKE_CURRENT_LIST_DIR}/../../pn532.c
	${CMAKE_CURRENT_LIST_DIR}/../../pn532_pico.c
)

target_include_directories(pico-i2c-felica PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../..)

target_link_libraries(pico-i2c-felica
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(pico-i2c-felica 1)
pico_enable_stdio_uart(pico-i2c-felica 0)

pico_add_extra_outputs(pico-i2c-felica)

