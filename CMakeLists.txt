cmake_minimum_required(VERSION 3.13)

# Pico SDK Import
include(pico_sdk_import.cmake)

project(poc_protocol C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# -------------------------------------------------------------
# C-7: Build Identity Header Generation
# -------------------------------------------------------------
set(BUILD_INFO_GEN_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/build_info_gen.h")

file(GLOB_RECURSE POC_ALL_C "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
file(GLOB_RECURSE POC_ALL_H "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB_RECURSE POC_ALL_PIO "${CMAKE_CURRENT_SOURCE_DIR}/*.pio")
file(GLOB_RECURSE POC_ALL_CMAKE "${CMAKE_CURRENT_SOURCE_DIR}/*CMakeLists.txt" "${CMAKE_CURRENT_SOURCE_DIR}/*.cmake")

set(POC_RAW_SOURCES ${POC_ALL_C} ${POC_ALL_H} ${POC_ALL_PIO} ${POC_ALL_CMAKE})
set(POC_MAJOR_SOURCES "")
foreach(F ${POC_RAW_SOURCES})
    if(NOT F MATCHES "/(build|build[^/]*)/" AND NOT F MATCHES "/generated/" AND NOT F MATCHES "/\\.")
        list(APPEND POC_MAJOR_SOURCES "${F}")
    endif()
endforeach()
list(REMOVE_DUPLICATES POC_MAJOR_SOURCES)
list(SORT POC_MAJOR_SOURCES)

# Generate at configure time to ensure generated header is immediately available
execute_process(
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DOUTPUT_FILE=${BUILD_INFO_GEN_HEADER}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBuildInfo.cmake
)

# Custom command to regenerate build_info_gen.h when any major source file changes
add_custom_command(
    OUTPUT ${BUILD_INFO_GEN_HEADER}
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DOUTPUT_FILE=${BUILD_INFO_GEN_HEADER}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBuildInfo.cmake
    DEPENDS ${POC_MAJOR_SOURCES} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBuildInfo.cmake"
    COMMENT "Generating build_info_gen.h"
    VERBATIM
)

add_custom_target(generate_build_info DEPENDS ${BUILD_INFO_GEN_HEADER})

# -------------------------------------------------------------
# Common Library (CRC16, Packet, Ring Cursor, Benchmark, Build Info)
# H-9: poc_common links pico_stdlib PRIVATE
# -------------------------------------------------------------
add_library(poc_common STATIC
    ${CMAKE_CURRENT_LIST_DIR}/common/crc16.c
    ${CMAKE_CURRENT_LIST_DIR}/common/packet.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ring_cursor.c
    ${CMAKE_CURRENT_LIST_DIR}/common/benchmark_common.c
    ${CMAKE_CURRENT_LIST_DIR}/common/build_info.c
)

add_dependencies(poc_common generate_build_info)

target_include_directories(poc_common PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/common
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_link_libraries(poc_common PRIVATE
    pico_stdlib
)

# -------------------------------------------------------------
# Host Tests (Runnable via CTest on Host Platforms) (M-8)
# -------------------------------------------------------------
if(NOT PICO_ON_DEVICE)
    enable_testing()
    add_subdirectory(tests)
endif()

# -------------------------------------------------------------
# Target 1: BMC Loopback Benchmark
# -------------------------------------------------------------
add_executable(poc_bmc_loopback
    ${CMAKE_CURRENT_LIST_DIR}/bmc/main_bmc_loopback.c
    ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_driver.c
)

pico_generate_pio_header(poc_bmc_loopback ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p.pio)
pico_generate_pio_header(poc_bmc_loopback ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_fast.pio)
pico_generate_pio_header(poc_bmc_loopback ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_mid12.pio)
pico_generate_pio_header(poc_bmc_loopback ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_mid10.pio)

target_include_directories(poc_bmc_loopback PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/bmc
    ${CMAKE_CURRENT_LIST_DIR}/common
)

target_link_libraries(poc_bmc_loopback PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_gpio
    poc_common
)

pico_enable_stdio_usb(poc_bmc_loopback 1)
pico_enable_stdio_uart(poc_bmc_loopback 0)

pico_add_extra_outputs(poc_bmc_loopback)

# -------------------------------------------------------------
# Target 2: UART Loopback Benchmark
# -------------------------------------------------------------
add_executable(poc_uart_loopback
    ${CMAKE_CURRENT_LIST_DIR}/uart/main_uart_loopback.c
    ${CMAKE_CURRENT_LIST_DIR}/uart/uart_driver.c
)

target_include_directories(poc_uart_loopback PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/uart
    ${CMAKE_CURRENT_LIST_DIR}/common
)

target_link_libraries(poc_uart_loopback PRIVATE
    pico_stdlib
    hardware_uart
    hardware_dma
    hardware_gpio
    poc_common
)

pico_enable_stdio_usb(poc_uart_loopback 1)
pico_enable_stdio_uart(poc_uart_loopback 0)

pico_add_extra_outputs(poc_uart_loopback)

# -------------------------------------------------------------
# Target 3: Unified Benchmark (Runtime Switchable BMC vs UART)
# -------------------------------------------------------------
add_executable(poc_benchmark
    ${CMAKE_CURRENT_LIST_DIR}/main_benchmark.c
    ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/uart/uart_driver.c
)

pico_generate_pio_header(poc_benchmark ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p.pio)
pico_generate_pio_header(poc_benchmark ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_fast.pio)
pico_generate_pio_header(poc_benchmark ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_mid12.pio)
pico_generate_pio_header(poc_benchmark ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_mid10.pio)

target_include_directories(poc_benchmark PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/bmc
    ${CMAKE_CURRENT_LIST_DIR}/uart
    ${CMAKE_CURRENT_LIST_DIR}/common
)

target_link_libraries(poc_benchmark PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_uart
    hardware_dma
    hardware_clocks
    hardware_gpio
    poc_common
)

pico_enable_stdio_usb(poc_benchmark 1)
pico_enable_stdio_uart(poc_benchmark 0)

pico_add_extra_outputs(poc_benchmark)

# -------------------------------------------------------------
# Target 4: Standalone Flash Write/Erase Benchmark (B-4)
# -------------------------------------------------------------
add_executable(poc_flash_bench
    ${CMAKE_CURRENT_LIST_DIR}/flash/main_flash_bench.c
    ${CMAKE_CURRENT_LIST_DIR}/flash/flash_bench.c
    ${CMAKE_CURRENT_LIST_DIR}/flash/flash_utils.c
)

target_include_directories(poc_flash_bench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/flash
    ${CMAKE_CURRENT_LIST_DIR}/common
)

target_link_libraries(poc_flash_bench PRIVATE
    pico_stdlib
    pico_flash
    hardware_flash
    hardware_sync
    poc_common
)

# Declare core1 safe for flash execution since core1 is never launched
target_compile_definitions(poc_flash_bench PRIVATE
    PICO_FLASH_ASSUME_CORE1_SAFE=1
)

pico_enable_stdio_usb(poc_flash_bench 1)
pico_enable_stdio_uart(poc_flash_bench 0)

pico_add_extra_outputs(poc_flash_bench)

# Post-build size verification: Ensure binary strictly fits inside 1MB firmware slot
add_custom_command(TARGET poc_flash_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DBINARY_FILE=${CMAKE_CURRENT_BINARY_DIR}/poc_flash_bench.bin
        -DMAX_BYTES=1048576
        -DTARGET_NAME=poc_flash_bench
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckBinarySize.cmake
    COMMENT "Verifying poc_flash_bench binary size against 1MB limit"
    VERBATIM
)

# -------------------------------------------------------------
# Target 5: BMC OTA Firmware Transfer Benchmark
# -------------------------------------------------------------
add_executable(poc_ota_bench
    ${CMAKE_CURRENT_LIST_DIR}/ota/main_ota_bench.c
    ${CMAKE_CURRENT_LIST_DIR}/ota/ota_protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/ota/ota_receiver.c
    ${CMAKE_CURRENT_LIST_DIR}/ota/ota_sender.c
    ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/flash/flash_bench.c
    ${CMAKE_CURRENT_LIST_DIR}/flash/flash_utils.c
)

pico_generate_pio_header(poc_ota_bench ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p.pio)
pico_generate_pio_header(poc_ota_bench ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_fast.pio)
pico_generate_pio_header(poc_ota_bench ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_mid12.pio)
pico_generate_pio_header(poc_ota_bench ${CMAKE_CURRENT_LIST_DIR}/bmc/bmc_p2p_mid10.pio)

target_include_directories(poc_ota_bench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/ota
    ${CMAKE_CURRENT_LIST_DIR}/bmc
    ${CMAKE_CURRENT_LIST_DIR}/flash
    ${CMAKE_CURRENT_LIST_DIR}/common
)

target_link_libraries(poc_ota_bench PRIVATE
    pico_stdlib
    pico_multicore
    pico_flash
    hardware_pio
    hardware_dma
    hardware_flash
    hardware_sync
    hardware_clocks
    hardware_gpio
    poc_common
)

pico_enable_stdio_usb(poc_ota_bench 1)
pico_enable_stdio_uart(poc_ota_bench 0)

pico_add_extra_outputs(poc_ota_bench)

# Post-build size verification: Ensure binary strictly fits inside 1MB firmware slot
add_custom_command(TARGET poc_ota_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DBINARY_FILE=${CMAKE_CURRENT_BINARY_DIR}/poc_ota_bench.bin
        -DMAX_BYTES=1048576
        -DTARGET_NAME=poc_ota_bench
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckBinarySize.cmake
    COMMENT "Verifying poc_ota_bench binary size against 1MB limit"
    VERBATIM
)

