cmake_minimum_required(VERSION 3.13)
project(poc_tests C)

set(CMAKE_C_STANDARD 11)

enable_testing()

add_executable(test_packet
    test_packet.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/packet.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/crc16.c
)

target_include_directories(test_packet PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../common
)

add_test(NAME test_packet COMMAND test_packet)

add_executable(test_ring
    test_ring.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/ring_cursor.c
)

target_include_directories(test_ring PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../common
)

add_test(NAME test_ring COMMAND test_ring)

add_executable(test_rx_extract
    test_rx_extract.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/packet.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/crc16.c
)

target_include_directories(test_rx_extract PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../common
)

add_test(NAME test_rx_extract COMMAND test_rx_extract)

add_executable(test_dma_counter
    test_dma_counter.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/packet.c
    ${CMAKE_CURRENT_LIST_DIR}/../common/crc16.c
)

target_include_directories(test_dma_counter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../common
)

add_test(NAME test_dma_counter COMMAND test_dma_counter)

add_executable(test_flash_bench
    test_flash_bench.c
    ${CMAKE_CURRENT_LIST_DIR}/../flash/flash_utils.c
)

target_include_directories(test_flash_bench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../flash
)

add_test(NAME test_flash_bench COMMAND test_flash_bench)

add_executable(test_ota_transfer
    test_ota_transfer.c
    ${CMAKE_CURRENT_LIST_DIR}/../ota/ota_protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/../ota/ota_receiver.c
    ${CMAKE_CURRENT_LIST_DIR}/../ota/ota_sender.c
)

target_include_directories(test_ota_transfer PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../common
    ${CMAKE_CURRENT_LIST_DIR}/../ota
    ${CMAKE_CURRENT_LIST_DIR}/../flash
)

add_test(NAME test_ota_transfer COMMAND test_ota_transfer)
