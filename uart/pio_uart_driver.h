#ifndef PIO_UART_DRIVER_H
#define PIO_UART_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "common/packet.h"
#include "common/ring_cursor.h"
#include "common/benchmark_common.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PIO_UART_RX_RING_BYTES 1024u
#define PIO_UART_RX_RING_BITS 10u

typedef struct __attribute__((aligned(PIO_UART_RX_RING_BYTES))) {
    uint8_t rx_ring_buf[PIO_UART_RX_RING_BYTES] __attribute__((aligned(PIO_UART_RX_RING_BYTES)));
    PIO pio_tx;
    PIO pio_rx;
    uint sm_tx;
    uint sm_rx;
    uint offset_tx;
    uint offset_rx;
    uint tx_pin;
    uint rx_pin;
    uint32_t baudrate;
    int dma_tx_chan;
    int dma_rx_chan;
    ring_cursor_t ring;
    uint8_t last_seq;
    bool has_received_first;
    bool is_initialized;
    poc_stats_t stats;
} pio_uart_driver_t;

void pio_uart_driver_zero(pio_uart_driver_t *drv);
bool pio_uart_driver_init(pio_uart_driver_t *drv, uint tx_pin, uint rx_pin, uint32_t baudrate);
void pio_uart_driver_deinit(pio_uart_driver_t *drv);
bool pio_uart_tx_send_packet_blocking(pio_uart_driver_t *drv, const uint8_t *frame, size_t frame_bytes, uint8_t payload_len);
bool pio_uart_rx_poll_packet(pio_uart_driver_t *drv, uint8_t *out_type, uint8_t *out_seq, uint8_t *out_payload, uint8_t *out_len);
void pio_uart_get_stats(const pio_uart_driver_t *drv, poc_stats_t *out_stats);
void pio_uart_reset_stats(pio_uart_driver_t *drv);

#endif
