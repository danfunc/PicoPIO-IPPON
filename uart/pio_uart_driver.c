#include "pio_uart_driver.h"

#include "hardware/gpio.h"
#include "hardware/structs/pio.h"
#include "hardware/structs/dma.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

#include <string.h>

void pio_uart_driver_zero(pio_uart_driver_t *drv) {
    if (!drv) return;
    memset(drv, 0, sizeof(*drv));
    drv->dma_tx_chan = -1;
    drv->dma_rx_chan = -1;
    drv->offset_tx = 0xFFFFFFFFu;
    drv->offset_rx = 0xFFFFFFFFu;
}

void pio_uart_driver_deinit(pio_uart_driver_t *drv) {
    if (!drv || !drv->is_initialized) return;
    if (drv->dma_tx_chan >= 0) {
        dma_channel_abort(drv->dma_tx_chan);
        dma_channel_unclaim(drv->dma_tx_chan);
        drv->dma_tx_chan = -1;
    }
    if (drv->dma_rx_chan >= 0) {
        dma_channel_abort(drv->dma_rx_chan);
        dma_channel_unclaim(drv->dma_rx_chan);
        drv->dma_rx_chan = -1;
    }
    pio_sm_set_enabled(drv->pio_tx, drv->sm_tx, false);
    pio_sm_set_enabled(drv->pio_rx, drv->sm_rx, false);
    pio_sm_clear_fifos(drv->pio_tx, drv->sm_tx);
    pio_sm_clear_fifos(drv->pio_rx, drv->sm_rx);
    if (drv->offset_tx != 0xFFFFFFFFu) {
        pio_remove_program(drv->pio_tx, &uart_tx_program, drv->offset_tx);
        drv->offset_tx = 0xFFFFFFFFu;
    }
    if (drv->offset_rx != 0xFFFFFFFFu) {
        pio_remove_program(drv->pio_rx, &uart_rx_mini_program, drv->offset_rx);
        drv->offset_rx = 0xFFFFFFFFu;
    }
    drv->is_initialized = false;
}

bool pio_uart_driver_init(pio_uart_driver_t *drv, uint tx_pin, uint rx_pin, uint32_t baudrate) {
    if (!drv || baudrate == 0) return false;
    if (drv->is_initialized) pio_uart_driver_deinit(drv);
    pio_uart_driver_zero(drv);

    drv->pio_tx = pio0;
    drv->pio_rx = pio1;
    drv->sm_tx = 0;
    drv->sm_rx = 0;
    drv->tx_pin = tx_pin;
    drv->rx_pin = rx_pin;
    drv->baudrate = baudrate;
    ring_cursor_init(&drv->ring, PIO_UART_RX_RING_BYTES);
    poc_stats_reset(&drv->stats);

    int tx_offset = pio_add_program(drv->pio_tx, &uart_tx_program);
    if (tx_offset < 0) return false;
    drv->offset_tx = (uint)tx_offset;

    int rx_offset = pio_add_program(drv->pio_rx, &uart_rx_mini_program);
    if (rx_offset < 0) {
        pio_remove_program(drv->pio_tx, &uart_tx_program, drv->offset_tx);
        drv->offset_tx = 0xFFFFFFFFu;
        return false;
    }
    drv->offset_rx = (uint)rx_offset;

    drv->dma_tx_chan = dma_claim_unused_channel(false);
    drv->dma_rx_chan = dma_claim_unused_channel(false);
    if (drv->dma_tx_chan < 0 || drv->dma_rx_chan < 0) {
        if (drv->dma_tx_chan >= 0) dma_channel_unclaim(drv->dma_tx_chan);
        if (drv->dma_rx_chan >= 0) dma_channel_unclaim(drv->dma_rx_chan);
        drv->dma_tx_chan = drv->dma_rx_chan = -1;
        pio_remove_program(drv->pio_rx, &uart_rx_mini_program, drv->offset_rx);
        pio_remove_program(drv->pio_tx, &uart_tx_program, drv->offset_tx);
        drv->offset_rx = drv->offset_tx = 0xFFFFFFFFu;
        return false;
    }

    uart_tx_program_init(drv->pio_tx, drv->sm_tx, drv->offset_tx, tx_pin, baudrate);
    gpio_set_slew_rate(tx_pin, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(tx_pin, GPIO_DRIVE_STRENGTH_8MA);

    uart_rx_mini_program_init(drv->pio_rx, drv->sm_rx, drv->offset_rx, rx_pin, baudrate);
    gpio_set_input_hysteresis_enabled(rx_pin, true);
    pio_sm_set_enabled(drv->pio_rx, drv->sm_rx, false);

    dma_channel_config txc = dma_channel_get_default_config(drv->dma_tx_chan);
    channel_config_set_transfer_data_size(&txc, DMA_SIZE_8);
    channel_config_set_read_increment(&txc, true);
    channel_config_set_write_increment(&txc, false);
    channel_config_set_dreq(&txc, pio_get_dreq(drv->pio_tx, drv->sm_tx, true));
    dma_channel_configure(drv->dma_tx_chan, &txc, &drv->pio_tx->txf[drv->sm_tx], NULL, 0, false);

    dma_channel_config rxc = dma_channel_get_default_config(drv->dma_rx_chan);
    channel_config_set_transfer_data_size(&rxc, DMA_SIZE_8);
    channel_config_set_read_increment(&rxc, false);
    channel_config_set_write_increment(&rxc, true);
    channel_config_set_ring(&rxc, true, PIO_UART_RX_RING_BITS);
    channel_config_set_dreq(&rxc, pio_get_dreq(drv->pio_rx, drv->sm_rx, false));
    io_rw_8 *rx_fifo_high_byte = (io_rw_8 *)&drv->pio_rx->rxf[drv->sm_rx] + 3;
    dma_channel_configure(drv->dma_rx_chan, &rxc, drv->rx_ring_buf, rx_fifo_high_byte, 0xFFFFFFFFu, true);

    pio_sm_set_enabled(drv->pio_rx, drv->sm_rx, true);
    drv->is_initialized = true;
    return true;
}

bool pio_uart_tx_send_packet_blocking(pio_uart_driver_t *drv, const uint8_t *frame,
                                      size_t frame_bytes, uint8_t payload_len) {
    if (!drv || !drv->is_initialized || !frame || frame_bytes == 0) {
        if (drv) drv->stats.tx_failures++;
        return false;
    }

    dma_channel_wait_for_finish_blocking(drv->dma_tx_chan);
    absolute_time_t start = get_absolute_time();
    dma_channel_transfer_from_buffer_now(drv->dma_tx_chan, frame, frame_bytes);

    absolute_time_t timeout = make_timeout_time_ms(10);
    while (dma_channel_is_busy(drv->dma_tx_chan)) {
        if (time_reached(timeout)) {
            dma_channel_abort(drv->dma_tx_chan);
            drv->stats.tx_timeouts++;
            drv->stats.tx_failures++;
            return false;
        }
        tight_loop_contents();
    }

    /* The official TX program wraps to its blocking pull instruction after the final stop bit. */
    while (pio_sm_get_pc(drv->pio_tx, drv->sm_tx) != drv->offset_tx ||
           !pio_sm_is_tx_fifo_empty(drv->pio_tx, drv->sm_tx)) {
        if (time_reached(timeout)) {
            dma_channel_abort(drv->dma_tx_chan);
            drv->stats.tx_timeouts++;
            drv->stats.tx_failures++;
            return false;
        }
        tight_loop_contents();
    }

    absolute_time_t end = get_absolute_time();
    drv->stats.tx_phy_duration_us += (uint64_t)absolute_time_diff_us(start, end);
    drv->stats.tx_packets++;
    drv->stats.tx_bytes += payload_len;
    drv->stats.tx_wire_bytes += frame_bytes;
    return true;
}

bool pio_uart_rx_poll_packet(pio_uart_driver_t *drv, uint8_t *out_type, uint8_t *out_seq,
                             uint8_t *out_payload, uint8_t *out_len) {
    if (!drv || !drv->is_initialized) return false;

    uint32_t write_addr = dma_hw->ch[drv->dma_rx_chan].write_addr;
    size_t rx_head = (write_addr - (uintptr_t)drv->rx_ring_buf) & (PIO_UART_RX_RING_BYTES - 1);
    size_t avail = ring_cursor_update_head(&drv->ring, rx_head);
    drv->stats.rx_overrun_errors = drv->ring.overrun_count;

    if (avail < (BMC_HEADER_LEN + BMC_MIN_PAYLOAD_LEN + BMC_CRC_LEN)) return false;

    uint8_t len = 0;
    ring_cursor_peek(&drv->ring, drv->rx_ring_buf, &len, 1);
    if (len < BMC_MIN_PAYLOAD_LEN || len > BMC_MAX_PAYLOAD_LEN) {
        drv->stats.rx_len_errors++;
        ring_cursor_advance(&drv->ring, 1);
        return false;
    }

    size_t total_bytes = bmc_packet_total_bytes(len);
    if (avail < total_bytes) return false;

    uint8_t temp_frame[BMC_MAX_FRAME_BYTES];
    ring_cursor_peek(&drv->ring, drv->rx_ring_buf, temp_frame, total_bytes);
    uint8_t type = 0, seq = 0, plen = 0;
    int res = bmc_packet_decode(temp_frame, total_bytes, &type, &seq, out_payload, &plen);
    if (res > 0) {
        ring_cursor_advance(&drv->ring, total_bytes);
        if (drv->has_received_first) {
            uint8_t expected_seq = (drv->last_seq + 1) & 0xFF;
            if (seq != expected_seq) drv->stats.rx_seq_drops += (seq - expected_seq) & 0xFF;
        } else {
            drv->has_received_first = true;
        }
        drv->last_seq = seq;
        if (out_type) *out_type = type;
        if (out_seq) *out_seq = seq;
        if (out_len) *out_len = plen;
        drv->stats.rx_packets++;
        drv->stats.rx_bytes += plen;
        return true;
    }

    if (res == BMC_ERR_CRC_MISMATCH) drv->stats.rx_crc_errors++;
    ring_cursor_advance(&drv->ring, 1);
    return false;
}

void pio_uart_get_stats(const pio_uart_driver_t *drv, poc_stats_t *out_stats) {
    if (drv && out_stats) *out_stats = drv->stats;
}

void pio_uart_reset_stats(pio_uart_driver_t *drv) {
    if (!drv) return;
    poc_stats_reset(&drv->stats);
    drv->ring.overrun_count = 0;
}
