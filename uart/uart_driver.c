#include "uart_driver.h"
#include "hardware/gpio.h"
#include <string.h>

void uart_driver_zero(uart_driver_t *drv) {
    if (!drv) return;
    memset(drv, 0, sizeof(*drv));
    drv->dma_tx_chan = -1;
    drv->dma_rx_chan = -1;
    drv->is_initialized = false;
}

bool uart_driver_init(uart_driver_t *drv, uart_inst_t *uart_id, uint tx_pin, uint rx_pin, uint32_t baudrate) {
    if (!drv || !uart_id) return false;

    memset(drv->rx_ring_buf, 0, sizeof(drv->rx_ring_buf));
    drv->uart_id = uart_id;
    drv->tx_pin = tx_pin;
    drv->rx_pin = rx_pin;
    drv->baudrate = baudrate;
    drv->dma_tx_chan = -1;
    drv->dma_rx_chan = -1;
    drv->last_seq = 0;
    drv->has_received_first = false;
    poc_stats_reset(&drv->stats);
    ring_cursor_init(&drv->ring, UART_RX_RING_BYTES);

    if (drv->is_initialized) {
        uart_driver_deinit(drv);
    }

    // 1. DMA チャンネル確保 (panic=false で安全確保)
    drv->dma_tx_chan = dma_claim_unused_channel(false);
    drv->dma_rx_chan = dma_claim_unused_channel(false);
    if (drv->dma_tx_chan < 0 || drv->dma_rx_chan < 0) {
        if (drv->dma_tx_chan >= 0) { dma_channel_unclaim(drv->dma_tx_chan); drv->dma_tx_chan = -1; }
        if (drv->dma_rx_chan >= 0) { dma_channel_unclaim(drv->dma_rx_chan); drv->dma_rx_chan = -1; }
        return false;
    }

    // 2. UART ハードウェア初期化
    uart_init(uart_id, baudrate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin); // UART アイドルHigh保持のためプルアップ
    gpio_set_slew_rate(tx_pin, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(tx_pin, GPIO_DRIVE_STRENGTH_4MA);
    uart_set_hw_flow(uart_id, false, false);
    uart_set_format(uart_id, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_id, true);

    // 3. TX DMA 設定 (8bit転送, DREQ_UARTx_TX)
    dma_channel_config txc = dma_channel_get_default_config(drv->dma_tx_chan);
    channel_config_set_transfer_data_size(&txc, DMA_SIZE_8);
    channel_config_set_read_increment(&txc, true);
    channel_config_set_write_increment(&txc, false);
    channel_config_set_dreq(&txc, uart_get_dreq(uart_id, true));
    dma_channel_configure(drv->dma_tx_chan, &txc, &uart_get_hw(uart_id)->dr, NULL, 0, false);

    // 4. ピン切替・UART初期化時の過渡信号沈静化待ち (100us: 3Mbpsで約300ビット時間、ラインHigh安定に十分)
    sleep_us(100);

    // 5. 過渡信号等によりRX FIFOに入り込んだバイトをDMA起動前に空読みして破棄
    while (uart_is_readable(uart_id)) {
        (void)uart_getc(uart_id);
    }

    // 6. RX DMA 設定・起動 (1024B リングバッファ, 8bit転送, DREQ_UARTx_RX)
    dma_channel_config rxc = dma_channel_get_default_config(drv->dma_rx_chan);
    channel_config_set_transfer_data_size(&rxc, DMA_SIZE_8);
    channel_config_set_read_increment(&rxc, false);
    channel_config_set_write_increment(&rxc, true);
    channel_config_set_ring(&rxc, true, UART_RX_RING_BITS);
    channel_config_set_dreq(&rxc, uart_get_dreq(uart_id, false));
    dma_channel_configure(drv->dma_rx_chan, &rxc, drv->rx_ring_buf, &uart_get_hw(uart_id)->dr, 0xFFFFFFFF, true);

    drv->is_initialized = true;
    return true;
}

void uart_driver_deinit(uart_driver_t *drv) {
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
    if (drv->uart_id) {
        uart_deinit(drv->uart_id);
        drv->uart_id = NULL;
    }
    drv->is_initialized = false;
}

bool uart_driver_set_baudrate(uart_driver_t *drv, uint32_t baudrate) {
    if (!drv || !drv->uart_id) return false;
    drv->baudrate = baudrate;
    uart_set_baudrate(drv->uart_id, baudrate);
    return true;
}

bool uart_tx_send_packet_blocking(uart_driver_t *drv, const uint8_t *post_sync_frame, size_t frame_bytes, uint8_t payload_len) {
    if (!drv || !post_sync_frame || frame_bytes == 0 || drv->dma_tx_chan < 0) {
        if (drv) {
            drv->stats.tx_failures++;
        }
        return false;
    }

    // 2. 前回DMA転送の残存完了待ち (このパケット自身のPHY計測には含めない)
    dma_channel_wait_for_finish_blocking(drv->dma_tx_chan);

    // 1. 純粋なPHY送信開始時刻の計測 (H-1)
    absolute_time_t t_start = get_absolute_time();
    dma_channel_transfer_from_buffer_now(drv->dma_tx_chan, post_sync_frame, frame_bytes);

    // 最大 10ms の完了待機
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

    // 3. UART FIFO から全ビットが物理的に出終わるまで待機
    uart_tx_wait_blocking(drv->uart_id);

    // 4. 純粋なPHY送信完了時刻の計測 (H-1: エンコードやスリープは除外)
    absolute_time_t t_end = get_absolute_time();
    drv->stats.tx_phy_duration_us += (uint64_t)absolute_time_diff_us(t_start, t_end);

    drv->stats.tx_packets++;
    drv->stats.tx_bytes += payload_len;
    drv->stats.tx_wire_bytes += frame_bytes;

    return true;
}

// ============================================================================
// M-2: プロジェクト唯一の UART パケット受信ポーリング実装
// ============================================================================
bool uart_rx_poll_packet(uart_driver_t *drv, uint8_t *out_type, uint8_t *out_seq, uint8_t *out_payload, uint8_t *out_len) {
    if (!drv || drv->dma_rx_chan < 0) return false;

    // 1. DMA 書き込みヘッド位置を ring_cursor へ反映
    uint32_t write_addr = dma_hw->ch[drv->dma_rx_chan].write_addr;
    size_t rx_head = (write_addr - (uintptr_t)drv->rx_ring_buf) & (UART_RX_RING_BYTES - 1);
    size_t avail = ring_cursor_update_head(&drv->ring, rx_head);
    drv->stats.rx_overrun_errors = drv->ring.overrun_count;

    // 最小パケット長チェック (LEN + TYPE + SEQ + RESERVED + MIN_PAYLOAD + CRC = 8 バイト)
    if (avail < (BMC_HEADER_LEN + BMC_MIN_PAYLOAD_LEN + BMC_CRC_LEN)) {
        return false;
    }

    // 2. LEN バイトの検査
    uint8_t len = 0;
    ring_cursor_peek(&drv->ring, drv->rx_ring_buf, &len, 1);
    if (len < BMC_MIN_PAYLOAD_LEN || len > BMC_MAX_PAYLOAD_LEN) {
        drv->stats.rx_len_errors++;
        ring_cursor_advance(&drv->ring, 1);
        return false;
    }

    // 3. パケット全体の到着確認
    size_t total_bytes = bmc_packet_total_bytes(len);
    if (avail < total_bytes) {
        return false; // パケット全体のDMA転送完了待ち
    }

    // 4. パケットフレームを一時バッファへコピーしてデコード検証
    uint8_t temp_frame[BMC_MAX_FRAME_BYTES];
    ring_cursor_peek(&drv->ring, drv->rx_ring_buf, temp_frame, total_bytes);

    uint8_t type = 0, seq = 0, plen = 0;
    int res = bmc_packet_decode(temp_frame, total_bytes, &type, &seq, out_payload, &plen);

    if (res > 0) {
        // デコード成功！リングカーソルをパケット長分進める
        ring_cursor_advance(&drv->ring, total_bytes);

        if (drv->has_received_first) {
            uint8_t expected_seq = (drv->last_seq + 1) & 0xFF;
            if (seq != expected_seq) {
                uint8_t drop = (seq - expected_seq) & 0xFF;
                drv->stats.rx_seq_drops += drop;
            }
        } else {
            drv->has_received_first = true;
        }
        drv->last_seq = seq;

        if (out_type) *out_type = type;
        if (out_seq)  *out_seq  = seq;
        if (out_len)  *out_len  = plen;

        drv->stats.rx_packets++;
        drv->stats.rx_bytes += plen;
        return true;
    } else {
        // CRCエラーまたはデコードエラー: 1バイト進めて再同期
        if (res == BMC_ERR_CRC_MISMATCH) {
            drv->stats.rx_crc_errors++;
        }
        ring_cursor_advance(&drv->ring, 1);
        return false;
    }
}

void uart_get_stats(const uart_driver_t *drv, poc_stats_t *out_stats) {
    if (drv && out_stats) {
        *out_stats = drv->stats;
    }
}

void uart_reset_stats(uart_driver_t *drv) {
    if (drv) {
        poc_stats_reset(&drv->stats);
        drv->ring.overrun_count = 0;
    }
}
