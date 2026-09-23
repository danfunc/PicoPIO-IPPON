#include "bmc_driver.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

// ============================================================================
// TX ピン初期化
// ============================================================================
void bmc_tx_pin_init(uint pin) {
    // 1. まずSIOで能動的にHigh (3.3V) を出力固定
    gpio_put(pin, 1);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_input_enabled(pin, true); // mov pins, !pins による自ピン読み戻しを許可
}

// ============================================================================
// TX コントローラ内部ヘルパー: 常駐SMのピンおよびモード確立
// ============================================================================
static void bmc_tx_ensure_sm_resident(bmc_tx_controller_t *tx, uint tx_pin) {
    if (tx->current_tx_pin == tx_pin) {
        return; // 既に当該ピンで常駐稼働中
    }

    PIO pio = tx->pio;
    uint sm = tx->sm_tx;
    float clkdiv = (tx->clkdiv > 0.0f) ? tx->clkdiv : bmc_get_clkdiv();

    // 1. SM停止 & DMAアボート
    pio_sm_set_enabled(pio, sm, false);
    if (tx->dma_tx_chan >= 0) {
        dma_channel_abort(tx->dma_tx_chan);
    }

    // 2. PIOプログラム初期化 (SIO High保持 -> PIOラッチHigh/方向OUT準備 -> Mux PIO)
    switch (tx->rate_family) {
        case BMC_RATE_MID_12CYC:
            bmc_mid12_tx_program_init(pio, sm, tx->offset_tx, tx_pin, clkdiv);
            break;
        case BMC_RATE_MID_10CYC:
            bmc_mid10_tx_program_init(pio, sm, tx->offset_tx, tx_pin, clkdiv);
            break;
        case BMC_RATE_FAST_8CYC:
            bmc_fast_tx_program_init(pio, sm, tx->offset_tx, tx_pin, clkdiv);
            break;
        case BMC_RATE_STD_16CYC:
        default:
            bmc_tx_program_init(pio, sm, tx->offset_tx, tx_pin, clkdiv);
            break;
    }

    // B-1: パッド設定バリアントの適用
    // 制約: この変種は rate_family == BMC_RATE_FAST_8CYC のときのみ適用可能とし、
    //       BMC_RATE_STD_16CYC には絶対に適用しない
    if (tx->pad_fast_variant && tx->rate_family == BMC_RATE_FAST_8CYC) {
        gpio_set_slew_rate(tx_pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(tx_pin, GPIO_DRIVE_STRENGTH_8MA);
    }

    // 3. FIFOクリア & 初期PC設定 (idle: set pins, 1; pull block)
    pio_sm_clear_fifos(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));
    pio_sm_exec(pio, sm, pio_encode_jmp(tx->offset_tx));

    // 4. TX SM を常駐有効化 (pull block で待機)
    pio_sm_set_enabled(pio, sm, true);
    tx->current_tx_pin = tx_pin;
}

// ============================================================================
// TX コントローラ初期化
// ============================================================================
void bmc_tx_controller_zero(bmc_tx_controller_t *tx) {
    if (!tx) return;
    memset(tx, 0, sizeof(*tx));
    tx->dma_tx_chan = -1;
    tx->current_tx_pin = 0xFFFFFFFF;
    tx->rate_family = BMC_RATE_STD_16CYC;
    tx->pad_fast_variant = false;
    tx->pending_active = false;
    tx->is_initialized = false;
}

void bmc_tx_controller_deinit(bmc_tx_controller_t *tx) {
    if (!tx || !tx->is_initialized) return;
    if (tx->dma_tx_chan >= 0) {
        dma_channel_abort(tx->dma_tx_chan);
        dma_channel_unclaim(tx->dma_tx_chan);
        tx->dma_tx_chan = -1;
    }
    if (tx->pio) {
        pio_sm_set_enabled(tx->pio, tx->sm_tx, false);
    }
    tx->current_tx_pin = 0xFFFFFFFF;
    tx->is_initialized = false;
}

bool bmc_tx_controller_init(bmc_tx_controller_t *tx, PIO pio, uint sm) {
    if (!tx) return false;
    if (tx->is_initialized) {
        bmc_tx_controller_deinit(tx);
    }

    bmc_tx_controller_zero(tx);
    tx->pio = pio;
    tx->sm_tx = sm;
    // NIPPON: TXプログラムは "irq set 0 rel" で送出完了を通知する (RP2350データシート 3.4.7.3:
    // rel 指定時の実効IRQ番号 = (命令中のIRQ番号 + (SM番号 & 3)) & 7)。命令中のIRQ番号は常に0の
    // ため、実効フラグ番号は sm & 3 に一致する。同一PIOに複数のTX SMを常駐させても
    // (bmc_p2p*.pio 全4種で共通) フラグが衝突しない。
    tx->irq_num = sm & 3u;
    tx->offset_tx = 0;
    tx->dma_tx_chan = -1;
    tx->current_tx_pin = 0xFFFFFFFF;
    tx->clkdiv = bmc_get_clkdiv();
    tx->rate_family = BMC_RATE_STD_16CYC;
    tx->pad_fast_variant = false;
    poc_stats_reset(&tx->stats);

    // PIOプログラムの空き領域確認・登録
    if (!pio_can_add_program(pio, &bmc_tx_program)) {
        return false;
    }
    tx->offset_tx = pio_add_program(pio, &bmc_tx_program);

    // TX DMA設定 (32bit転送, DREQ_PIOx_TX, Byte-swap有効, panic=false)
    tx->dma_tx_chan = dma_claim_unused_channel(false);
    if (tx->dma_tx_chan < 0) {
        pio_remove_program(pio, &bmc_tx_program, tx->offset_tx);
        return false;
    }

    dma_channel_config c = dma_channel_get_default_config(tx->dma_tx_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_bswap(&c, true); // エンディアン整合
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    dma_channel_configure(tx->dma_tx_chan, &c, &pio->txf[sm], NULL, 0, false);

    tx->is_initialized = true;
    return true;
}

void bmc_tx_controller_set_mode(bmc_tx_controller_t *tx, uint offset_tx, float clkdiv, bmc_rate_family_t rate_family) {
    if (!tx) return;
    tx->offset_tx = offset_tx;
    tx->clkdiv = (clkdiv > 0.0f) ? clkdiv : bmc_get_clkdiv();
    tx->rate_family = rate_family;

    // 既にピンが割当済みの場合は常駐SMを再初期化
    if (tx->current_tx_pin != 0xFFFFFFFF) {
        uint pin = tx->current_tx_pin;
        tx->current_tx_pin = 0xFFFFFFFF; // 強制再構成
        bmc_tx_ensure_sm_resident(tx, pin);
    }
}

void bmc_tx_controller_set_pad_variant(bmc_tx_controller_t *tx, bool pad_fast_variant) {
    if (!tx) return;
    tx->pad_fast_variant = pad_fast_variant;

    // 既にピンが割当済みの場合は常駐SMを再初期化
    if (tx->current_tx_pin != 0xFFFFFFFF) {
        uint pin = tx->current_tx_pin;
        tx->current_tx_pin = 0xFFFFFFFF; // 強制再構成
        bmc_tx_ensure_sm_resident(tx, pin);
    }
}

void bmc_tx_controller_prime(bmc_tx_controller_t *tx, uint tx_pin) {
    if (!tx) return;
    tx->current_tx_pin = 0xFFFFFFFF; // 強制再初期化 (常に最新の offset_tx/clkdiv/rate_family で確立)
    bmc_tx_ensure_sm_resident(tx, tx_pin);
}

// ============================================================================
// TX パケット送信 (常駐SM・IRQ0検知)
// ============================================================================
bool bmc_tx_start_packet_async(bmc_tx_controller_t *tx, uint tx_pin,
                               const uint32_t *dma_buf, size_t total_dma_words) {
    if (!tx || !dma_buf || total_dma_words < 2) {
        if (tx) tx->stats.tx_failures++;
        return false;
    }

    bmc_tx_ensure_sm_resident(tx, tx_pin);

    pio_interrupt_clear(tx->pio, tx->irq_num);
    tx->pending_t_start = get_absolute_time();
    dma_channel_transfer_from_buffer_now(tx->dma_tx_chan, dma_buf, total_dma_words);

    tx->pending_dma_buf = dma_buf;
    tx->pending_total_dma_words = total_dma_words;
    tx->pending_active = true;
    return true;
}

bool bmc_tx_wait_packet_done(bmc_tx_controller_t *tx) {
    if (!tx || !tx->pending_active) return true;

    absolute_time_t timeout = make_timeout_time_ms(10);
    bool success = true;
    while (!pio_interrupt_get(tx->pio, tx->irq_num)) {
        if (time_reached(timeout)) {
            success = false;
            break;
        }
        tight_loop_contents();
    }
    pio_interrupt_clear(tx->pio, tx->irq_num);
    tx->pending_active = false;

    if (success) {
        absolute_time_t t_end = get_absolute_time();
        tx->stats.tx_phy_duration_us += (uint64_t)absolute_time_diff_us(tx->pending_t_start, t_end);

        tx->stats.tx_packets++;
        const uint8_t *wire_bytes = (const uint8_t *)&tx->pending_dma_buf[1];
        tx->stats.tx_bytes += wire_bytes[BMC_PREAMBLE_SYNC_BYTES];
        tx->stats.tx_wire_bytes += (tx->pending_total_dma_words - 1) * 4;
        return true;
    } else {
        dma_channel_abort(tx->dma_tx_chan);
        tx->stats.tx_timeouts++;
        tx->stats.tx_failures++;

        pio_sm_exec(tx->pio, tx->sm_tx, pio_encode_set(pio_pins, 1));
        pio_sm_clear_fifos(tx->pio, tx->sm_tx);
        pio_sm_exec(tx->pio, tx->sm_tx, pio_encode_jmp(tx->offset_tx));
        return false;
    }
}

bool bmc_tx_send_packet_blocking(bmc_tx_controller_t *tx, uint tx_pin,
                                 const uint32_t *dma_buf, size_t total_dma_words) {
    if (!bmc_tx_start_packet_async(tx, tx_pin, dma_buf, total_dma_words)) {
        return false;
    }
    return bmc_tx_wait_packet_done(tx);
}

// ============================================================================
// RX ポート初期化
// ============================================================================
void bmc_rx_port_zero(bmc_rx_port_t *rx) {
    if (!rx) return;
    memset(rx, 0, sizeof(*rx));
    rx->dma_rx_chan = -1;
    rx->rate_family = BMC_RATE_STD_16CYC;
    rx->last_transfer_count = BMC_RX_DMA_RELOAD_WORDS;
    rx->is_initialized = false;
}

void bmc_rx_port_deinit(bmc_rx_port_t *rx) {
    if (!rx || !rx->is_initialized) return;
    if (rx->dma_rx_chan >= 0) {
        dma_channel_abort(rx->dma_rx_chan);
        dma_channel_unclaim(rx->dma_rx_chan);
        rx->dma_rx_chan = -1;
    }
    if (rx->pio) {
        pio_sm_set_enabled(rx->pio, rx->sm_rx, false);
    }
    rx->is_initialized = false;
}

bool bmc_rx_port_init(bmc_rx_port_t *rx, PIO pio, uint sm, uint offset_rx, uint pin) {
    return bmc_rx_port_init_ex(rx, pio, sm, offset_rx, pin, bmc_get_clkdiv(), BMC_RATE_STD_16CYC);
}

bool bmc_rx_port_init_ex(bmc_rx_port_t *rx, PIO pio, uint sm, uint offset_rx,
                         uint pin, float clkdiv, bmc_rate_family_t rate_family) {
    if (!rx) return false;
    if (rx->is_initialized) {
        bmc_rx_port_deinit(rx);
    }

    bmc_rx_port_zero(rx);

    // 1. DMAチャネル確保 (panic=false で安全確保)
    rx->dma_rx_chan = dma_claim_unused_channel(false);
    if (rx->dma_rx_chan < 0) return false;

    // 2. 全メンバーの明示的初期化
    rx->pio = pio;
    rx->sm_rx = sm;
    rx->offset_rx = offset_rx;
    rx->rx_pin = pin;
    rx->clkdiv = (clkdiv > 0.0f) ? clkdiv : bmc_get_clkdiv();
    rx->rate_family = rate_family;
    rx->tail_bit = 0;
    rx->last_head_byte = 0;
    rx->lap_count = 0;
    rx->last_transfer_count = BMC_RX_DMA_RELOAD_WORDS;
    rx->total_words_written = 0;
    rx->total_bytes_written = 0;
    rx->total_bits_read = 0;
    rx->last_seq = 0;
    rx->has_received_first = false;
    poc_stats_reset(&rx->stats);

    // 3. SM停止
    pio_sm_set_enabled(pio, sm, false);

    // 4. ピン初期化 (プルダウン + 入力シュミットトリガ)
    switch (rate_family) {
        case BMC_RATE_MID_12CYC:
            bmc_mid12_rx_program_init(pio, sm, offset_rx, pin, rx->clkdiv);
            break;
        case BMC_RATE_MID_10CYC:
            bmc_mid10_rx_program_init(pio, sm, offset_rx, pin, rx->clkdiv);
            break;
        case BMC_RATE_FAST_8CYC:
            bmc_fast_rx_program_init(pio, sm, offset_rx, pin, rx->clkdiv);
            break;
        case BMC_RATE_STD_16CYC:
        default:
            bmc_rx_program_init(pio, sm, offset_rx, pin, rx->clkdiv);
            break;
    }

    // 5. FIFO、ISR、入力カウント、PC、Xレジスタの厳密な初期化
    pio_sm_clear_fifos(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_null));
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_x, pio_null));
    pio_sm_exec(pio, sm, pio_encode_jmp(offset_rx));

    // 6. RX DMA設定 (16384バイト循環リングバッファ, 32bit転送, Byte-swap有効, TRIGGER_SELF カウント)
    dma_channel_config dc = dma_channel_get_default_config(rx->dma_rx_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_bswap(&dc, true);
    channel_config_set_ring(&dc, true, BMC_RX_RING_BITS); // 16384バイト境界 (14ビット)
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, false));

    // 7. 必ず RX DMA を先行起動し、その後に RX SM を有効化する
    // RP2350: TRIGGER_SELF モード + 28bit RELOAD COUNT (0x0FFFFFFF ≈ 268M transfers ≈ 1.07GB)
    uint32_t encoded_tc = dma_encode_transfer_count_with_self_trigger(BMC_RX_DMA_RELOAD_WORDS);
    dma_channel_configure(rx->dma_rx_chan, &dc, rx->rx_ring_buf, &pio->rxf[sm], encoded_tc, true);
    pio_sm_set_enabled(pio, sm, true);

    rx->is_initialized = true;
    return true;
}

bool BMC_SRAM_FUNC(bmc_rx_poll_packet)(bmc_rx_port_t *rx,
                        uint8_t *out_type,
                        uint8_t *out_seq,
                        uint8_t *out_payload,
                        uint8_t *out_len) {
    if (!rx) return false;

    // 1. transfer_count レジスタ (COUNT部) から真の累計転送語数・バイト数を算出 (エイリアシング解消)。
    //    この1回のレジスタ読み出しのみを行い、write_addr の別読みは行わない
    //    (旧実装は write_addr と transfer_count を別々に読んでおり、2つの読み出しの間に新規
    //    DMA転送が挟まるとヘッド位置と累計転送語数が僅かに食い違う観測不整合が起こり得た。
    //    オーバーラン復旧時の safe_tail_byte もこのヘッド位置を基準にするため、同一の読み出し
    //    から一貫して導出する)。
    uint32_t raw_tc = dma_hw->ch[rx->dma_rx_chan].transfer_count;
    uint32_t curr_tc = raw_tc & DMA_CH0_TRANS_COUNT_COUNT_BITS;
    uint32_t prev_tc = rx->last_transfer_count;

    // B-X: 直前の抽出試行が「現在届いているデータでは判定不能/シンク未検出」で終わっており
    // (last_extract_exhausted == true)、かつ前回ポーリング以降に新規DMA転送が1語も発生して
    // いなければ (curr_tc == prev_tc)、再度 bmc_rx_extract_frame_from_ring() を呼んでも入力
    // データが一切変化していないため結果は必ず同じになる。この場合のみ抽出処理を省略する。
    // 逆に直前がフレーム受理/CRCエラー/LENエラーで終わった場合は、新規DMA転送が無くても
    // リング内に未抽出の複数フレームが残っている可能性があるため、必ず抽出を試みる。
    if (rx->last_extract_exhausted && curr_tc == prev_tc) {
        return false;
    }

    rx->total_words_written = bmc_rx_update_transfer_state(
        &rx->lap_count, &rx->last_transfer_count, curr_tc, BMC_RX_DMA_RELOAD_WORDS);
    rx->total_bytes_written = rx->total_words_written * sizeof(uint32_t);

    // 2. DMA 書き込みヘッド位置は、上で読んだ累計転送語数から導出する (レジスタ再読み出し無し)。
    size_t head_byte = (size_t)(rx->total_bytes_written & (BMC_RX_RING_BYTES - 1));
    rx->last_head_byte = head_byte;

    uint64_t total_bits_written = rx->total_bytes_written * 8;
    uint64_t unread_bits = (total_bits_written >= rx->total_bits_read) ?
                           (total_bits_written - rx->total_bits_read) : 0;

    if (unread_bits >= (uint64_t)(BMC_RX_RING_BYTES * 8)) {
        // オーバーラン検出: 未読データが16KBリングサイズを超過
        rx->stats.rx_overrun_errors++;
        // tail_bit を直近の安全な位置 (head_byte より 256 バイト手前) へリカバリ
        size_t safe_tail_byte = (head_byte >= 256) ? (head_byte - 256) : (BMC_RX_RING_BYTES + head_byte - 256);
        rx->tail_bit = (safe_tail_byte * 8) & ((BMC_RX_RING_BYTES * 8) - 1);
        rx->total_bits_read = total_bits_written - (256 * 8);
        unread_bits = 256 * 8;
    }

    // 3-8. 受信範囲計算・シンク探索・フレーム抽出は純粋関数 bmc_rx_extract_frame_from_ring()
    //      (packet.c, ホストテスト対象) に集約。ハードウェアレジスタへのアクセスはここで終わり、
    //      以降はリングバッファの生データと tail_bit/unread_bits のみで完結する。
    bmc_rx_frame_t frame;
    size_t advance_bits = 0;
    bmc_rx_extract_status_t status = bmc_rx_extract_frame_from_ring(
        (const uint8_t *)rx->rx_ring_buf, BMC_RX_RING_BYTES,
        rx->tail_bit, unread_bits, &frame, &advance_bits);

    if (advance_bits > 0) {
        rx->tail_bit = (rx->tail_bit + advance_bits) & ((BMC_RX_RING_BYTES * 8) - 1);
        rx->total_bits_read += advance_bits;
    }

    // INVALID_LEN/CRC_MISMATCH/FRAME_OK はいずれも「入力を消費して進んだ」結果であり、
    // 同一 transfer_count のままでもリングに次のフレームが残っている可能性があるため
    // exhausted=false (次回ポーリングで新規DMA語が無くても再度抽出を試みる)。
    // INCOMPLETE/SYNC_NOT_FOUND (default節) は「現在の未読データを使い切った」ことを意味する
    // ため exhausted=true とする。
    switch (status) {
        case BMC_RX_EXTRACT_INVALID_LEN:
            rx->stats.rx_len_errors++;
            rx->last_extract_exhausted = false;
            return false;

        case BMC_RX_EXTRACT_CRC_MISMATCH:
            // CRCエラー等の不正フレーム (H-2: printf は完全禁止)
            rx->stats.rx_crc_errors++;
            rx->last_extract_exhausted = false;
            return false;

        case BMC_RX_EXTRACT_FRAME_OK:
            if (rx->has_received_first) {
                uint8_t expected_seq = (rx->last_seq + 1) & 0xFF;
                if (frame.seq != expected_seq) {
                    uint8_t drop = (frame.seq - expected_seq) & 0xFF;
                    rx->stats.rx_seq_drops += drop;
                }
            } else {
                rx->has_received_first = true;
            }
            rx->last_seq = frame.seq;

            if (out_type) *out_type = frame.type;
            if (out_seq)  *out_seq  = frame.seq;
            if (out_len)  *out_len  = frame.len;
            if (out_payload) memcpy(out_payload, frame.payload, frame.len);

            rx->stats.rx_packets++;
            rx->stats.rx_bytes += frame.len;
            rx->last_extract_exhausted = false;
            return true;

        case BMC_RX_EXTRACT_SYNC_NOT_FOUND:
        case BMC_RX_EXTRACT_INCOMPLETE:
        default:
            rx->last_extract_exhausted = true;
            return false;
    }
}

// ============================================================================
// 統計情報アクセサ関数 (M-9)
// ============================================================================
void bmc_tx_get_stats(const bmc_tx_controller_t *tx, poc_stats_t *out_stats) {
    if (tx && out_stats) {
        *out_stats = tx->stats;
    }
}

void bmc_tx_reset_stats(bmc_tx_controller_t *tx) {
    if (tx) {
        poc_stats_reset(&tx->stats);
    }
}

void bmc_rx_get_stats(const bmc_rx_port_t *rx, poc_stats_t *out_stats) {
    if (rx && out_stats) {
        *out_stats = rx->stats;
    }
}

void bmc_rx_reset_stats(bmc_rx_port_t *rx) {
    if (rx) {
        poc_stats_reset(&rx->stats);
    }
}

// ============================================================================
// RX 生データ診断ダンプ (B-2)
// ============================================================================
void bmc_rx_dump_diagnostic(const bmc_rx_port_t *rx) {
    if (!rx) return;
    printf("\n======================================================================\n");
    printf("  [RX DIAGNOSTIC DUMP]\n");
    printf("  Port: pio%u sm%u GP%u, Rate Family: %d\n",
           pio_get_index(rx->pio), rx->sm_rx, rx->rx_pin, (int)rx->rate_family);
    printf("  1. Total bytes written by DMA: %llu bytes (%llu words, lap: %llu, last_tc: 0x%08lx)\n",
           (unsigned long long)rx->total_bytes_written,
           (unsigned long long)rx->total_words_written,
           (unsigned long long)rx->lap_count,
           (unsigned long)rx->last_transfer_count);
    if (rx->dma_rx_chan >= 0) {
        uint32_t write_addr = dma_hw->ch[rx->dma_rx_chan].write_addr;
        size_t head_byte = (write_addr - (uintptr_t)rx->rx_ring_buf) & (BMC_RX_RING_BYTES - 1);
        printf("     Current DMA write offset in ring: %lu / %d (0x%04lx)\n",
               (unsigned long)head_byte, BMC_RX_RING_BYTES, (unsigned long)head_byte);
    }
    printf("     Tail bit: %lu (tail byte: %lu), Total bits read: %llu\n",
           (unsigned long)rx->tail_bit, (unsigned long)(rx->tail_bit / 8),
           (unsigned long long)rx->total_bits_read);

    // 2. リングバッファ先頭から最大32ワード(128バイト)程度の生データを16進ダンプ
    printf("  2. First 32 words (128 bytes) raw data from ring buffer:\n");
    const uint8_t *ring = (const uint8_t *)rx->rx_ring_buf;
    for (size_t i = 0; i < 128; i += 16) {
        printf("     %04lx: ", (unsigned long)i);
        for (size_t j = 0; j < 16; ++j) {
            printf("%02x ", ring[i + j]);
        }
        printf(" |");
        for (size_t j = 0; j < 16; ++j) {
            uint8_t c = ring[i + j];
            printf("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        printf("|\n");
    }

    // 3. シンクワード 0x93C7 (0x93, 0xC7) のバイト列部分一致の有無 (バイト境界簡易検索)
    bool sync_in_128 = false;
    size_t sync_pos_128 = 0;
    for (size_t i = 0; i < 127; ++i) {
        if (ring[i] == BMC_SYNC_BYTE_1 && ring[i + 1] == BMC_SYNC_BYTE_2) {
            sync_in_128 = true;
            sync_pos_128 = i;
            break;
        }
    }

    bool sync_in_ring = false;
    size_t sync_pos_ring = 0;
    for (size_t i = 0; i < BMC_RX_RING_BYTES - 1; ++i) {
        if (ring[i] == BMC_SYNC_BYTE_1 && ring[i + 1] == BMC_SYNC_BYTE_2) {
            sync_in_ring = true;
            sync_pos_ring = i;
            break;
        }
    }

    if (sync_in_128) {
        printf("  3. Sync word [0x93, 0xC7] check: FOUND in first 128B at byte offset %lu (0x%02lx)\n",
               (unsigned long)sync_pos_128, (unsigned long)sync_pos_128);
    } else if (sync_in_ring) {
        printf("  3. Sync word [0x93, 0xC7] check: NOT in first 128B, but FOUND in full %dB ring at byte offset %lu (0x%04lx)\n",
               BMC_RX_RING_BYTES, (unsigned long)sync_pos_ring, (unsigned long)sync_pos_ring);
    } else {
        printf("  3. Sync word [0x93, 0xC7] check: NOT FOUND at byte boundaries in entire %dB buffer\n",
               BMC_RX_RING_BYTES);
    }
    printf("======================================================================\n\n");
}
