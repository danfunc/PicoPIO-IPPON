#include "packet.h"
#include "sram_attrs.h"
#include <string.h>

// ============================================================================
// bmc_packet_total_bytes
// ============================================================================
size_t BMC_SRAM_FUNC(bmc_packet_total_bytes)(uint8_t payload_len) {
    size_t raw_len = BMC_HEADER_LEN + payload_len + BMC_CRC_LEN;
    size_t padding = 4 - (raw_len & 3); // 1..4 バイト
    return raw_len + padding;
}

// ============================================================================
// bmc_packet_encode
// ============================================================================
size_t BMC_SRAM_FUNC(bmc_packet_encode)(uint32_t *dma_buf, uint8_t type, uint8_t seq,
                         const uint8_t *payload, uint8_t payload_len) {
    // ------------------------------------------------------------------------
    // SECTION 1: 引数検証 & バッファ安全性チェック
    // ------------------------------------------------------------------------
    if (!dma_buf) {
        return 0;
    }
    if (payload_len < BMC_MIN_PAYLOAD_LEN || payload_len > BMC_MAX_PAYLOAD_LEN) {
        return 0;
    }
    if (payload_len > 0 && !payload) {
        // ペイロード長が正であるにもかかわらずポインタがNULLの場合は明確に拒否
        return 0;
    }

    // dma_buf[0] は PIO TX の pull block で消費されるワード長ヘッダ用
    // dma_buf[1] は ワード1: プリアンブル + シンクワード (4バイト)
    // dma_buf[2] は ワード2: ポストシンクフレーム (LEN を bit 0 とする)
    uint8_t *wire_bytes = (uint8_t *)&dma_buf[1];

    // ------------------------------------------------------------------------
    // SECTION 2: シリアライズ (プリアンブル、シンク、ヘッダ、ペイロード)
    // ------------------------------------------------------------------------
    // 2.1 プリアンブル (0xAAAA) & シンクワード (0x93C7) の配置 [4バイト]
    wire_bytes[0] = BMC_PREAMBLE_BYTE_1; // 0xAA (10101010b)
    wire_bytes[1] = BMC_PREAMBLE_BYTE_2; // 0xAA (10101010b)
    wire_bytes[2] = BMC_SYNC_BYTE_1;     // 0x93 (10010011b)
    wire_bytes[3] = BMC_SYNC_BYTE_2;     // 0xC7 (11000111b)

    // 2.2 ポストシンクフレームヘッダ [4バイト]
    // ワード2の第0バイト (wire_bytes[4]) が厳密に LEN となる
    wire_bytes[4] = payload_len;         // LEN (1..128)
    wire_bytes[5] = type;                // TYPE
    wire_bytes[6] = seq;                 // SEQ
    wire_bytes[7] = 0x00;                // RESERVED

    // 2.3 ペイロードデータのコピー
    memcpy(&wire_bytes[8], payload, payload_len);

    // ------------------------------------------------------------------------
    // SECTION 3: CRC16-CCITT 計算 (Big-Endian MSB / LSB)
    // ------------------------------------------------------------------------
    // CRC16 計算範囲: LEN から PAYLOAD 末尾まで (4 + payload_len バイト)
    // プリアンブルとシンクワードはフレーミング用同期列のためCRC対象外とし、
    // ランダムデータ内の偶発的シンク出現はCRC不一致により確実に排除される
    uint8_t *post_sync_ptr = &wire_bytes[BMC_PREAMBLE_SYNC_BYTES];
    size_t post_sync_data_len = BMC_HEADER_LEN + payload_len;
    uint16_t crc = crc16_ccitt(post_sync_ptr, post_sync_data_len);

    size_t crc_offset = BMC_PREAMBLE_SYNC_BYTES + post_sync_data_len;
    wire_bytes[crc_offset]     = (uint8_t)(crc >> 8);       // MSB
    wire_bytes[crc_offset + 1] = (uint8_t)(crc & 0xFF);     // LSB

    // ------------------------------------------------------------------------
    // SECTION 4: ゼロカウント偶数パリティ & 32bit境界パディング計算
    // ------------------------------------------------------------------------
    // 【WHY: BMC極性とゼロカウントパリティの物理的理由】
    // BMC (Biphase Mark Code / Differential Manchester) では、全ビット境界で必ず極性反転が発生し、
    // ビット1ではセル中央でもう一度反転する (計2回反転 -> セル通過前後で極性不変)。
    // 一方、ビット0では中央反転がないため (計1回反転 -> セル通過前後で極性が反転する)。
    // したがって、送信される全ビット中の「0」の総数が偶数であれば、フレーム終了時の信号極性は
    // アイドル時の初期極性 (High / 3.3V) と完全に一致する。
    // これにより、EOP送出完了時の set pins, 1 による急峻な偽エッジや極性不整合が100%根絶される。
    size_t raw_total_len = BMC_PREAMBLE_SYNC_BYTES + BMC_HEADER_LEN + payload_len + BMC_CRC_LEN;
    size_t post_sync_raw_len = BMC_HEADER_LEN + payload_len + BMC_CRC_LEN;
    size_t padding = 4 - (post_sync_raw_len & 3); // 1..4 バイト (32bitワード境界合わせ)

    // 送信全バイト (プリアンブル・シンク・ヘッダ・ペイロード・CRC) のビット0の個数の偶奇のみを求める。
    // 【最適化】: 元実装は毎バイト __builtin_popcount を呼んでいたが、必要なのは「0の総数の偶奇」
    // だけである。全バイトをXOR集約した1バイト値の1ビット数の偶奇は、元の全バイトの1ビット総数の
    // 偶奇と一致する(各ビット列を8個のビット桁に分解し、桁ごとのXORが桁のパリティを保存するため)。
    // さらに全送信ビット数 raw_total_len*8 は常に偶数なので、1の総数の偶奇と0の総数の偶奇は一致する。
    // これにより popcount 呼び出しを raw_total_len 回から1回に削減する (安価な XOR ループに置換)。
    uint8_t xor_acc = 0;
    for (size_t i = 0; i < raw_total_len; ++i) {
        xor_acc ^= wire_bytes[i];
    }
    bool zero_count_is_even = (__builtin_popcount(xor_acc) & 1) == 0;

    // パディングの先頭 (padding - 1) バイトは 0xFF (0の個数: 0)
    for (size_t i = 0; i < padding - 1; ++i) {
        wire_bytes[raw_total_len + i] = 0xFF;
    }

    // 最後のパディングバイトで全送信ビット中の「0」の総数を偶数に整合
    if (zero_count_is_even) {
        wire_bytes[raw_total_len + padding - 1] = 0xFF; // 既に偶数のため 0xFF (0の追加: 0個)
    } else {
        wire_bytes[raw_total_len + padding - 1] = 0xFE; // 奇数のため 0xFE (11111110b -> 0を1個追加して偶数化)
    }

    size_t total_wire_bytes = raw_total_len + padding;

    // B-3案b: ポストアンブル (全1の32bit) をフレーム直後に追加
    for (size_t i = 0; i < BMC_POSTAMBLE_BYTES; ++i) {
        wire_bytes[total_wire_bytes + i] = 0xFF;
    }
    size_t total_data_words = (total_wire_bytes + BMC_POSTAMBLE_BYTES) / 4;

    // ワード0: (total_data_words - 1) を PIO TX の Y レジスタ用カウンタとして格納
    // DMA転送時に channel_config_set_bswap(&c, true) を使用するため、
    // PIO FIFO に正しく (total_data_words - 1) が入るよう __builtin_bswap32 を施す
    dma_buf[0] = __builtin_bswap32((uint32_t)(total_data_words - 1));

    return total_data_words + 1; // ヘッダワード1 + 総データワード数
}

// ============================================================================
// bmc_packet_decode
// ============================================================================
int BMC_SRAM_FUNC(bmc_packet_decode)(const uint8_t *frame_bytes, size_t max_bytes,
                      uint8_t *out_type, uint8_t *out_seq,
                      uint8_t *out_payload, uint8_t *out_len) {
    if (!frame_bytes || max_bytes < (BMC_HEADER_LEN + BMC_MIN_PAYLOAD_LEN + BMC_CRC_LEN)) {
        return BMC_ERR_BUFFER_TOO_SMALL;
    }

    uint8_t len = frame_bytes[0];
    if (len < BMC_MIN_PAYLOAD_LEN || len > BMC_MAX_PAYLOAD_LEN) {
        return BMC_ERR_INVALID_LEN;
    }

    size_t total_frame_bytes = bmc_packet_total_bytes(len);
    if (max_bytes < total_frame_bytes) {
        return BMC_ERR_FRAME_INCOMPLETE;
    }

    // CRC16-CCITT 検証
    uint16_t expected_crc = crc16_ccitt(frame_bytes, BMC_HEADER_LEN + len);
    size_t crc_offset = BMC_HEADER_LEN + len;
    uint16_t received_crc = ((uint16_t)frame_bytes[crc_offset] << 8) | frame_bytes[crc_offset + 1];

    if (expected_crc != received_crc) {
        return BMC_ERR_CRC_MISMATCH;
    }

    if (out_type)    *out_type = frame_bytes[1];
    if (out_seq)     *out_seq  = frame_bytes[2];
    if (out_len)     *out_len  = len;
    if (out_payload) {
        memcpy(out_payload, &frame_bytes[BMC_HEADER_LEN], len);
    }

    return (int)total_frame_bytes;
}

// ============================================================================
// bmc_find_sync_in_bits
// ============================================================================
int BMC_SRAM_FUNC(bmc_find_sync_in_bits)(const uint8_t *buf, size_t buf_len_bytes,
                          size_t start_bit, size_t end_bit,
                          size_t *out_sync_end_bit) {
    if (!buf || !out_sync_end_bit || start_bit >= end_bit || end_bit > buf_len_bytes * 8) {
        return BMC_ERR_INVALID_ARG;
    }

    if (end_bit < 16) {
        return BMC_ERR_SYNC_NOT_FOUND;
    }

    // シンク語 (16bit: 0x93C7) の終端ビット b は [start_bit, end_bit) かつ b >= 15 を満たす必要がある。
    // 開始ビット s = b - 15 の探索範囲:
    // s >= min_s, s <= max_s
    size_t min_s = (start_bit >= 15) ? (start_bit - 15) : 0;
    size_t max_s = end_bit - 16;
    if (min_s > max_s) {
        return BMC_ERR_SYNC_NOT_FOUND;
    }

    size_t first_byte = min_s / 8;
    size_t last_byte = max_s / 8;

    // 8通りのビット位相 (0..7) における0x93C7の32bitシフトパターンとマスク
    static const struct {
        uint32_t pat;
        uint32_t mask;
    } s_sync_patterns[8] = {
        { 0x93C70000u, 0xFFFF0000u },
        { 0x49E38000u, 0x7FFF8000u },
        { 0x24F1C000u, 0x3FFFC000u },
        { 0x1278E000u, 0x1FFFE000u },
        { 0x093C7000u, 0x0FFFF000u },
        { 0x049E3800u, 0x07FFF800u },
        { 0x024F1C00u, 0x03FFFC00u },
        { 0x01278E00u, 0x01FFFE00u },
    };

    for (size_t byte_idx = first_byte; byte_idx <= last_byte; ++byte_idx) {
        uint32_t u32 = 0;
        if (byte_idx + 4 <= buf_len_bytes) {
            uint32_t raw;
            memcpy(&raw, &buf[byte_idx], 4);
            u32 = __builtin_bswap32(raw);
        } else {
            uint8_t tmp[4] = {0};
            size_t rem = buf_len_bytes - byte_idx;
            memcpy(tmp, &buf[byte_idx], rem);
            uint32_t raw;
            memcpy(&raw, tmp, 4);
            u32 = __builtin_bswap32(raw);
        }

        for (int phase = 0; phase < 8; ++phase) {
            size_t s = byte_idx * 8 + (size_t)phase;
            if (s < min_s) continue;
            if (s > max_s) break;
            if ((u32 & s_sync_patterns[phase].mask) == s_sync_patterns[phase].pat) {
                *out_sync_end_bit = s + 15;
                return BMC_OK;
            }
        }
    }

    return BMC_ERR_SYNC_NOT_FOUND;
}

// ============================================================================
// bmc_extract_bits_as_bytes
// ============================================================================
int BMC_SRAM_FUNC(bmc_extract_bits_as_bytes)(const uint8_t *buf, size_t buf_len_bytes,
                              size_t start_bit, size_t num_bytes,
                              uint8_t *out_bytes) {
    if (!buf || !out_bytes) {
        return BMC_ERR_INVALID_ARG;
    }
    if (num_bytes == 0) {
        return BMC_OK;
    }
    if (start_bit + num_bytes * 8 > buf_len_bytes * 8) {
        return BMC_ERR_BUFFER_TOO_SMALL;
    }

    size_t byte_idx = start_bit / 8;
    size_t bit_offset = start_bit % 8;

    if (bit_offset == 0) {
        memcpy(out_bytes, &buf[byte_idx], num_bytes);
    } else {
        size_t shift_left = bit_offset;
        size_t shift_right = 8 - bit_offset;
        for (size_t i = 0; i < num_bytes; ++i) {
            out_bytes[i] = (uint8_t)((buf[byte_idx + i] << shift_left) |
                                     (buf[byte_idx + i + 1] >> shift_right));
        }
    }

    return BMC_OK;
}

// ============================================================================
// bmc_rx_extract_frame_from_ring
// ============================================================================
bmc_rx_extract_status_t BMC_SRAM_FUNC(bmc_rx_extract_frame_from_ring)(
    const uint8_t *ring_buf, size_t ring_bytes,
    size_t tail_bit, uint64_t unread_bits,
    bmc_rx_frame_t *out_frame,
    size_t *out_advance_bits) {
    if (out_advance_bits) {
        *out_advance_bits = 0;
    }
    if (!ring_buf || ring_bytes == 0 || !out_advance_bits) {
        return BMC_RX_EXTRACT_INCOMPLETE;
    }

    size_t ring_bits = ring_bytes * 8;
    tail_bit &= (ring_bits - 1);

    // 最小フレーム (LEN=BMC_MIN_PAYLOAD_LEN) を受理するのに真に必要な最小ビット数:
    // シンクワード16ビット + 最小ポストシンクフレーム。この判定は「同期語探索」段階の
    // 入口ガードであり、実際のフレーム完了判定 (needed_bits) とは別に、無駄な線形コピーを
    // 避けるためだけに存在する。過大な固定値を置くと、単発の短いフレームが後続データなしに
    // 永久に受理されない不具合になるため、理論上の最小値に一致させる。
    size_t min_needed_bits = 16 + bmc_packet_total_bytes(BMC_MIN_PAYLOAD_LEN) * 8;
    if (unread_bits < min_needed_bits) {
        return BMC_RX_EXTRACT_INCOMPLETE;
    }

    size_t tail_byte = (tail_bit / 8) & (ring_bytes - 1);
    size_t bit_start = tail_bit % 8;

    // linear_buf はバイト境界 tail_byte から始まるため、有効ビット範囲は
    // [bit_start, bit_start + unread_bits) であり、必要バイト数は
    // ceil((bit_start + unread_bits) / 8)。
    uint64_t total_avail_bits_needed = (uint64_t)bit_start + unread_bits;
    size_t avail_bytes = (size_t)((total_avail_bits_needed + 7) / 8);
    if (avail_bytes > 384) {
        avail_bytes = 384;
    }

    uint8_t linear_buf[384];
    size_t first_part = ring_bytes - tail_byte;
    if (avail_bytes <= first_part) {
        memcpy(linear_buf, &ring_buf[tail_byte], avail_bytes);
    } else {
        memcpy(linear_buf, &ring_buf[tail_byte], first_part);
        memcpy(linear_buf + first_part, ring_buf, avail_bytes - first_part);
    }

    size_t total_avail_bits = (size_t)total_avail_bits_needed;
    if (total_avail_bits > avail_bytes * 8) {
        total_avail_bits = avail_bytes * 8;
    }

    size_t sync_end_bit = 0;
    int find_res = bmc_find_sync_in_bits(linear_buf, avail_bytes, bit_start, total_avail_bits, &sync_end_bit);
    if (find_res != BMC_OK) {
        // 現在の探索範囲にシンク未検出 -> 次回探索のため tail_bit を進める
        // (15bitのシンク跨ぎマージンを残す)
        if (total_avail_bits > bit_start + 15) {
            *out_advance_bits = total_avail_bits - bit_start - 15;
        }
        return BMC_RX_EXTRACT_SYNC_NOT_FOUND;
    }

    // シンク検出成功。シンク直後のビット (LEN の bit 7) からポストシンクフレーム開始
    size_t post_sync_start_bit = sync_end_bit + 1;

    // LEN バイト (8ビット) を抽出できるか確認
    if (post_sync_start_bit + 8 > total_avail_bits) {
        // シンク開始位置まで tail_bit を進めて次回まで待機
        size_t sync_start_bit = (sync_end_bit >= 15) ? (sync_end_bit - 15) : 0;
        if (sync_start_bit > bit_start) {
            *out_advance_bits = sync_start_bit - bit_start;
        }
        return BMC_RX_EXTRACT_INCOMPLETE; // まだ LEN のビットが届ききっていない
    }

    uint8_t len = 0;
    bmc_extract_bits_as_bytes(linear_buf, avail_bytes, post_sync_start_bit, 1, &len);

    if (len < BMC_MIN_PAYLOAD_LEN || len > BMC_MAX_PAYLOAD_LEN) {
        // 不正なLEN -> 偽シンクと判定し、検出シンクの直後からビット単位探索を即時再開
        *out_advance_bits = (post_sync_start_bit > bit_start) ? (post_sync_start_bit - bit_start) : 1;
        return BMC_RX_EXTRACT_INVALID_LEN;
    }

    // フレーム全体のバイト数と必要ビット数を算出
    size_t needed_bytes = bmc_packet_total_bytes(len);
    size_t needed_bits = needed_bytes * 8;

    if (post_sync_start_bit + needed_bits > total_avail_bits) {
        // フレーム全体が揃うまで待機。シンク開始位置まで tail_bit を進める
        size_t sync_start_bit = (sync_end_bit >= 15) ? (sync_end_bit - 15) : 0;
        if (sync_start_bit > bit_start) {
            *out_advance_bits = sync_start_bit - bit_start;
        }
        return BMC_RX_EXTRACT_INCOMPLETE; // フレーム全体の到着待ち
    }

    // フレーム全体をビット抽出して検証
    uint8_t frame_buf[BMC_MAX_FRAME_BYTES];
    bmc_extract_bits_as_bytes(linear_buf, avail_bytes, post_sync_start_bit, needed_bytes, frame_buf);

    uint8_t type = 0, seq = 0, plen = 0;
    uint8_t payload_tmp[BMC_MAX_PAYLOAD_LEN];
    int dec_res = bmc_packet_decode(frame_buf, needed_bytes, &type, &seq, payload_tmp, &plen);

    if (dec_res > 0) {
        *out_advance_bits = (post_sync_start_bit + needed_bits) - bit_start;
        if (out_frame) {
            out_frame->type = type;
            out_frame->seq = seq;
            out_frame->len = plen;
            memcpy(out_frame->payload, payload_tmp, plen);
        }
        return BMC_RX_EXTRACT_FRAME_OK;
    } else {
        // CRCエラー等の不正フレーム -> 検出シンクの直後から次ビット探索を再開
        *out_advance_bits = (post_sync_start_bit > bit_start) ? (post_sync_start_bit - bit_start) : 1;
        return BMC_RX_EXTRACT_CRC_MISMATCH;
    }
}

// ============================================================================
// bmc_rx_calc_total_words
// ============================================================================
uint64_t bmc_rx_calc_total_words(uint64_t lap_count, uint32_t current_transfer_count, uint32_t reload_count) {
    if (reload_count == 0) return 0;
    uint32_t consumed = (current_transfer_count <= reload_count) ?
                        (reload_count - current_transfer_count) : 0;
    return lap_count * (uint64_t)reload_count + (uint64_t)consumed;
}

// ============================================================================
// bmc_rx_calc_delta_words
// ============================================================================
uint64_t bmc_rx_calc_delta_words(uint64_t prev_lap_count, uint32_t prev_transfer_count,
                                 uint64_t curr_lap_count, uint32_t curr_transfer_count,
                                 uint32_t reload_count) {
    if (reload_count == 0 || curr_lap_count < prev_lap_count) return 0;
    uint64_t prev_total = bmc_rx_calc_total_words(prev_lap_count, prev_transfer_count, reload_count);
    uint64_t curr_total = bmc_rx_calc_total_words(curr_lap_count, curr_transfer_count, reload_count);
    if (curr_total >= prev_total) {
        return curr_total - prev_total;
    }
    return 0;
}

// ============================================================================
// bmc_rx_update_transfer_state
// ============================================================================
uint64_t BMC_SRAM_FUNC(bmc_rx_update_transfer_state)(uint64_t *inout_lap_count,
                                      uint32_t *inout_last_transfer_count,
                                      uint32_t curr_transfer_count,
                                      uint32_t reload_count) {
    if (!inout_lap_count || !inout_last_transfer_count || reload_count == 0) return 0;

    // 前提 (呼出し側 bmc_rx_poll_packet の呼出し間隔に依存): 1回のポーリング間隔中に
    // reload_count (28bit, 最大約2.68億語 ≈ 1.07GB) 語以上の転送が発生しないこと。
    // これを超えると "COUNTが増加した" という1周回検知が飽和し、2周以上のラップを
    // 1回として誤検出しうる (エイリアシング)。本PoCの想定転送量では十分な安全マージンがある。
    uint32_t last_tc = *inout_last_transfer_count;
    if (curr_transfer_count > last_tc) {
        // カウントダウンレジスタが増加した = 0 を通過して reload_count に自動リロードされた
        (*inout_lap_count)++;
    }
    *inout_last_transfer_count = curr_transfer_count;
    return bmc_rx_calc_total_words(*inout_lap_count, curr_transfer_count, reload_count);
}


