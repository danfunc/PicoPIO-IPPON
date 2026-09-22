#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include "../common/crc16.h"
#include "../common/packet.h"

// ============================================================================
// H-5 Diagnostic Assertion Macros
// ============================================================================
#define CHECK_EQ_INT(actual, expected) do { \
    long long _a = (long long)(actual); \
    long long _e = (long long)(expected); \
    if (_a != _e) { \
        fprintf(stderr, "FAIL [%s:%d]: " #actual " (%lld) != " #expected " (%lld)\n", \
                __FILE__, __LINE__, _a, _e); \
        assert(_a == _e); \
    } \
} while(0)

#define CHECK_EQ_UINT(actual, expected) do { \
    unsigned long long _a = (unsigned long long)(actual); \
    unsigned long long _e = (unsigned long long)(expected); \
    if (_a != _e) { \
        fprintf(stderr, "FAIL [%s:%d]: " #actual " (%llu / 0x%llX) != " #expected " (%llu / 0x%llX)\n", \
                __FILE__, __LINE__, _a, _a, _e, _e); \
        assert(_a == _e); \
    } \
} while(0)

#define CHECK_TRUE(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL [%s:%d]: condition failed: " #cond "\n", __FILE__, __LINE__); \
        assert(cond); \
    } \
} while(0)

// Helper: Bitstream serialization
static void write_bits_to_stream(uint8_t *stream, size_t *bit_pos, const uint8_t *src_bytes, size_t num_bits) {
    for (size_t i = 0; i < num_bits; ++i) {
        uint8_t src_byte = src_bytes[i / 8];
        uint8_t bit = (src_byte >> (7 - (i % 8))) & 1;
        size_t pos = *bit_pos;
        if (bit) {
            stream[pos / 8] |= (uint8_t)(1u << (7 - (pos % 8)));
        } else {
            stream[pos / 8] &= (uint8_t)~(1u << (7 - (pos % 8)));
        }
        (*bit_pos)++;
    }
}

int main(void) {
    printf("Running Host Unit Tests for BMC Packet Protocol...\n");

    // =========================================================================
    // Test 1: CRC16-CCITT Known Vector Check
    // "123456789" -> 0x29B1 (CCITT standard vector)
    // =========================================================================
    {
        const uint8_t test_vector[] = "123456789";
        uint16_t crc_val = crc16_ccitt(test_vector, 9);
        printf("[TEST 1] CRC16 test vector '123456789': 0x%04X (Expected: 0x29B1)\n", crc_val);
        CHECK_EQ_UINT(crc_val, 0x29B1);
    }

    // =========================================================================
    // Test 2: Variable Length Packet Encode & Decode (All Lengths 1 to 128) [C-5]
    // =========================================================================
    static uint32_t dma_buf[BMC_TX_DMA_MAX_WORDS];
    uint8_t payload_in[BMC_MAX_PAYLOAD_LEN];
    uint8_t payload_out[BMC_MAX_PAYLOAD_LEN];

    printf("[TEST 2] Testing ALL payload lengths 1..128 for framing, parity, and decode...\n");

    for (uint8_t len = 1; len <= BMC_MAX_PAYLOAD_LEN; ++len) {
        for (int i = 0; i < len; ++i) {
            payload_in[i] = (uint8_t)(0xAA ^ (i * 7) ^ len);
        }

        uint8_t seq = (uint8_t)(len * 13);
        uint8_t type = BMC_PKT_TYPE_DATA;

        memset(dma_buf, 0x55, sizeof(dma_buf));
        size_t dma_words = bmc_packet_encode(dma_buf, type, seq, payload_in, len);
        CHECK_TRUE(dma_words > 0);

        // Word 0 check (総データワード数 - 1)
        size_t post_sync_bytes = bmc_packet_total_bytes(len);
        size_t expected_total_data_words = BMC_PREAMBLE_SYNC_WORDS + (post_sync_bytes / 4) + BMC_POSTAMBLE_WORDS;
        CHECK_EQ_UINT(dma_words, expected_total_data_words + 1);

        uint32_t w0 = __builtin_bswap32(dma_buf[0]);
        CHECK_EQ_UINT(w0, (uint32_t)(expected_total_data_words - 1));

        // プリアンブル (0xAAAA) & シンクワード (0x93C7) の配置検証
        const uint8_t *wire_bytes = (const uint8_t *)&dma_buf[1];
        CHECK_EQ_UINT(wire_bytes[0], (uint8_t)BMC_PREAMBLE_BYTE_1);
        CHECK_EQ_UINT(wire_bytes[1], (uint8_t)BMC_PREAMBLE_BYTE_2);
        CHECK_EQ_UINT(wire_bytes[2], (uint8_t)BMC_SYNC_BYTE_1);
        CHECK_EQ_UINT(wire_bytes[3], (uint8_t)BMC_SYNC_BYTE_2);

        // ポストシンク第0ワードのアライメント検証 (wire_bytes[4] == LEN)
        CHECK_EQ_UINT(wire_bytes[4], len);
        CHECK_EQ_UINT(wire_bytes[5], type);
        CHECK_EQ_UINT(wire_bytes[6], seq);
        CHECK_EQ_UINT(wire_bytes[7], 0x00);

        // パリティチェック: 送信全ビット中の0の総数が必ず偶数であることを検証 (EOP High保証)
        size_t total_wire_bytes = expected_total_data_words * 4;
        size_t total_zeros = 0;
        for (size_t b = 0; b < total_wire_bytes; ++b) {
            total_zeros += (8 - __builtin_popcount(wire_bytes[b]));
        }
        CHECK_EQ_UINT(total_zeros & 1, 0);

        // ポストシンクデコード検証
        const uint8_t *post_sync_bytes_ptr = &wire_bytes[BMC_PREAMBLE_SYNC_BYTES];
        uint8_t dec_type = 0, dec_seq = 0, dec_len = 0;
        memset(payload_out, 0, sizeof(payload_out));

        int dec_res = bmc_packet_decode(post_sync_bytes_ptr, post_sync_bytes,
                                        &dec_type, &dec_seq, payload_out, &dec_len);
        CHECK_EQ_INT(dec_res, (int)post_sync_bytes);
        CHECK_EQ_UINT(dec_type, type);
        CHECK_EQ_UINT(dec_seq, seq);
        CHECK_EQ_UINT(dec_len, len);
        CHECK_EQ_INT(memcmp(payload_in, payload_out, len), 0);

        if (len == 1 || len == 128 || (len % 16 == 0)) {
            printf("  - Length %3u: DMA words=%2zu, Post-sync bytes=%3zu, Zeros=%3zu [PASS]\n",
                   len, dma_words, post_sync_bytes, total_zeros);
        }
    }
    printf("[TEST 2] All 128 payload lengths passed framing, parity, and decode tests! [PASS]\n");

    // =========================================================================
    // Test 3: Corruption & CRC Error Check with Named Error Constants (M-7)
    // =========================================================================
    {
        uint8_t len = 32;
        memset(payload_in, 0x12, len);
        size_t dma_words = bmc_packet_encode(dma_buf, BMC_PKT_TYPE_DATA, 1, payload_in, len);
        CHECK_TRUE(dma_words > 0);

        uint8_t *wire_bytes = (uint8_t *)&dma_buf[1];
        uint8_t *post_sync = &wire_bytes[BMC_PREAMBLE_SYNC_BYTES];
        size_t post_sync_bytes = bmc_packet_total_bytes(len);

        // ペイロードの1バイトを反転
        post_sync[4 + 10] ^= 0x01;
        uint8_t dummy_pl[128], dummy_type, dummy_seq, dummy_len;
        int res = bmc_packet_decode(post_sync, post_sync_bytes, &dummy_type, &dummy_seq, dummy_pl, &dummy_len);
        CHECK_EQ_INT(res, BMC_ERR_CRC_MISMATCH);
        printf("[TEST 3] Bit corruption detected by CRC: res=%d (BMC_ERR_CRC_MISMATCH) [PASS]\n", res);
    }

    // =========================================================================
    // Test 4: Invalid Length Boundaries Rejected (M-7)
    // =========================================================================
    {
        uint8_t bad_len_0 = 0;
        CHECK_EQ_UINT(bmc_packet_encode(dma_buf, 0, 0, payload_in, bad_len_0), 0);
        uint8_t bad_len_129 = 129;
        CHECK_EQ_UINT(bmc_packet_encode(dma_buf, 0, 0, payload_in, bad_len_129), 0);

        // デコード時も不正なLENが名前付き定数で拒否されることを検証
        uint8_t bad_frame_0[16] = {0}; // len = 0
        int res0 = bmc_packet_decode(bad_frame_0, sizeof(bad_frame_0), NULL, NULL, NULL, NULL);
        CHECK_EQ_INT(res0, BMC_ERR_INVALID_LEN);

        uint8_t bad_frame_129[16] = {129, 0, 0, 0}; // len = 129
        int res129 = bmc_packet_decode(bad_frame_129, sizeof(bad_frame_129), NULL, NULL, NULL, NULL);
        CHECK_EQ_INT(res129, BMC_ERR_INVALID_LEN);

        printf("[TEST 4] Invalid length boundaries rejected (encode=0, decode=BMC_ERR_INVALID_LEN) [PASS]\n");
    }

    // =========================================================================
    // Test 5: NULL Payload Protection Check
    // =========================================================================
    {
        // payload_len > 0 にもかかわらず payload が NULL の場合は 0 を返却
        size_t res = bmc_packet_encode(dma_buf, BMC_PKT_TYPE_DATA, 1, NULL, 10);
        CHECK_EQ_UINT(res, 0);

        // dma_buf 自体が NULL の場合も 0
        res = bmc_packet_encode(NULL, BMC_PKT_TYPE_DATA, 1, payload_in, 10);
        CHECK_EQ_UINT(res, 0);

        // decode 時にバッファが NULL または短すぎる場合
        CHECK_EQ_INT(bmc_packet_decode(NULL, 100, NULL, NULL, NULL, NULL), BMC_ERR_BUFFER_TOO_SMALL);
        CHECK_EQ_INT(bmc_packet_decode(payload_in, 3, NULL, NULL, NULL, NULL), BMC_ERR_BUFFER_TOO_SMALL);
        printf("[TEST 5] NULL payload and buffer safety guards verified [PASS]\n");
    }

    // =========================================================================
    // Test 6: Bitstream Scanner & Recovery from Arbitrary Bit Phase Offsets 0..7
    // (Mandatory Upper-Layer Audit Test: line-level exact phase shift verification)
    // =========================================================================
    {
        printf("[TEST 6] Testing bit-granular sync detection and phase recovery across offsets 0..7...\n");
        static uint8_t bitstream[4096];
        memset(bitstream, 0xFF, sizeof(bitstream)); // 初期アイドル状態 (High=1)

        size_t write_bit_pos = 0;
        const uint8_t phase_offsets[8] = {0, 1, 2, 3, 4, 5, 6, 7};
        const uint8_t test_payload_lens[8] = {5, 12, 1, 64, 3, 20, 8, 31};

        // 8フレームを各ビットオフセット (0..7) を意図的に挿入してシリアライズ
        for (int frame_idx = 0; frame_idx < 8; ++frame_idx) {
            uint8_t pad_bits = phase_offsets[frame_idx];
            // 各フレームの前に pad_bits 個のアイドルビット (1) を挿入して位相をずらす
            for (size_t p = 0; p < pad_bits; ++p) {
                size_t pos = write_bit_pos++;
                bitstream[pos / 8] |= (uint8_t)(1u << (7 - (pos % 8)));
            }

            // テストパケット生成
            uint8_t cur_len = test_payload_lens[frame_idx];
            uint8_t cur_seq = (uint8_t)(0x10 + frame_idx);
            uint8_t cur_type = BMC_PKT_TYPE_BENCH;
            uint8_t cur_payload[128];
            for (int i = 0; i < cur_len; ++i) {
                cur_payload[i] = (uint8_t)(frame_idx * 17 + i);
            }

            size_t enc_words = bmc_packet_encode(dma_buf, cur_type, cur_seq, cur_payload, cur_len);
            CHECK_TRUE(enc_words > 0);

            // 送信される物理ワイヤバイト列 (プリアンブル + シンク + ポストシンク)
            const uint8_t *wire_data = (const uint8_t *)&dma_buf[1];
            size_t wire_bits = (enc_words - 1) * 32;

            // ビットストリームへ正確に書き込み
            write_bits_to_stream(bitstream, &write_bit_pos, wire_data, wire_bits);

            // フレーム間に数ビット〜数十ビットのアイドルHighを挿入
            size_t inter_frame_idle = 11 + frame_idx * 3;
            for (size_t idl = 0; idl < inter_frame_idle; ++idl) {
                size_t pos = write_bit_pos++;
                bitstream[pos / 8] |= (uint8_t)(1u << (7 - (pos % 8)));
            }
        }

        size_t total_stream_bits = write_bit_pos;
        size_t total_stream_bytes = (total_stream_bits + 7) / 8;

        // --- 受信側ビットスキャナによる連続パケット探索・復号 ---
        size_t scan_bit = 0;
        for (int frame_idx = 0; frame_idx < 8; ++frame_idx) {
            size_t sync_end_bit = 0;
            // ビット単位でシンクワード (0x93C7) を探索
            int find_res = bmc_find_sync_in_bits(bitstream, total_stream_bytes,
                                                 scan_bit, total_stream_bits, &sync_end_bit);
            CHECK_EQ_INT(find_res, BMC_OK);

            // シンクの直後がポストシンクフレームの先頭ビット (LEN の bit 7)
            size_t post_sync_start_bit = sync_end_bit + 1;

            // 1. LEN バイトの抽出
            uint8_t extracted_len = 0;
            int ext_res = bmc_extract_bits_as_bytes(bitstream, total_stream_bytes,
                                                    post_sync_start_bit, 1, &extracted_len);
            CHECK_EQ_INT(ext_res, BMC_OK);
            CHECK_EQ_UINT(extracted_len, test_payload_lens[frame_idx]);

            // 2. フレーム全体の抽出
            size_t needed_bytes = bmc_packet_total_bytes(extracted_len);
            uint8_t frame_buf[BMC_MAX_FRAME_BYTES];
            ext_res = bmc_extract_bits_as_bytes(bitstream, total_stream_bytes,
                                                post_sync_start_bit, needed_bytes, frame_buf);
            CHECK_EQ_INT(ext_res, BMC_OK);

            // 3. パケットのデコード & CRC検証
            uint8_t dec_type = 0, dec_seq = 0, dec_plen = 0;
            uint8_t dec_payload[128];
            int dec_res = bmc_packet_decode(frame_buf, needed_bytes,
                                            &dec_type, &dec_seq, dec_payload, &dec_plen);
            CHECK_EQ_INT(dec_res, (int)needed_bytes);
            CHECK_EQ_UINT(dec_type, BMC_PKT_TYPE_BENCH);
            CHECK_EQ_UINT(dec_seq, (uint8_t)(0x10 + frame_idx));
            CHECK_EQ_UINT(dec_plen, test_payload_lens[frame_idx]);

            uint8_t exp_payload[128];
            uint8_t cur_len = test_payload_lens[frame_idx];
            for (int i = 0; i < cur_len; ++i) {
                exp_payload[i] = (uint8_t)(frame_idx * 17 + i);
            }
            CHECK_EQ_INT(memcmp(exp_payload, dec_payload, cur_len), 0);

            printf("  - Phase Offset %u-bit: Synced at bit %zu, Extracted Len %u, Seq 0x%02X [PASS]\n",
                   phase_offsets[frame_idx], sync_end_bit, dec_plen, dec_seq);

            // 次フレームの探索開始位置を現在のフレーム終端へ進める
            scan_bit = post_sync_start_bit + (needed_bytes * 8);
        }
        printf("[TEST 6] Bit-granular sync and recovery across ALL 8 phase offsets passed flawlessly! [PASS]\n");
    }

    // =========================================================================
    // Test 7: Automatic Frame Recovery on the Very Next Frame After Corruption
    // =========================================================================
    {
        printf("[TEST 7] Testing next-frame recovery after corrupted bitstream...\n");
        static uint8_t stream[2048];
        memset(stream, 0xFF, sizeof(stream));

        size_t w_bit = 0;

        // 1. 不正フレームの生成 (LEN=0 の不正シンクフレーム)
        // 偽のプリアンブル+シンクワードを書き込む
        uint8_t fake_hdr[5] = {0xAA, 0xAA, 0x93, 0xC7, 0x00}; // len = 0 (不正)
        write_bits_to_stream(stream, &w_bit, fake_hdr, 40);

        // アイドルを挟む
        for (int i = 0; i < 20; ++i) {
            size_t pos = w_bit++;
            stream[pos / 8] |= (uint8_t)(1u << (7 - (pos % 8)));
        }

        // 2. 正常なフレーム (位相オフセット 5 ビット)
        uint8_t good_pl[] = "Hello RP2350 Recovery!";
        uint8_t good_len = (uint8_t)strlen((char *)good_pl);
        size_t enc_words = bmc_packet_encode(dma_buf, BMC_PKT_TYPE_DATA, 42, good_pl, good_len);
        CHECK_TRUE(enc_words > 0);
        write_bits_to_stream(stream, &w_bit, (const uint8_t *)&dma_buf[1], (enc_words - 1) * 32);

        size_t stream_bytes = (w_bit + 7) / 8;

        // スキャナによる処理
        size_t scan_bit = 0;
        size_t sync_bit = 0;

        // 最初に見つかるシンクは偽フレーム (LEN=0)
        int find_res = bmc_find_sync_in_bits(stream, stream_bytes, scan_bit, w_bit, &sync_bit);
        CHECK_EQ_INT(find_res, BMC_OK);

        uint8_t bad_len = 0;
        int ext_res = bmc_extract_bits_as_bytes(stream, stream_bytes, sync_bit + 1, 1, &bad_len);
        CHECK_EQ_INT(ext_res, BMC_OK);
        CHECK_EQ_UINT(bad_len, 0); // 不正LEN検出！

        // 不正フレームと判定し、シンク直後から探索を再開
        scan_bit = sync_bit + 1;

        // 次のシンク (正常フレーム) をビット単位で検出
        find_res = bmc_find_sync_in_bits(stream, stream_bytes, scan_bit, w_bit, &sync_bit);
        CHECK_EQ_INT(find_res, BMC_OK);

        uint8_t rec_len = 0;
        ext_res = bmc_extract_bits_as_bytes(stream, stream_bytes, sync_bit + 1, 1, &rec_len);
        CHECK_EQ_INT(ext_res, BMC_OK);
        CHECK_EQ_UINT(rec_len, good_len);

        size_t total_bytes = bmc_packet_total_bytes(rec_len);
        uint8_t rec_frame[BMC_MAX_FRAME_BYTES];
        ext_res = bmc_extract_bits_as_bytes(stream, stream_bytes, sync_bit + 1, total_bytes, rec_frame);
        CHECK_EQ_INT(ext_res, BMC_OK);

        uint8_t type = 0, seq = 0, plen = 0;
        uint8_t pl[128];
        int dec_res = bmc_packet_decode(rec_frame, total_bytes, &type, &seq, pl, &plen);
        CHECK_EQ_INT(dec_res, (int)total_bytes);
        CHECK_EQ_UINT(type, BMC_PKT_TYPE_DATA);
        CHECK_EQ_UINT(seq, 42);
        CHECK_EQ_UINT(plen, good_len);
        CHECK_EQ_INT(memcmp(pl, good_pl, good_len), 0);

        printf("[TEST 7] Successfully recovered and decoded valid frame on the very next frame! [PASS]\n");
    }

    // =========================================================================
    // Test 8: Table-driven CRC16 vs Bitwise Reference Cross-Verification
    // =========================================================================
    {
        printf("[TEST 8] Cross-verifying table-driven CRC16 vs bitwise reference...\n");
        const size_t test_lengths[] = {0, 1, 2, 3, 7, 8, 16, 31, 32, 64, 127, 128, 255, 512, 1024};
        const size_t num_lens = sizeof(test_lengths) / sizeof(test_lengths[0]);
        uint8_t rand_buf[1024];

        // LCG による決定論的擬似乱数 (再現性確保)
        for (uint32_t seed = 1; seed <= 20; ++seed) {
            uint32_t lcg = seed * 1664525u + 1013904223u;
            for (size_t i = 0; i < sizeof(rand_buf); ++i) {
                lcg = lcg * 1664525u + 1013904223u;
                rand_buf[i] = (uint8_t)(lcg >> 24);
            }

            for (size_t li = 0; li < num_lens; ++li) {
                size_t len = test_lengths[li];

                // 1. 一括計算の比較
                // ローカルのビット単位リファレンス計算
                uint16_t ref_crc = 0xFFFF;
                for (size_t b = 0; b < len; ++b) {
                    ref_crc ^= (uint16_t)rand_buf[b] << 8;
                    for (int bit = 0; bit < 8; ++bit) {
                        if (ref_crc & 0x8000) {
                            ref_crc = (ref_crc << 1) ^ 0x1021;
                        } else {
                            ref_crc = ref_crc << 1;
                        }
                    }
                }
                uint16_t tbl_crc = crc16_ccitt(rand_buf, len);
                CHECK_EQ_UINT(tbl_crc, ref_crc);

                // 2. 逐次アップデート (crc16_ccitt_update) の比較
                if (len >= 3) {
                    size_t s1 = len / 3;
                    size_t s2 = (len * 2) / 3;

                    uint16_t inc_ref = 0xFFFF;
                    for (size_t b = 0; b < s1; ++b) {
                        inc_ref ^= (uint16_t)rand_buf[b] << 8;
                        for (int bit = 0; bit < 8; ++bit) inc_ref = (inc_ref & 0x8000) ? ((inc_ref << 1) ^ 0x1021) : (inc_ref << 1);
                    }
                    for (size_t b = s1; b < s2; ++b) {
                        inc_ref ^= (uint16_t)rand_buf[b] << 8;
                        for (int bit = 0; bit < 8; ++bit) inc_ref = (inc_ref & 0x8000) ? ((inc_ref << 1) ^ 0x1021) : (inc_ref << 1);
                    }
                    for (size_t b = s2; b < len; ++b) {
                        inc_ref ^= (uint16_t)rand_buf[b] << 8;
                        for (int bit = 0; bit < 8; ++bit) inc_ref = (inc_ref & 0x8000) ? ((inc_ref << 1) ^ 0x1021) : (inc_ref << 1);
                    }

                    uint16_t inc_tbl = 0xFFFF;
                    inc_tbl = crc16_ccitt_update(inc_tbl, rand_buf, s1);
                    inc_tbl = crc16_ccitt_update(inc_tbl, rand_buf + s1, s2 - s1);
                    inc_tbl = crc16_ccitt_update(inc_tbl, rand_buf + s2, len - s2);

                    CHECK_EQ_UINT(inc_tbl, inc_ref);
                    CHECK_EQ_UINT(inc_tbl, tbl_crc);
                }
            }
        }
        printf("[TEST 8] Table-driven CRC16 matched bitwise reference across all seeds and lengths! [PASS]\n");
    }

    // =========================================================================
    // Test 9: Postamble Verification for RX ISR Bit Residual Immunity (B-3案b)
    // =========================================================================
    {
        printf("[TEST 9] Testing postamble presence, word count increase, and decode transparency...\n");
        const uint8_t test_lens[] = {1, 2, 3, 4, 7, 8, 15, 16, 31, 32, 64, 127, 128};
        const size_t num_lens = sizeof(test_lens) / sizeof(test_lens[0]);

        for (size_t li = 0; li < num_lens; ++li) {
            uint8_t len = test_lens[li];
            uint8_t payload[BMC_MAX_PAYLOAD_LEN];
            for (int i = 0; i < len; ++i) {
                payload[i] = (uint8_t)(0x55 + i * 3);
            }

            memset(dma_buf, 0x00, sizeof(dma_buf));
            size_t dma_words = bmc_packet_encode(dma_buf, BMC_PKT_TYPE_BENCH, (uint8_t)li, payload, len);
            CHECK_TRUE(dma_words > 0);

            // 1. ワード数検証: 修正前総データワード数 + BMC_POSTAMBLE_WORDS + 1 (ヘッダワード)
            size_t legacy_total_data_words = BMC_PREAMBLE_SYNC_WORDS + (bmc_packet_total_bytes(len) / 4);
            size_t expected_dma_words = legacy_total_data_words + BMC_POSTAMBLE_WORDS + 1;
            CHECK_EQ_UINT(dma_words, expected_dma_words);

            // 2. dma_buf[0] (ワード0) が (総データワード数 - 1) と厳密に一致
            uint32_t w0 = __builtin_bswap32(dma_buf[0]);
            size_t total_data_words = dma_words - 1;
            CHECK_EQ_UINT(w0, (uint32_t)(total_data_words - 1));

            // 3. エンコードされたバッファ末尾4バイト (ポストアンブル領域) が全て 0xFF
            const uint8_t *wire_bytes = (const uint8_t *)&dma_buf[1];
            size_t total_wire_bytes = total_data_words * 4;
            for (size_t p = 0; p < BMC_POSTAMBLE_BYTES; ++p) {
                size_t postamble_idx = total_wire_bytes - BMC_POSTAMBLE_BYTES + p;
                CHECK_EQ_UINT(wire_bytes[postamble_idx], 0xFF);
            }

            // 4. bmc_packet_decode() の透過性検証: ポストアンブルを含んだ拡張バッファを渡しても、
            //    LEN由来の実フレーム長分のみを消費して正しくデコード成功すること
            const uint8_t *post_sync_frame = &wire_bytes[BMC_PREAMBLE_SYNC_BYTES];
            size_t post_sync_wire_bytes = (total_data_words - BMC_PREAMBLE_SYNC_WORDS) * 4; // ポストアンブル含む
            size_t expected_frame_bytes = bmc_packet_total_bytes(len);
            uint8_t dec_type = 0, dec_seq = 0, dec_len = 0;
            uint8_t dec_pl[BMC_MAX_PAYLOAD_LEN];
            memset(dec_pl, 0, sizeof(dec_pl));

            int dec_res = bmc_packet_decode(post_sync_frame, post_sync_wire_bytes,
                                            &dec_type, &dec_seq, dec_pl, &dec_len);
            CHECK_EQ_INT(dec_res, (int)expected_frame_bytes);
            CHECK_EQ_UINT(dec_type, BMC_PKT_TYPE_BENCH);
            CHECK_EQ_UINT(dec_seq, (uint8_t)li);
            CHECK_EQ_UINT(dec_len, len);
            CHECK_EQ_INT(memcmp(payload, dec_pl, len), 0);
        }

        // 5. ビットストリームレベルでの autopush (32bit) 境界保証のシミュレーション検証
        for (size_t residual = 0; residual < 32; ++residual) {
            size_t bits_before_postamble = residual + 136 * 8; // 最大フレーム長
            size_t pushes_before = bits_before_postamble / 32;
            size_t bits_after_postamble = bits_before_postamble + (BMC_POSTAMBLE_BYTES * 8);
            size_t pushes_after = bits_after_postamble / 32;
            CHECK_TRUE(pushes_after > pushes_before);
        }

        printf("[TEST 9] Postamble validation passed on all lengths and autopush boundaries! [PASS]\n");
    }

    printf("\n>>> ALL HOST UNIT TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
