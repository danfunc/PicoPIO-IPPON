#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include "../common/crc16.h"
#include "../common/packet.h"

// ============================================================================
// H-5 Diagnostic Assertion Macros (test_packet.c と同一の作法)
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

#define RING_BYTES 1024

// フレームをビット単位でリングバッファへ書き込む (base_bit から開始し、末尾は境界を跨いでラップする)
static void write_bits_to_ring_sz(uint8_t *ring, size_t ring_bytes, size_t base_bit, const uint8_t *src_bytes, size_t num_bits) {
    for (size_t i = 0; i < num_bits; ++i) {
        uint8_t src_byte = src_bytes[i / 8];
        uint8_t bit = (src_byte >> (7 - (i % 8))) & 1;
        size_t pos = (base_bit + i) % (ring_bytes * 8);
        size_t byte_idx = pos / 8;
        size_t bit_idx = pos % 8;
        if (bit) {
            ring[byte_idx] |= (uint8_t)(1u << (7 - bit_idx));
        } else {
            ring[byte_idx] &= (uint8_t)~(1u << (7 - bit_idx));
        }
    }
}

static void write_bits_to_ring(uint8_t *ring, size_t base_bit, const uint8_t *src_bytes, size_t num_bits) {
    write_bits_to_ring_sz(ring, RING_BYTES, base_bit, src_bytes, num_bits);
}

// 1フレーム分の生ワイヤバイト列 (プリアンブル2B + シンク2B + ポストシンクフレーム) を生成する
static size_t build_wire_frame(uint8_t *wire_out, uint8_t type, uint8_t seq,
                               const uint8_t *payload, uint8_t len) {
    static uint32_t dma_buf[BMC_TX_DMA_MAX_WORDS];
    size_t enc_words = bmc_packet_encode(dma_buf, type, seq, payload, len);
    CHECK_TRUE(enc_words > 0);
    const uint8_t *wire_bytes = (const uint8_t *)&dma_buf[1];
    // ポストアンブル(末尾4B)を除いた「シンク直後からCRC末尾を含むフレーム全体」までを使う。
    // bmc_rx_extract_frame_from_ring() は needed_bytes = bmc_packet_total_bytes(len) 分しか
    // 消費しないため、ポストアンブルは同期後方に残ってもテストの受理判定には影響しない。
    size_t post_sync_bytes = bmc_packet_total_bytes(len);
    size_t total_bytes = BMC_PREAMBLE_SYNC_BYTES + post_sync_bytes;
    memcpy(wire_out, wire_bytes, total_bytes);
    return total_bytes; // プリアンブル(2)+シンク(2)+ポストシンクフレーム(post_sync_bytes)
}

int main(void) {
    printf("Running Host Unit Tests for RX Extract Range Calculation Core...\n");

    // =========================================================================
    // Test 1: 全32通りのワード位相 (0..31 ビット) でフレーム先頭をずらしても受理できる
    // =========================================================================
    {
        printf("[TEST 1] 32-way word-phase alignment (bit_start 0..31 within a 32bit word)...\n");
        uint8_t payload[16];
        for (int i = 0; i < 16; ++i) payload[i] = (uint8_t)(0x40 + i);

        for (size_t phase = 0; phase < 32; ++phase) {
            static uint8_t ring[RING_BYTES];
            memset(ring, 0xFF, sizeof(ring)); // アイドル(1)で初期化

            uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
            size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_DATA, (uint8_t)(phase + 1), payload, 16);
            size_t wire_bits = wire_bytes * 8;

            size_t tail_bit = phase; // リング先頭からphaseビットずらした位置がtail
            write_bits_to_ring(ring, tail_bit, wire, wire_bits);

            bmc_rx_frame_t frame;
            size_t advance_bits = 0;
            bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                ring, RING_BYTES, tail_bit, wire_bits, &frame, &advance_bits);

            CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
            CHECK_EQ_UINT(frame.type, BMC_PKT_TYPE_DATA);
            CHECK_EQ_UINT(frame.seq, (uint8_t)(phase + 1));
            CHECK_EQ_UINT(frame.len, 16);
            CHECK_EQ_INT(memcmp(frame.payload, payload, 16), 0);
            CHECK_EQ_UINT(advance_bits, wire_bits);
        }
        printf("[TEST 1] All 32 word phases accepted correctly [PASS]\n");
    }

    // =========================================================================
    // Test 2: LEN 1..128 の全長で受理できる (bit_start = 3 で固定し非バイト整列を確認)
    // =========================================================================
    {
        printf("[TEST 2] All LEN 1..128 accepted at a non-byte-aligned tail_bit...\n");
        const size_t bit_start = 3;
        uint8_t payload[BMC_MAX_PAYLOAD_LEN];

        for (int len = 1; len <= BMC_MAX_PAYLOAD_LEN; ++len) {
            for (int i = 0; i < len; ++i) payload[i] = (uint8_t)(0x77 ^ (i * 5) ^ len);

            static uint8_t ring[RING_BYTES];
            memset(ring, 0xFF, sizeof(ring));

            uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
            size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_BENCH, (uint8_t)len, payload, (uint8_t)len);
            size_t wire_bits = wire_bytes * 8;

            write_bits_to_ring(ring, bit_start, wire, wire_bits);

            bmc_rx_frame_t frame;
            size_t advance_bits = 0;
            bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                ring, RING_BYTES, bit_start, wire_bits, &frame, &advance_bits);

            CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
            CHECK_EQ_UINT(frame.len, (uint8_t)len);
            CHECK_EQ_INT(memcmp(frame.payload, payload, len), 0);
        }
        printf("[TEST 2] All 128 payload lengths accepted [PASS]\n");
    }

    // =========================================================================
    // Test 3: ちょうど必要な分だけ届いた時点で即座に受理できる。1ビット足りなければ不可
    //         (欠陥1の直接回帰テスト: 非バイト整列 bit_start != 0 で検証)
    // =========================================================================
    {
        printf("[TEST 3] Exact-arrival acceptance vs one-bit-short rejection (defect 1 regression)...\n");
        const size_t bit_starts[] = {0, 1, 3, 6, 7};
        const uint8_t test_lens[] = {1, 5, 16, 64, 128};

        for (size_t bsi = 0; bsi < sizeof(bit_starts) / sizeof(bit_starts[0]); ++bsi) {
            size_t bit_start = bit_starts[bsi];
            for (size_t li = 0; li < sizeof(test_lens) / sizeof(test_lens[0]); ++li) {
                uint8_t len = test_lens[li];
                uint8_t payload[BMC_MAX_PAYLOAD_LEN];
                for (int i = 0; i < len; ++i) payload[i] = (uint8_t)(0x11 + i);

                static uint8_t ring[RING_BYTES];
                memset(ring, 0xFF, sizeof(ring));

                uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
                size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_DATA, 0x55, payload, len);
                size_t wire_bits = wire_bytes * 8;

                write_bits_to_ring(ring, bit_start, wire, wire_bits);

                // 3a. ちょうど必要な分 (wire_bits) だけ届いた場合 -> 即座に受理
                {
                    bmc_rx_frame_t frame;
                    size_t advance_bits = 0;
                    bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                        ring, RING_BYTES, bit_start, wire_bits, &frame, &advance_bits);
                    CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
                    CHECK_EQ_UINT(frame.len, len);
                    CHECK_EQ_UINT(advance_bits, wire_bits);
                }

                // 3b. 1ビット足りない場合 -> 受理してはならない (FRAME_OKを返さない)
                {
                    bmc_rx_frame_t frame;
                    size_t advance_bits = 0;
                    bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                        ring, RING_BYTES, bit_start, wire_bits - 1, &frame, &advance_bits);
                    CHECK_TRUE(st != BMC_RX_EXTRACT_FRAME_OK);
                }
            }
        }
        printf("[TEST 3] Exact-arrival accepted, one-bit-short correctly rejected for all cases [PASS]\n");
    }

    // =========================================================================
    // Test 4: 途中到着 (フレームの一部しか届いていない) は受理されない
    // =========================================================================
    {
        printf("[TEST 4] Partial arrival (mid-frame) is never accepted...\n");
        uint8_t payload[32];
        for (int i = 0; i < 32; ++i) payload[i] = (uint8_t)(0x90 + i);

        static uint8_t ring[RING_BYTES];
        memset(ring, 0xFF, sizeof(ring));

        uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
        size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_DATA, 0x77, payload, 32);
        size_t wire_bits = wire_bytes * 8;

        const size_t bit_start = 5;
        write_bits_to_ring(ring, bit_start, wire, wire_bits);

        // シンク+LENしか届いていない状態から、フレーム末尾直前まで、複数の途中到着量を確認
        for (size_t got = 24; got < wire_bits; got += 17) {
            bmc_rx_frame_t frame;
            size_t advance_bits = 0;
            bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                ring, RING_BYTES, bit_start, got, &frame, &advance_bits);
            CHECK_TRUE(st != BMC_RX_EXTRACT_FRAME_OK);
        }
        printf("[TEST 4] All partial-arrival amounts correctly withheld [PASS]\n");
    }

    // =========================================================================
    // Test 5: リングの折り返しをまたぐフレーム
    // =========================================================================
    {
        printf("[TEST 5] Frame spanning the ring wrap-around boundary...\n");
        uint8_t payload[64];
        for (int i = 0; i < 64; ++i) payload[i] = (uint8_t)(0xC0 ^ i);

        static uint8_t ring[RING_BYTES];
        memset(ring, 0xFF, sizeof(ring));

        uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
        size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_DATA, 0xAB, payload, 64);
        size_t wire_bits = wire_bytes * 8;

        // tail_bit をリング末尾付近に置き、フレームがバイト0へ折り返すようにする
        size_t tail_bit = (RING_BYTES - 3) * 8 + 4; // 末尾3バイト+4ビット残しの位置、折り返し確実
        write_bits_to_ring(ring, tail_bit, wire, wire_bits);

        bmc_rx_frame_t frame;
        size_t advance_bits = 0;
        bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
            ring, RING_BYTES, tail_bit, wire_bits, &frame, &advance_bits);

        CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
        CHECK_EQ_UINT(frame.type, BMC_PKT_TYPE_DATA);
        CHECK_EQ_UINT(frame.seq, 0xAB);
        CHECK_EQ_UINT(frame.len, 64);
        CHECK_EQ_INT(memcmp(frame.payload, payload, 64), 0);
        CHECK_EQ_UINT(advance_bits, wire_bits);
        printf("[TEST 5] Ring wrap-around frame decoded correctly [PASS]\n");
    }

    // =========================================================================
    // Test 6: 欠陥2の直接回帰テスト — tail_bit がシンク語先頭に既に到達した状態で、
    //         LEN=1 のフレームに真に必要な最小ビット数 (シンク16+最小フレーム64=80ビット)
    //         しか届いていなくても受理できる (旧実装は12バイト=96ビット閾値で永久に待ち続けた)。
    //         プリアンブル(2B)は tail より前に既読み終えた想定でリングへは含めない。
    // =========================================================================
    {
        printf("[TEST 6] Sync-aligned tail with only the true minimum (80 bits) available (defect 2 regression)...\n");
        uint8_t payload[1] = {0x5A};

        static uint8_t ring[RING_BYTES];
        memset(ring, 0xFF, sizeof(ring));

        uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
        size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_ECHO_REQ, 0x01, payload, 1);

        // プリアンブル2バイトを除いた「シンク語 + ポストシンクフレーム」のみをリングへ書く
        const uint8_t *sync_onward = wire + 2;
        size_t sync_onward_bytes = wire_bytes - 2;
        size_t sync_onward_bits = sync_onward_bytes * 8;

        // 真に必要な最小ビット数 (16+64=80) と一致し、旧閾値 96 より確実に少ないことを保証
        CHECK_EQ_UINT(sync_onward_bits, 80);
        CHECK_TRUE(sync_onward_bits < 96);

        const size_t bit_start = 0; // tail_bit はシンク語の先頭に既に位置している想定
        write_bits_to_ring(ring, bit_start, sync_onward, sync_onward_bits);

        bmc_rx_frame_t frame;
        size_t advance_bits = 0;
        bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
            ring, RING_BYTES, bit_start, sync_onward_bits, &frame, &advance_bits);

        CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
        CHECK_EQ_UINT(frame.len, 1);
        CHECK_EQ_UINT(frame.payload[0], 0x5A);
        CHECK_EQ_UINT(advance_bits, sync_onward_bits);
        printf("[TEST 6] LEN=1 frame accepted with exactly the true minimum 80 bits (no follow-up data) [PASS]\n");

        // 79ビットしかない場合は受理してはならない
        {
            bmc_rx_frame_t frame2;
            size_t advance_bits2 = 0;
            bmc_rx_extract_status_t st2 = bmc_rx_extract_frame_from_ring(
                ring, RING_BYTES, bit_start, sync_onward_bits - 1, &frame2, &advance_bits2);
            CHECK_TRUE(st2 != BMC_RX_EXTRACT_FRAME_OK);
        }
    }

    // =========================================================================
    // Test 7: データが全く足りない場合 (シンク16ビット未満等) は INCOMPLETE、advance_bits=0
    // =========================================================================
    {
        printf("[TEST 7] Not-enough-data-at-all yields INCOMPLETE with zero advance...\n");
        static uint8_t ring[RING_BYTES];
        memset(ring, 0xFF, sizeof(ring));

        bmc_rx_frame_t frame;
        size_t advance_bits = 999;
        bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
            ring, RING_BYTES, 0, 10, &frame, &advance_bits);
        CHECK_EQ_INT(st, BMC_RX_EXTRACT_INCOMPLETE);
        CHECK_EQ_UINT(advance_bits, 0);
        printf("[TEST 7] Tiny unread_bits correctly withheld without advancing [PASS]\n");
    }

    // =========================================================================
    // Test 8: 本番相当 16384 バイト (16KB) リングバッファでのフレーム抽出と境界ラップ
    // =========================================================================
    {
        printf("[TEST 8] 16384-byte (16KB) ring buffer acceptance and boundary wrap...\n");
        const size_t RING_16K = 16384;
        static uint8_t ring16k[16384];

        // 8a. 通常位置 (tail_bit = 4096B+5b) での抽出
        {
            memset(ring16k, 0xFF, sizeof(ring16k));
            uint8_t payload[64];
            for (int i = 0; i < 64; ++i) payload[i] = (uint8_t)(0x33 + i);

            uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
            size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_DATA, 0x42, payload, 64);
            size_t wire_bits = wire_bytes * 8;

            size_t tail_bit = 4096 * 8 + 5; // オフセット4096バイト + 5ビット
            write_bits_to_ring_sz(ring16k, RING_16K, tail_bit, wire, wire_bits);

            bmc_rx_frame_t frame;
            size_t advance_bits = 0;
            bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                ring16k, RING_16K, tail_bit, wire_bits, &frame, &advance_bits);

            CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
            CHECK_EQ_UINT(frame.type, BMC_PKT_TYPE_DATA);
            CHECK_EQ_UINT(frame.seq, 0x42);
            CHECK_EQ_UINT(frame.len, 64);
            CHECK_EQ_INT(memcmp(frame.payload, payload, 64), 0);
            CHECK_EQ_UINT(advance_bits, wire_bits);
        }

        // 8b. 16KBリング終端 (16384バイト境界) を跨ぐ折り返しフレームの抽出 (最大128B)
        {
            memset(ring16k, 0xFF, sizeof(ring16k));
            uint8_t payload[128];
            for (int i = 0; i < 128; ++i) payload[i] = (uint8_t)(0xAA ^ (i * 3));

            uint8_t wire[BMC_PREAMBLE_SYNC_BYTES + BMC_MAX_FRAME_BYTES];
            size_t wire_bytes = build_wire_frame(wire, BMC_PKT_TYPE_BENCH, 0x99, payload, 128);
            size_t wire_bits = wire_bytes * 8;

            // 16KBリングの終端から 50バイト手前の非バイト整列位置に配置
            size_t tail_bit = (RING_16K - 50) * 8 + 3;
            write_bits_to_ring_sz(ring16k, RING_16K, tail_bit, wire, wire_bits);

            bmc_rx_frame_t frame;
            size_t advance_bits = 0;
            bmc_rx_extract_status_t st = bmc_rx_extract_frame_from_ring(
                ring16k, RING_16K, tail_bit, wire_bits, &frame, &advance_bits);

            CHECK_EQ_INT(st, BMC_RX_EXTRACT_FRAME_OK);
            CHECK_EQ_UINT(frame.type, BMC_PKT_TYPE_BENCH);
            CHECK_EQ_UINT(frame.seq, 0x99);
            CHECK_EQ_UINT(frame.len, 128);
            CHECK_EQ_INT(memcmp(frame.payload, payload, 128), 0);
            CHECK_EQ_UINT(advance_bits, wire_bits);
        }
        printf("[TEST 8] 16KB ring frame extraction & boundary wrap passed! [PASS]\n");
    }

    printf("\n>>> ALL RX EXTRACT RANGE-CALCULATION TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
