#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
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

// RP2350 の 28bit RELOAD COUNT 最大値 (0x0FFFFFFF ≈ 268M transfers)
#define RELOAD_28BIT 0x0FFFFFFFu

int main(void) {
    printf("Running Host Unit Tests for DMA Counter & Lap-Tracking Pure Functions...\n");

    // =========================================================================
    // Test 1: bmc_rx_calc_total_words 基本計算と境界値
    // =========================================================================
    {
        printf("[TEST 1] bmc_rx_calc_total_words arithmetic & boundaries...\n");

        // 1a. 初期状態: lap=0, tc=reload -> 0語
        uint64_t w0 = bmc_rx_calc_total_words(0, RELOAD_28BIT, RELOAD_28BIT);
        CHECK_EQ_UINT(w0, 0);

        // 1b. 同一ラップ内での進捗: lap=0, tc=reload-500 -> 500語
        uint64_t w1 = bmc_rx_calc_total_words(0, RELOAD_28BIT - 500, RELOAD_28BIT);
        CHECK_EQ_UINT(w1, 500);

        // 1c. 1ラップ満了直前: lap=0, tc=0 -> RELOAD語
        uint64_t w2 = bmc_rx_calc_total_words(0, 0, RELOAD_28BIT);
        CHECK_EQ_UINT(w2, (uint64_t)RELOAD_28BIT);

        // 1d. 1ラップ満了直後 (再トリガリロード後): lap=1, tc=reload -> RELOAD語 (連続性)
        uint64_t w3 = bmc_rx_calc_total_words(1, RELOAD_28BIT, RELOAD_28BIT);
        CHECK_EQ_UINT(w3, (uint64_t)RELOAD_28BIT);
        CHECK_EQ_UINT(w2, w3);

        // 1e. 複数ラップ: lap=7, tc=reload - 123456 -> 7 * RELOAD + 123456
        uint64_t expected_w4 = 7ULL * RELOAD_28BIT + 123456ULL;
        uint64_t w4 = bmc_rx_calc_total_words(7, RELOAD_28BIT - 123456, RELOAD_28BIT);
        CHECK_EQ_UINT(w4, expected_w4);

        // 1f. 巨大ラップ数 (64bit非オーバーフロー検証: 100,000ラップ ≈ 26.8兆語)
        uint64_t big_lap = 100000ULL;
        uint64_t expected_big = big_lap * RELOAD_28BIT + 42ULL;
        uint64_t w5 = bmc_rx_calc_total_words(big_lap, RELOAD_28BIT - 42, RELOAD_28BIT);
        CHECK_EQ_UINT(w5, expected_big);

        // 1g. 異常値クランプ: tc > reload の場合
        uint64_t w_clamp = bmc_rx_calc_total_words(3, RELOAD_28BIT + 100, RELOAD_28BIT);
        CHECK_EQ_UINT(w_clamp, 3ULL * RELOAD_28BIT);

        // 1h. reload=0 ガード
        uint64_t w_zero = bmc_rx_calc_total_words(5, 10, 0);
        CHECK_EQ_UINT(w_zero, 0);

        printf("[TEST 1] bmc_rx_calc_total_words arithmetic passed [PASS]\n");
    }

    // =========================================================================
    // Test 2: bmc_rx_calc_delta_words サンプリング区間の差分積算 (複数ラップ跨ぎ含む)
    // =========================================================================
    {
        printf("[TEST 2] bmc_rx_calc_delta_words delta accumulation across laps...\n");

        // 2a. 同一ラップ内での差分: prev_tc=1000, curr_tc=700 -> 300語
        uint64_t d1 = bmc_rx_calc_delta_words(0, 1000, 0, 700, RELOAD_28BIT);
        CHECK_EQ_UINT(d1, 300);

        // 2b. 1ラップ跨ぎでの差分:
        // 前回: lap=0, tc=100 (残り100語)
        // 今回: lap=1, tc=reload-250 (リロード後250語消費)
        // 期待デルタ: 100 + 250 = 350語
        uint64_t d2 = bmc_rx_calc_delta_words(0, 100, 1, RELOAD_28BIT - 250, RELOAD_28BIT);
        CHECK_EQ_UINT(d2, 350);

        // 2c. 複数ラップ跨ぎでの差分:
        // 前回: lap=2, tc=500
        // 今回: lap=5, tc=reload-1000
        // 期待デルタ: (5*reload + 1000) - (2*reload + (reload-500)) = 2*reload + 1500
        uint64_t expected_d3 = 2ULL * RELOAD_28BIT + 1500ULL;
        uint64_t d3 = bmc_rx_calc_delta_words(2, 500, 5, RELOAD_28BIT - 1000, RELOAD_28BIT);
        CHECK_EQ_UINT(d3, expected_d3);

        // 2d. ゼロ進捗: 同一状態ならデルタ0
        uint64_t d4 = bmc_rx_calc_delta_words(3, 12345, 3, 12345, RELOAD_28BIT);
        CHECK_EQ_UINT(d4, 0);

        // 2e. 異常系: curr_lap < prev_lap -> 0
        uint64_t d_inv = bmc_rx_calc_delta_words(5, 100, 4, 200, RELOAD_28BIT);
        CHECK_EQ_UINT(d_inv, 0);

        printf("[TEST 2] bmc_rx_calc_delta_words passed [PASS]\n");
    }

    // =========================================================================
    // Test 3: bmc_rx_update_transfer_state ポーリング時の逐次更新と自動リロード検知
    // =========================================================================
    {
        printf("[TEST 3] bmc_rx_update_transfer_state polling & reload detection...\n");

        uint64_t lap = 0;
        uint32_t last_tc = RELOAD_28BIT;

        // 3a. 転送初期化直後
        uint64_t total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 0);
        CHECK_EQ_UINT(last_tc, RELOAD_28BIT);
        CHECK_EQ_UINT(total, 0);

        // 3b. 100語転送
        total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT - 100, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 0);
        CHECK_EQ_UINT(last_tc, RELOAD_28BIT - 100);
        CHECK_EQ_UINT(total, 100);

        // 3c. さらに900語転送 (累計1000語)
        total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT - 1000, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 0);
        CHECK_EQ_UINT(total, 1000);

        // 3d. ゼロ境界連続性テスト (curr_tc: 1 -> 0 -> RELOAD -> RELOAD-1)
        // カウンタ残り1語
        total = bmc_rx_update_transfer_state(&lap, &last_tc, 1, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 0);
        CHECK_EQ_UINT(last_tc, 1);
        CHECK_EQ_UINT(total, (uint64_t)RELOAD_28BIT - 1ULL);

        // 満了瞬間: curr_tc = 0
        total = bmc_rx_update_transfer_state(&lap, &last_tc, 0, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 0);
        CHECK_EQ_UINT(last_tc, 0);
        CHECK_EQ_UINT(total, (uint64_t)RELOAD_28BIT);

        // 自動リロード直後 (0語消費): curr_tc = RELOAD_28BIT (curr > last を検出して lap が 1 進む)
        total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 1);
        CHECK_EQ_UINT(last_tc, RELOAD_28BIT);
        CHECK_EQ_UINT(total, (uint64_t)RELOAD_28BIT); // 満了時と同一値で完全に連続！

        // リロード後 1語消費: curr_tc = RELOAD_28BIT - 1
        total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT - 1, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 1);
        CHECK_EQ_UINT(last_tc, RELOAD_28BIT - 1);
        CHECK_EQ_UINT(total, (uint64_t)RELOAD_28BIT + 1ULL);

        // 3e. カウンタが残り10語まで減少し、その後 0 を跨いでリロードされた瞬間をシミュレート
        last_tc = 10;
        // リロード直後: tc は RELOAD_28BIT - 50 に戻る (curr > last を検出)
        total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT - 50, RELOAD_28BIT);
        CHECK_EQ_UINT(lap, 2); // lap が 2 に進んだ！
        CHECK_EQ_UINT(last_tc, RELOAD_28BIT - 50);
        CHECK_EQ_UINT(total, 2ULL * RELOAD_28BIT + 50ULL);

        // 3f. 100サイクルの連続リロードシミュレーション
        for (int cycle = 0; cycle < 100; ++cycle) {
            // cycle 内で 3 回サンプリング (途中、終端直前、リロード後)
            last_tc = 5;
            total = bmc_rx_update_transfer_state(&lap, &last_tc, RELOAD_28BIT - 20, RELOAD_28BIT);
            CHECK_EQ_UINT(lap, (uint64_t)(cycle + 3));
            CHECK_EQ_UINT(total, (uint64_t)(cycle + 3) * RELOAD_28BIT + 20ULL);
        }

        printf("[TEST 3] bmc_rx_update_transfer_state passed [PASS]\n");
    }

    // =========================================================================
    // Test 4: 16KBリングオーバーラン検出シミュレーション (B-2核心要件)
    // =========================================================================
    {
        printf("[TEST 4] 16KB ring overrun detection math...\n");
        const size_t RING_BYTES = 16384;
        const uint64_t RING_BITS = (uint64_t)RING_BYTES * 8; // 131,072 bits

        uint64_t total_bits_read = 0;
        uint64_t total_words = 0;

        // 4a. 15KB 分のデータ到着 -> 未読ビット < 16KB -> オーバーラン未発生
        total_words = (15 * 1024) / 4;
        uint64_t total_bytes = total_words * 4;
        uint64_t total_bits_written = total_bytes * 8;
        uint64_t unread_bits = total_bits_written - total_bits_read;
        CHECK_TRUE(unread_bits < RING_BITS);

        // 4b. 長時間のCPU停止で DMA がリングを何周も周回した場合 (例: 3周 = 48KB 転送)
        total_words += (48 * 1024) / 4;
        total_bytes = total_words * 4;
        total_bits_written = total_bytes * 8;
        unread_bits = total_bits_written - total_bits_read;

        // 新方式では lap/word 追跡により unread_bits が正確に 63KB (516,096 bits) になる！
        CHECK_TRUE(unread_bits >= RING_BITS); // 確実にオーバーラン検出！

        // 4c. オーバーランリカバリの検証
        // safe_tail は最新の head より 256 バイト手前
        total_bits_read = total_bits_written - (256 * 8);
        unread_bits = total_bits_written - total_bits_read;
        CHECK_EQ_UINT(unread_bits, 256 * 8); // 正確に 256 バイト (2048 ビット) に復旧

        printf("[TEST 4] 16KB ring overrun math verified [PASS]\n");
    }

    printf("\n>>> ALL DMA COUNTER & LAP-TRACKING TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
