#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include "../common/ring_cursor.h"

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

int main(void) {
    printf("Running Host Unit Tests for Ring Cursor / Overrun Math...\n");

    const size_t RING_CAP = 1024;
    static uint8_t ring_buf[1024];
    ring_cursor_t rc;

    // =========================================================================
    // Test 1: Empty Ring Initialization
    // =========================================================================
    {
        ring_cursor_init(&rc, RING_CAP);
        CHECK_EQ_UINT(rc.capacity, RING_CAP);
        CHECK_EQ_UINT(rc.tail, 0);
        CHECK_EQ_UINT(rc.total_written, 0);
        CHECK_EQ_UINT(rc.total_read, 0);
        CHECK_EQ_UINT(rc.overrun_count, 0);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 0);
        printf("[TEST 1] Empty ring initialization [PASS]\n");
    }

    // =========================================================================
    // Test 2: Sequential Write & Read Without Wrap
    // =========================================================================
    {
        ring_cursor_reset(&rc);
        // シミュレーション: プロデューサが100バイト書き込み (head=100)
        for (size_t i = 0; i < 100; ++i) {
            ring_buf[i] = (uint8_t)(i + 1);
        }
        size_t avail = ring_cursor_update_head(&rc, 100);
        CHECK_EQ_UINT(avail, 100);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 100);
        CHECK_EQ_UINT(rc.overrun_count, 0);

        // Peek 50バイト
        uint8_t peek_buf[50];
        size_t peeked = ring_cursor_peek(&rc, ring_buf, peek_buf, 50);
        CHECK_EQ_UINT(peeked, 50);
        for (size_t i = 0; i < 50; ++i) {
            CHECK_EQ_UINT(peek_buf[i], (uint8_t)(i + 1));
        }
        CHECK_EQ_UINT(rc.tail, 0); // peek では tail は不変
        CHECK_EQ_UINT(ring_cursor_available(&rc), 100);

        // Read 50バイト (tail が進む)
        uint8_t read_buf[100];
        size_t read_cnt = ring_cursor_read(&rc, ring_buf, read_buf, 50);
        CHECK_EQ_UINT(read_cnt, 50);
        CHECK_EQ_UINT(rc.tail, 50);
        CHECK_EQ_UINT(rc.total_read, 50);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 50);

        // 残り50バイト読み出し
        read_cnt = ring_cursor_read(&rc, ring_buf, read_buf + 50, 50);
        CHECK_EQ_UINT(read_cnt, 50);
        CHECK_EQ_UINT(rc.tail, 100);
        CHECK_EQ_UINT(rc.total_read, 100);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 0);

        // 読み出した100バイトの内容検証
        for (size_t i = 0; i < 100; ++i) {
            CHECK_EQ_UINT(read_buf[i], (uint8_t)(i + 1));
        }
        printf("[TEST 2] Sequential write & read [PASS]\n");
    }

    // =========================================================================
    // Test 3: Ring Buffer Wrap-Around Across Boundary
    // =========================================================================
    {
        // tail を 1000 に進める
        ring_cursor_reset(&rc);
        ring_cursor_update_head(&rc, 1000);
        ring_cursor_advance(&rc, 1000);
        CHECK_EQ_UINT(rc.tail, 1000);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 0);

        // 1000 から 1024 境界をまたいで 100 バイト書き込み (head=76)
        // [1000..1023] (24B) + [0..75] (76B) = 100B
        for (size_t i = 0; i < 100; ++i) {
            size_t idx = (1000 + i) % RING_CAP;
            ring_buf[idx] = (uint8_t)(0x80 ^ i);
        }
        size_t avail = ring_cursor_update_head(&rc, 76);
        CHECK_EQ_UINT(avail, 100);
        CHECK_EQ_UINT(rc.overrun_count, 0);

        // 境界をまたぐ 100 バイトの一括読み出し
        uint8_t wrap_buf[100];
        size_t read_cnt = ring_cursor_read(&rc, ring_buf, wrap_buf, 100);
        CHECK_EQ_UINT(read_cnt, 100);
        CHECK_EQ_UINT(rc.tail, 76);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 0);

        for (size_t i = 0; i < 100; ++i) {
            CHECK_EQ_UINT(wrap_buf[i], (uint8_t)(0x80 ^ i));
        }
        printf("[TEST 3] Wrap-around read across boundary [PASS]\n");
    }

    // =========================================================================
    // Test 4: Full Lap (Exact Capacity Bytes Stored)
    // =========================================================================
    {
        ring_cursor_reset(&rc);
        // 1024 バイトまるまる書き込み (head=0, last_head=0 だが 1024 バイト差分)
        // 途中で 512, 1024 と進める
        ring_cursor_update_head(&rc, 512);
        size_t avail = ring_cursor_update_head(&rc, 0); // 512 -> 1024 (idx 0)
        CHECK_EQ_UINT(avail, 1024);
        CHECK_EQ_UINT(rc.overrun_count, 0);
        CHECK_EQ_UINT(rc.total_written, 1024);
        CHECK_EQ_UINT(rc.total_read, 0);
        printf("[TEST 4] Full lap exact capacity [PASS]\n");
    }

    // =========================================================================
    // Test 5: Producer Overtaking Consumer (Overrun Detection & Recovery)
    // =========================================================================
    {
        ring_cursor_reset(&rc);
        // まず 800 バイト書き込み
        ring_cursor_update_head(&rc, 800);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 800);

        // コンシューマは未読のまま、プロデューサがさらに 500 バイト書き込み (計1300B > 1024B)
        // head: (800 + 500) % 1024 = 276
        size_t avail = ring_cursor_update_head(&rc, 276);

        // オーバーランが検知され、overrun_count が 1 に増えることを検証
        CHECK_EQ_UINT(rc.overrun_count, 1);
        // 安全マージン (capacity/4 = 256B) 手前にリカバリされることを検証
        CHECK_EQ_UINT(avail, 256);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 256);

        // tail が head (276) の 256 バイト手前 (20) にあることを検証
        CHECK_EQ_UINT(rc.tail, 20);
        // total_written - total_read が 256 であることを検証
        CHECK_EQ_UINT(rc.total_written - rc.total_read, 256);

        // 再度オーバーランを発生させてカウンタが正確にインクリメントされるか検証
        ring_cursor_update_head(&rc, (276 + 800) % 1024);
        CHECK_EQ_UINT(rc.overrun_count, 2);

        printf("[TEST 5] Producer overtaking consumer (overrun detection & recovery) [PASS]\n");
    }

    // =========================================================================
    // Test 6: Advance Guard Beyond Available
    // =========================================================================
    {
        ring_cursor_reset(&rc);
        ring_cursor_update_head(&rc, 50);
        // 50バイトしかないのに100バイトadvance要求
        ring_cursor_advance(&rc, 100);
        CHECK_EQ_UINT(rc.tail, 50); // 50でクランプ
        CHECK_EQ_UINT(rc.total_read, 50);
        CHECK_EQ_UINT(ring_cursor_available(&rc), 0);
        printf("[TEST 6] Advance clamped to available [PASS]\n");
    }

    printf("\n>>> ALL RING CURSOR TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
