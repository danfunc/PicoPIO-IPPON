#ifndef RING_CURSOR_H
#define RING_CURSOR_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 純粋リングバッファ・カーソル管理 (ハードウェア非依存)
// ============================================================================
typedef struct {
    size_t capacity;              // バッファ容量 (2のべき乗必須)
    size_t capacity_mask;         // capacity - 1
    size_t last_head;             // 直前の書き込みヘッド位置 (0..capacity-1)
    uint64_t total_written;       // 累計書き込みバイト数
    uint64_t total_read;          // 累計読み出しバイト数
    size_t tail;                  // 現在の読み出し位置 (0..capacity-1)
    uint32_t overrun_count;       // 検出されたオーバーラン回数
} ring_cursor_t;

void ring_cursor_init(ring_cursor_t *rc, size_t capacity);
void ring_cursor_reset(ring_cursor_t *rc);

// 書き込みヘッド位置(DMA等)を更新し、現在の未読バイト数を返す。オーバーラン検知時は自動復旧する
size_t ring_cursor_update_head(ring_cursor_t *rc, size_t current_head);

// 未読バイト数を取得 (最大 capacity)
size_t ring_cursor_available(const ring_cursor_t *rc);

// 指定バイト数を読み進める (tail および total_read を前進)
void ring_cursor_advance(ring_cursor_t *rc, size_t num_bytes);

// リングバッファからデータを覗き見 (tail は進めない)
size_t ring_cursor_peek(const ring_cursor_t *rc, const uint8_t *ring_buf, uint8_t *dest, size_t max_len);

// リングバッファからデータを読み出し (tail を進める)
size_t ring_cursor_read(ring_cursor_t *rc, const uint8_t *ring_buf, uint8_t *dest, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif // RING_CURSOR_H
