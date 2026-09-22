#include "ring_cursor.h"

void ring_cursor_init(ring_cursor_t *rc, size_t capacity) {
    if (!rc) return;
    rc->capacity = capacity;
    rc->capacity_mask = (capacity > 0) ? (capacity - 1) : 0;
    ring_cursor_reset(rc);
}

void ring_cursor_reset(ring_cursor_t *rc) {
    if (!rc) return;
    rc->last_head = 0;
    rc->total_written = 0;
    rc->total_read = 0;
    rc->tail = 0;
    rc->overrun_count = 0;
}

size_t ring_cursor_update_head(ring_cursor_t *rc, size_t current_head) {
    if (!rc || rc->capacity == 0) return 0;

    current_head &= rc->capacity_mask;
    size_t delta = (current_head >= rc->last_head) ?
                   (current_head - rc->last_head) :
                   (rc->capacity - rc->last_head + current_head);
    rc->last_head = current_head;
    rc->total_written += delta;

    uint64_t unread = (rc->total_written >= rc->total_read) ?
                      (rc->total_written - rc->total_read) : 0;

    if (unread > rc->capacity) {
        rc->overrun_count++;
        // オーバーラン復旧: tail を current_head より安全なマージン (capacity/4) 手前へ配置
        size_t safe_margin = rc->capacity / 4;
        if (safe_margin == 0) safe_margin = 1;
        rc->tail = (current_head >= safe_margin) ?
                   (current_head - safe_margin) :
                   (rc->capacity + current_head - safe_margin);
        rc->total_read = rc->total_written - safe_margin;
        unread = safe_margin;
    }

    return (size_t)unread;
}

size_t ring_cursor_available(const ring_cursor_t *rc) {
    if (!rc || rc->capacity == 0) return 0;
    uint64_t unread = (rc->total_written >= rc->total_read) ?
                      (rc->total_written - rc->total_read) : 0;
    return (unread > rc->capacity) ? rc->capacity : (size_t)unread;
}

void ring_cursor_advance(ring_cursor_t *rc, size_t num_bytes) {
    if (!rc || rc->capacity == 0 || num_bytes == 0) return;
    size_t avail = ring_cursor_available(rc);
    if (num_bytes > avail) num_bytes = avail;
    rc->tail = (rc->tail + num_bytes) & rc->capacity_mask;
    rc->total_read += num_bytes;
}

size_t ring_cursor_peek(const ring_cursor_t *rc, const uint8_t *ring_buf, uint8_t *dest, size_t max_len) {
    if (!rc || !ring_buf || !dest || max_len == 0 || rc->capacity == 0) return 0;
    size_t avail = ring_cursor_available(rc);
    if (max_len > avail) max_len = avail;
    for (size_t i = 0; i < max_len; ++i) {
        dest[i] = ring_buf[(rc->tail + i) & rc->capacity_mask];
    }
    return max_len;
}

size_t ring_cursor_read(ring_cursor_t *rc, const uint8_t *ring_buf, uint8_t *dest, size_t max_len) {
    size_t read_bytes = ring_cursor_peek(rc, ring_buf, dest, max_len);
    if (read_bytes > 0) {
        ring_cursor_advance(rc, read_bytes);
    }
    return read_bytes;
}
