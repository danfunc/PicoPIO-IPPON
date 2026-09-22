#ifndef POC_FLASH_UTILS_H
#define POC_FLASH_UTILS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ===========================================================================
 * CRC32 (IEEE 802.3 Standard)
 * ===========================================================================
 */
#define POC_CRC32_INIT (0xFFFFFFFFu)

/**
 * \brief Incrementally update CRC32 checksum with input buffer.
 */
uint32_t flash_crc32_update(uint32_t crc, const uint8_t *data, size_t length);

/**
 * \brief Compute CRC32 checksum of buffer in one shot.
 */
uint32_t flash_crc32(const uint8_t *data, size_t length);

/*
 * ===========================================================================
 * PRNG (Xorshift32) Test Pattern Generator
 * ===========================================================================
 */
#define POC_DEFAULT_PRNG_SEED (0x12345678u)

/**
 * \brief Return next 32-bit pseudo-random value and advance state.
 */
static inline uint32_t flash_prng_next(uint32_t *state) {
    uint32_t x = *state;
    if (x == 0) {
        x = POC_DEFAULT_PRNG_SEED;
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/**
 * \brief Fill buffer with pseudo-random test pattern (never pure 0xFF or 0x00).
 */
void flash_prng_fill_pattern(uint8_t *buffer, size_t length, uint32_t seed);

/*
 * ===========================================================================
 * Benchmark Statistics Accumulator
 * ===========================================================================
 */
typedef struct {
    uint32_t count;
    uint64_t min_us;
    uint64_t max_us;
    uint64_t total_us;
} bench_stats_t;

void bench_stats_init(bench_stats_t *stats);
void bench_stats_record(bench_stats_t *stats, uint64_t duration_us);
uint64_t bench_stats_avg_us(const bench_stats_t *stats);
double bench_stats_throughput_kb_s(size_t total_bytes, uint64_t total_us);

#ifdef __cplusplus
}
#endif

#endif /* POC_FLASH_UTILS_H */
