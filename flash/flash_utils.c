#include "flash_utils.h"

/*
 * IEEE 802.3 CRC32 Nibble Table (Polynomial: 0xEDB88320)
 * Consumes only 64 bytes of ROM/RAM while providing high speed.
 */
static const uint32_t CRC32_NIBBLE[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu
};

uint32_t flash_crc32_update(uint32_t crc, const uint8_t *data, size_t length) {
    if (!data) return crc;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        crc = (crc >> 4) ^ CRC32_NIBBLE[crc & 0x0Fu];
        crc = (crc >> 4) ^ CRC32_NIBBLE[crc & 0x0Fu];
    }
    return crc;
}

uint32_t flash_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = POC_CRC32_INIT;
    crc = flash_crc32_update(crc, data, length);
    return crc ^ 0xFFFFFFFFu;
}

void flash_prng_fill_pattern(uint8_t *buffer, size_t length, uint32_t seed) {
    if (!buffer || length == 0) return;

    uint32_t state = (seed != 0) ? seed : POC_DEFAULT_PRNG_SEED;
    size_t words = length / sizeof(uint32_t);
    size_t rem = length % sizeof(uint32_t);

    /* Write 4 bytes at a time safely without unaligned access */
    for (size_t i = 0; i < words; ++i) {
        uint32_t val = flash_prng_next(&state);
        /* Guard against all 0xFF (erased state) or all 0x00 */
        if (val == 0xFFFFFFFFu || val == 0u) {
            val ^= 0x5A5A5A5Au;
        }
        uint8_t *dest = buffer + (i * sizeof(uint32_t));
        dest[0] = (uint8_t)(val & 0xFF);
        dest[1] = (uint8_t)((val >> 8) & 0xFF);
        dest[2] = (uint8_t)((val >> 16) & 0xFF);
        dest[3] = (uint8_t)((val >> 24) & 0xFF);
    }

    if (rem > 0) {
        uint32_t val = flash_prng_next(&state);
        if (val == 0xFFFFFFFFu || val == 0u) {
            val ^= 0x5A5A5A5Au;
        }
        uint8_t *dest = buffer + (words * sizeof(uint32_t));
        for (size_t i = 0; i < rem; ++i) {
            dest[i] = (uint8_t)((val >> (i * 8)) & 0xFF);
        }
    }
}

void bench_stats_init(bench_stats_t *stats) {
    if (!stats) return;
    stats->count = 0;
    stats->min_us = UINT64_MAX;
    stats->max_us = 0;
    stats->total_us = 0;
}

void bench_stats_record(bench_stats_t *stats, uint64_t duration_us) {
    if (!stats) return;
    stats->count++;
    if (duration_us < stats->min_us) {
        stats->min_us = duration_us;
    }
    if (duration_us > stats->max_us) {
        stats->max_us = duration_us;
    }
    stats->total_us += duration_us;
}

uint64_t bench_stats_avg_us(const bench_stats_t *stats) {
    if (!stats || stats->count == 0) return 0;
    return stats->total_us / stats->count;
}

double bench_stats_throughput_kb_s(size_t total_bytes, uint64_t total_us) {
    if (total_us == 0) return 0.0;
    return ((double)total_bytes * 1000000.0) / ((double)total_us * 1024.0);
}
