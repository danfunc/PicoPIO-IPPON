#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>

#include "flash_map.h"
#include "flash_utils.h"

/*
 * ===========================================================================
 * Test 1: Compile-time Layout & Alignment Invariants
 * ===========================================================================
 */
static void test_flash_map_layout(void) {
    printf("[TEST 1] Testing Flash Map constants and alignments...\n");

    /* Fundamental geometry */
    assert(POC_FLASH_PAGE_SIZE == 256u);
    assert(POC_FLASH_SECTOR_SIZE == 4096u);
    assert(POC_FLASH_BLOCK_SIZE == 65536u);
    assert(POC_FLASH_TOTAL_BYTES == 4u * 1024u * 1024u);

    /* Partition sizes & offsets */
    assert(POC_FLASH_FIRMWARE_OFFSET == 0u);
    assert(POC_FLASH_FIRMWARE_BYTES == 1024u * 1024u); /* 1MB */

    assert(POC_FLASH_STAGING_OFFSET == 1024u * 1024u); /* Starts at 1MB (0x100000) */
    assert(POC_FLASH_BT_RESERVED_BYTES == 12288u);      /* 12KB (3 sectors) */
    assert(POC_FLASH_BT_RESERVED_OFFSET == (4194304u - 12288u)); /* 0x3FD000 */

    assert(POC_FLASH_STAGING_BYTES == (POC_FLASH_BT_RESERVED_OFFSET - POC_FLASH_STAGING_OFFSET));
    assert(POC_FLASH_STAGING_BYTES == 3133440u); /* Exactly 3060 KB = 765 sectors */

    /* Alignments */
    assert(POC_FLASH_FIRMWARE_OFFSET % POC_FLASH_SECTOR_SIZE == 0);
    assert(POC_FLASH_FIRMWARE_BYTES % POC_FLASH_SECTOR_SIZE == 0);
    assert(POC_FLASH_STAGING_OFFSET % POC_FLASH_SECTOR_SIZE == 0);
    assert(POC_FLASH_STAGING_BYTES % POC_FLASH_SECTOR_SIZE == 0);
    assert(POC_FLASH_BT_RESERVED_OFFSET % POC_FLASH_SECTOR_SIZE == 0);
    assert(POC_FLASH_BT_RESERVED_BYTES % POC_FLASH_SECTOR_SIZE == 0);

    /* Capacity check */
    assert(POC_FLASH_STAGING_BYTES >= POC_FLASH_FIRMWARE_BYTES);

    /* Test region */
    assert(POC_FLASH_BENCH_TEST_OFFSET == POC_FLASH_STAGING_OFFSET);
    assert(POC_FLASH_BENCH_TEST_BYTES == 384u * 1024u);
    assert(POC_FLASH_BENCH_TEST_BYTES % POC_FLASH_BLOCK_SIZE == 0);
    assert(POC_FLASH_BENCH_TEST_BYTES % POC_FLASH_SECTOR_SIZE == 0);

    printf("  Flash Map layout invariants passed! [PASS]\n");
}

/*
 * ===========================================================================
 * Test 2: Runtime Safety Boundary Guards (Staging range & alignment)
 * ===========================================================================
 */
static void test_flash_map_boundary_guards(void) {
    printf("[TEST 2] Testing Flash Map boundary guards and alignment checks...\n");

    /* 1. Staging boundary checking */
    /* Inside staging */
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, 4096) == true);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, POC_FLASH_STAGING_BYTES) == true);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES - 4096, 4096) == true);

    /* Out of bounds: inside firmware */
    assert(flash_map_is_staging_range(0, 4096) == false);
    assert(flash_map_is_staging_range(POC_FLASH_FIRMWARE_OFFSET, POC_FLASH_FIRMWARE_BYTES) == false);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET - 4096, 4096) == false);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET - 1, 4096) == false);

    /* Out of bounds: encroaching on BT reserved bank */
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, POC_FLASH_STAGING_BYTES + 1) == false);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES - 4096 + 1, 4096) == false);
    assert(flash_map_is_staging_range(POC_FLASH_BT_RESERVED_OFFSET, 4096) == false);
    assert(flash_map_is_staging_range(POC_FLASH_TOTAL_BYTES, 4096) == false);

    /* Edge case: count = 0 */
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, 0) == false);

    /* 2. Sector alignment check */
    assert(flash_map_is_sector_aligned(0x100000, 4096) == true);
    assert(flash_map_is_sector_aligned(0x100000, 65536) == true);
    assert(flash_map_is_sector_aligned(0x100001, 4096) == false);
    assert(flash_map_is_sector_aligned(0x100000, 4095) == false);

    /* 3. Page alignment check */
    assert(flash_map_is_page_aligned(0x100000, 256) == true);
    assert(flash_map_is_page_aligned(0x100000, 4096) == true);
    assert(flash_map_is_page_aligned(0x100100, 256) == true);
    assert(flash_map_is_page_aligned(0x100001, 256) == false);
    assert(flash_map_is_page_aligned(0x100000, 255) == false);

    printf("  Boundary guard tests passed! [PASS]\n");
}

/*
 * ===========================================================================
 * Test 3: PRNG Test Pattern Generation & Reproducibility
 * ===========================================================================
 */
static void test_flash_prng(void) {
    printf("[TEST 3] Testing PRNG pattern generator (Xorshift32)...\n");

    uint8_t buf1[4096];
    uint8_t buf2[4096];
    uint8_t buf3[4096];

    /* Deterministic reproducibility check */
    flash_prng_fill_pattern(buf1, sizeof(buf1), 0x12345678u);
    flash_prng_fill_pattern(buf2, sizeof(buf2), 0x12345678u);
    assert(memcmp(buf1, buf2, sizeof(buf1)) == 0);

    /* Different seed generates distinct pattern */
    flash_prng_fill_pattern(buf3, sizeof(buf3), 0x87654321u);
    assert(memcmp(buf1, buf3, sizeof(buf1)) != 0);

    /* Verify non-erased state: ensure no pure 0xFF or pure 0x00 */
    bool has_non_ff = false;
    bool has_non_00 = false;
    for (size_t i = 0; i < sizeof(buf1); ++i) {
        if (buf1[i] != 0xFF) has_non_ff = true;
        if (buf1[i] != 0x00) has_non_00 = true;
    }
    assert(has_non_ff == true);
    assert(has_non_00 == true);

    printf("  PRNG generator tests passed! [PASS]\n");
}

/*
 * ===========================================================================
 * Test 4: CRC32 Standard Test Vectors & Integrity
 * ===========================================================================
 */
static void test_flash_crc32(void) {
    printf("[TEST 4] Testing CRC32 implementation against IEEE 802.3 vectors...\n");

    /* Test empty buffer */
    uint32_t crc_empty = flash_crc32(NULL, 0);
    assert(crc_empty == 0x00000000u);

    /* Standard test vector: "123456789" -> 0xCBF43926 */
    const char *vector = "123456789";
    uint32_t crc_vector = flash_crc32((const uint8_t *)vector, 9);
    assert(crc_vector == 0xCBF43926u);

    /* Incremental update vs one-shot */
    uint32_t inc_crc = POC_CRC32_INIT;
    inc_crc = flash_crc32_update(inc_crc, (const uint8_t *)"12345", 5);
    inc_crc = flash_crc32_update(inc_crc, (const uint8_t *)"6789", 4);
    inc_crc ^= 0xFFFFFFFFu;
    assert(inc_crc == 0xCBF43926u);

    /* Bit corruption detection */
    char corrupted[10];
    memcpy(corrupted, vector, 10);
    corrupted[4] ^= 0x01; /* 1-bit flip */
    uint32_t crc_corrupted = flash_crc32((const uint8_t *)corrupted, 9);
    assert(crc_corrupted != crc_vector);

    printf("  CRC32 test vector verification passed! [PASS]\n");
}

/*
 * ===========================================================================
 * Test 5: Benchmark Statistics Accumulator
 * ===========================================================================
 */
static void test_bench_stats(void) {
    printf("[TEST 5] Testing benchmark statistics accumulator...\n");

    bench_stats_t stats;
    bench_stats_init(&stats);

    assert(stats.count == 0);
    assert(stats.min_us == UINT64_MAX);
    assert(stats.max_us == 0);
    assert(stats.total_us == 0);
    assert(bench_stats_avg_us(&stats) == 0);

    /* Record samples */
    bench_stats_record(&stats, 100);
    bench_stats_record(&stats, 200);
    bench_stats_record(&stats, 300);

    assert(stats.count == 3);
    assert(stats.min_us == 100);
    assert(stats.max_us == 300);
    assert(stats.total_us == 600);
    assert(bench_stats_avg_us(&stats) == 200);

    /* Throughput check: 1024 bytes in 1000 us (1 ms) -> exactly 1000.0 KB/s */
    double tp = bench_stats_throughput_kb_s(1024, 1000);
    assert(tp >= 999.9 && tp <= 1000.1);

    printf("  Benchmark stats tests passed! [PASS]\n");
}

int main(void) {
    printf("\nRunning Host Unit Tests for Flash Benchmark Module...\n");
    test_flash_map_layout();
    test_flash_map_boundary_guards();
    test_flash_prng();
    test_flash_crc32();
    test_bench_stats();

    printf("\n>>> ALL FLASH BENCHMARK HOST TESTS PASSED! <<<\n\n");
    return 0;
}
