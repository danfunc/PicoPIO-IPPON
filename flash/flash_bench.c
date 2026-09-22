#include "flash_bench.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* Static 4KB Work Buffer placed in SRAM to avoid stack overflow */
static uint8_t s_work_buf[POC_BENCH_WORK_BUFFER_BYTES] __attribute__((aligned(4)));

/* Parameter structure passed to flash_safe_execute callback */
typedef struct {
    flash_bench_op_type_t op_type;
    uint32_t offset;
    const uint8_t *data;
    size_t count;
    uint64_t callback_duration_us;
} flash_bench_param_t;

/**
 * \brief Pure flash operation executed while interrupts are disabled and core1 is safe.
 *
 * NOTE: Measuring time_us_64() directly inside this callback measures the exact duration
 * that the CPU was halted / interrupts were disabled for this single flash operation.
 */
static void flash_bench_safe_callback(void *param) {
    flash_bench_param_t *p = (flash_bench_param_t *)param;
    uint64_t t0 = time_us_64();
    if (p->op_type == FLASH_BENCH_OP_ERASE) {
        flash_range_erase(p->offset, p->count);
    } else {
        flash_range_program(p->offset, p->data, p->count);
    }
    uint64_t t1 = time_us_64();
    p->callback_duration_us = (t1 >= t0) ? (t1 - t0) : 0;
}

int flash_bench_execute(flash_bench_op_type_t op_type, uint32_t offset,
                        const uint8_t *data, size_t count, uint64_t *out_cb_us) {
    /* Hard boundary safety guard */
    if (!flash_map_is_staging_range(offset, count)) {
        printf("[FLASH_GUARD] REJECTED op=%d at offset=0x%08" PRIX32 ", count=%zu (out of staging bounds [0x%08" PRIX32 ", 0x%08" PRIX32 "))\n",
               (int)op_type, offset, count,
               (uint32_t)POC_FLASH_STAGING_OFFSET,
               (uint32_t)(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES));
        return PICO_ERROR_NOT_PERMITTED;
    }

    /* Alignment checks */
    if (op_type == FLASH_BENCH_OP_ERASE) {
        if (!flash_map_is_sector_aligned(offset, count)) {
            printf("[FLASH_GUARD] REJECTED erase not sector aligned: offs=0x%" PRIX32 ", count=%zu\n",
                   offset, count);
            return PICO_ERROR_BAD_ALIGNMENT;
        }
    } else {
        if (!flash_map_is_page_aligned(offset, count)) {
            printf("[FLASH_GUARD] REJECTED program not page aligned: offs=0x%" PRIX32 ", count=%zu\n",
                   offset, count);
            return PICO_ERROR_BAD_ALIGNMENT;
        }
        if (!data) {
            return PICO_ERROR_INVALID_ARG;
        }
    }

    flash_bench_param_t param = {
        .op_type = op_type,
        .offset = offset,
        .data = data,
        .count = count,
        .callback_duration_us = 0,
    };

    /* Timeout of 5000ms is generous for 64KB block operations */
    int rc = flash_safe_execute(flash_bench_safe_callback, &param, 5000);
    if (rc == PICO_OK && out_cb_us) {
        *out_cb_us = param.callback_duration_us;
    }
    return rc;
}

int flash_bench_measure_sector_erase(bench_stats_t *stats, uint32_t num_sectors) {
    bench_stats_init(stats);
    for (uint32_t i = 0; i < num_sectors; ++i) {
        uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (i * POC_FLASH_SECTOR_SIZE);
        uint64_t cb_us = 0;
        int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL,
                                    POC_FLASH_SECTOR_SIZE, &cb_us);
        if (rc != PICO_OK) {
            return rc;
        }
        bench_stats_record(stats, cb_us);
    }
    return PICO_OK;
}

int flash_bench_measure_block_erase(bench_stats_t *stats, uint32_t num_blocks) {
    bench_stats_init(stats);
    for (uint32_t i = 0; i < num_blocks; ++i) {
        uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (i * POC_FLASH_BLOCK_SIZE);
        uint64_t cb_us = 0;
        int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL,
                                    POC_FLASH_BLOCK_SIZE, &cb_us);
        if (rc != PICO_OK) {
            return rc;
        }
        bench_stats_record(stats, cb_us);
    }
    return PICO_OK;
}

int flash_bench_measure_page_program(bench_stats_t *stats, uint32_t num_pages) {
    bench_stats_init(stats);
    /* Calculate how many sectors need erasing before programming pages */
    uint32_t total_bytes = num_pages * POC_FLASH_PAGE_SIZE;
    uint32_t sectors = (total_bytes + POC_FLASH_SECTOR_SIZE - 1) / POC_FLASH_SECTOR_SIZE;
    for (uint32_t s = 0; s < sectors; ++s) {
        uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (s * POC_FLASH_SECTOR_SIZE);
        int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL, POC_FLASH_SECTOR_SIZE, NULL);
        if (rc != PICO_OK) return rc;
    }

    /* Program each 256B page with PRNG test pattern */
    for (uint32_t i = 0; i < num_pages; ++i) {
        uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (i * POC_FLASH_PAGE_SIZE);
        flash_prng_fill_pattern(s_work_buf, POC_FLASH_PAGE_SIZE, 0xABC00000u + i);
        uint64_t cb_us = 0;
        int rc = flash_bench_execute(FLASH_BENCH_OP_PROGRAM, offs, s_work_buf,
                                    POC_FLASH_PAGE_SIZE, &cb_us);
        if (rc != PICO_OK) {
            return rc;
        }
        bench_stats_record(stats, cb_us);
    }
    return PICO_OK;
}

int flash_bench_measure_chunk_program(bench_stats_t *stats, uint32_t num_chunks) {
    bench_stats_init(stats);
    /* Erase sectors first */
    for (uint32_t i = 0; i < num_chunks; ++i) {
        uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (i * POC_FLASH_SECTOR_SIZE);
        int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL, POC_FLASH_SECTOR_SIZE, NULL);
        if (rc != PICO_OK) return rc;
    }

    /* Program each 4KB chunk */
    for (uint32_t i = 0; i < num_chunks; ++i) {
        uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (i * POC_FLASH_SECTOR_SIZE);
        flash_prng_fill_pattern(s_work_buf, POC_FLASH_SECTOR_SIZE, 0xDEF00000u + i);
        uint64_t cb_us = 0;
        int rc = flash_bench_execute(FLASH_BENCH_OP_PROGRAM, offs, s_work_buf,
                                    POC_FLASH_SECTOR_SIZE, &cb_us);
        if (rc != PICO_OK) {
            return rc;
        }
        bench_stats_record(stats, cb_us);
    }
    return PICO_OK;
}

int flash_bench_measure_xip_read(uint32_t offset, size_t total_bytes, uint32_t seed,
                                 uint64_t *out_read_us, uint32_t *out_read_crc32,
                                 bool *out_matched) {
    if (!flash_map_is_staging_range(offset, total_bytes)) {
        return PICO_ERROR_NOT_PERMITTED;
    }

    uintptr_t xip_addr = flash_map_offset_to_xip(offset);
    const uint8_t *xip_ptr = (const uint8_t *)xip_addr;

    uint32_t running_crc = POC_CRC32_INIT;
    bool matched = true;

    uint64_t t0 = time_us_64();

    size_t remaining = total_bytes;
    size_t chunk_idx = 0;

    while (remaining > 0) {
        size_t step = (remaining > POC_BENCH_WORK_BUFFER_BYTES) ?
                       POC_BENCH_WORK_BUFFER_BYTES : remaining;

        /* Re-generate expected reference pattern */
        flash_prng_fill_pattern(s_work_buf, step, seed + chunk_idx);

        const uint8_t *current_xip = xip_ptr + (chunk_idx * POC_BENCH_WORK_BUFFER_BYTES);

        /* Accumulate CRC32 directly from XIP memory */
        running_crc = flash_crc32_update(running_crc, current_xip, step);

        /* Verify content matches reference */
        if (matched && memcmp(s_work_buf, current_xip, step) != 0) {
            matched = false;
        }

        remaining -= step;
        chunk_idx++;
    }

    uint64_t t1 = time_us_64();
    uint32_t final_crc = running_crc ^ 0xFFFFFFFFu;

    if (out_read_us) *out_read_us = (t1 >= t0) ? (t1 - t0) : 0;
    if (out_read_crc32) *out_read_crc32 = final_crc;
    if (out_matched) *out_matched = matched;

    return PICO_OK;
}

int flash_bench_measure_image_strategies(strategy_result_t results[3], size_t total_bytes) {
    if (total_bytes == 0 || (total_bytes % POC_FLASH_BLOCK_SIZE) != 0) {
        return PICO_ERROR_INVALID_ARG;
    }
    if (!flash_map_is_staging_range(POC_FLASH_BENCH_TEST_OFFSET, total_bytes)) {
        return PICO_ERROR_NOT_PERMITTED;
    }

    const uint32_t num_sectors = total_bytes / POC_FLASH_SECTOR_SIZE;
    const uint32_t num_blocks = total_bytes / POC_FLASH_BLOCK_SIZE;
    const uint32_t num_pages = total_bytes / POC_FLASH_PAGE_SIZE;
    const uint32_t test_seed = 0x55AA1234u;

    /* -----------------------------------------------------------------------
     * Strategy (a): Alternating 4KB Sector Erase -> 4KB Chunk Program
     * ----------------------------------------------------------------------- */
    {
        strategy_result_t *res = &results[0];
        memset(res, 0, sizeof(*res));
        res->strategy_name = "(a) Interleaved 4KB Erase -> 4KB Write";

        uint64_t t_start = time_us_64();
        for (uint32_t s = 0; s < num_sectors; ++s) {
            uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (s * POC_FLASH_SECTOR_SIZE);

            /* Prepare test data */
            flash_prng_fill_pattern(s_work_buf, POC_FLASH_SECTOR_SIZE, test_seed + s);

            /* Erase sector */
            uint64_t cb_us = 0;
            int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL, POC_FLASH_SECTOR_SIZE, &cb_us);
            if (rc != PICO_OK) return rc;
            res->total_erase_us += cb_us;
            if (cb_us > res->max_stall_us) res->max_stall_us = cb_us;

            /* Program sector */
            rc = flash_bench_execute(FLASH_BENCH_OP_PROGRAM, offs, s_work_buf, POC_FLASH_SECTOR_SIZE, &cb_us);
            if (rc != PICO_OK) return rc;
            res->total_program_us += cb_us;
            if (cb_us > res->max_stall_us) res->max_stall_us = cb_us;
        }
        uint64_t t_end = time_us_64();
        res->total_elapsed_us = (t_end >= t_start) ? (t_end - t_start) : 0;
        res->throughput_kb_s = bench_stats_throughput_kb_s(total_bytes, res->total_elapsed_us);

        /* Verify */
        flash_bench_measure_xip_read(POC_FLASH_BENCH_TEST_OFFSET, total_bytes, test_seed,
                                     NULL, &res->crc32, &res->verified);
    }

    /* -----------------------------------------------------------------------
     * Strategy (b): Batch 64KB Block Erase first, then 256B Page Program
     * ----------------------------------------------------------------------- */
    {
        strategy_result_t *res = &results[1];
        memset(res, 0, sizeof(*res));
        res->strategy_name = "(b) Batch 64KB Erase -> 256B Page Write";

        uint64_t t_start = time_us_64();
        /* Erase all 64KB blocks first */
        for (uint32_t b = 0; b < num_blocks; ++b) {
            uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (b * POC_FLASH_BLOCK_SIZE);
            uint64_t cb_us = 0;
            int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL, POC_FLASH_BLOCK_SIZE, &cb_us);
            if (rc != PICO_OK) return rc;
            res->total_erase_us += cb_us;
            if (cb_us > res->max_stall_us) res->max_stall_us = cb_us;
        }

        /* Program page by page */
        for (uint32_t p = 0; p < num_pages; ++p) {
            uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (p * POC_FLASH_PAGE_SIZE);
            uint32_t s = p / (POC_FLASH_SECTOR_SIZE / POC_FLASH_PAGE_SIZE);
            uint32_t rel = (p % (POC_FLASH_SECTOR_SIZE / POC_FLASH_PAGE_SIZE)) * POC_FLASH_PAGE_SIZE;

            /* Regenerate sector buffer on boundary */
            if (rel == 0) {
                flash_prng_fill_pattern(s_work_buf, POC_FLASH_SECTOR_SIZE, test_seed + s);
            }

            uint64_t cb_us = 0;
            int rc = flash_bench_execute(FLASH_BENCH_OP_PROGRAM, offs, s_work_buf + rel,
                                         POC_FLASH_PAGE_SIZE, &cb_us);
            if (rc != PICO_OK) return rc;
            res->total_program_us += cb_us;
            if (cb_us > res->max_stall_us) res->max_stall_us = cb_us;
        }
        uint64_t t_end = time_us_64();
        res->total_elapsed_us = (t_end >= t_start) ? (t_end - t_start) : 0;
        res->throughput_kb_s = bench_stats_throughput_kb_s(total_bytes, res->total_elapsed_us);

        /* Verify */
        flash_bench_measure_xip_read(POC_FLASH_BENCH_TEST_OFFSET, total_bytes, test_seed,
                                     NULL, &res->crc32, &res->verified);
    }

    /* -----------------------------------------------------------------------
     * Strategy (c): Batch 64KB Block Erase first, then 4KB Chunk Program
     * ----------------------------------------------------------------------- */
    {
        strategy_result_t *res = &results[2];
        memset(res, 0, sizeof(*res));
        res->strategy_name = "(c) Batch 64KB Erase -> 4KB Chunk Write";

        uint64_t t_start = time_us_64();
        /* Erase all 64KB blocks first */
        for (uint32_t b = 0; b < num_blocks; ++b) {
            uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (b * POC_FLASH_BLOCK_SIZE);
            uint64_t cb_us = 0;
            int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, offs, NULL, POC_FLASH_BLOCK_SIZE, &cb_us);
            if (rc != PICO_OK) return rc;
            res->total_erase_us += cb_us;
            if (cb_us > res->max_stall_us) res->max_stall_us = cb_us;
        }

        /* Program 4KB chunk by chunk */
        for (uint32_t s = 0; s < num_sectors; ++s) {
            uint32_t offs = POC_FLASH_BENCH_TEST_OFFSET + (s * POC_FLASH_SECTOR_SIZE);
            flash_prng_fill_pattern(s_work_buf, POC_FLASH_SECTOR_SIZE, test_seed + s);

            uint64_t cb_us = 0;
            int rc = flash_bench_execute(FLASH_BENCH_OP_PROGRAM, offs, s_work_buf,
                                         POC_FLASH_SECTOR_SIZE, &cb_us);
            if (rc != PICO_OK) return rc;
            res->total_program_us += cb_us;
            if (cb_us > res->max_stall_us) res->max_stall_us = cb_us;
        }
        uint64_t t_end = time_us_64();
        res->total_elapsed_us = (t_end >= t_start) ? (t_end - t_start) : 0;
        res->throughput_kb_s = bench_stats_throughput_kb_s(total_bytes, res->total_elapsed_us);

        /* Verify */
        flash_bench_measure_xip_read(POC_FLASH_BENCH_TEST_OFFSET, total_bytes, test_seed,
                                     NULL, &res->crc32, &res->verified);
    }

    return PICO_OK;
}

static void print_stats_row(const char *label, const bench_stats_t *stats, size_t unit_bytes) {
    if (!stats || stats->count == 0) {
        printf("| %-28s | (no samples recorded)                           |\n", label);
        return;
    }
    uint64_t avg = bench_stats_avg_us(stats);
    double tp = (avg > 0) ? bench_stats_throughput_kb_s(unit_bytes, avg) : 0.0;
    printf("| %-28s | %5" PRIu32 " | %8" PRIu64 " | %8" PRIu64 " | %8" PRIu64 " | %8.1f KB/s |\n",
           label, stats->count, stats->min_us, avg, stats->max_us, tp);
}

void flash_bench_run_full_suite(void) {
    printf("\n");
    printf("=================================================================================\n");
    printf("        RP2350 FLASH WRITE / ERASE BENCHMARK - FULL EXECUTION\n");
    printf("=================================================================================\n");
    printf(" Flash Total Size : 4096 KB (0x%08" PRIX32 ")\n", (uint32_t)POC_FLASH_TOTAL_BYTES);
    printf(" Firmware Window  :    0 - %4u KB [0x%08" PRIX32 " - 0x%08" PRIX32 ") (Read-Only)\n",
           (unsigned)(POC_FLASH_FIRMWARE_BYTES / 1024),
           (uint32_t)POC_FLASH_FIRMWARE_OFFSET,
           (uint32_t)(POC_FLASH_FIRMWARE_OFFSET + POC_FLASH_FIRMWARE_BYTES));
    printf(" Staging Area     : %4u - %4u KB [0x%08" PRIX32 " - 0x%08" PRIX32 ") (Active Benchmark)\n",
           (unsigned)(POC_FLASH_STAGING_OFFSET / 1024),
           (unsigned)((POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES) / 1024),
           (uint32_t)POC_FLASH_STAGING_OFFSET,
           (uint32_t)(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES));
    printf(" BT Reserved Bank : %4u - %4u KB [0x%08" PRIX32 " - 0x%08" PRIX32 ") (Protected)\n",
           (unsigned)(POC_FLASH_BT_RESERVED_OFFSET / 1024),
           (unsigned)(POC_FLASH_TOTAL_BYTES / 1024),
           (uint32_t)POC_FLASH_BT_RESERVED_OFFSET,
           (uint32_t)POC_FLASH_TOTAL_BYTES);
    printf(" Test Region      :  384 KB [0x%08" PRIX32 " - 0x%08" PRIX32 ")\n",
           (uint32_t)POC_FLASH_BENCH_TEST_OFFSET,
           (uint32_t)(POC_FLASH_BENCH_TEST_OFFSET + POC_FLASH_BENCH_TEST_BYTES));
    printf("---------------------------------------------------------------------------------\n\n");

    bench_stats_t stats_sec_erase, stats_blk_erase, stats_page_prog, stats_chunk_prog;

    /* Item 1: 4KB Sector Erase (8 samples) */
    printf("[1/6] Measuring 4KB Sector Erase (8 sectors)...\n");
    int rc = flash_bench_measure_sector_erase(&stats_sec_erase, 8);
    if (rc != PICO_OK) {
        printf("  FAILED: error code %d\n", rc);
        return;
    }

    /* Item 2: 64KB Block Erase (4 samples) */
    printf("[2/6] Measuring 64KB Block Erase (4 blocks)...\n");
    rc = flash_bench_measure_block_erase(&stats_blk_erase, 4);
    if (rc != PICO_OK) {
        printf("  FAILED: error code %d\n", rc);
        return;
    }

    /* Item 3: 256B Page Program (16 samples) */
    printf("[3/6] Measuring 256B Page Program (16 pages)...\n");
    rc = flash_bench_measure_page_program(&stats_page_prog, 16);
    if (rc != PICO_OK) {
        printf("  FAILED: error code %d\n", rc);
        return;
    }

    /* Item 4: 4KB Chunk Program (8 samples) */
    printf("[4/6] Measuring 4KB Chunk Program (8 chunks)...\n");
    rc = flash_bench_measure_chunk_program(&stats_chunk_prog, 8);
    if (rc != PICO_OK) {
        printf("  FAILED: error code %d\n", rc);
        return;
    }

    /* Print Micro-benchmark Table */
    printf("\n--- Micro-benchmark Results ---\n");
    printf("+------------------------------+-------+----------+----------+----------+----------------+\n");
    printf("| Operation                    | Count |  Min(us) |  Avg(us) |  Max(us) | Avg Throughput |\n");
    printf("+------------------------------+-------+----------+----------+----------+----------------+\n");
    print_stats_row("4KB Sector Erase", &stats_sec_erase, POC_FLASH_SECTOR_SIZE);
    print_stats_row("64KB Block Erase", &stats_blk_erase, POC_FLASH_BLOCK_SIZE);
    print_stats_row("256B Page Program", &stats_page_prog, POC_FLASH_PAGE_SIZE);
    print_stats_row("4KB Chunk Program", &stats_chunk_prog, POC_FLASH_SECTOR_SIZE);
    printf("+------------------------------+-------+----------+----------+----------+----------------+\n\n");

    /* Item 5: 384KB Large Image Transfer Strategies */
    printf("[5/6] Measuring 384KB Image Write Strategies ((a), (b), (c))...\n");
    strategy_result_t strat_results[3];
    rc = flash_bench_measure_image_strategies(strat_results, POC_BENCH_DEFAULT_IMAGE_BYTES);
    if (rc != PICO_OK) {
        printf("  FAILED: error code %d\n", rc);
        return;
    }

    printf("\n--- 384KB Image Strategy Comparison ---\n");
    printf("+------------------------------------------+------------+------------+------------+------------+--------------+---------+\n");
    printf("| Strategy                                 | Erase (ms) | Write (ms) | Total (ms) | MaxStall ms| Throughput   | Verify  |\n");
    printf("+------------------------------------------+------------+------------+------------+------------+--------------+---------+\n");
    for (int i = 0; i < 3; ++i) {
        strategy_result_t *r = &strat_results[i];
        printf("| %-40s | %10.2f | %10.2f | %10.2f | %10.2f | %8.1f KB/s | %s |\n",
               r->strategy_name,
               (double)r->total_erase_us / 1000.0,
               (double)r->total_program_us / 1000.0,
               (double)r->total_elapsed_us / 1000.0,
               (double)r->max_stall_us / 1000.0,
               r->throughput_kb_s,
               r->verified ? "PASS(CRC)" : "FAIL(CRC)");
    }
    printf("+------------------------------------------+------------+------------+------------+------------+--------------+---------+\n\n");

    /* Item 6: XIP Read Speed & Verification */
    printf("[6/6] Measuring 384KB XIP Memory-Mapped Read Speed...\n");
    uint64_t read_us = 0;
    uint32_t read_crc = 0;
    bool matched = false;
    rc = flash_bench_measure_xip_read(POC_FLASH_BENCH_TEST_OFFSET, POC_BENCH_DEFAULT_IMAGE_BYTES,
                                      0x55AA1234u, &read_us, &read_crc, &matched);
    if (rc == PICO_OK) {
        double read_mb_s = ((double)POC_BENCH_DEFAULT_IMAGE_BYTES * 1000000.0) / ((double)read_us * 1024.0 * 1024.0);
        printf("  Read Elapsed   : %" PRIu64 " us (%.2f ms)\n", read_us, (double)read_us / 1000.0);
        printf("  Read Speed     : %.2f MB/s\n", read_mb_s);
        printf("  CRC32 Checksum : 0x%08" PRIX32 "\n", read_crc);
        printf("  Data Integrity : %s\n", matched ? "ALL BYTES MATCHED PERFECTLY [PASS]" : "MISMATCH DETECTED [FAIL]");
    } else {
        printf("  FAILED: error code %d\n", rc);
    }

    printf("\n=================================================================================\n");
    printf(" [BENCHMARK COMPLETE] All tests executed safely inside staging area.\n");
    printf("=================================================================================\n\n");
}
