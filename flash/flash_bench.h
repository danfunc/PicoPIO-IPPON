#ifndef POC_FLASH_BENCH_H
#define POC_FLASH_BENCH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "flash_map.h"
#include "flash_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Benchmark Configuration Parameters
 */
#define POC_BENCH_DEFAULT_IMAGE_BYTES (384u * 1024u) /* 384 KB */
#define POC_BENCH_WORK_BUFFER_BYTES   (POC_FLASH_SECTOR_SIZE) /* 4 KB RAM work buffer */

/*
 * Execution Results for Flash Transfer Strategy Comparison
 */
typedef struct {
    const char *strategy_name;
    uint64_t total_erase_us;
    uint64_t total_program_us;
    uint64_t total_elapsed_us;
    uint64_t max_stall_us;       /* Maximum single callback duration (CPU stall) */
    double throughput_kb_s;     /* Effective overall throughput in KB/s */
    uint32_t crc32;             /* Verification CRC32 */
    bool verified;              /* Readback match status */
} strategy_result_t;

/*
 * Low-level Flash Safe Execution Interface
 */
typedef enum {
    FLASH_BENCH_OP_ERASE,
    FLASH_BENCH_OP_PROGRAM,
} flash_bench_op_type_t;

/**
 * \brief Safely execute flash erase or program within staging area.
 * \param op_type FLASH_BENCH_OP_ERASE or FLASH_BENCH_OP_PROGRAM
 * \param offset Flash byte offset (must be inside staging area)
 * \param data Pointer to data (ignored for erase)
 * \param count Number of bytes (sector-aligned for erase, page-aligned for program)
 * \param[out] out_cb_us Elapsed microseconds inside callback (pure CPU stall)
 * \return PICO_OK (0) on success, or negative Pico SDK error code
 */
int flash_bench_execute(flash_bench_op_type_t op_type, uint32_t offset,
                        const uint8_t *data, size_t count, uint64_t *out_cb_us);

/*
 * Benchmark Suites
 */

/**
 * \brief Item 1: Benchmark 4KB Sector Erase (flash_range_erase, count=4096)
 */
int flash_bench_measure_sector_erase(bench_stats_t *stats, uint32_t num_sectors);

/**
 * \brief Item 2: Benchmark 64KB Block Erase (flash_range_erase, count=65536)
 */
int flash_bench_measure_block_erase(bench_stats_t *stats, uint32_t num_blocks);

/**
 * \brief Item 3: Benchmark 256B Page Program (flash_range_program, count=256)
 */
int flash_bench_measure_page_program(bench_stats_t *stats, uint32_t num_pages);

/**
 * \brief Item 4: Benchmark 4KB Chunk Program (flash_range_program, count=4096)
 */
int flash_bench_measure_chunk_program(bench_stats_t *stats, uint32_t num_chunks);

/**
 * \brief Item 5: Benchmark 384KB Image Write Strategy Comparison ((a), (b), (c))
 */
int flash_bench_measure_image_strategies(strategy_result_t results[3], size_t total_bytes);

/**
 * \brief Item 6: Benchmark XIP Read & CRC32 Verification
 */
int flash_bench_measure_xip_read(uint32_t offset, size_t total_bytes, uint32_t seed,
                                 uint64_t *out_read_us, uint32_t *out_read_crc32,
                                 bool *out_matched);

/**
 * \brief Execute complete benchmark suite and print comprehensive report.
 */
void flash_bench_run_full_suite(void);

#ifdef __cplusplus
}
#endif

#endif /* POC_FLASH_BENCH_H */
