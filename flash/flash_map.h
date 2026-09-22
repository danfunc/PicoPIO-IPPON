#ifndef POC_FLASH_MAP_H
#define POC_FLASH_MAP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

#if defined(__has_include)
#  if __has_include("hardware/flash.h")
#    include "hardware/flash.h"
#  endif
#endif

// Fallbacks for host environment or when hardware/flash.h is unavailable
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE (256u)
#endif

#ifndef FLASH_SECTOR_SIZE
#define FLASH_SECTOR_SIZE (4096u)
#endif

#ifndef FLASH_BLOCK_SIZE
#define FLASH_BLOCK_SIZE (65536u)
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4u * 1024u * 1024u)
#endif

#ifndef XIP_BASE
#define XIP_BASE (0x10000000u)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ===========================================================================
 * RP2350 Flash Memory Map for Real Undanger Bomb Game PoC
 * ===========================================================================
 *
 * Physical Flash: 4MB (W25Q32 / Winbond or equivalent on Pico 2 / Pico 2 W)
 *
 * [0x0000_0000 - 0x0010_0000] : Running Firmware (1MB window)
 * [0x0010_0000 - 0x003F_D000] : Staging Area (3060KB = 3,133,440 B)
 * [0x003F_D000 - 0x0040_0000] : BTstack Bonding Bank Reserved (12KB = 3 sectors)
 *
 * Note on Safety Policy:
 * 1. The benchmark and future OTA transfer MUST ONLY erase or write into
 *    the staging area [0x0010_0000, 0x003F_D000).
 * 2. Firmware area [0, 0x0010_0000) is NEVER touched during benchmark.
 * 3. BTstack bonding bank [0x003F_D000, 0x0040_0000) is reserved by Pico SDK
 *    (PICO_FLASH_BANK_STORAGE_OFFSET = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE * 3)
 *    and must never be modified by application code.
 * ===========================================================================
 */

#define POC_FLASH_PAGE_SIZE           FLASH_PAGE_SIZE        /* 256 B */
#define POC_FLASH_SECTOR_SIZE         FLASH_SECTOR_SIZE      /* 4096 B (4 KB) */
#define POC_FLASH_BLOCK_SIZE          FLASH_BLOCK_SIZE       /* 65536 B (64 KB) */
#define POC_FLASH_TOTAL_BYTES         PICO_FLASH_SIZE_BYTES  /* 4MB = 4,194,304 B */

/* 1. BTstack reserved tail area (3 sectors = 12KB) */
#define POC_FLASH_BT_RESERVED_BYTES   (POC_FLASH_SECTOR_SIZE * 3u)
#define POC_FLASH_BT_RESERVED_OFFSET  (POC_FLASH_TOTAL_BYTES - POC_FLASH_BT_RESERVED_BYTES)

/* 2. Running firmware area (1MB) */
#define POC_FLASH_FIRMWARE_OFFSET     (0u)
#define POC_FLASH_FIRMWARE_BYTES      (1024u * 1024u)

/* 3. Staging area (occupies space between firmware and BTstack reserved tail) */
#define POC_FLASH_STAGING_OFFSET      (POC_FLASH_FIRMWARE_OFFSET + POC_FLASH_FIRMWARE_BYTES)
#define POC_FLASH_STAGING_BYTES       (POC_FLASH_BT_RESERVED_OFFSET - POC_FLASH_STAGING_OFFSET)

/* 4. Benchmark test region (inside staging area: 384KB = 6 blocks = 96 sectors) */
#define POC_FLASH_BENCH_TEST_OFFSET   (POC_FLASH_STAGING_OFFSET)
#define POC_FLASH_BENCH_TEST_BYTES    (384u * 1024u)

/*
 * ===========================================================================
 * Static Assertions for Flash Map Integrity
 * ===========================================================================
 */
static_assert(POC_FLASH_PAGE_SIZE == 256u, "FLASH_PAGE_SIZE must be 256");
static_assert(POC_FLASH_SECTOR_SIZE == 4096u, "FLASH_SECTOR_SIZE must be 4096");
static_assert(POC_FLASH_BLOCK_SIZE == 65536u, "FLASH_BLOCK_SIZE must be 65536");
static_assert(POC_FLASH_TOTAL_BYTES == (4u * 1024u * 1024u), "PICO_FLASH_SIZE_BYTES must be 4MB");

/* Alignments */
static_assert(POC_FLASH_FIRMWARE_OFFSET % POC_FLASH_SECTOR_SIZE == 0, "FIRMWARE_OFFSET must be sector-aligned");
static_assert(POC_FLASH_FIRMWARE_BYTES % POC_FLASH_SECTOR_SIZE == 0, "FIRMWARE_BYTES must be sector-aligned");
static_assert(POC_FLASH_STAGING_OFFSET % POC_FLASH_SECTOR_SIZE == 0, "STAGING_OFFSET must be sector-aligned");
static_assert(POC_FLASH_STAGING_BYTES % POC_FLASH_SECTOR_SIZE == 0, "STAGING_BYTES must be sector-aligned");
static_assert(POC_FLASH_BT_RESERVED_OFFSET % POC_FLASH_SECTOR_SIZE == 0, "BT_RESERVED_OFFSET must be sector-aligned");
static_assert(POC_FLASH_BT_RESERVED_BYTES % POC_FLASH_SECTOR_SIZE == 0, "BT_RESERVED_BYTES must be sector-aligned");

/* Non-overlapping and boundary checks */
static_assert(POC_FLASH_FIRMWARE_OFFSET + POC_FLASH_FIRMWARE_BYTES <= POC_FLASH_STAGING_OFFSET,
              "Firmware area overlaps staging area");
static_assert(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES <= POC_FLASH_BT_RESERVED_OFFSET,
              "Staging area overlaps BT reserved bank");
static_assert(POC_FLASH_BT_RESERVED_OFFSET + POC_FLASH_BT_RESERVED_BYTES == POC_FLASH_TOTAL_BYTES,
              "BT reserved bank must reach exact end of flash");

/* Capacity invariant: Staging area must be capable of receiving a full firmware image */
static_assert(POC_FLASH_STAGING_BYTES >= POC_FLASH_FIRMWARE_BYTES,
              "Staging area must be at least as large as firmware area");

/* Test region bounds check */
static_assert(POC_FLASH_BENCH_TEST_OFFSET >= POC_FLASH_STAGING_OFFSET,
              "Benchmark test region must start within staging area");
static_assert(POC_FLASH_BENCH_TEST_OFFSET + POC_FLASH_BENCH_TEST_BYTES <= POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES,
              "Benchmark test region must not exceed staging area");
static_assert(POC_FLASH_BENCH_TEST_BYTES % POC_FLASH_BLOCK_SIZE == 0,
              "Benchmark test region must be block-aligned (64KB multiple)");
static_assert(POC_FLASH_BENCH_TEST_BYTES % POC_FLASH_SECTOR_SIZE == 0,
              "Benchmark test region must be sector-aligned (4KB multiple)");

/*
 * ===========================================================================
 * Runtime Safety Guard Helpers (Pure Functions)
 * ===========================================================================
 */

/**
 * \brief Check if [offset, offset + count) is strictly within the staging area.
 */
static inline bool flash_map_is_staging_range(uint32_t offset, size_t count) {
    if (count == 0) {
        return false;
    }
    if (offset < POC_FLASH_STAGING_OFFSET) {
        return false;
    }
    uint64_t end = (uint64_t)offset + count;
    uint64_t staging_end = (uint64_t)POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES;
    return (end <= staging_end);
}

/**
 * \brief Check sector alignment (required for flash_range_erase).
 */
static inline bool flash_map_is_sector_aligned(uint32_t offset, size_t count) {
    return ((offset % POC_FLASH_SECTOR_SIZE) == 0) &&
           ((count % POC_FLASH_SECTOR_SIZE) == 0);
}

/**
 * \brief Check page alignment (required for flash_range_program).
 */
static inline bool flash_map_is_page_aligned(uint32_t offset, size_t count) {
    return ((offset % POC_FLASH_PAGE_SIZE) == 0) &&
           ((count % POC_FLASH_PAGE_SIZE) == 0);
}

/**
 * \brief Convert flash relative offset to memory mapped XIP address.
 */
static inline uintptr_t flash_map_offset_to_xip(uint32_t offset) {
    return (uintptr_t)XIP_BASE + offset;
}

#ifdef __cplusplus
}
#endif

#endif /* POC_FLASH_MAP_H */
