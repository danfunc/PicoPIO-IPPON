#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "flash_map.h"
#include "flash_utils.h"
#include "flash_bench.h"

/* Linker symbols defining binary boundary */
extern char __flash_binary_start;
extern char __flash_binary_end;

static void print_banner(void) {
    uintptr_t bin_start = (uintptr_t)&__flash_binary_start;
    uintptr_t bin_end = (uintptr_t)&__flash_binary_end;
    size_t bin_size = (bin_end >= bin_start) ? (bin_end - bin_start) : 0;

    printf("\n");
    printf("*****************************************************************\n");
    printf("   RP2350 Standalone Flash Write/Erase Speed Benchmark (B-4)    \n");
    printf("*****************************************************************\n");
    printf(" Binary Flash Range : 0x%08" PRIXPTR " - 0x%08" PRIXPTR " (%zu bytes / %zu KB)\n",
           bin_start, bin_end, bin_size, bin_size / 1024);
    printf(" Firmware Limit     : %u KB (0x%08" PRIX32 ")\n",
           (unsigned)(POC_FLASH_FIRMWARE_BYTES / 1024),
           (uint32_t)POC_FLASH_FIRMWARE_BYTES);
    printf(" Staging Area       : [0x%08" PRIX32 ", 0x%08" PRIX32 ") (%u KB)\n",
           (uint32_t)POC_FLASH_STAGING_OFFSET,
           (uint32_t)(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES),
           (unsigned)(POC_FLASH_STAGING_BYTES / 1024));
    printf(" BT Reserved Bank   : [0x%08" PRIX32 ", 0x%08" PRIX32 ") (%u KB)\n",
           (uint32_t)POC_FLASH_BT_RESERVED_OFFSET,
           (uint32_t)POC_FLASH_TOTAL_BYTES,
           (unsigned)(POC_FLASH_BT_RESERVED_BYTES / 1024));
    printf(" Test Region        : [0x%08" PRIX32 ", 0x%08" PRIX32 ") (%u KB)\n",
           (uint32_t)POC_FLASH_BENCH_TEST_OFFSET,
           (uint32_t)(POC_FLASH_BENCH_TEST_OFFSET + POC_FLASH_BENCH_TEST_BYTES),
           (unsigned)(POC_FLASH_BENCH_TEST_BYTES / 1024));
    printf(" Core1 Policy       : PICO_FLASH_ASSUME_CORE1_SAFE=1 (Core1 Dormant)\n");
    printf("*****************************************************************\n");
}

static void print_menu(void) {
    printf("\nAvailable Commands:\n");
    printf("  [f] Run Full Benchmark Suite (Items 1-6) - [Destructive, Requires 'y']\n");
    printf("  [1] Test 4KB Sector Erase (8 sectors)     - [Destructive, Requires 'y']\n");
    printf("  [2] Test 64KB Block Erase (4 blocks)      - [Destructive, Requires 'y']\n");
    printf("  [3] Test 256B Page Program (16 pages)     - [Destructive, Requires 'y']\n");
    printf("  [4] Test 4KB Chunk Program (8 chunks)     - [Destructive, Requires 'y']\n");
    printf("  [5] Test 384KB Strategy Comparison        - [Destructive, Requires 'y']\n");
    printf("  [6] Test 384KB XIP Read Speed & Verify    - [Non-Destructive Read]\n");
    printf("  [h] Show this menu\n");
    printf("\nEnter command: ");
}

static bool confirm_action(const char *action_desc) {
    printf("\n[CONFIRMATION REQUIRED] %s\n", action_desc);
    printf("  Type 'y' to confirm execution, any other key to cancel: ");
    int c = getchar();
    printf("%c\n", (c >= 32 && c <= 126) ? c : ' ');
    if (c == 'y' || c == 'Y') {
        printf("[CONFIRMED] Running operation...\n\n");
        return true;
    }
    printf("[CANCELLED] Operation aborted by user.\n\n");
    return false;
}

int main(void) {
    stdio_init_all();

    /* Allow USB serial CDC terminal to attach */
    sleep_ms(2000);

    print_banner();

    /* Runtime safety check: Binary size must strictly fit inside 1MB firmware window */
    uintptr_t bin_end_rel = (uintptr_t)&__flash_binary_end - (uintptr_t)XIP_BASE;
    if (bin_end_rel > POC_FLASH_FIRMWARE_BYTES) {
        printf("\n[FATAL ERROR] Binary end offset 0x%08" PRIXPTR " exceeds 1MB firmware limit 0x%08" PRIX32 "!\n",
               bin_end_rel, (uint32_t)POC_FLASH_FIRMWARE_BYTES);
        while (1) {
            tight_loop_contents();
        }
    }
    printf("[INIT CHECK] Binary footprint within 1MB window (End Offset: 0x%08" PRIXPTR " <= 0x%08" PRIX32 ") [PASS]\n",
           bin_end_rel, (uint32_t)POC_FLASH_FIRMWARE_BYTES);

    print_menu();

    while (1) {
        int ch = getchar();
        if (ch == EOF || ch == '\r' || ch == '\n') {
            continue;
        }

        printf("%c\n", (ch >= 32 && ch <= 126) ? ch : ' ');

        switch (ch) {
            case 'f':
            case 'F':
                if (confirm_action("Run Complete Flash Benchmark Suite (Items 1-6)")) {
                    flash_bench_run_full_suite();
                }
                break;

            case '1':
                if (confirm_action("Run 4KB Sector Erase Test (8 sectors)")) {
                    bench_stats_t stats;
                    int rc = flash_bench_measure_sector_erase(&stats, 8);
                    if (rc == PICO_OK) {
                        uint64_t avg = bench_stats_avg_us(&stats);
                        printf("[RESULT] 4KB Sector Erase (8 samples):\n");
                        printf("  Min: %" PRIu64 " us, Avg: %" PRIu64 " us (%.2f ms), Max: %" PRIu64 " us (%.2f ms)\n",
                               stats.min_us, avg, (double)avg / 1000.0, stats.max_us, (double)stats.max_us / 1000.0);
                        printf("  Average Throughput: %.1f KB/s\n\n",
                               bench_stats_throughput_kb_s(POC_FLASH_SECTOR_SIZE, avg));
                    } else {
                        printf("[ERROR] Failed with code %d\n\n", rc);
                    }
                }
                break;

            case '2':
                if (confirm_action("Run 64KB Block Erase Test (4 blocks)")) {
                    bench_stats_t stats;
                    int rc = flash_bench_measure_block_erase(&stats, 4);
                    if (rc == PICO_OK) {
                        uint64_t avg = bench_stats_avg_us(&stats);
                        printf("[RESULT] 64KB Block Erase (4 samples):\n");
                        printf("  Min: %" PRIu64 " us (%.2f ms), Avg: %" PRIu64 " us (%.2f ms), Max: %" PRIu64 " us (%.2f ms)\n",
                               stats.min_us, (double)stats.min_us / 1000.0,
                               avg, (double)avg / 1000.0,
                               stats.max_us, (double)stats.max_us / 1000.0);
                        printf("  Average Throughput: %.1f KB/s\n\n",
                               bench_stats_throughput_kb_s(POC_FLASH_BLOCK_SIZE, avg));
                    } else {
                        printf("[ERROR] Failed with code %d\n\n", rc);
                    }
                }
                break;

            case '3':
                if (confirm_action("Run 256B Page Program Test (16 pages)")) {
                    bench_stats_t stats;
                    int rc = flash_bench_measure_page_program(&stats, 16);
                    if (rc == PICO_OK) {
                        uint64_t avg = bench_stats_avg_us(&stats);
                        printf("[RESULT] 256B Page Program (16 samples):\n");
                        printf("  Min: %" PRIu64 " us, Avg: %" PRIu64 " us (%.2f ms), Max: %" PRIu64 " us (%.2f ms)\n",
                               stats.min_us, avg, (double)avg / 1000.0, stats.max_us, (double)stats.max_us / 1000.0);
                        printf("  Average Throughput: %.1f KB/s\n\n",
                               bench_stats_throughput_kb_s(POC_FLASH_PAGE_SIZE, avg));
                    } else {
                        printf("[ERROR] Failed with code %d\n\n", rc);
                    }
                }
                break;

            case '4':
                if (confirm_action("Run 4KB Chunk Program Test (8 chunks)")) {
                    bench_stats_t stats;
                    int rc = flash_bench_measure_chunk_program(&stats, 8);
                    if (rc == PICO_OK) {
                        uint64_t avg = bench_stats_avg_us(&stats);
                        printf("[RESULT] 4KB Chunk Program (8 samples):\n");
                        printf("  Min: %" PRIu64 " us, Avg: %" PRIu64 " us (%.2f ms), Max: %" PRIu64 " us (%.2f ms)\n",
                               stats.min_us, avg, (double)avg / 1000.0, stats.max_us, (double)stats.max_us / 1000.0);
                        printf("  Average Throughput: %.1f KB/s\n\n",
                               bench_stats_throughput_kb_s(POC_FLASH_SECTOR_SIZE, avg));
                    } else {
                        printf("[ERROR] Failed with code %d\n\n", rc);
                    }
                }
                break;

            case '5':
                if (confirm_action("Run 384KB Strategy Comparison ((a), (b), (c))")) {
                    strategy_result_t strats[3];
                    int rc = flash_bench_measure_image_strategies(strats, POC_BENCH_DEFAULT_IMAGE_BYTES);
                    if (rc == PICO_OK) {
                        printf("\n[RESULT] 384KB Flash Strategy Comparison:\n");
                        for (int i = 0; i < 3; ++i) {
                            printf("  Strategy: %s\n", strats[i].strategy_name);
                            printf("    Erase   : %.2f ms\n", (double)strats[i].total_erase_us / 1000.0);
                            printf("    Program : %.2f ms\n", (double)strats[i].total_program_us / 1000.0);
                            printf("    Total   : %.2f ms\n", (double)strats[i].total_elapsed_us / 1000.0);
                            printf("    Max CPU Stall: %.2f ms\n", (double)strats[i].max_stall_us / 1000.0);
                            printf("    Throughput   : %.1f KB/s\n", strats[i].throughput_kb_s);
                            printf("    Verify       : %s (CRC32: 0x%08" PRIX32 ")\n",
                                   strats[i].verified ? "PASS" : "FAIL", strats[i].crc32);
                        }
                        printf("\n");
                    } else {
                        printf("[ERROR] Failed with code %d\n\n", rc);
                    }
                }
                break;

            case '6':
                printf("\n[RUNNING] Measuring 384KB XIP Read Speed...\n");
                {
                    uint64_t read_us = 0;
                    uint32_t read_crc = 0;
                    bool matched = false;
                    int rc = flash_bench_measure_xip_read(POC_FLASH_BENCH_TEST_OFFSET,
                                                          POC_BENCH_DEFAULT_IMAGE_BYTES,
                                                          0x55AA1234u, &read_us, &read_crc, &matched);
                    if (rc == PICO_OK) {
                        double read_mb_s = ((double)POC_BENCH_DEFAULT_IMAGE_BYTES * 1000000.0) /
                                           ((double)read_us * 1024.0 * 1024.0);
                        printf("[RESULT] 384KB XIP Read:\n");
                        printf("  Time Elapsed : %" PRIu64 " us (%.2f ms)\n", read_us, (double)read_us / 1000.0);
                        printf("  Throughput   : %.2f MB/s\n", read_mb_s);
                        printf("  CRC32        : 0x%08" PRIX32 "\n", read_crc);
                        printf("  Verify Status: %s\n\n", matched ? "MATCHED PASS" : "MISMATCH (needs prior write test)");
                    } else {
                        printf("[ERROR] Read failed with code %d\n\n", rc);
                    }
                }
                break;

            case 'h':
            case 'H':
            case '?':
                print_banner();
                print_menu();
                break;

            default:
                printf("[UNKNOWN COMMAND '%c'] Press 'h' for help.\n", (char)ch);
                print_menu();
                break;
        }

        printf("Enter command: ");
    }

    return 0;
}
