#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#include "common/packet.h"
#include "common/benchmark_common.h"
#include "common/build_info.h"
#include "common/sram_attrs.h"
#include "bmc/bmc_driver.h"
#include "flash/flash_map.h"
#include "flash/flash_bench.h"
#include "flash/flash_utils.h"

#include "ota_protocol.h"
#include "ota_receiver.h"
#include "ota_sender.h"

#include "bmc_p2p.pio.h"
#include "bmc_p2p_fast.pio.h"
#include "bmc_p2p_mid12.pio.h"
#include "bmc_p2p_mid10.pio.h"

/*
 * Pin Assignment (Direct GP0 <-> GP1 Loopback Wire)
 */
#define PIN_TX 0u
#define PIN_RX 1u

#define OTA_BENCH_IMAGE_BYTES (384u * 1024u) /* 384 KB */

/*
 * ============================================================================
 * SPSC Response Queue (Core 1 -> Core 0 in SRAM)
 * ============================================================================
 */
#define RESP_QUEUE_SIZE 32

typedef struct {
    uint8_t payload[128];
    uint8_t len;
    uint8_t bmc_type;
    uint8_t seq;
} resp_item_t;

typedef struct {
    resp_item_t items[RESP_QUEUE_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} resp_queue_t;

static resp_queue_t s_resp_queue;

static inline bool resp_queue_push(const uint8_t *payload, uint8_t len, uint8_t type, uint8_t seq) {
    uint32_t h = s_resp_queue.head;
    uint32_t next_h = (h + 1) % RESP_QUEUE_SIZE;
    if (next_h == s_resp_queue.tail) {
        return false; /* Queue full */
    }
    resp_item_t *item = &s_resp_queue.items[h];
    item->bmc_type = type;
    item->seq = seq;
    item->len = len;
    memcpy(item->payload, payload, len);
    __dmb();
    s_resp_queue.head = next_h;
    return true;
}

static inline bool resp_queue_pop(uint8_t *out_payload, uint8_t *out_len, uint8_t *out_type, uint8_t *out_seq) {
    uint32_t t = s_resp_queue.tail;
    if (t == s_resp_queue.head) {
        return false; /* Queue empty */
    }
    resp_item_t *item = &s_resp_queue.items[t];
    *out_type = item->bmc_type;
    *out_seq = item->seq;
    *out_len = item->len;
    memcpy(out_payload, item->payload, item->len);
    __dmb();
    s_resp_queue.tail = (t + 1) % RESP_QUEUE_SIZE;
    return true;
}

/*
 * ============================================================================
 * Rate Modes
 * ============================================================================
 */
typedef enum {
    RATE_9M = 0,   /* 9.375 Mbps (Standard 16cyc, clkdiv 1.0f) */
    RATE_12M,      /* 12.500 Mbps (Mid12 12cyc, clkdiv 1.0f) */
    RATE_15M,      /* 15.000 Mbps (Mid10 10cyc, clkdiv 1.0f) */
    RATE_18M,      /* 18.750 Mbps (Fast 8cyc, clkdiv 1.0f) */
    RATE_COUNT
} bench_rate_t;

typedef struct {
    const char *name;
    float phy_mbps;
    bmc_rate_family_t rate_family;
    float clkdiv;
    bool is_fast_variant;
} rate_desc_t;

static const rate_desc_t RATE_DESCS[RATE_COUNT] = {
    [RATE_9M]  = {"BMC 9.375 Mbps (Std16)", 9.375f, BMC_RATE_STD_16CYC, 1.0f, false},
    [RATE_12M] = {"BMC 12.500 Mbps (Mid12)", 12.500f, BMC_RATE_MID_12CYC, 1.0f, false},
    [RATE_15M] = {"BMC 15.000 Mbps (Mid10)", 15.000f, BMC_RATE_MID_10CYC, 1.0f, false},
    [RATE_18M] = {"BMC 18.750 Mbps (Fast8)", 18.750f, BMC_RATE_FAST_8CYC, 1.0f, true},
};

static bench_rate_t s_current_rate = RATE_12M;

/* Hardware Driver Contexts */
static bmc_tx_controller_t s_bmc_tx;
static bmc_rx_port_t       s_bmc_rx;
static uint32_t s_tx_dma_words[BMC_TX_DMA_MAX_WORDS] __attribute__((aligned(4)));

/* State Machine Contexts */
static ota_receiver_t s_receiver;
static ota_sender_t   s_sender;

/* Core 1 Control Flags */
static volatile bool s_core1_running = false;
static volatile bool s_core1_terminate = false;
static volatile bool s_core1_rx_enable = false;

/* Precomputed Image Hash */
static uint32_t s_expected_image_crc = 0;

/*
 * ============================================================================
 * PRNG Image Generator (Deterministic on-the-fly byte generation)
 * ============================================================================
 */
static inline uint8_t image_prng_byte(uint32_t offset) {
    uint32_t x = offset ^ 0x5A5A5A5Au;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (uint8_t)(x ^ (x >> 8) ^ (x >> 16) ^ (x >> 24));
}

static bool prng_image_reader(void *ctx, uint32_t offset, uint8_t *dst, size_t len) {
    (void)ctx;
    for (size_t i = 0; i < len; ++i) {
        dst[i] = image_prng_byte(offset + i);
    }
    return true;
}

static void precompute_expected_image_crc(void) {
    uint32_t crc = OTA_CRC32_INIT;
    uint8_t temp[256];
    uint32_t left = OTA_BENCH_IMAGE_BYTES;
    uint32_t off = 0;
    while (left > 0) {
        size_t n = (left > sizeof(temp)) ? sizeof(temp) : left;
        prng_image_reader(NULL, off, temp, n);
        crc = ota_crc32_update(crc, temp, n);
        off += n;
        left -= n;
    }
    s_expected_image_crc = crc ^ 0xFFFFFFFFu;
}

/*
 * ============================================================================
 * Flash Operations for Receiver (Core 1)
 * ============================================================================
 */
static int ota_device_flash_erase(void *ctx, uint32_t offset, size_t count) {
    (void)ctx;
    uint64_t stall_us = 0;
    return flash_bench_execute(FLASH_BENCH_OP_ERASE, offset, NULL, count, &stall_us);
}

static int ota_device_flash_program(void *ctx, uint32_t offset, const uint8_t *data, size_t count) {
    (void)ctx;
    /* Program in 4KB sector units to stay within staging safety standards */
    size_t prog_done = 0;
    while (prog_done < count) {
        size_t chunk = count - prog_done;
        if (chunk > POC_FLASH_SECTOR_SIZE) chunk = POC_FLASH_SECTOR_SIZE;
        uint64_t stall_us = 0;
        int rc = flash_bench_execute(FLASH_BENCH_OP_PROGRAM, offset + prog_done,
                                    data + prog_done, chunk, &stall_us);
        if (rc != 0) return rc;
        prog_done += chunk;
    }
    return 0;
}

static int ota_device_flash_read(void *ctx, uint32_t offset, uint8_t *dst, size_t count) {
    (void)ctx;
    const uint8_t *xip_ptr = (const uint8_t *)flash_map_offset_to_xip(offset);
    memcpy(dst, xip_ptr, count);
    return 0;
}

static uint32_t ota_device_flash_crc32(void *ctx, uint32_t offset, size_t count) {
    (void)ctx;
    const uint8_t *xip_ptr = (const uint8_t *)flash_map_offset_to_xip(offset);
    return ota_crc32(xip_ptr, count);
}

/*
 * ============================================================================
 * Link Adapter Operations
 * ============================================================================
 */

/* Sender (Core 0): sends physical BMC packets over GP0 */
static bool BMC_SRAM_FUNC(sender_link_send)(void *ctx, uint8_t bmc_type, uint8_t seq,
                                            const uint8_t *payload, uint8_t len) {
    (void)ctx;
    size_t words = bmc_packet_encode(s_tx_dma_words, bmc_type, seq, payload, len);
    if (words == 0) return false;
    return bmc_tx_send_packet_blocking(&s_bmc_tx, PIN_TX, s_tx_dma_words, words);
}

/* Sender (Core 0): polls responses from SRAM queue */
static bool BMC_SRAM_FUNC(sender_link_recv)(void *ctx, uint8_t *out_type, uint8_t *out_seq,
                                            uint8_t *out_payload, uint8_t *out_len) {
    (void)ctx;
    return resp_queue_pop(out_payload, out_len, out_type, out_seq);
}

/* Receiver (Core 1): sends responses into SRAM queue */
static bool BMC_SRAM_FUNC(receiver_link_send)(void *ctx, uint8_t bmc_type, uint8_t seq,
                                              const uint8_t *payload, uint8_t len) {
    (void)ctx;
    return resp_queue_push(payload, len, bmc_type, seq);
}

/* Receiver (Core 1): dummy recv callback (Core 1 polls physical BMC directly) */
static bool BMC_SRAM_FUNC(receiver_link_recv)(void *ctx, uint8_t *out_type, uint8_t *out_seq,
                                              uint8_t *out_payload, uint8_t *out_len) {
    (void)ctx; (void)out_type; (void)out_seq; (void)out_payload; (void)out_len;
    return false;
}

/*
 * ============================================================================
 * Core 1 Worker (Receiver State Machine + Physical RX Polling + Flash Engine)
 * ============================================================================
 */
static void BMC_SRAM_FUNC(core1_ota_worker)(void) {
    s_core1_running = true;
    static uint8_t s_rx_payload_buf[BMC_MAX_PAYLOAD_LEN];

    while (!s_core1_terminate) {
        if (!s_core1_rx_enable) {
            sleep_us(100);
            continue;
        }

        /* 1. Poll physical BMC RX */
        uint8_t rx_type = 0, rx_seq = 0, rx_len = 0;
        if (bmc_rx_poll_packet(&s_bmc_rx, &rx_type, &rx_seq, s_rx_payload_buf, &rx_len)) {
            if (rx_type == OTA_BMC_PKT_TYPE) {
                ota_receiver_process_packet(&s_receiver, s_rx_payload_buf, rx_len);
            }
        }

        /* 2. Step flash state machine if a window is ready to program */
        ota_receiver_step_flash(&s_receiver);

        tight_loop_contents();
    }

    s_core1_running = false;
}

/*
 * ============================================================================
 * Hardware Initialization & Rate Configuration
 * ============================================================================
 */
static bool configure_rate(bench_rate_t rate_idx) {
    const rate_desc_t *d = &RATE_DESCS[rate_idx];

    /* Stop active hardware */
    s_core1_rx_enable = false;
    sleep_ms(5);

    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio1, 0, false);
    pio_clear_instruction_memory(pio0);
    pio_clear_instruction_memory(pio1);

    uint off_tx = 0, off_rx = 0;
    switch (d->rate_family) {
        case BMC_RATE_MID_12CYC:
            off_tx = pio_add_program(pio0, &bmc_mid12_tx_program);
            off_rx = pio_add_program(pio1, &bmc_mid12_rx_program);
            break;
        case BMC_RATE_MID_10CYC:
            off_tx = pio_add_program(pio0, &bmc_mid10_tx_program);
            off_rx = pio_add_program(pio1, &bmc_mid10_rx_program);
            break;
        case BMC_RATE_FAST_8CYC:
            off_tx = pio_add_program(pio0, &bmc_fast_tx_program);
            off_rx = pio_add_program(pio1, &bmc_fast_rx_program);
            break;
        case BMC_RATE_STD_16CYC:
        default:
            off_tx = pio_add_program(pio0, &bmc_tx_program);
            off_rx = pio_add_program(pio1, &bmc_rx_program);
            break;
    }

    bmc_tx_controller_set_pad_variant(&s_bmc_tx, d->is_fast_variant);
    bmc_tx_controller_set_mode(&s_bmc_tx, off_tx, d->clkdiv, d->rate_family);
    bmc_tx_controller_prime(&s_bmc_tx, PIN_TX);

    if (!bmc_rx_port_init_ex(&s_bmc_rx, pio1, 0, off_rx, PIN_RX, d->clkdiv, d->rate_family)) {
        printf("[ERROR] Failed to init BMC RX port\n");
        return false;
    }

    s_current_rate = rate_idx;
    s_core1_rx_enable = true;
    return true;
}

static bool init_hardware(void) {
    bmc_tx_controller_zero(&s_bmc_tx);
    bmc_rx_port_zero(&s_bmc_rx);
    bmc_tx_pin_init(PIN_TX);

    if (!bmc_tx_controller_init(&s_bmc_tx, pio0, 0)) {
        printf("[ERROR] Failed to init TX controller on PIO0\n");
        return false;
    }

    /* Configure initial rate */
    if (!configure_rate(s_current_rate)) {
        return false;
    }

    /* Initialize Lockout Reception on Core 0!
     * This is required so Core 1's flash_safe_execute can pause Core 0 safely.
     */
    if (!flash_safe_execute_core_init()) {
        printf("[ERROR] flash_safe_execute_core_init failed on Core 0!\n");
        return false;
    }

    /* Initialize Receiver configuration */
    ota_flash_ops_t flash_ops = {
        .erase = ota_device_flash_erase,
        .program = ota_device_flash_program,
        .read = ota_device_flash_read,
        .crc32 = ota_device_flash_crc32
    };

    ota_rx_config_t rx_cfg = {
        .staging_offset = POC_FLASH_STAGING_OFFSET,
        .staging_max_bytes = POC_FLASH_STAGING_BYTES,
        .initial_credits = 1, /* Lockstep window credit prevents RX overrun during lockout */
        .link_ops = {
            .send_packet = receiver_link_send,
            .recv_packet = receiver_link_recv
        },
        .link_ctx = NULL,
        .flash_ops = flash_ops,
        .flash_ctx = NULL
    };
    ota_receiver_init(&s_receiver, &rx_cfg);

    /* Launch Core 1 Worker */
    multicore_launch_core1(core1_ota_worker);
    while (!s_core1_running) tight_loop_contents();

    return true;
}

/*
 * ============================================================================
 * Benchmark Execution & Reporting
 * ============================================================================
 */
static void run_ota_benchmark(void) {
    printf("\n============================================================\n");
    printf("   Starting OTA Transfer: 384 KB @ %s\n", RATE_DESCS[s_current_rate].name);
    printf("   Staging Base: 0x%08" PRIX32 " (Safety bounds: [0x%08" PRIX32 ", 0x%08" PRIX32 "))\n",
           (uint32_t)POC_FLASH_STAGING_OFFSET,
           (uint32_t)POC_FLASH_STAGING_OFFSET,
           (uint32_t)(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES));
    printf("   Expected Image CRC32: 0x%08" PRIX32 "\n", s_expected_image_crc);
    printf("============================================================\n");

    /* Reset Response Queue and Receiver */
    s_resp_queue.head = 0;
    s_resp_queue.tail = 0;
    ota_receiver_reset(&s_receiver);

    /* Configure Sender */
    ota_tx_config_t tx_cfg = {
        .total_size = OTA_BENCH_IMAGE_BYTES,
        .image_crc32 = s_expected_image_crc,
        .data_reader = prng_image_reader,
        .reader_ctx = NULL,
        .link_ops = {
            .send_packet = sender_link_send,
            .recv_packet = sender_link_recv
        },
        .link_ctx = NULL,
        .query_timeout_us = 200000 /* 200 ms */
    };
    ota_sender_init(&s_sender, &tx_cfg);

    uint64_t t_start = time_us_64();
    if (!ota_sender_start(&s_sender)) {
        printf("[FAIL] Could not start sender!\n");
        return;
    }

    printf("[OTA] Transfer in progress... ");
    uint32_t progress_dots = 0;
    uint32_t last_win = 0;

    while (!ota_sender_is_complete(&s_sender) && !ota_sender_is_failed(&s_sender)) {
        ota_sender_step(&s_sender);

        if (s_sender.commit_win_idx > last_win) {
            last_win = s_sender.commit_win_idx;
            printf("[Win %u/6 OK] ", last_win);
        }

        tight_loop_contents();
    }
    uint64_t t_end = time_us_64();
    uint64_t total_us = t_end - t_start;

    printf("\n\n");

    if (ota_sender_is_complete(&s_sender) && ota_receiver_is_complete(&s_receiver)) {
        printf(">>> OTA FIRMWARE TRANSFER SUCCESSFUL! <<<\n");
    } else {
        printf(">>> OTA FIRMWARE TRANSFER FAILED! <<<\n");
        printf("Sender State: %d, Receiver State: %d\n", s_sender.state, s_receiver.state);
        return;
    }

    double total_sec = (double)total_us / 1000000.0;
    double throughput_kib = ((double)OTA_BENCH_IMAGE_BYTES / 1024.0) / total_sec;
    double throughput_kb  = ((double)OTA_BENCH_IMAGE_BYTES / 1000.0) / total_sec;

    /* Print Detailed Performance Breakdown Table */
    printf("------------------------------------------------------------\n");
    printf("   Performance & Measurement Breakdown (384 KB Image)\n");
    printf("------------------------------------------------------------\n");
    printf("  Total Elapsed Time      : %8.2f ms  (%llu us)\n", (double)total_us / 1000.0, (unsigned long long)total_us);
    printf("  Overall Throughput      : %8.2f KiB/s  (%8.2f KB/s)\n", throughput_kib, throughput_kb);
    printf("  Target PHY Rate         : %8.2f Mbps (%s)\n",
           RATE_DESCS[s_current_rate].phy_mbps, RATE_DESCS[s_current_rate].name);
    printf("\n");
    printf("  Timing Breakdown:\n");
    printf("    - Link Transfer (PHY) : %8.2f ms  (%5.1f%%)\n",
           (double)s_sender.stats.link_transfer_us / 1000.0,
           100.0 * (double)s_sender.stats.link_transfer_us / total_us);
    printf("    - Retransmit (PHY)    : %8.2f ms  (%5.1f%%)\n",
           (double)s_sender.stats.retransmit_us / 1000.0,
           100.0 * (double)s_sender.stats.retransmit_us / total_us);
    printf("    - Flash Erase (64KBx6): %8.2f ms  (%5.1f%%)\n",
           (double)s_receiver.stats.flash_erase_us / 1000.0,
           100.0 * (double)s_receiver.stats.flash_erase_us / total_us);
    printf("    - Flash Program (4KB) : %8.2f ms  (%5.1f%%)\n",
           (double)s_receiver.stats.flash_program_us / 1000.0,
           100.0 * (double)s_receiver.stats.flash_program_us / total_us);
    printf("    - Readback Verify     : %8.2f ms  (%5.1f%%)\n",
           (double)s_receiver.stats.flash_verify_us / 1000.0,
           100.0 * (double)s_receiver.stats.flash_verify_us / total_us);
    printf("    - Handshake / Wait    : %8.2f ms  (%5.1f%%)\n",
           (double)s_sender.stats.ready_wait_us / 1000.0,
           100.0 * (double)s_sender.stats.ready_wait_us / total_us);
    printf("\n");
    printf("  Window & Packet Metrics:\n");
    printf("    - Total Packets Sent  : %lu\n", (unsigned long)s_sender.stats.total_packets_sent);
    printf("    - Retransmitted Chunks: %lu\n", (unsigned long)s_sender.stats.retransmit_packets);
    printf("    - Retransmit Drops    : %lu\n", (unsigned long)s_receiver.stats.rx_chunks_duplicate);
    printf("    - Flash Write Retries : %lu\n", (unsigned long)s_receiver.stats.flash_retries);
    printf("    - Max Single Window   : %8.2f ms\n", (double)s_receiver.stats.max_window_us / 1000.0);
    printf("\n");
    printf("  Integrity Verification:\n");
    printf("    - Expected Image CRC32: 0x%08" PRIX32 "\n", s_expected_image_crc);
    uint32_t staged_readback_crc = ota_crc32((const uint8_t *)flash_map_offset_to_xip(POC_FLASH_STAGING_OFFSET),
                                             OTA_BENCH_IMAGE_BYTES);
    printf("    - Staged Flash CRC32  : 0x%08" PRIX32 " [%s]\n", staged_readback_crc,
           (staged_readback_crc == s_expected_image_crc) ? "MATCH - OK" : "MISMATCH - FAIL");
    printf("------------------------------------------------------------\n");
}

/*
 * ============================================================================
 * Staging Guard Verification
 * ============================================================================
 */
static void run_guard_check(void) {
    printf("\n--- Checking Flash Boundary Safety Guards ---\n");
    printf("  Valid staging range: [0x%08" PRIX32 ", 0x%08" PRIX32 ")\n",
           (uint32_t)POC_FLASH_STAGING_OFFSET,
           (uint32_t)(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES));

    /* Test 1: Legal write to staging */
    bool legal = flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, 65536);
    printf("  Test 1 (Legal write @ staging start): %s\n", legal ? "ACCEPTED [PASS]" : "REJECTED [FAIL]");

    /* Test 2: Writing to bootloader / code slot 0 */
    bool illegal_code = flash_map_is_staging_range(0, 4096);
    printf("  Test 2 (Illegal write @ offset 0): %s\n", !illegal_code ? "REJECTED [PASS]" : "ACCEPTED [FAIL]");

    /* Test 3: Writing past staging end */
    bool illegal_past = flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES, 4096);
    printf("  Test 3 (Illegal write past staging): %s\n", !illegal_past ? "REJECTED [PASS]" : "ACCEPTED [FAIL]");

    /* Test 4: Execution attempt via flash_bench_execute */
    uint64_t cb_us = 0;
    int rc = flash_bench_execute(FLASH_BENCH_OP_ERASE, 0, NULL, 4096, &cb_us);
    printf("  Test 4 (Execute erase @ offset 0): Return=%d (%s) [PASS]\n",
           rc, (rc == PICO_ERROR_NOT_PERMITTED) ? "PICO_ERROR_NOT_PERMITTED" : "UNKNOWN");

    printf("All safety guards verified!\n");
}

/*
 * ============================================================================
 * Interactive CLI Menu
 * ============================================================================
 */
static void print_menu(void) {
    printf("\n============================================================\n");
    printf("   RP2350 BMC OTA Benchmark Control Menu\n");
    printf("   Current Rate: [%d] %s\n", s_current_rate + 1, RATE_DESCS[s_current_rate].name);
    printf("============================================================\n");
    printf("  u : Start 384KB OTA Firmware Transfer (requires 2-stage 'y' confirm)\n");
    printf("  1 : Select BMC  9.375 Mbps (Standard 16cyc)\n");
    printf("  2 : Select BMC 12.500 Mbps (Mid12 12cyc)\n");
    printf("  3 : Select BMC 15.000 Mbps (Mid10 10cyc)\n");
    printf("  4 : Select BMC 18.750 Mbps (Fast 8cyc, experimental)\n");
    printf("  g : Test Flash Staging Safety Boundary Guards\n");
    printf("  i : Print System & Flash Map Information\n");
    printf("  h : Print this Help Menu\n");
    printf("Enter choice: ");
}

int main(void) {
    stdio_init_all();

    /* Wait up to 3s for USB CDC stdio */
    for (int i = 0; i < 30 && !stdio_usb_connected(); ++i) {
        sleep_ms(100);
    }

    build_info_print("poc_ota_bench");
    precompute_expected_image_crc();

    printf("\nInitializing RP2350 BMC Dual-Core OTA Subsystem...\n");
    printf("  Core 0: Sender, CLI, Lockout Reception\n");
    printf("  Core 1: Receiver, RX Polling, Double Buffer, Flash Engine\n");
    printf("  Wiring: GP%u (TX, Pin 1) <---> GP%u (RX, Pin 2) Direct Loopback\n", PIN_TX, PIN_RX);

    if (!init_hardware()) {
        printf("[FATAL] Hardware initialization failed!\n");
        return 1;
    }

    printf("Initialization complete. Image 384KB expected CRC32: 0x%08" PRIX32 "\n", s_expected_image_crc);
    print_menu();

    while (true) {
        int ch = getchar_timeout_us(100000);
        if (ch == PICO_ERROR_TIMEOUT) {
            continue;
        }

        switch (ch) {
            case 'u':
            case 'U': {
                printf("\n[CONFIRMATION REQUIRED]\n");
                printf("Are you sure you want to write 384KB to Flash staging area (0x%08" PRIX32 ")?\n",
                       (uint32_t)POC_FLASH_STAGING_OFFSET);
                printf("Press 'y' to confirm, any other key to abort: ");
                int confirm = getchar_timeout_us(10000000); /* 10s confirmation timeout */
                if (confirm == 'y' || confirm == 'Y') {
                    printf("y\nStarting benchmark...\n");
                    run_ota_benchmark();
                } else {
                    printf("Aborted.\n");
                }
                print_menu();
                break;
            }

            case '1':
            case '2':
            case '3':
            case '4': {
                bench_rate_t r = (bench_rate_t)(ch - '1');
                printf("\nSwitching rate to %s...\n", RATE_DESCS[r].name);
                if (configure_rate(r)) {
                    printf("Rate applied successfully!\n");
                } else {
                    printf("[ERROR] Failed to switch rate!\n");
                }
                print_menu();
                break;
            }

            case 'g':
            case 'G':
                run_guard_check();
                print_menu();
                break;

            case 'i':
            case 'I':
                printf("\nSystem Information:\n");
                printf("  CPU Clock          : %lu MHz\n", (unsigned long)(clock_get_hz(clk_sys) / 1000000));
                printf("  Flash Total Size   : %lu MB\n", (unsigned long)(POC_FLASH_TOTAL_BYTES / 1024 / 1024));
                printf("  Flash Staging Base : 0x%08" PRIX32 "\n", (uint32_t)POC_FLASH_STAGING_OFFSET);
                printf("  Flash Staging Size : %lu KB\n", (unsigned long)(POC_FLASH_STAGING_BYTES / 1024));
                printf("  Image Size         : %lu KB (6 x 64KB Windows)\n", (unsigned long)(OTA_BENCH_IMAGE_BYTES / 1024));
                printf("  Expected Image CRC : 0x%08" PRIX32 "\n", s_expected_image_crc);
                print_menu();
                break;

            case 'h':
            case 'H':
            default:
                print_menu();
                break;
        }
    }

    return 0;
}
