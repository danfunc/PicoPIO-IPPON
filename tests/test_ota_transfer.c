#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "ota/ota_protocol.h"
#include "ota/ota_receiver.h"
#include "ota/ota_sender.h"
#include "flash/flash_map.h"

/*
 * ============================================================================
 * Mock Flash Implementation for Host Testing
 * ============================================================================
 */
#define MOCK_FLASH_TOTAL_SIZE (4u * 1024u * 1024u) /* 4 MB */
static uint8_t s_mock_flash[MOCK_FLASH_TOTAL_SIZE];

typedef struct {
    int   corrupt_window_idx;     /* Window index to deliberately corrupt on first attempt */
    int   corrupt_attempts_left;  /* How many times to corrupt before succeeding */
    int   erase_calls;
    int   program_calls;
    int   crc_calls;
} mock_flash_ctx_t;

static int mock_flash_erase(void *ctx, uint32_t offset, size_t count) {
    mock_flash_ctx_t *m = (mock_flash_ctx_t *)ctx;
    m->erase_calls++;
    if (offset + count > MOCK_FLASH_TOTAL_SIZE) return -1;
    memset(s_mock_flash + offset, 0xFF, count);
    return 0;
}

static int mock_flash_program(void *ctx, uint32_t offset, const uint8_t *data, size_t count) {
    mock_flash_ctx_t *m = (mock_flash_ctx_t *)ctx;
    m->program_calls++;
    if (offset + count > MOCK_FLASH_TOTAL_SIZE) return -1;
    memcpy(s_mock_flash + offset, data, count);
    return 0;
}

static int mock_flash_read(void *ctx, uint32_t offset, uint8_t *dst, size_t count) {
    (void)ctx;
    if (offset + count > MOCK_FLASH_TOTAL_SIZE) return -1;
    memcpy(dst, s_mock_flash + offset, count);
    return 0;
}

static uint32_t mock_flash_crc32(void *ctx, uint32_t offset, size_t count) {
    mock_flash_ctx_t *m = (mock_flash_ctx_t *)ctx;
    m->crc_calls++;
    if (offset + count > MOCK_FLASH_TOTAL_SIZE) return 0;

    uint32_t calculated = ota_crc32(s_mock_flash + offset, count);

    /* Fault injection: deliberate readback corruption */
    if (m->corrupt_attempts_left > 0) {
        uint32_t target_win = (offset - POC_FLASH_STAGING_OFFSET) / OTA_WINDOW_SIZE;
        if ((int)target_win == m->corrupt_window_idx) {
            m->corrupt_attempts_left--;
            printf("    [FAULT_INJECTION] Corrupting flash readback CRC on win=%d! (Remaining faults: %d)\n",
                   target_win, m->corrupt_attempts_left);
            return calculated ^ 0xDEADBEEFu;
        }
    }

    return calculated;
}

/*
 * ============================================================================
 * Mock Bidirectional Link with Loss & Duplication Simulation
 * ============================================================================
 */
#define MOCK_QUEUE_CAPACITY 4096

typedef struct {
    uint8_t payload[128];
    uint8_t len;
    uint8_t bmc_type;
    uint8_t seq;
} mock_pkt_t;

typedef struct {
    mock_pkt_t pkts[MOCK_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
} mock_queue_t;

static void queue_push(mock_queue_t *q, const mock_pkt_t *p) {
    assert(q->count < MOCK_QUEUE_CAPACITY);
    q->pkts[q->head] = *p;
    q->head = (q->head + 1) % MOCK_QUEUE_CAPACITY;
    q->count++;
}

static bool queue_pop(mock_queue_t *q, mock_pkt_t *out) {
    if (q->count == 0) return false;
    *out = q->pkts[q->tail];
    q->tail = (q->tail + 1) % MOCK_QUEUE_CAPACITY;
    q->count--;
    return true;
}

typedef struct {
    mock_queue_t fwd_queue; /* Sender -> Receiver */
    mock_queue_t rev_queue; /* Receiver -> Sender */
    uint32_t prng_state;
    uint32_t loss_pct;       /* 0..100 */
    uint32_t dup_pct;        /* 0..100 */
    uint32_t dropped_count;
    uint32_t duplicated_count;
} mock_link_t;

static uint32_t mock_prng_next(uint32_t *st) {
    uint32_t x = *st;
    if (x == 0) x = 0x12345678u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *st = x;
    return x;
}

static bool link_sender_send(void *ctx, uint8_t bmc_type, uint8_t seq,
                             const uint8_t *payload, uint8_t len) {
    mock_link_t *link = (mock_link_t *)ctx;
    mock_pkt_t pkt;
    pkt.bmc_type = bmc_type;
    pkt.seq = seq;
    pkt.len = len;
    memcpy(pkt.payload, payload, len);

    /* Packet loss simulation (apply only to DATA packets to test XNOR recovery) */
    if (payload[0] == OTA_PKT_TYPE_DATA && link->loss_pct > 0) {
        uint32_t r = mock_prng_next(&link->prng_state) % 100;
        if (r < link->loss_pct) {
            link->dropped_count++;
            return true; /* Dropped silently */
        }
    }

    /* Packet duplication simulation */
    if (payload[0] == OTA_PKT_TYPE_DATA && link->dup_pct > 0) {
        uint32_t r = mock_prng_next(&link->prng_state) % 100;
        if (r < link->dup_pct) {
            link->duplicated_count++;
            queue_push(&link->fwd_queue, &pkt);
        }
    }

    queue_push(&link->fwd_queue, &pkt);
    return true;
}

static bool link_sender_recv(void *ctx, uint8_t *out_bmc_type, uint8_t *out_seq,
                             uint8_t *out_payload, uint8_t *out_len) {
    mock_link_t *link = (mock_link_t *)ctx;
    mock_pkt_t pkt;
    if (!queue_pop(&link->rev_queue, &pkt)) return false;
    *out_bmc_type = pkt.bmc_type;
    *out_seq = pkt.seq;
    *out_len = pkt.len;
    memcpy(out_payload, pkt.payload, pkt.len);
    return true;
}

static bool link_receiver_send(void *ctx, uint8_t bmc_type, uint8_t seq,
                               const uint8_t *payload, uint8_t len) {
    mock_link_t *link = (mock_link_t *)ctx;
    mock_pkt_t pkt;
    pkt.bmc_type = bmc_type;
    pkt.seq = seq;
    pkt.len = len;
    memcpy(pkt.payload, payload, len);
    queue_push(&link->rev_queue, &pkt);
    return true;
}

static bool link_receiver_recv(void *ctx, uint8_t *out_bmc_type, uint8_t *out_seq,
                               uint8_t *out_payload, uint8_t *out_len) {
    mock_link_t *link = (mock_link_t *)ctx;
    mock_pkt_t pkt;
    if (!queue_pop(&link->fwd_queue, &pkt)) return false;
    *out_bmc_type = pkt.bmc_type;
    *out_seq = pkt.seq;
    *out_len = pkt.len;
    memcpy(out_payload, pkt.payload, pkt.len);
    return true;
}

/*
 * ============================================================================
 * Image Generation & Verification Helpers
 * ============================================================================
 */
static void generate_test_image(uint8_t *buf, size_t size, uint32_t seed) {
    uint32_t st = seed;
    for (size_t i = 0; i < size; ++i) {
        buf[i] = (uint8_t)(mock_prng_next(&st) & 0xFF);
    }
}

typedef struct {
    const uint8_t *image;
    size_t total_size;
} buffer_reader_ctx_t;

static bool buffer_data_reader(void *ctx, uint32_t offset, uint8_t *dst, size_t len) {
    buffer_reader_ctx_t *b = (buffer_reader_ctx_t *)ctx;
    if (offset + len > b->total_size) return false;
    memcpy(dst, b->image + offset, len);
    return true;
}

/*
 * ============================================================================
 * Test Runner Loop
 * ============================================================================
 */
static bool run_full_transfer(size_t image_size, uint32_t seed, uint32_t loss_pct,
                              uint32_t dup_pct, int corrupt_win, int corrupt_count,
                              ota_tx_stats_t *out_tx_stats, ota_rx_stats_t *out_rx_stats) {
    uint8_t *source_image = (uint8_t *)malloc(image_size);
    assert(source_image != NULL);
    generate_test_image(source_image, image_size, seed);
    uint32_t expected_crc = ota_crc32(source_image, image_size);

    /* Initialize Mock Flash */
    memset(s_mock_flash, 0xFF, sizeof(s_mock_flash));
    mock_flash_ctx_t flash_ctx = {
        .corrupt_window_idx = corrupt_win,
        .corrupt_attempts_left = corrupt_count,
        .erase_calls = 0,
        .program_calls = 0,
        .crc_calls = 0
    };
    ota_flash_ops_t flash_ops = {
        .erase = mock_flash_erase,
        .program = mock_flash_program,
        .read = mock_flash_read,
        .crc32 = mock_flash_crc32
    };

    /* Initialize Mock Link */
    mock_link_t link;
    memset(&link, 0, sizeof(link));
    link.prng_state = seed ^ 0xA5A5A5A5u;
    link.loss_pct = loss_pct;
    link.dup_pct = dup_pct;

    /* Initialize Receiver */
    ota_rx_config_t rx_cfg = {
        .staging_offset = POC_FLASH_STAGING_OFFSET,
        .staging_max_bytes = POC_FLASH_STAGING_BYTES,
        .initial_credits = 2, /* Pipelined double buffering */
        .link_ops = {
            .send_packet = link_receiver_send,
            .recv_packet = link_receiver_recv
        },
        .link_ctx = &link,
        .flash_ops = flash_ops,
        .flash_ctx = &flash_ctx
    };
    ota_receiver_t rx;
    ota_receiver_init(&rx, &rx_cfg);

    /* Initialize Sender */
    buffer_reader_ctx_t reader_ctx = {
        .image = source_image,
        .total_size = image_size
    };
    ota_tx_config_t tx_cfg = {
        .total_size = (uint32_t)image_size,
        .image_crc32 = expected_crc,
        .data_reader = buffer_data_reader,
        .reader_ctx = &reader_ctx,
        .link_ops = {
            .send_packet = link_sender_send,
            .recv_packet = link_sender_recv
        },
        .link_ctx = &link,
        .query_timeout_us = 50000 /* 50ms for speedy unit test */
    };
    ota_sender_t tx;
    ota_sender_init(&tx, &tx_cfg);

    bool start_ok = ota_sender_start(&tx);
    assert(start_ok);

    /* Main execution event loop */
    uint32_t iterations = 0;
    const uint32_t MAX_ITERATIONS = 500000;

    while (iterations++ < MAX_ITERATIONS) {
        /* Step Sender */
        ota_sender_step(&tx);

        /* Step Receiver Network Input */
        uint8_t rx_bmc_type = 0, rx_seq = 0, rx_len = 0;
        uint8_t rx_buf[128];
        while (rx_cfg.link_ops.recv_packet(rx_cfg.link_ctx, &rx_bmc_type, &rx_seq, rx_buf, &rx_len)) {
            ota_receiver_process_packet(&rx, rx_buf, rx_len);
        }

        /* Step Receiver Flash Operations */
        ota_receiver_step_flash(&rx);

        /* Check Completion */
        if (ota_sender_is_complete(&tx) && ota_receiver_is_complete(&rx)) {
            break;
        }

        if (ota_sender_is_failed(&tx) || ota_receiver_is_aborted(&rx)) {
            break;
        }
    }

    if (out_tx_stats) *out_tx_stats = tx.stats;
    if (out_rx_stats) *out_rx_stats = rx.stats;

    /* Verify correctness */
    bool success = ota_sender_is_complete(&tx) && ota_receiver_is_complete(&rx);
    if (success) {
        /* 1. Verify staged image data against source image */
        int cmp = memcmp(s_mock_flash + POC_FLASH_STAGING_OFFSET, source_image, image_size);
        assert(cmp == 0);

        /* 2. Verify CRC32 of staged flash */
        uint32_t staged_crc = ota_crc32(s_mock_flash + POC_FLASH_STAGING_OFFSET, image_size);
        assert(staged_crc == expected_crc);
    }

    free(source_image);
    return success;
}

/*
 * ============================================================================
 * Individual Test Cases
 * ============================================================================
 */

/* Test 1: 384KB Happy Path (0% loss) */
static void test_384kb_loss_0(void) {
    printf("[TEST 1] 384KB transfer with 0%% packet loss (Happy Path)...\n");
    ota_tx_stats_t tx_st;
    ota_rx_stats_t rx_st;
    bool ok = run_full_transfer(384 * 1024, 0x11223344, 0, 0, -1, 0, &tx_st, &rx_st);
    assert(ok);
    assert(tx_st.windows_completed == 6);
    assert(tx_st.retransmit_packets == 0);
    assert(rx_st.windows_flashed == 6);
    assert(rx_st.flash_retries == 0);
    printf("  [PASS] 384KB transfer completed in %lu total packets, 0 retransmits, all 6 windows verified!\n",
           (unsigned long)tx_st.total_packets_sent);
}

/* Test 2: 384KB with 1% Packet Loss */
static void test_384kb_loss_1(void) {
    printf("[TEST 2] 384KB transfer with 1%% packet loss (XNOR missing retransmission)...\n");
    ota_tx_stats_t tx_st;
    ota_rx_stats_t rx_st;
    bool ok = run_full_transfer(384 * 1024, 0x55667788, 1, 0, -1, 0, &tx_st, &rx_st);
    assert(ok);
    assert(tx_st.windows_completed == 6);
    assert(tx_st.retransmit_packets > 0);
    assert(rx_st.windows_flashed == 6);
    printf("  [PASS] 384KB transfer recovered from 1%% loss! Retransmitted %lu chunks, full CRC matched!\n",
           (unsigned long)tx_st.retransmit_packets);
}

/* Test 3: 384KB with 10% Heavy Packet Loss */
static void test_384kb_loss_10(void) {
    printf("[TEST 3] 384KB transfer with 10%% heavy packet loss...\n");
    ota_tx_stats_t tx_st;
    ota_rx_stats_t rx_st;
    bool ok = run_full_transfer(384 * 1024, 0x99AABBCC, 10, 0, -1, 0, &tx_st, &rx_st);
    assert(ok);
    assert(tx_st.windows_completed == 6);
    assert(tx_st.retransmit_packets > 100);
    assert(rx_st.windows_flashed == 6);
    printf("  [PASS] 384KB transfer recovered from 10%% loss! Retransmitted %lu chunks successfully!\n",
           (unsigned long)tx_st.retransmit_packets);
}

/* Test 4: Fractional Size (300KB = 4 full windows + 1 partial window of 45,056 bytes) */
static void test_fractional_size_300kb(void) {
    printf("[TEST 4] Fractional image size (300KB = 4 full windows + 1 partial window)...\n");
    ota_tx_stats_t tx_st;
    ota_rx_stats_t rx_st;
    bool ok = run_full_transfer(300 * 1024, 0xCAFEBABE, 1, 0, -1, 0, &tx_st, &rx_st);
    assert(ok);
    assert(tx_st.windows_completed == 5);
    assert(rx_st.windows_flashed == 5);
    printf("  [PASS] 300KB fractional transfer verified! 5 windows (4 full + 1 partial) matched exactly!\n");
}

/* Test 5: Duplicate Packet Arrival */
static void test_duplicate_packets(void) {
    printf("[TEST 5] Packet duplication tolerance (5%% duplicate packet arrival)...\n");
    ota_tx_stats_t tx_st;
    ota_rx_stats_t rx_st;
    bool ok = run_full_transfer(128 * 1024, 0x12344321, 0, 5, -1, 0, &tx_st, &rx_st);
    assert(ok);
    assert(rx_st.rx_chunks_duplicate > 0);
    assert(tx_st.windows_completed == 2);
    printf("  [PASS] Duplicates handled correctly (%lu duplicate chunks filtered by bitmap)!\n",
           (unsigned long)rx_st.rx_chunks_duplicate);
}

/* Test 6: Flash Readback Verification Failure Injection & Recovery */
static void test_flash_readback_failure_recovery(void) {
    printf("[TEST 6] Flash readback CRC mismatch injection on window 2 & recovery...\n");
    ota_tx_stats_t tx_st;
    ota_rx_stats_t rx_st;
    /* Deliberately corrupt window 2 readback once, then succeed on retry */
    bool ok = run_full_transfer(256 * 1024, 0xFEEDFACE, 0, 0, 2, 1, &tx_st, &rx_st);
    assert(ok);
    assert(rx_st.flash_retries == 1);
    assert(tx_st.windows_completed == 4);
    assert(rx_st.windows_flashed == 4);
    printf("  [PASS] Flash retry recovery succeeded! Window 2 retried and committed cleanly!\n");
}

/* Test 7: Double Buffer Concurrency Verification */
static void test_double_buffer_concurrency(void) {
    printf("[TEST 7] Verifying double buffer concurrency (simultaneous flash and receive)...\n");

    /* Create receiver with credits = 2 */
    mock_link_t link;
    memset(&link, 0, sizeof(link));
    mock_flash_ctx_t flash_ctx;
    memset(&flash_ctx, 0, sizeof(flash_ctx));

    ota_flash_ops_t flash_ops = {
        .erase = mock_flash_erase,
        .program = mock_flash_program,
        .read = mock_flash_read,
        .crc32 = mock_flash_crc32
    };

    ota_rx_config_t rx_cfg = {
        .staging_offset = POC_FLASH_STAGING_OFFSET,
        .staging_max_bytes = POC_FLASH_STAGING_BYTES,
        .initial_credits = 2,
        .link_ops = { .send_packet = link_receiver_send, .recv_packet = link_receiver_recv },
        .link_ctx = &link,
        .flash_ops = flash_ops,
        .flash_ctx = &flash_ctx
    };

    ota_receiver_t rx;
    ota_receiver_init(&rx, &rx_cfg);

    /* 1. Send START */
    ota_start_pkt_t start = {
        .pkt_type = OTA_PKT_TYPE_START,
        .reserved = 0,
        .magic = OTA_MAGIC_START,
        .total_size = 128 * 1024,
        .image_crc32 = 0x12345678,
        .total_windows = 2,
        .chunk_data_size = OTA_CHUNK_DATA_SIZE
    };
    bool p_ok = ota_receiver_process_packet(&rx, (const uint8_t *)&start, sizeof(start));
    assert(p_ok);
    assert(rx.state == OTA_RX_STATE_TRANSFER);

    /* 2. Feed all chunks of Window 0 into slot 0 */
    ota_data_pkt_t data_pkt;
    data_pkt.pkt_type = OTA_PKT_TYPE_DATA;
    data_pkt.win_idx = 0;
    memset(data_pkt.data, 0xAA, sizeof(data_pkt.data));
    for (uint16_t c = 0; c < OTA_CHUNKS_PER_WINDOW; ++c) {
        data_pkt.chunk_idx = c;
        ota_receiver_process_packet(&rx, (const uint8_t *)&data_pkt, sizeof(data_pkt));
    }

    /* 3. Query Window 0 -> slot 0 transitions to READY_TO_FLASH */
    uint32_t exp_win0_crc = ota_crc32(rx.slots[0].buffer, OTA_WINDOW_SIZE);
    ota_query_pkt_t q0 = {
        .pkt_type = OTA_PKT_TYPE_QUERY,
        .win_idx = 0,
        .reserved = 0,
        .win_crc32 = exp_win0_crc,
        .win_bytes = OTA_WINDOW_SIZE
    };
    ota_receiver_process_packet(&rx, (const uint8_t *)&q0, sizeof(q0));
    assert(rx.slots[0].state == OTA_SLOT_READY_TO_FLASH);

    /* 4. NOW: while Slot 0 is in READY_TO_FLASH, feed chunks of Window 1 into Slot 1! */
    data_pkt.win_idx = 1;
    memset(data_pkt.data, 0xBB, sizeof(data_pkt.data));
    for (uint16_t c = 0; c < 50; ++c) {
        data_pkt.chunk_idx = c;
        ota_receiver_process_packet(&rx, (const uint8_t *)&data_pkt, sizeof(data_pkt));
    }

    /* Verify Slot 1 is actively receiving while Slot 0 is READY_TO_FLASH */
    assert(rx.slots[0].state == OTA_SLOT_READY_TO_FLASH);
    assert(rx.slots[1].state == OTA_SLOT_RECEIVING);
    assert(rx.slots[1].received_chunks == 50);
    assert(rx.slots[0].win_idx == 0);
    assert(rx.slots[1].win_idx == 1);

    /* Step flash for Slot 0 */
    bool flashed = ota_receiver_step_flash(&rx);
    assert(flashed);
    assert(rx.windows_committed == 1);
    /* Slot 0 is now committed/free, while Slot 1 still preserves its 50 chunks */
    assert(rx.slots[1].received_chunks == 50);
    assert(rx.slots[1].state == OTA_SLOT_RECEIVING);

    printf("  [PASS] Double buffer concurrency verified! Window 1 received chunks while Window 0 was flashing!\n");
}

/* Test 8: Staging Boundary Hard Guard Check */
static void test_staging_boundary_guard(void) {
    printf("[TEST 8] Staging boundary safety guard rejection...\n");

    /* Staging range is [1MB, 4MB - 12KB) */
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, 65536) == true);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES - 65536, 65536) == true);

    /* Writing below staging (e.g. bootloader or main firmware slot at offset 0) MUST be rejected */
    assert(flash_map_is_staging_range(0, 4096) == false);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET - 4096, 4096) == false);

    /* Writing past staging boundary MUST be rejected */
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET + POC_FLASH_STAGING_BYTES, 4096) == false);
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, POC_FLASH_STAGING_BYTES + 4096) == false);

    /* Zero count MUST be rejected */
    assert(flash_map_is_staging_range(POC_FLASH_STAGING_OFFSET, 0) == false);

    printf("  [PASS] Staging boundary guards firmly rejected invalid ranges!\n");
}

int main(void) {
    printf("\n============================================================\n");
    printf("   Running BMC OTA Protocol Host Verification Suite\n");
    printf("============================================================\n");

    test_384kb_loss_0();
    test_384kb_loss_1();
    test_384kb_loss_10();
    test_fractional_size_300kb();
    test_duplicate_packets();
    test_flash_readback_failure_recovery();
    test_double_buffer_concurrency();
    test_staging_boundary_guard();

    printf("\n>>> ALL 8 OTA HOST UNIT TESTS PASSED SUCCESSFULLY! <<<\n\n");
    return 0;
}
