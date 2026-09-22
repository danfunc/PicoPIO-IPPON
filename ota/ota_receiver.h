#ifndef POC_OTA_RECEIVER_H
#define POC_OTA_RECEIVER_H

#include "ota_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Receiver State & Slot Definitions
 * ============================================================================
 */

typedef enum {
    OTA_RX_STATE_IDLE = 0,
    OTA_RX_STATE_TRANSFER,
    OTA_RX_STATE_COMPLETE,
    OTA_RX_STATE_ABORTED,
} ota_rx_state_t;

typedef enum {
    OTA_SLOT_FREE = 0,
    OTA_SLOT_RECEIVING,
    OTA_SLOT_READY_TO_FLASH,
    OTA_SLOT_FLASHING,
    OTA_SLOT_COMMITTED,
    OTA_SLOT_ERROR,
} ota_slot_state_t;

/* Double Buffer Slot (64KB SRAM buffer + bitmap) */
typedef struct {
    uint8_t          buffer[OTA_WINDOW_SIZE] __attribute__((aligned(4)));
    uint8_t          bitmap[OTA_BITMAP_BYTES];
    uint16_t         win_idx;
    uint16_t         total_chunks;
    uint32_t         win_bytes;
    uint32_t         expected_crc32;
    uint32_t         received_chunks;
    uint8_t          retries;
    ota_slot_state_t state;
} ota_window_slot_t;

/* Receiver Statistics */
typedef struct {
    uint32_t rx_chunks_total;
    uint32_t rx_chunks_duplicate;
    uint32_t rx_queries;
    uint32_t flash_retries;
    uint32_t windows_flashed;
    uint64_t flash_erase_us;
    uint64_t flash_program_us;
    uint64_t flash_verify_us;
    uint64_t total_flash_us;
    uint64_t max_window_us;
} ota_rx_stats_t;

/* Receiver Configuration */
typedef struct {
    uint32_t staging_offset;      /* Flash offset for staging area */
    uint32_t staging_max_bytes;   /* Maximum staging area size */
    uint16_t initial_credits;     /* 1 for lockstep, 2 for pipelined double buffer */
    ota_link_ops_t  link_ops;     /* Link interface for sending responses */
    void           *link_ctx;
    ota_flash_ops_t flash_ops;    /* Flash operations interface */
    void           *flash_ctx;
} ota_rx_config_t;

/* Receiver Context */
typedef struct {
    ota_rx_config_t   config;
    ota_rx_state_t    state;
    uint32_t          total_size;
    uint32_t          image_crc32;
    uint16_t          total_windows;
    uint16_t          windows_committed;
    uint8_t           tx_seq;
    ota_window_slot_t slots[2];   /* 2 x 64KB double buffer */
    ota_rx_stats_t    stats;
} ota_receiver_t;

/*
 * ============================================================================
 * Receiver API Functions
 * ============================================================================
 */

/**
 * \brief Initialize OTA receiver context.
 */
void ota_receiver_init(ota_receiver_t *rx, const ota_rx_config_t *config);

/**
 * \brief Process an incoming packet from the link.
 * \return true if packet was recognized and processed, false otherwise.
 */
bool ota_receiver_process_packet(ota_receiver_t *rx, const uint8_t *payload, uint8_t len);

/**
 * \brief Execute pending flash operations (erase, program, readback verify).
 *        If a slot is READY_TO_FLASH, this executes the flash write and readback,
 *        and if verified, transmits the READY packet granting credit.
 * \return true if a flash operation was executed, false if idle.
 */
bool ota_receiver_step_flash(ota_receiver_t *rx);

/**
 * \brief Reset receiver to IDLE.
 */
void ota_receiver_reset(ota_receiver_t *rx);

/**
 * \brief Check if full transfer is complete and verified.
 */
static inline bool ota_receiver_is_complete(const ota_receiver_t *rx) {
    return (rx->state == OTA_RX_STATE_COMPLETE);
}

/**
 * \brief Check if receiver encountered a fatal error or abort.
 */
static inline bool ota_receiver_is_aborted(const ota_receiver_t *rx) {
    return (rx->state == OTA_RX_STATE_ABORTED);
}

#ifdef __cplusplus
}
#endif

#endif /* POC_OTA_RECEIVER_H */
