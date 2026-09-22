#ifndef POC_OTA_SENDER_H
#define POC_OTA_SENDER_H

#include "ota_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Sender State & Context
 * ============================================================================
 */

typedef enum {
    OTA_TX_STATE_IDLE = 0,
    OTA_TX_STATE_START,
    OTA_TX_STATE_SEND_WINDOW,
    OTA_TX_STATE_QUERY,
    OTA_TX_STATE_RETRANSMIT,
    OTA_TX_STATE_WAIT_READY,
    OTA_TX_STATE_FINALIZE,
    OTA_TX_STATE_COMPLETE,
    OTA_TX_STATE_FAILED,
} ota_tx_state_t;

/* Sender Statistics */
typedef struct {
    uint64_t total_time_us;
    uint64_t link_transfer_us;     /* Active TX of original chunks */
    uint64_t retransmit_us;        /* Active TX of retransmitted chunks */
    uint64_t ready_wait_us;        /* Time spent waiting for READY credit */
    uint64_t query_turnaround_us;  /* Turnaround wait for QUERY_RESP */
    uint64_t max_window_us;        /* Maximum single-window duration */
    uint32_t total_packets_sent;
    uint32_t retransmit_packets;
    uint32_t queries_sent;
    uint32_t windows_completed;
    double   effective_kib_s;
} ota_tx_stats_t;

/* Data Provider Callback (allows PRNG generation or buffer access) */
typedef bool (*ota_data_reader_fn)(void *ctx, uint32_t offset, uint8_t *dst, size_t len);

/* Sender Configuration */
typedef struct {
    uint32_t            total_size;      /* Total image size in bytes */
    uint32_t            image_crc32;     /* Precomputed full image CRC32 */
    ota_data_reader_fn  data_reader;     /* Data read callback */
    void               *reader_ctx;
    ota_link_ops_t      link_ops;        /* Link interface */
    void               *link_ctx;
    uint32_t            query_timeout_us;/* Timeout waiting for QUERY_RESP/ACK */
} ota_tx_config_t;

/* Sender Context */
typedef struct {
    ota_tx_config_t     config;
    ota_tx_state_t      state;
    uint16_t            total_windows;
    uint16_t            send_win_idx;      /* Window currently being sent */
    uint16_t            commit_win_idx;    /* Window waiting for READY */
    uint16_t            current_chunk_idx;
    uint16_t            total_chunks_in_win;
    uint32_t            current_win_bytes;
    uint32_t            current_win_crc32;
    int16_t             credits;           /* Available window credits */
    uint8_t             tx_seq;
    uint8_t             retries;
    uint64_t            state_enter_us;
    uint64_t            t_start_us;
    uint64_t            t_win_start_us;
    /* Retransmission Bitmap Cache from peer QUERY_RESP */
    uint8_t             retransmit_bitmap[OTA_BITMAP_BYTES];
    uint16_t            retransmit_next_chunk;
    uint16_t            retransmit_remaining;
    ota_tx_stats_t      stats;
} ota_sender_t;

/*
 * ============================================================================
 * Sender API Functions
 * ============================================================================
 */

/**
 * \brief Initialize OTA sender context.
 */
void ota_sender_init(ota_sender_t *tx, const ota_tx_config_t *config);

/**
 * \brief Start OTA transmission sequence.
 * \return true on success, false if invalid arguments.
 */
bool ota_sender_start(ota_sender_t *tx);

/**
 * \brief Step the sender state machine forward.
 *        Polls link, sends outgoing chunks/queries, handles timeouts.
 * \return true while transmission is still active, false when complete or failed.
 */
bool ota_sender_step(ota_sender_t *tx);

/**
 * \brief Pass a received response packet directly to sender.
 * \return true if processed, false otherwise.
 */
bool ota_sender_handle_response(ota_sender_t *tx, const uint8_t *payload, uint8_t len);

/**
 * \brief Check if sender has successfully completed all windows and final verification.
 */
static inline bool ota_sender_is_complete(const ota_sender_t *tx) {
    return (tx->state == OTA_TX_STATE_COMPLETE);
}

/**
 * \brief Check if sender failed.
 */
static inline bool ota_sender_is_failed(const ota_sender_t *tx) {
    return (tx->state == OTA_TX_STATE_FAILED);
}

#ifdef __cplusplus
}
#endif

#endif /* POC_OTA_SENDER_H */
