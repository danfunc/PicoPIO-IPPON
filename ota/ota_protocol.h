#ifndef POC_OTA_PROTOCOL_H
#define POC_OTA_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "common/sram_attrs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 1. Protocol Invariants & Constants
 * ============================================================================
 */
#define OTA_MAGIC_START         0x4F54u  /* 'OT' in ASCII */
#define OTA_BMC_PKT_TYPE        0x50u    /* BMC packet type for OTA frames */

#define OTA_WINDOW_SIZE         65536u   /* 64 KB per erase block window */
#define OTA_CHUNK_DATA_SIZE     124u     /* Data payload bytes per BMC frame */
#define OTA_CHUNKS_PER_WINDOW   529u     /* (65536 + 124 - 1) / 124 = 529 */
#define OTA_BITMAP_BYTES        67u      /* (529 + 7) / 8 = 67 bytes */
#define OTA_MAX_FLASH_RETRIES   3u       /* Max write/verify retries on sector failure */

/* Status Codes */
#define OTA_STATUS_OK                   0x00
#define OTA_STATUS_ERR_MAGIC            0x01
#define OTA_STATUS_ERR_SIZE             0x02
#define OTA_STATUS_ERR_CRC_MISMATCH     0x03
#define OTA_STATUS_ERR_FLASH_ERASE      0x04
#define OTA_STATUS_ERR_FLASH_PROGRAM    0x05
#define OTA_STATUS_ERR_FLASH_VERIFY     0x06
#define OTA_STATUS_ERR_TIMEOUT          0x07
#define OTA_STATUS_ERR_ABORTED          0x08

/*
 * ============================================================================
 * 2. Packet Types
 * ============================================================================
 */
typedef enum {
    OTA_PKT_TYPE_START       = 0x01, /* Sender -> Receiver: Begin session */
    OTA_PKT_TYPE_START_ACK   = 0x02, /* Receiver -> Sender: Session accepted, initial credit */
    OTA_PKT_TYPE_DATA        = 0x03, /* Sender -> Receiver: Window chunk data */
    OTA_PKT_TYPE_QUERY       = 0x04, /* Sender -> Receiver: Query missing chunks for window */
    OTA_PKT_TYPE_QUERY_RESP  = 0x05, /* Receiver -> Sender: Missing count & bitmap */
    OTA_PKT_TYPE_READY       = 0x06, /* Receiver -> Sender: Window committed & verified, grant credit */
    OTA_PKT_TYPE_FINALIZE    = 0x07, /* Sender -> Receiver: Request final image verification */
    OTA_PKT_TYPE_STATUS      = 0x08, /* Receiver -> Sender: Final image CRC status */
    OTA_PKT_TYPE_ABORT       = 0x09, /* Sender <-> Receiver: Abort transfer */
} ota_pkt_type_t;

/*
 * ============================================================================
 * 3. Wire Formats (All packed, fits strictly in 128-byte BMC payload)
 * ============================================================================
 */
#pragma pack(push, 1)

/* 0x01: START (16 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_START */
    uint8_t  reserved;
    uint16_t magic;           /* OTA_MAGIC_START */
    uint32_t total_size;      /* Total image bytes (e.g. 393216 for 384KB) */
    uint32_t image_crc32;     /* Expected CRC32 of full image */
    uint16_t total_windows;   /* Number of 64KB windows */
    uint16_t chunk_data_size; /* OTA_CHUNK_DATA_SIZE (124) */
} ota_start_pkt_t;

/* 0x02: START_ACK (8 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_START_ACK */
    uint8_t  status;          /* OTA_STATUS_OK or error */
    uint16_t initial_credit;  /* Number of window credits granted (1 or 2) */
    uint32_t staging_offset;  /* Staging base offset in flash */
} ota_start_ack_pkt_t;

/* 0x03: DATA (128 bytes = 4B header + 124B data, exact 128B BMC payload) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_DATA */
    uint8_t  win_idx;         /* Window index (0..total_windows-1) */
    uint16_t chunk_idx;       /* Chunk index in window (0..528) */
    uint8_t  data[OTA_CHUNK_DATA_SIZE]; /* 124 bytes of firmware data */
} ota_data_pkt_t;

/* 0x04: QUERY (12 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_QUERY */
    uint8_t  win_idx;         /* Target window index */
    uint16_t reserved;
    uint32_t win_crc32;       /* Expected CRC32 of this 64KB window */
    uint32_t win_bytes;       /* Valid bytes in this window (<= 65536) */
} ota_query_pkt_t;

/* 0x05: QUERY_RESP (75 bytes = 8B header + 67B bitmap) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_QUERY_RESP */
    uint8_t  win_idx;         /* Window index */
    uint16_t missing_count;   /* 0 = all chunks received */
    uint16_t total_chunks;    /* Total chunks in this window */
    uint16_t bitmap_bytes;    /* Number of valid bytes in bitmap (<= 67) */
    uint8_t  bitmap[OTA_BITMAP_BYTES]; /* bit i: 1=received, 0=missing */
} ota_query_resp_pkt_t;

/* 0x06: READY (8 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_READY */
    uint8_t  win_idx;         /* Window index written & verified */
    uint8_t  status;          /* OTA_STATUS_OK or error */
    uint8_t  credit_granted;  /* Number of credits granted (usually 1) */
    uint32_t readback_crc32;  /* Verified XIP readback CRC32 */
} ota_ready_pkt_t;

/* 0x07: FINALIZE (8 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_FINALIZE */
    uint8_t  reserved[3];
    uint32_t image_crc32;     /* Full image expected CRC32 */
} ota_finalize_pkt_t;

/* 0x08: STATUS (12 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_STATUS */
    uint8_t  status;          /* OTA_STATUS_OK or error */
    uint16_t reserved;
    uint32_t staged_bytes;    /* Total bytes staged */
    uint32_t staged_crc32;    /* Total CRC32 computed from flash */
} ota_status_pkt_t;

/* 0x09: ABORT (4 bytes) */
typedef struct {
    uint8_t  pkt_type;        /* OTA_PKT_TYPE_ABORT */
    uint8_t  reason;          /* Abort reason code */
    uint16_t reserved;
} ota_abort_pkt_t;

#pragma pack(pop)

/*
 * ============================================================================
 * 4. Bitmap Helper Functions (Pure inline functions)
 * ============================================================================
 */
static inline bool ota_bm_get(const uint8_t *bm, uint16_t idx) {
    return ((bm[idx >> 3] >> (idx & 7u)) & 1u) != 0;
}

static inline void ota_bm_set(uint8_t *bm, uint16_t idx) {
    bm[idx >> 3] |= (uint8_t)(1u << (idx & 7u));
}

static inline void ota_bm_clear(uint8_t *bm, uint16_t idx) {
    bm[idx >> 3] &= (uint8_t)~(1u << (idx & 7u));
}

uint16_t ota_bm_count_missing(const uint8_t *bm, uint16_t total_chunks);
uint16_t ota_bm_find_next_missing(const uint8_t *bm, uint16_t start_idx, uint16_t total_chunks);

/*
 * ============================================================================
 * 5. Hardware Abstraction Interfaces (Function Pointers)
 * ============================================================================
 */

/* Link Operations: Send and Receive Raw Packets */
typedef struct {
    /* Send packet. Returns true on success. */
    bool (*send_packet)(void *user_ctx, uint8_t bmc_type, uint8_t seq,
                        const uint8_t *payload, uint8_t payload_len);
    /* Poll packet. Returns true if packet was available and fetched. */
    bool (*recv_packet)(void *user_ctx, uint8_t *out_bmc_type, uint8_t *out_seq,
                        uint8_t *out_payload, uint8_t *out_len);
} ota_link_ops_t;

/* Flash Operations: Erase, Program, Read, CRC32 */
typedef struct {
    /* Erase flash block/sector. Returns 0 on success. */
    int (*erase)(void *user_ctx, uint32_t offset, size_t count);
    /* Program flash pages. Returns 0 on success. */
    int (*program)(void *user_ctx, uint32_t offset, const uint8_t *data, size_t count);
    /* Read flash range. Returns 0 on success. */
    int (*read)(void *user_ctx, uint32_t offset, uint8_t *dst, size_t count);
    /* Compute CRC32 of flash range directly. */
    uint32_t (*crc32)(void *user_ctx, uint32_t offset, size_t count);
} ota_flash_ops_t;

/*
 * ============================================================================
 * 6. CRC32 Calculation
 * ============================================================================
 */
#define OTA_CRC32_INIT (0xFFFFFFFFu)

uint32_t ota_crc32_update(uint32_t crc, const uint8_t *data, size_t length);
uint32_t ota_crc32(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* POC_OTA_PROTOCOL_H */
