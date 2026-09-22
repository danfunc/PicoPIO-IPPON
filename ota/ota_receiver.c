#include "ota_receiver.h"
#include <string.h>

#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
#include "pico/time.h"
static inline uint64_t get_time_us(void) { return time_us_64(); }
#else
#include <time.h>
static inline uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000ull);
}
#endif

void ota_receiver_init(ota_receiver_t *rx, const ota_rx_config_t *config) {
    if (!rx) return;
    memset(rx, 0, sizeof(*rx));
    if (config) {
        rx->config = *config;
    }
    rx->state = OTA_RX_STATE_IDLE;
    for (int i = 0; i < 2; ++i) {
        rx->slots[i].state = OTA_SLOT_FREE;
    }
}

void ota_receiver_reset(ota_receiver_t *rx) {
    if (!rx) return;
    rx->state = OTA_RX_STATE_IDLE;
    rx->total_size = 0;
    rx->image_crc32 = 0;
    rx->total_windows = 0;
    rx->windows_committed = 0;
    for (int i = 0; i < 2; ++i) {
        rx->slots[i].state = OTA_SLOT_FREE;
        rx->slots[i].win_idx = 0;
        rx->slots[i].received_chunks = 0;
        rx->slots[i].retries = 0;
        memset(rx->slots[i].bitmap, 0, sizeof(rx->slots[i].bitmap));
    }
}

static bool rx_send_response(ota_receiver_t *rx, const void *payload, uint8_t len) {
    if (!rx->config.link_ops.send_packet) return false;
    uint8_t seq = rx->tx_seq++;
    return rx->config.link_ops.send_packet(rx->config.link_ctx, OTA_BMC_PKT_TYPE, seq,
                                           (const uint8_t *)payload, len);
}

bool BMC_SRAM_FUNC(ota_receiver_process_packet)(ota_receiver_t *rx, const uint8_t *payload, uint8_t len) {
    if (!rx || !payload || len < 1) return false;

    uint8_t pkt_type = payload[0];

    switch (pkt_type) {
        case OTA_PKT_TYPE_START: {
            if (len < sizeof(ota_start_pkt_t)) return false;
            const ota_start_pkt_t *pkt = (const ota_start_pkt_t *)payload;

            ota_start_ack_pkt_t ack;
            memset(&ack, 0, sizeof(ack));
            ack.pkt_type = OTA_PKT_TYPE_START_ACK;
            ack.staging_offset = rx->config.staging_offset;

            if (pkt->magic != OTA_MAGIC_START) {
                ack.status = OTA_STATUS_ERR_MAGIC;
                ack.initial_credit = 0;
                rx_send_response(rx, &ack, sizeof(ack));
                return true;
            }

            if (pkt->total_size == 0 ||
                (rx->config.staging_max_bytes > 0 && pkt->total_size > rx->config.staging_max_bytes)) {
                ack.status = OTA_STATUS_ERR_SIZE;
                ack.initial_credit = 0;
                rx_send_response(rx, &ack, sizeof(ack));
                return true;
            }

            ota_receiver_reset(rx);
            rx->total_size = pkt->total_size;
            rx->image_crc32 = pkt->image_crc32;
            rx->total_windows = pkt->total_windows;
            rx->state = OTA_RX_STATE_TRANSFER;

            uint16_t cred = (rx->config.initial_credits > 0) ? rx->config.initial_credits : 1;
            ack.status = OTA_STATUS_OK;
            ack.initial_credit = cred;
            rx_send_response(rx, &ack, sizeof(ack));
            return true;
        }

        case OTA_PKT_TYPE_DATA: {
            if (rx->state != OTA_RX_STATE_TRANSFER) return false;
            if (len < 4) return false;
            const ota_data_pkt_t *pkt = (const ota_data_pkt_t *)payload;
            uint8_t data_len = len - 4;

            uint8_t slot_idx = pkt->win_idx % 2;
            ota_window_slot_t *slot = &rx->slots[slot_idx];

            /* If slot is FREE or finished for an older window, initialize for this window */
            if (slot->state == OTA_SLOT_FREE ||
                (slot->state == OTA_SLOT_COMMITTED && slot->win_idx != pkt->win_idx)) {
                slot->win_idx = pkt->win_idx;
                uint32_t offset = (uint32_t)pkt->win_idx * OTA_WINDOW_SIZE;
                uint32_t remaining = (rx->total_size > offset) ? (rx->total_size - offset) : 0;
                slot->win_bytes = (remaining > OTA_WINDOW_SIZE) ? OTA_WINDOW_SIZE : remaining;
                slot->total_chunks = (uint16_t)((slot->win_bytes + OTA_CHUNK_DATA_SIZE - 1) / OTA_CHUNK_DATA_SIZE);
                slot->received_chunks = 0;
                slot->retries = 0;
                memset(slot->bitmap, 0, sizeof(slot->bitmap));
                slot->state = OTA_SLOT_RECEIVING;
            }

            if (slot->win_idx != pkt->win_idx) {
                /* Conflicting window for this slot, drop */
                return false;
            }

            if (slot->state != OTA_SLOT_RECEIVING && slot->state != OTA_SLOT_READY_TO_FLASH) {
                return false;
            }

            if (pkt->chunk_idx >= slot->total_chunks) {
                return false;
            }

            if (ota_bm_get(slot->bitmap, pkt->chunk_idx)) {
                rx->stats.rx_chunks_duplicate++;
                return true;
            }

            uint32_t byte_offset = (uint32_t)pkt->chunk_idx * OTA_CHUNK_DATA_SIZE;
            uint32_t copy_bytes = data_len;
            if (byte_offset + copy_bytes > slot->win_bytes) {
                copy_bytes = slot->win_bytes - byte_offset;
            }
            if (copy_bytes > 0) {
                memcpy(slot->buffer + byte_offset, pkt->data, copy_bytes);
            }

            ota_bm_set(slot->bitmap, pkt->chunk_idx);
            slot->received_chunks++;
            rx->stats.rx_chunks_total++;
            return true;
        }

        case OTA_PKT_TYPE_QUERY: {
            if (rx->state != OTA_RX_STATE_TRANSFER) return false;
            if (len < sizeof(ota_query_pkt_t)) return false;
            const ota_query_pkt_t *pkt = (const ota_query_pkt_t *)payload;
            rx->stats.rx_queries++;

            uint8_t slot_idx = pkt->win_idx % 2;
            ota_window_slot_t *slot = &rx->slots[slot_idx];

            if (slot->state == OTA_SLOT_FREE || slot->win_idx != pkt->win_idx) {
                /* Unknown or uninitialized window */
                ota_query_resp_pkt_t resp;
                memset(&resp, 0, sizeof(resp));
                resp.pkt_type = OTA_PKT_TYPE_QUERY_RESP;
                resp.win_idx = pkt->win_idx;
                resp.total_chunks = (uint16_t)((pkt->win_bytes + OTA_CHUNK_DATA_SIZE - 1) / OTA_CHUNK_DATA_SIZE);
                resp.missing_count = resp.total_chunks;
                resp.bitmap_bytes = (resp.total_chunks + 7) / 8;
                rx_send_response(rx, &resp, 8 + resp.bitmap_bytes);
                return true;
            }

            slot->expected_crc32 = pkt->win_crc32;
            slot->win_bytes = pkt->win_bytes;
            slot->total_chunks = (uint16_t)((pkt->win_bytes + OTA_CHUNK_DATA_SIZE - 1) / OTA_CHUNK_DATA_SIZE);

            uint16_t missing = ota_bm_count_missing(slot->bitmap, slot->total_chunks);

            if (missing == 0) {
                /* All chunks arrived, verify SRAM CRC */
                uint32_t sram_crc = ota_crc32(slot->buffer, slot->win_bytes);
                if (sram_crc == slot->expected_crc32) {
                    slot->state = OTA_SLOT_READY_TO_FLASH;
                } else {
                    /* SRAM CRC mismatch, invalidate bitmap to force retransmission */
                    memset(slot->bitmap, 0, sizeof(slot->bitmap));
                    slot->received_chunks = 0;
                    missing = slot->total_chunks;
                }
            }

            ota_query_resp_pkt_t resp;
            memset(&resp, 0, sizeof(resp));
            resp.pkt_type = OTA_PKT_TYPE_QUERY_RESP;
            resp.win_idx = slot->win_idx;
            resp.missing_count = missing;
            resp.total_chunks = slot->total_chunks;
            resp.bitmap_bytes = (slot->total_chunks + 7) / 8;
            memcpy(resp.bitmap, slot->bitmap, resp.bitmap_bytes);

            rx_send_response(rx, &resp, 8 + resp.bitmap_bytes);
            return true;
        }

        case OTA_PKT_TYPE_FINALIZE: {
            if (rx->state != OTA_RX_STATE_TRANSFER) return false;
            if (len < sizeof(ota_finalize_pkt_t)) return false;

            ota_status_pkt_t st;
            memset(&st, 0, sizeof(st));
            st.pkt_type = OTA_PKT_TYPE_STATUS;
            st.staged_bytes = rx->total_size;

            if (rx->windows_committed < rx->total_windows) {
                st.status = OTA_STATUS_ERR_SIZE;
                rx_send_response(rx, &st, sizeof(st));
                return true;
            }

            /* Final Full Image CRC32 Verification via Flash Readback */
            uint64_t t0 = get_time_us();
            uint32_t staged_crc = 0;
            if (rx->config.flash_ops.crc32) {
                staged_crc = rx->config.flash_ops.crc32(rx->config.flash_ctx,
                                                        rx->config.staging_offset,
                                                        rx->total_size);
            }
            uint64_t t1 = get_time_us();
            rx->stats.flash_verify_us += (t1 - t0);

            st.staged_crc32 = staged_crc;
            if (staged_crc == rx->image_crc32) {
                st.status = OTA_STATUS_OK;
                rx->state = OTA_RX_STATE_COMPLETE;
            } else {
                st.status = OTA_STATUS_ERR_CRC_MISMATCH;
                rx->state = OTA_RX_STATE_ABORTED;
            }

            rx_send_response(rx, &st, sizeof(st));
            return true;
        }

        case OTA_PKT_TYPE_ABORT: {
            rx->state = OTA_RX_STATE_ABORTED;
            return true;
        }

        default:
            return false;
    }
}

bool BMC_SRAM_FUNC(ota_receiver_step_flash)(ota_receiver_t *rx) {
    if (!rx || rx->state != OTA_RX_STATE_TRANSFER) return false;

    /* Search for the next in-order slot that is READY_TO_FLASH */
    ota_window_slot_t *target_slot = NULL;
    for (int i = 0; i < 2; ++i) {
        if (rx->slots[i].state == OTA_SLOT_READY_TO_FLASH &&
            rx->slots[i].win_idx == rx->windows_committed) {
            target_slot = &rx->slots[i];
            break;
        }
    }

    if (!target_slot) {
        return false;
    }

    target_slot->state = OTA_SLOT_FLASHING;
    uint32_t flash_offset = rx->config.staging_offset + (uint32_t)target_slot->win_idx * OTA_WINDOW_SIZE;
    uint32_t write_bytes = target_slot->win_bytes;

    /* Hard boundary safety check */
    if (rx->config.staging_max_bytes > 0) {
        uint64_t end = (uint64_t)flash_offset + write_bytes;
        uint64_t max_end = (uint64_t)rx->config.staging_offset + rx->config.staging_max_bytes;
        if (end > max_end) {
            target_slot->state = OTA_SLOT_ERROR;
            rx->state = OTA_RX_STATE_ABORTED;
            ota_ready_pkt_t ready = {
                .pkt_type = OTA_PKT_TYPE_READY,
                .win_idx = (uint8_t)target_slot->win_idx,
                .status = OTA_STATUS_ERR_SIZE,
                .credit_granted = 0,
                .readback_crc32 = 0
            };
            rx_send_response(rx, &ready, sizeof(ready));
            return true;
        }
    }

    uint64_t t_win_start = get_time_us();

    /* 1. Flash Erase 64KB block (must erase at least the block) */
    uint64_t t_erase0 = get_time_us();
    size_t erase_bytes = OTA_WINDOW_SIZE; /* 64KB block erase */
    int erase_rc = rx->config.flash_ops.erase(rx->config.flash_ctx, flash_offset, erase_bytes);
    uint64_t t_erase1 = get_time_us();
    rx->stats.flash_erase_us += (t_erase1 - t_erase0);

    if (erase_rc != 0) {
        target_slot->retries++;
        rx->stats.flash_retries++;
        if (target_slot->retries < OTA_MAX_FLASH_RETRIES) {
            target_slot->state = OTA_SLOT_READY_TO_FLASH;
        } else {
            target_slot->state = OTA_SLOT_ERROR;
            ota_ready_pkt_t ready = {
                .pkt_type = OTA_PKT_TYPE_READY,
                .win_idx = (uint8_t)target_slot->win_idx,
                .status = OTA_STATUS_ERR_FLASH_ERASE,
                .credit_granted = 0,
                .readback_crc32 = 0
            };
            rx_send_response(rx, &ready, sizeof(ready));
        }
        return true;
    }

    /* 2. Flash Program: write data in 4KB (or page) units */
    uint64_t t_prog0 = get_time_us();
    int prog_rc = rx->config.flash_ops.program(rx->config.flash_ctx, flash_offset,
                                               target_slot->buffer, write_bytes);
    uint64_t t_prog1 = get_time_us();
    rx->stats.flash_program_us += (t_prog1 - t_prog0);

    if (prog_rc != 0) {
        target_slot->retries++;
        rx->stats.flash_retries++;
        if (target_slot->retries < OTA_MAX_FLASH_RETRIES) {
            target_slot->state = OTA_SLOT_READY_TO_FLASH;
        } else {
            target_slot->state = OTA_SLOT_ERROR;
            ota_ready_pkt_t ready = {
                .pkt_type = OTA_PKT_TYPE_READY,
                .win_idx = (uint8_t)target_slot->win_idx,
                .status = OTA_STATUS_ERR_FLASH_PROGRAM,
                .credit_granted = 0,
                .readback_crc32 = 0
            };
            rx_send_response(rx, &ready, sizeof(ready));
        }
        return true;
    }

    /* 3. XIP Readback CRC32 Verification */
    uint64_t t_vfy0 = get_time_us();
    uint32_t readback_crc = rx->config.flash_ops.crc32(rx->config.flash_ctx, flash_offset, write_bytes);
    uint64_t t_vfy1 = get_time_us();
    rx->stats.flash_verify_us += (t_vfy1 - t_vfy0);

    uint64_t win_dur = get_time_us() - t_win_start;
    if (win_dur > rx->stats.max_window_us) {
        rx->stats.max_window_us = win_dur;
    }
    rx->stats.total_flash_us += (t_vfy1 - t_erase0);

    if (readback_crc == target_slot->expected_crc32) {
        /* Success! Commit window and free slot */
        target_slot->state = OTA_SLOT_FREE;
        rx->windows_committed++;
        rx->stats.windows_flashed++;

        ota_ready_pkt_t ready = {
            .pkt_type = OTA_PKT_TYPE_READY,
            .win_idx = (uint8_t)target_slot->win_idx,
            .status = OTA_STATUS_OK,
            .credit_granted = 1,
            .readback_crc32 = readback_crc
        };
        rx_send_response(rx, &ready, sizeof(ready));
    } else {
        /* Readback mismatch! */
        target_slot->retries++;
        rx->stats.flash_retries++;
        if (target_slot->retries < OTA_MAX_FLASH_RETRIES) {
            target_slot->state = OTA_SLOT_READY_TO_FLASH;
        } else {
            target_slot->state = OTA_SLOT_ERROR;
            ota_ready_pkt_t ready = {
                .pkt_type = OTA_PKT_TYPE_READY,
                .win_idx = (uint8_t)target_slot->win_idx,
                .status = OTA_STATUS_ERR_FLASH_VERIFY,
                .credit_granted = 0,
                .readback_crc32 = readback_crc
            };
            rx_send_response(rx, &ready, sizeof(ready));
        }
    }

    return true;
}
