#include "ota_sender.h"
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

#define DEFAULT_TIMEOUT_US 250000ull /* 250 ms */
#define MAX_TIMEOUT_RETRIES 10

void ota_sender_init(ota_sender_t *tx, const ota_tx_config_t *config) {
    if (!tx) return;
    memset(tx, 0, sizeof(*tx));
    if (config) {
        tx->config = *config;
    }
    if (tx->config.query_timeout_us == 0) {
        tx->config.query_timeout_us = DEFAULT_TIMEOUT_US;
    }
    tx->state = OTA_TX_STATE_IDLE;
}

static bool tx_send_pkt(ota_sender_t *tx, const void *payload, uint8_t len) {
    if (!tx->config.link_ops.send_packet) return false;
    uint8_t seq = tx->tx_seq++;
    bool ok = tx->config.link_ops.send_packet(tx->config.link_ctx, OTA_BMC_PKT_TYPE, seq,
                                             (const uint8_t *)payload, len);
    if (ok) {
        tx->stats.total_packets_sent++;
    }
    return ok;
}

static void setup_window(ota_sender_t *tx, uint16_t win_idx) {
    uint32_t offset = (uint32_t)win_idx * OTA_WINDOW_SIZE;
    uint32_t remaining = (tx->config.total_size > offset) ? (tx->config.total_size - offset) : 0;
    tx->current_win_bytes = (remaining > OTA_WINDOW_SIZE) ? OTA_WINDOW_SIZE : remaining;
    tx->total_chunks_in_win = (uint16_t)((tx->current_win_bytes + OTA_CHUNK_DATA_SIZE - 1) / OTA_CHUNK_DATA_SIZE);
    tx->current_chunk_idx = 0;

    /* Compute CRC32 of this window data using reader */
    uint32_t win_crc = OTA_CRC32_INIT;
    uint8_t temp[OTA_CHUNK_DATA_SIZE];
    uint32_t cur = offset;
    uint32_t left = tx->current_win_bytes;
    while (left > 0) {
        size_t n = (left > sizeof(temp)) ? sizeof(temp) : left;
        if (tx->config.data_reader) {
            tx->config.data_reader(tx->config.reader_ctx, cur, temp, n);
        }
        win_crc = ota_crc32_update(win_crc, temp, n);
        cur += n;
        left -= n;
    }
    tx->current_win_crc32 = win_crc ^ 0xFFFFFFFFu;
    tx->t_win_start_us = get_time_us();
}

bool ota_sender_start(ota_sender_t *tx) {
    if (!tx || tx->config.total_size == 0) return false;

    tx->total_windows = (uint16_t)((tx->config.total_size + OTA_WINDOW_SIZE - 1) / OTA_WINDOW_SIZE);
    tx->send_win_idx = 0;
    tx->commit_win_idx = 0;
    tx->credits = 0;
    tx->retries = 0;
    tx->t_start_us = get_time_us();
    tx->state_enter_us = tx->t_start_us;

    ota_start_pkt_t start_pkt = {
        .pkt_type = OTA_PKT_TYPE_START,
        .reserved = 0,
        .magic = OTA_MAGIC_START,
        .total_size = tx->config.total_size,
        .image_crc32 = tx->config.image_crc32,
        .total_windows = tx->total_windows,
        .chunk_data_size = OTA_CHUNK_DATA_SIZE
    };

    tx->state = OTA_TX_STATE_START;
    return tx_send_pkt(tx, &start_pkt, sizeof(start_pkt));
}

bool BMC_SRAM_FUNC(ota_sender_handle_response)(ota_sender_t *tx, const uint8_t *payload, uint8_t len) {
    if (!tx || !payload || len < 1) return false;

    uint8_t pkt_type = payload[0];

    switch (pkt_type) {
        case OTA_PKT_TYPE_START_ACK: {
            if (tx->state != OTA_TX_STATE_START) return false;
            if (len < sizeof(ota_start_ack_pkt_t)) return false;
            const ota_start_ack_pkt_t *ack = (const ota_start_ack_pkt_t *)payload;
            if (ack->status != OTA_STATUS_OK) {
                tx->state = OTA_TX_STATE_FAILED;
                return true;
            }
            tx->credits = ack->initial_credit;
            tx->retries = 0;
            setup_window(tx, tx->send_win_idx);
            tx->state = OTA_TX_STATE_SEND_WINDOW;
            tx->state_enter_us = get_time_us();
            return true;
        }

        case OTA_PKT_TYPE_QUERY_RESP: {
            if (tx->state != OTA_TX_STATE_QUERY) return false;
            if (len < 8) return false;
            const ota_query_resp_pkt_t *resp = (const ota_query_resp_pkt_t *)payload;
            if (resp->win_idx != tx->send_win_idx) return false;

            if (resp->missing_count > 0) {
                uint8_t bm_len = (resp->bitmap_bytes <= OTA_BITMAP_BYTES) ? (uint8_t)resp->bitmap_bytes : OTA_BITMAP_BYTES;
                memcpy(tx->retransmit_bitmap, resp->bitmap, bm_len);
                tx->retransmit_remaining = resp->missing_count;
                tx->retransmit_next_chunk = 0;
                tx->state = OTA_TX_STATE_RETRANSMIT;
                tx->state_enter_us = get_time_us();
            } else {
                /* All chunks received for this window in receiver SRAM */
                tx->credits--;
                tx->send_win_idx++;
                tx->retries = 0;

                if (tx->send_win_idx < tx->total_windows && tx->credits > 0) {
                    /* Double buffering: send next window immediately */
                    setup_window(tx, tx->send_win_idx);
                    tx->state = OTA_TX_STATE_SEND_WINDOW;
                    tx->state_enter_us = get_time_us();
                } else {
                    /* Wait for READY (flash write and readback verify) */
                    tx->state = OTA_TX_STATE_WAIT_READY;
                    tx->state_enter_us = get_time_us();
                }
            }
            return true;
        }

        case OTA_PKT_TYPE_READY: {
            if (len < sizeof(ota_ready_pkt_t)) return false;
            const ota_ready_pkt_t *ready = (const ota_ready_pkt_t *)payload;

            if (ready->status != OTA_STATUS_OK) {
                tx->state = OTA_TX_STATE_FAILED;
                return true;
            }

            tx->credits += ready->credit_granted;
            tx->commit_win_idx = ready->win_idx + 1;
            tx->stats.windows_completed++;

            uint64_t win_dur = get_time_us() - tx->t_win_start_us;
            if (win_dur > tx->stats.max_window_us) {
                tx->stats.max_window_us = win_dur;
            }

            if (tx->commit_win_idx == tx->total_windows) {
                /* All windows committed to flash! Request final full verification */
                ota_finalize_pkt_t finalize_pkt = {
                    .pkt_type = OTA_PKT_TYPE_FINALIZE,
                    .reserved = {0, 0, 0},
                    .image_crc32 = tx->config.image_crc32
                };
                tx->state = OTA_TX_STATE_FINALIZE;
                tx->state_enter_us = get_time_us();
                tx->retries = 0;
                tx_send_pkt(tx, &finalize_pkt, sizeof(finalize_pkt));
            } else if (tx->state == OTA_TX_STATE_WAIT_READY) {
                /* Start next window now that credit was granted */
                if (tx->send_win_idx < tx->total_windows) {
                    setup_window(tx, tx->send_win_idx);
                    tx->state = OTA_TX_STATE_SEND_WINDOW;
                    tx->state_enter_us = get_time_us();
                }
            }
            return true;
        }

        case OTA_PKT_TYPE_STATUS: {
            if (tx->state != OTA_TX_STATE_FINALIZE) return false;
            if (len < sizeof(ota_status_pkt_t)) return false;
            const ota_status_pkt_t *st = (const ota_status_pkt_t *)payload;

            if (st->status == OTA_STATUS_OK && st->staged_crc32 == tx->config.image_crc32) {
                tx->state = OTA_TX_STATE_COMPLETE;
                tx->stats.total_time_us = get_time_us() - tx->t_start_us;
                double sec = (double)tx->stats.total_time_us / 1000000.0;
                if (sec > 0.0) {
                    tx->stats.effective_kib_s = ((double)tx->config.total_size / 1024.0) / sec;
                }
            } else {
                tx->state = OTA_TX_STATE_FAILED;
            }
            return true;
        }

        case OTA_PKT_TYPE_ABORT: {
            tx->state = OTA_TX_STATE_FAILED;
            return true;
        }

        default:
            return false;
    }
}

bool BMC_SRAM_FUNC(ota_sender_step)(ota_sender_t *tx) {
    if (!tx) return false;

    /* 1. Poll incoming link packets */
    if (tx->config.link_ops.recv_packet) {
        uint8_t bmc_type = 0, seq = 0, rx_len = 0;
        uint8_t buf[128];
        while (tx->config.link_ops.recv_packet(tx->config.link_ctx, &bmc_type, &seq, buf, &rx_len)) {
            ota_sender_handle_response(tx, buf, rx_len);
        }
    }

    /* 2. Process state machine */
    switch (tx->state) {
        case OTA_TX_STATE_SEND_WINDOW: {
            uint64_t t0 = get_time_us();
            /* Stream chunks in window */
            uint32_t win_base = (uint32_t)tx->send_win_idx * OTA_WINDOW_SIZE;

            /* Send up to 32 chunks per step to allow polling */
            for (int batch = 0; batch < 32 && tx->current_chunk_idx < tx->total_chunks_in_win; ++batch) {
                uint16_t c_idx = tx->current_chunk_idx;
                uint32_t c_offset = win_base + (uint32_t)c_idx * OTA_CHUNK_DATA_SIZE;
                uint32_t left = (tx->config.total_size > c_offset) ? (tx->config.total_size - c_offset) : 0;
                uint8_t c_len = (left > OTA_CHUNK_DATA_SIZE) ? OTA_CHUNK_DATA_SIZE : (uint8_t)left;

                ota_data_pkt_t pkt;
                pkt.pkt_type = OTA_PKT_TYPE_DATA;
                pkt.win_idx = (uint8_t)tx->send_win_idx;
                pkt.chunk_idx = c_idx;
                if (tx->config.data_reader) {
                    tx->config.data_reader(tx->config.reader_ctx, c_offset, pkt.data, c_len);
                }

                tx_send_pkt(tx, &pkt, 4 + c_len);
                tx->current_chunk_idx++;
            }
            uint64_t t1 = get_time_us();
            tx->stats.link_transfer_us += (t1 - t0);

            if (tx->current_chunk_idx >= tx->total_chunks_in_win) {
                /* All chunks sent, send QUERY */
                ota_query_pkt_t q = {
                    .pkt_type = OTA_PKT_TYPE_QUERY,
                    .win_idx = (uint8_t)tx->send_win_idx,
                    .reserved = 0,
                    .win_crc32 = tx->current_win_crc32,
                    .win_bytes = tx->current_win_bytes
                };
                tx_send_pkt(tx, &q, sizeof(q));
                tx->stats.queries_sent++;
                tx->state = OTA_TX_STATE_QUERY;
                tx->state_enter_us = get_time_us();
                tx->retries = 0;
            }
            return true;
        }

        case OTA_TX_STATE_RETRANSMIT: {
            uint64_t t0 = get_time_us();
            uint32_t win_base = (uint32_t)tx->send_win_idx * OTA_WINDOW_SIZE;

            /* Retransmit missing chunks */
            for (int batch = 0; batch < 32 && tx->retransmit_remaining > 0; ++batch) {
                uint16_t missing_chunk = ota_bm_find_next_missing(tx->retransmit_bitmap,
                                                                  tx->retransmit_next_chunk,
                                                                  tx->total_chunks_in_win);
                if (missing_chunk >= tx->total_chunks_in_win) {
                    tx->retransmit_remaining = 0;
                    break;
                }

                uint32_t c_offset = win_base + (uint32_t)missing_chunk * OTA_CHUNK_DATA_SIZE;
                uint32_t left = (tx->config.total_size > c_offset) ? (tx->config.total_size - c_offset) : 0;
                uint8_t c_len = (left > OTA_CHUNK_DATA_SIZE) ? OTA_CHUNK_DATA_SIZE : (uint8_t)left;

                ota_data_pkt_t pkt;
                pkt.pkt_type = OTA_PKT_TYPE_DATA;
                pkt.win_idx = (uint8_t)tx->send_win_idx;
                pkt.chunk_idx = missing_chunk;
                if (tx->config.data_reader) {
                    tx->config.data_reader(tx->config.reader_ctx, c_offset, pkt.data, c_len);
                }

                tx_send_pkt(tx, &pkt, 4 + c_len);
                tx->stats.retransmit_packets++;

                /* Mark as retransmitted in local cache */
                ota_bm_set(tx->retransmit_bitmap, missing_chunk);
                tx->retransmit_next_chunk = missing_chunk + 1;
                tx->retransmit_remaining--;
            }
            uint64_t t1 = get_time_us();
            tx->stats.retransmit_us += (t1 - t0);

            if (tx->retransmit_remaining == 0) {
                /* Retransmission burst done, query again */
                ota_query_pkt_t q = {
                    .pkt_type = OTA_PKT_TYPE_QUERY,
                    .win_idx = (uint8_t)tx->send_win_idx,
                    .reserved = 0,
                    .win_crc32 = tx->current_win_crc32,
                    .win_bytes = tx->current_win_bytes
                };
                tx_send_pkt(tx, &q, sizeof(q));
                tx->stats.queries_sent++;
                tx->state = OTA_TX_STATE_QUERY;
                tx->state_enter_us = get_time_us();
                tx->retries = 0;
            }
            return true;
        }

        case OTA_TX_STATE_QUERY: {
            uint64_t now = get_time_us();
            if (now - tx->state_enter_us > tx->config.query_timeout_us) {
                tx->retries++;
                if (tx->retries > MAX_TIMEOUT_RETRIES) {
                    tx->state = OTA_TX_STATE_FAILED;
                    return false;
                }
                ota_query_pkt_t q = {
                    .pkt_type = OTA_PKT_TYPE_QUERY,
                    .win_idx = (uint8_t)tx->send_win_idx,
                    .reserved = 0,
                    .win_crc32 = tx->current_win_crc32,
                    .win_bytes = tx->current_win_bytes
                };
                tx_send_pkt(tx, &q, sizeof(q));
                tx->stats.queries_sent++;
                tx->state_enter_us = now;
            }
            return true;
        }

        case OTA_TX_STATE_START: {
            uint64_t now = get_time_us();
            if (now - tx->state_enter_us > tx->config.query_timeout_us) {
                tx->retries++;
                if (tx->retries > MAX_TIMEOUT_RETRIES) {
                    tx->state = OTA_TX_STATE_FAILED;
                    return false;
                }
                ota_start_pkt_t start_pkt = {
                    .pkt_type = OTA_PKT_TYPE_START,
                    .reserved = 0,
                    .magic = OTA_MAGIC_START,
                    .total_size = tx->config.total_size,
                    .image_crc32 = tx->config.image_crc32,
                    .total_windows = tx->total_windows,
                    .chunk_data_size = OTA_CHUNK_DATA_SIZE
                };
                tx_send_pkt(tx, &start_pkt, sizeof(start_pkt));
                tx->state_enter_us = now;
            }
            return true;
        }

        case OTA_TX_STATE_WAIT_READY: {
            /* NIPPON: READY(受け手->送り手, 線B想定)が喪失した場合の回復。
             * query_timeout_us 毎に、まだ確定していない窓(tx->commit_win_idx)への
             * QUERYを再送する。受け手はcommit済みなら last_ready キャッシュを
             * 再送し(ota_receiver.c)、まだ書き込み中なら通常のQUERY_RESPを返す
             * (この時 tx->state はWAIT_READYのままなのでQUERY_RESPは無視される=害はない)。
             * MAX_TIMEOUT_RETRIES 回再送しても復帰しなければ FAILED とする。 */
            uint64_t now = get_time_us();
            if (now - tx->state_enter_us > tx->config.query_timeout_us) {
                tx->retries++;
                if (tx->retries > MAX_TIMEOUT_RETRIES) {
                    tx->state = OTA_TX_STATE_FAILED;
                    return false;
                }
                ota_query_pkt_t q = {
                    .pkt_type = OTA_PKT_TYPE_QUERY,
                    .win_idx = (uint8_t)tx->commit_win_idx,
                    .reserved = 0,
                    .win_crc32 = tx->current_win_crc32,
                    .win_bytes = tx->current_win_bytes
                };
                tx_send_pkt(tx, &q, sizeof(q));
                tx->stats.queries_sent++;
                tx->state_enter_us = now;
            }
            return true;
        }

        case OTA_TX_STATE_FINALIZE: {
            uint64_t now = get_time_us();
            if (now - tx->state_enter_us > tx->config.query_timeout_us) {
                tx->retries++;
                if (tx->retries > MAX_TIMEOUT_RETRIES) {
                    tx->state = OTA_TX_STATE_FAILED;
                    return false;
                }
                ota_finalize_pkt_t finalize_pkt = {
                    .pkt_type = OTA_PKT_TYPE_FINALIZE,
                    .reserved = {0, 0, 0},
                    .image_crc32 = tx->config.image_crc32
                };
                tx_send_pkt(tx, &finalize_pkt, sizeof(finalize_pkt));
                tx->state_enter_us = now;
            }
            return true;
        }

        case OTA_TX_STATE_COMPLETE:
        case OTA_TX_STATE_FAILED:
        case OTA_TX_STATE_IDLE:
        default:
            return false;
    }
}
