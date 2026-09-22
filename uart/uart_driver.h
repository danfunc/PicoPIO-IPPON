#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "common/packet.h"
#include "common/ring_cursor.h"
#include "common/benchmark_common.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// UART リングバッファ定数 (M-6: プロジェクト唯一の定義)
// ============================================================================
#define UART_RX_RING_BYTES 1024
#define UART_RX_RING_BITS  10

// ============================================================================
// UART ドライバ構造体 (H-4)
// ============================================================================
typedef struct __attribute__((aligned(UART_RX_RING_BYTES))) {
    uint8_t rx_ring_buf[UART_RX_RING_BYTES] __attribute__((aligned(UART_RX_RING_BYTES)));
    uart_inst_t *uart_id;
    uint tx_pin;
    uint rx_pin;
    uint32_t baudrate;
    int dma_tx_chan;
    int dma_rx_chan;
    ring_cursor_t ring;
    uint8_t last_seq;
    bool has_received_first;
    bool is_initialized;
    poc_stats_t stats;
} uart_driver_t;

/**
 * @brief UART ドライバ構造体を安全な未初期化状態にゼロクリア
 */
void uart_driver_zero(uart_driver_t *drv);

/**
 * @brief UART ドライバの初期化
 */
bool uart_driver_init(uart_driver_t *drv, uart_inst_t *uart_id, uint tx_pin, uint rx_pin, uint32_t baudrate);

/**
 * @brief UART ドライバの停止・クリーンアップ
 */
void uart_driver_deinit(uart_driver_t *drv);

/**
 * @brief ボーレート変更
 */
bool uart_driver_set_baudrate(uart_driver_t *drv, uint32_t baudrate);

/**
 * @brief ポストシンクパケット送信 (ブロッキング/PHY計測付き)
 */
bool uart_tx_send_packet_blocking(uart_driver_t *drv, const uint8_t *post_sync_frame, size_t frame_bytes, uint8_t payload_len);

/**
 * @brief UART パケット受信ポーリング (M-2: プロジェクト唯一の実装)
 */
bool uart_rx_poll_packet(uart_driver_t *drv, uint8_t *out_type, uint8_t *out_seq, uint8_t *out_payload, uint8_t *out_len);

/**
 * @brief UART 統計情報の取得
 */
void uart_get_stats(const uart_driver_t *drv, poc_stats_t *out_stats);

/**
 * @brief UART 統計情報のリセット
 */
void uart_reset_stats(uart_driver_t *drv);

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H
