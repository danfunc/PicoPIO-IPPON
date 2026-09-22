#ifndef BMC_DRIVER_H
#define BMC_DRIVER_H

// ============================================================================
// H-6: 生成されたPIOヘッダを最上部に集約配置
// ============================================================================
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "bmc_p2p.pio.h"
#include "bmc_p2p_fast.pio.h"
#include "bmc_p2p_mid12.pio.h"
#include "bmc_p2p_mid10.pio.h"
#include "common/packet.h"
#include "common/benchmark_common.h"
#include "common/sram_attrs.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 定数定義
// ============================================================================
// ターゲット PIO クロック: 50MHz (16cyc/bit -> 3.125 Mbps)
// RP2350 (150MHz): clkdiv = 3.0f
// RP2040 (125MHz): clkdiv = 2.5f
static inline float bmc_get_clkdiv(void) {
    return (float)clock_get_hz(clk_sys) / 50000000.0f;
}

// RXリングバッファサイズ: 16384バイト (4096ワード, 14bit ring)
#define BMC_RX_RING_BYTES 16384
#define BMC_RX_RING_WORDS (BMC_RX_RING_BYTES / sizeof(uint32_t))
#define BMC_RX_RING_BITS  14

// RX DMA reload ワード数 (RP2350 TRIGGER_SELF 28bit最大値: 0x0FFFFFFF ≈ 268M words ≈ 1.07GB)
#define BMC_RX_DMA_RELOAD_WORDS 0x0FFFFFFFu

// ============================================================================
// レートファミリー定義 (A-2)
// ============================================================================
typedef enum {
    BMC_RATE_STD_16CYC = 0,  // 16 cyc/bit (3.125M/9.375M, 既存標準)
    BMC_RATE_MID_12CYC,      // 12 cyc/bit (12.5Mbps, 新規)
    BMC_RATE_MID_10CYC,      // 10 cyc/bit (15.0Mbps, 新規)
    BMC_RATE_FAST_8CYC       // 8 cyc/bit (18.75Mbps, 既存高速実験)
} bmc_rate_family_t;

// ============================================================================
// TX共有コントローラ構造体 (常駐型SM)
// ============================================================================
typedef struct {
    PIO pio;
    uint sm_tx;
    uint offset_tx;
    int dma_tx_chan;
    uint current_tx_pin;
    float clkdiv;
    bmc_rate_family_t rate_family;
    bool pad_fast_variant;                // B-1: FASTスルーレート+8mA駆動変種 (FAST_8CYC専用)
    bool is_initialized;
    const uint32_t *pending_dma_buf;      // 非同期送信中のDMAソースバッファ (統計計算用)
    size_t pending_total_dma_words;       // 同上、総ワード数
    absolute_time_t pending_t_start;      // 非同期送信開始時刻
    bool pending_active;                  // 非同期送信が進行中かどうか
    poc_stats_t stats;
} bmc_tx_controller_t;

// ============================================================================
// RX専従ポート構造体 (16384バイト境界保証 & ビット単位同期スキャナ)
// ============================================================================
typedef struct __attribute__((aligned(BMC_RX_RING_BYTES))) {
    // DMA リングバッファ (必ず先頭配置 & 16384バイト境界保証)
    uint32_t rx_ring_buf[BMC_RX_RING_WORDS] __attribute__((aligned(BMC_RX_RING_BYTES)));
    PIO pio;
    uint sm_rx;
    uint offset_rx;
    int dma_rx_chan;
    uint rx_pin;
    float clkdiv;
    bmc_rate_family_t rate_family;
    size_t tail_bit;              // CPUビット読み出し位置 (0..131071, ring modulo)
    size_t last_head_byte;        // 直前のDMA書き込みヘッドバイト位置
    uint64_t lap_count;           // DMA TRIGGER_SELF のラップ回数 (28bit reload 周期)
    uint32_t last_transfer_count; // 直前の transfer_count レジスタ値 (COUNT部)
    uint64_t total_words_written; // 累計受信ワード数 (lap_count方式)
    uint64_t total_bytes_written; // 累計受信バイト数 (オーバーラン検出用, uint64_t)
    uint64_t total_bits_read;     // 累計読み出しビット数 (ビットドリフト防止)
    uint8_t last_seq;             // 直前のSEQ (欠落検知用)
    bool has_received_first;
    bool last_extract_exhausted;  // 直前の抽出試行がINCOMPLETE/SYNC_NOT_FOUNDで、
                                   // 現在のtransfer_countまでのデータを使い切ったかどうか
                                   // (新規DMA語が無ければ再抽出しても結果不変 -> スキップ判定に使用)
    bool is_initialized;
    poc_stats_t stats;
} bmc_rx_port_t;

// ============================================================================
// API 関数宣言
// ============================================================================

/**
 * @brief TXピンのGPIO初期化 (SIO High固定、読み戻しイネーブル)
 */
void bmc_tx_pin_init(uint pin);

/**
 * @brief TX共有コントローラの初期化
 */
void bmc_tx_controller_zero(bmc_tx_controller_t *tx);
void bmc_tx_controller_deinit(bmc_tx_controller_t *tx);
bool bmc_tx_controller_init(bmc_tx_controller_t *tx, PIO pio, uint sm);

void bmc_rx_port_zero(bmc_rx_port_t *rx);
void bmc_rx_port_deinit(bmc_rx_port_t *rx);

/**
 * @brief TXコントローラのモード設定
 */
void bmc_tx_controller_set_mode(bmc_tx_controller_t *tx, uint offset_tx,
                                float clkdiv, bmc_rate_family_t rate_family);

/**
 * @brief TX常駐SMのパッド設定バリアント更新 (B-1: FAST_8CYC専用)
 */
void bmc_tx_controller_set_pad_variant(bmc_tx_controller_t *tx, bool pad_fast_variant);

/**
 * @brief TX常駐SMを明示的に即時確立する (RX起動前の呼出しを保証するため)
 */
void bmc_tx_controller_prime(bmc_tx_controller_t *tx, uint tx_pin);

/**
 * @brief パケット送信を非ブロッキングで開始する (B-4: TXパイプライン化)
 *        物理送信完了は待たない。呼出し後は bmc_tx_wait_packet_done() で完了を待つこと。
 *        dma_buf は完了待ちが終わるまで内容を変更・再利用してはならない。
 */
bool bmc_tx_start_packet_async(bmc_tx_controller_t *tx, uint tx_pin,
                               const uint32_t *dma_buf, size_t total_dma_words);

/**
 * @brief bmc_tx_start_packet_async() で開始した送信の物理完了を待ち、統計を確定する
 * @return bool 送信成功時 true, タイムアウト時 false。進行中の送信がなければ true を返す。
 */
bool bmc_tx_wait_packet_done(bmc_tx_controller_t *tx);

/**
 * @brief パケット送信 (ブロッキング/物理完了IRQ0検知, 常駐SM駆動, PHY計測付き)
 *
 * @param tx TXコントローラ
 * @param tx_pin 送信先ピン
 * @param dma_buf 送信バッファ (bmc_packet_encodeで生成済み)
 * @param total_dma_words DMA転送総ワード数
 * @return bool 送信成功時 true, タイムアウト時 false
 */
bool bmc_tx_send_packet_blocking(bmc_tx_controller_t *tx, uint tx_pin,
                                 const uint32_t *dma_buf,
                                 size_t total_dma_words);

/**
 * @brief RX専従ポートの初期化 (通常: 3.125 Mbps @ 150MHz)
 */
bool bmc_rx_port_init(bmc_rx_port_t *rx, PIO pio, uint sm, uint offset_rx, uint pin);

/**
 * @brief RX専従ポートの初期化 (拡張: clkdiv, rate_family指定可能)
 */
bool bmc_rx_port_init_ex(bmc_rx_port_t *rx, PIO pio, uint sm, uint offset_rx,
                         uint pin, float clkdiv, bmc_rate_family_t rate_family);

/**
 * @brief RX生データおよびDMA状態の1回限り診断ダンプ (B-2)
 */
void bmc_rx_dump_diagnostic(const bmc_rx_port_t *rx);

/**
 * @brief 受信リングバッファからパケットを1件取り出してデコード (非ブロッキング, ビット単位同期復旧)
 *
 * @param rx RXポート
 * @param out_type 受信パケット種別出力先
 * @param out_seq 受信シーケンス番号出力先
 * @param out_payload 受信ペイロード格納先 (最低128バイト)
 * @param out_len 受信ペイロード長格納先
 * @return bool パケットを受信・検証完了した場合は true, なければ false
 */
bool BMC_SRAM_FUNC(bmc_rx_poll_packet)(bmc_rx_port_t *rx, uint8_t *out_type, uint8_t *out_seq,
                        uint8_t *out_payload, uint8_t *out_len);

/**
 * @brief TX統計情報の取得 (M-9)
 */
void bmc_tx_get_stats(const bmc_tx_controller_t *tx, poc_stats_t *out_stats);

/**
 * @brief TX統計情報のリセット (M-9)
 */
void bmc_tx_reset_stats(bmc_tx_controller_t *tx);

/**
 * @brief RX統計情報の取得 (M-9)
 */
void bmc_rx_get_stats(const bmc_rx_port_t *rx, poc_stats_t *out_stats);

/**
 * @brief RX統計情報のリセット (M-9)
 */
void bmc_rx_reset_stats(bmc_rx_port_t *rx);

#ifdef __cplusplus
}
#endif

#endif // BMC_DRIVER_H
