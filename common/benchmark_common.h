#ifndef BENCHMARK_COMMON_H
#define BENCHMARK_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 1. 統一統計構造体 (M-3: プロジェクト唯一の stats 型定義)
// ============================================================================
typedef struct {
    uint32_t tx_packets;            // 送信完了パケット数
    uint32_t tx_bytes;              // 送信完了ペイロードバイト数
    uint32_t tx_wire_bytes;         // 送信完了物理ワイヤバイト数
    uint32_t tx_timeouts;           // 送信完了待機タイムアウト回数
    uint32_t tx_failures;           // 送信失敗/アボート回数
    uint64_t tx_phy_duration_us;    // 純粋なPHY送信所要時間の累計 (µs)

    uint32_t rx_packets;            // 受信・検証完了パケット数
    uint32_t rx_bytes;              // 受信完了ペイロードバイト数
    uint32_t rx_crc_errors;         // CRCエラー検出回数
    uint32_t rx_len_errors;         // LENエラー検出回数
    uint32_t rx_seq_drops;          // シーケンス欠落パケット数
    uint32_t rx_overrun_errors;     // リングバッファオーバーラン回数
    uint32_t rx_timeouts;           // 受信ポーリングタイムアウト回数
    uint32_t verify_errors;         // アプリケーション層検証不一致回数

    uint64_t rtt_total_us;          // アプリケーションRound-Trip時間の累計 (µs)
    uint32_t rtt_samples;           // RTT計測回数

    float avg_tx_kibps;             // 送信実効レート (KiB/s)
    float avg_rx_kibps;             // 受信実効レート (KiB/s)

    // 追加フィールド (不具合1対策およびCPUコスト計装用)
    uint32_t late_arrival_drops;    // 期待seqより古い遅延到着パケット破棄数 (不具合1対処)
    uint64_t rx_cpu_cost_total_us;  // RXパケット処理所要時間の累計 (µs)
    uint32_t rx_cpu_cost_samples;   // RX処理時間計測サンプル数
    uint32_t rx_cpu_cost_max_us;    // RXパケット処理所要時間の最大値 (µs)
} poc_stats_t;

#define bmc_stats_t poc_stats_t
#define uart_stats_t poc_stats_t

// ============================================================================
// 2. 統一テストパケット長定義 (M-5: プロジェクト唯一の test_lens 定義)
// ============================================================================
#define POC_NUM_TEST_LENS 5
extern const uint8_t g_poc_test_lens[POC_NUM_TEST_LENS];
extern const size_t g_poc_num_test_lens;

// ============================================================================
// 3. 統一トランスポートインターフェース (Requirement 7)
// ============================================================================
typedef struct poc_transport poc_transport_t;

struct poc_transport {
    const char *name;
    bool is_experimental;
    float target_phy_mbps;
    bool (*send_packet)(void *ctx, unsigned int pin, const uint32_t *dma_buf, size_t dma_words, uint8_t payload_len);
    bool (*poll_packet)(void *ctx, uint8_t *out_type, uint8_t *out_seq, uint8_t *out_payload, uint8_t *out_len);
    void (*get_stats)(const void *ctx, poc_stats_t *out_stats);
    void (*reset_stats)(void *ctx);
    void *ctx;
};

void poc_stats_reset(poc_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // BENCHMARK_COMMON_H
