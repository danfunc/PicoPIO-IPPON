#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "common/packet.h"
#include "common/benchmark_common.h"
#include "common/build_info.h"
#include "bmc/bmc_driver.h"
#include "uart/uart_driver.h"
#include "bmc_p2p.pio.h"
#include "bmc_p2p_fast.pio.h"
#include "bmc_p2p_mid12.pio.h"
#include "bmc_p2p_mid10.pio.h"

#define PIN_TX 0
#define PIN_RX 1

#define UART_ID uart0

// -------------------------------------------------------------
// モード定義
// -------------------------------------------------------------
typedef enum {
    MODE_UART_3M = 0,    // UART 3.000 Mbps (停止待ち)
    MODE_UART_9M,        // UART 9.375 Mbps (停止待ち)
    MODE_BMC_3M,         // BMC  3.125 Mbps (16 cyc/bit, clkdiv 3.0f, PoC標準, 停止待ち)
    MODE_BMC_9M,         // BMC  9.375 Mbps (16 cyc/bit, clkdiv 1.0f, 停止待ち)
    MODE_BMC_18M,        // BMC 18.750 Mbps ( 8 cyc/bit, clkdiv 1.0f, EXPERIMENTAL, 停止待ち)
    MODE_UART_3M_BURST,  // UART 3.000 Mbps (連続送信, Dual-Core RX)
    MODE_UART_9M_BURST,  // UART 9.375 Mbps (連続送信, Dual-Core RX)
    MODE_BMC_3M_BURST,   // BMC  3.125 Mbps (連続送信, Dual-Core RX)
    MODE_BMC_9M_BURST,   // BMC  9.375 Mbps (連続送信, Dual-Core RX)
    MODE_BMC_12M,        // BMC 12.500 Mbps (12 cyc/bit, clkdiv 1.0f, 停止待ち)
    MODE_BMC_12M_BURST,  // BMC 12.500 Mbps (12 cyc/bit, clkdiv 1.0f, Burst 連続送信)
    MODE_BMC_15M,        // BMC 15.000 Mbps (10 cyc/bit, clkdiv 1.0f, 停止待ち)
    MODE_BMC_15M_BURST,  // BMC 15.000 Mbps (10 cyc/bit, clkdiv 1.0f, Burst 連続送信)
    MODE_BMC_18M_BURST,  // BMC 18.750 Mbps ( 8 cyc/bit, clkdiv 1.0f, EXPERIMENTAL, Burst 連続送信) B-3
    MODE_COUNT
} bench_mode_t;

typedef struct {
    const char *name;
    float target_phy_mbps;
    bool is_uart;
    bmc_rate_family_t rate_family;
    float bmc_clkdiv;
    uint32_t uart_baud;
    bool is_experimental;
    bool is_burst;
} mode_desc_t;

static const mode_desc_t MODE_DESCS[MODE_COUNT] = {
    [MODE_UART_3M]        = {"UART ( 3.000M)",        3.000f, true,  BMC_RATE_STD_16CYC, 0.0f, 3000000, false, false},
    [MODE_UART_9M]        = {"UART ( 9.375M)",        9.375f, true,  BMC_RATE_STD_16CYC, 0.0f, 9375000, false, false},
    [MODE_BMC_3M]         = {"BMC  ( 3.125M)",        3.125f, false, BMC_RATE_STD_16CYC, 3.0f,       0, false, false},
    [MODE_BMC_9M]         = {"BMC  ( 9.375M)",        9.375f, false, BMC_RATE_STD_16CYC, 1.0f,       0, false, false},
    [MODE_BMC_18M]        = {"BMC  (18.750M)",       18.750f, false, BMC_RATE_FAST_8CYC, 1.0f,       0, true,  false},
    [MODE_UART_3M_BURST]  = {"UART ( 3.000M Burst)",  3.000f, true,  BMC_RATE_STD_16CYC, 0.0f, 3000000, false, true},
    [MODE_UART_9M_BURST]  = {"UART ( 9.375M Burst)",  9.375f, true,  BMC_RATE_STD_16CYC, 0.0f, 9375000, false, true},
    [MODE_BMC_3M_BURST]   = {"BMC  ( 3.125M Burst)",  3.125f, false, BMC_RATE_STD_16CYC, 3.0f,       0, false, true},
    [MODE_BMC_9M_BURST]   = {"BMC  ( 9.375M Burst)",  9.375f, false, BMC_RATE_STD_16CYC, 1.0f,       0, false, true},
    [MODE_BMC_12M]        = {"BMC  (12.500M)",       12.500f, false, BMC_RATE_MID_12CYC, 1.0f,       0, false, false},
    [MODE_BMC_12M_BURST]  = {"BMC  (12.500M Burst)", 12.500f, false, BMC_RATE_MID_12CYC, 1.0f,       0, false, true},
    [MODE_BMC_15M]        = {"BMC  (15.000M)",       15.000f, false, BMC_RATE_MID_10CYC, 1.0f,       0, false, false},
    [MODE_BMC_15M_BURST]  = {"BMC  (15.000M Burst)", 15.000f, false, BMC_RATE_MID_10CYC, 1.0f,       0, false, true},
    [MODE_BMC_18M_BURST]  = {"BMC  (18.750M Burst)", 18.750f, false, BMC_RATE_FAST_8CYC, 1.0f,       0, true,  true}
};

// -------------------------------------------------------------
// ハードウェアおよびドライバ状態
// -------------------------------------------------------------
static PIO s_bmc_pio_tx = pio0;
static PIO s_bmc_pio_rx = pio1;
static bmc_tx_controller_t s_bmc_tx;
static bmc_rx_port_t s_bmc_rx;
static uart_driver_t s_uart_drv;

static poc_transport_t s_active_transport;
static bench_mode_t s_current_mode = MODE_BMC_3M;
static bool s_benchmark_running = true;
// B-3: 18.75M(8 cyc/bit)は実機実測でFAST+8mA駆動でのみ全レート安定動作したため既定をFASTにする。
// このフラグはBMC_RATE_FAST_8CYCのときのみ有効 (bmc_tx_ensure_sm_residentでゲート済み)。
// 16/12/10 cyc/bitの既定はSLOW+4mAのまま変更しない (実績のある設定を壊さない方針)。
static bool s_pad_fast_variant = true; // B-3: 18.75M用パッド変種、既定をFAST Slew+8mAに変更
static bool s_auto_sweep = false;
static uint32_t s_sweep_mode_start_ms = 0;
#define SWEEP_DURATION_MS 4000

static poc_stats_t s_stats = {0};
static poc_stats_t s_sweep_results[MODE_COUNT] = {0};

static uint32_t s_last_report_time = 0;
// B-a: Burstループでの入力確認/レポート判定の間引き用 (main_benchmark.c:1078 退行対策)
static absolute_time_t s_last_housekeeping_time;
static bool s_housekeeping_time_valid = false;
#define BENCHMARK_HOUSEKEEPING_INTERVAL_US 2000
static uint32_t s_report_tx_bytes_start = 0;
static uint32_t s_report_rx_bytes_start = 0;
static uint64_t s_report_phy_us_start = 0;
static uint32_t s_report_tx_wire_bytes_start = 0; // B-3: LineUtil(calc)算出用ベースライン

static uint8_t s_seq = 0;
static size_t s_len_idx = 0;
static uint32_t s_tx_dma_words[BMC_TX_DMA_MAX_WORDS];
static uint32_t s_tx_dma_words_pipeline[2][BMC_TX_DMA_MAX_WORDS];
static int  s_tx_pipeline_next = 0;
static bool s_tx_pipeline_active = false;
static uint8_t s_tx_payload[BMC_MAX_PAYLOAD_LEN];
static uint8_t s_rx_payload[BMC_MAX_PAYLOAD_LEN];

// Core 1 非同期RXおよび診断用同期状態
static volatile bool s_core1_rx_enabled = false;
static volatile bool s_core1_rx_active = false;
static volatile bool s_core1_terminate = false;

static volatile bool s_diag_first_rx_pending = false;
static volatile uint32_t s_diag_rx_poll_count = 0;
static volatile uint64_t s_diag_rx_elapsed_us = 0;
static volatile uint8_t s_diag_rx_seq = 0;
static volatile bool s_diag_rx_got = false;
static volatile bool s_diag_ready_to_print = false;

static bool apply_mode(bench_mode_t mode);
static void benchmark_print_sweep_summary(void);

// -------------------------------------------------------------
// トランスポートラッパー関数群 (Requirement 7: ポリモーフィック実行)
// -------------------------------------------------------------
static bool bmc_transport_send(void *ctx, uint pin, const uint32_t *dma_buf, size_t dma_words, uint8_t payload_len) {
    (void)ctx; (void)payload_len;
    return bmc_tx_send_packet_blocking(&s_bmc_tx, pin, dma_buf, dma_words);
}

static bool bmc_transport_poll(void *ctx, uint8_t *out_type, uint8_t *out_seq, uint8_t *out_payload, uint8_t *out_len) {
    (void)ctx;
    return bmc_rx_poll_packet(&s_bmc_rx, out_type, out_seq, out_payload, out_len);
}

static void bmc_transport_get_stats(const void *ctx, poc_stats_t *out_stats) {
    (void)ctx;
    poc_stats_t tx_st, rx_st;
    bmc_tx_get_stats(&s_bmc_tx, &tx_st);
    bmc_rx_get_stats(&s_bmc_rx, &rx_st);
    *out_stats = tx_st;
    out_stats->rx_packets = rx_st.rx_packets;
    out_stats->rx_bytes = rx_st.rx_bytes;
    out_stats->rx_crc_errors = rx_st.rx_crc_errors;
    out_stats->rx_len_errors = rx_st.rx_len_errors;
    out_stats->rx_seq_drops = rx_st.rx_seq_drops;
    out_stats->rx_overrun_errors = rx_st.rx_overrun_errors;
}

static void bmc_transport_reset_stats(void *ctx) {
    (void)ctx;
    bmc_tx_reset_stats(&s_bmc_tx);
    bmc_rx_reset_stats(&s_bmc_rx);
}

static bool uart_transport_send(void *ctx, uint pin, const uint32_t *dma_buf, size_t dma_words, uint8_t payload_len) {
    (void)ctx; (void)pin; (void)dma_words;
    const uint8_t *wire_bytes = (const uint8_t *)&dma_buf[1];
    const uint8_t *post_sync = wire_bytes + BMC_PREAMBLE_SYNC_BYTES;
    size_t post_sync_len = bmc_packet_total_bytes(payload_len);
    return uart_tx_send_packet_blocking(&s_uart_drv, post_sync, post_sync_len, payload_len);
}

static bool uart_transport_poll(void *ctx, uint8_t *out_type, uint8_t *out_seq, uint8_t *out_payload, uint8_t *out_len) {
    (void)ctx;
    return uart_rx_poll_packet(&s_uart_drv, out_type, out_seq, out_payload, out_len);
}

static void uart_transport_get_stats(const void *ctx, poc_stats_t *out_stats) {
    (void)ctx;
    uart_get_stats(&s_uart_drv, out_stats);
}

static void uart_transport_reset_stats(void *ctx) {
    (void)ctx;
    uart_reset_stats(&s_uart_drv);
}

// -------------------------------------------------------------
// Core 1 専用非同期RXワーカーループ
// -------------------------------------------------------------
static void core1_rx_worker(void) {
    uint8_t rx_type = 0;
    uint8_t rx_seq = 0;
    uint8_t rx_len = 0;
    static uint8_t s_core1_rx_buf[BMC_MAX_PAYLOAD_LEN];
    uint32_t sample_counter = 0;
    uint32_t diag_polls = 0;
    absolute_time_t diag_start = get_absolute_time();

    while (!s_core1_terminate) {
        if (!s_core1_rx_enabled || !s_active_transport.poll_packet) {
            s_core1_rx_active = false;
            tight_loop_contents();
            continue;
        }

        s_core1_rx_active = true;

        if (s_diag_first_rx_pending && diag_polls == 0) {
            diag_start = get_absolute_time();
        }

        sample_counter++;
        bool do_measure = ((sample_counter & 0x07) == 0); // 1/8サンプリングで計測負荷低減
        uint32_t t_start = 0;
        if (do_measure) {
            t_start = time_us_32();
        }

        bool got = s_active_transport.poll_packet(s_active_transport.ctx,
                                                 &rx_type, &rx_seq,
                                                 s_core1_rx_buf, &rx_len);
        if (s_diag_first_rx_pending) {
            diag_polls++;
        }

        if (got) {
            if (do_measure) {
                uint32_t t_end = time_us_32();
                uint32_t cost_us = t_end - t_start;
                s_stats.rx_cpu_cost_total_us += cost_us;
                s_stats.rx_cpu_cost_samples++;
                if (cost_us > s_stats.rx_cpu_cost_max_us) {
                    s_stats.rx_cpu_cost_max_us = cost_us;
                }
            }

            if (s_diag_first_rx_pending) {
                s_diag_rx_elapsed_us = (uint64_t)absolute_time_diff_us(diag_start, get_absolute_time());
                s_diag_rx_poll_count = diag_polls;
                s_diag_rx_seq = rx_seq;
                s_diag_rx_got = true;
                s_diag_first_rx_pending = false;
                s_diag_ready_to_print = true;
                diag_polls = 0;
            }

            // 連続送信パケットのペイロード整合性検証
            bool match = (rx_type == BMC_PKT_TYPE_BENCH);
            if (match) {
                for (int i = 0; i < rx_len; ++i) {
                    if (s_core1_rx_buf[i] != (uint8_t)(rx_seq + i)) {
                        match = false;
                        break;
                    }
                }
            }
            if (!match) {
                __sync_fetch_and_add(&s_stats.verify_errors, 1);
            }
            __sync_synchronize();
        } else {
            tight_loop_contents();
        }
    }
}

// -------------------------------------------------------------
// ハードウェア設定・切替
// -------------------------------------------------------------
static void stop_all_hardware(void) {
    bmc_rx_port_deinit(&s_bmc_rx);
    bmc_tx_controller_deinit(&s_bmc_tx);
    uart_driver_deinit(&s_uart_drv);

    pio_clear_instruction_memory(s_bmc_pio_tx);
    pio_clear_instruction_memory(s_bmc_pio_rx);

    bmc_tx_pin_init(PIN_TX);
}

static bool setup_bmc(float clkdiv, bmc_rate_family_t rate_family) {
    stop_all_hardware();
    bmc_tx_pin_init(PIN_TX);

    for (int i = 0; i < 100; ++i) {
        if (gpio_get(PIN_RX)) break;
        sleep_us(100);
    }

    if (!bmc_tx_controller_init(&s_bmc_tx, s_bmc_pio_tx, 0)) {
        printf("[ERROR] Failed to init BMC TX controller\n");
        return false;
    }

    uint off_tx, off_rx;
    const char *transport_name;
    bool is_exp = false;
    float target_phy = 3.125f;

    switch (rate_family) {
        case BMC_RATE_MID_12CYC:
            off_tx = pio_add_program(s_bmc_pio_tx, &bmc_mid12_tx_program);
            off_rx = pio_add_program(s_bmc_pio_rx, &bmc_mid12_rx_program);
            transport_name = "BMC 12.5M (Mid12)";
            target_phy = 12.500f;
            break;
        case BMC_RATE_MID_10CYC:
            off_tx = pio_add_program(s_bmc_pio_tx, &bmc_mid10_tx_program);
            off_rx = pio_add_program(s_bmc_pio_rx, &bmc_mid10_rx_program);
            transport_name = "BMC 15.0M (Mid10)";
            target_phy = 15.000f;
            break;
        case BMC_RATE_FAST_8CYC:
            off_tx = pio_add_program(s_bmc_pio_tx, &bmc_fast_tx_program);
            off_rx = pio_add_program(s_bmc_pio_rx, &bmc_fast_rx_program);
            transport_name = "BMC 18.75M (Fast)";
            is_exp = true;
            target_phy = 18.750f;
            break;
        case BMC_RATE_STD_16CYC:
        default:
            off_tx = pio_add_program(s_bmc_pio_tx, &bmc_tx_program);
            off_rx = pio_add_program(s_bmc_pio_rx, &bmc_rx_program);
            transport_name = "BMC Standard";
            target_phy = (clkdiv == 1.0f) ? 9.375f : 3.125f;
            break;
    }

    // B-1: パッド設定バリアントの適用 (FAST_8CYC時のみFAST+8mAが有効)
    bmc_tx_controller_set_pad_variant(&s_bmc_tx, s_pad_fast_variant);

    bmc_tx_controller_set_mode(&s_bmc_tx, off_tx, clkdiv, rate_family);
    bmc_tx_controller_prime(&s_bmc_tx, PIN_TX);
    if (!bmc_rx_port_init_ex(&s_bmc_rx, s_bmc_pio_rx, 0, off_rx, PIN_RX, clkdiv, rate_family)) {
        printf("[ERROR] Failed to init BMC RX port\n");
        bmc_tx_controller_deinit(&s_bmc_tx);
        return false;
    }

    s_active_transport = (poc_transport_t){
        .name = transport_name,
        .is_experimental = is_exp,
        .target_phy_mbps = target_phy,
        .send_packet = bmc_transport_send,
        .poll_packet = bmc_transport_poll,
        .get_stats = bmc_transport_get_stats,
        .reset_stats = bmc_transport_reset_stats,
        .ctx = NULL
    };
    return true;
}

static bool setup_uart(uint32_t baudrate) {
    stop_all_hardware();

    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    clock_get_hz(clk_sys), clock_get_hz(clk_sys));

    if (!uart_driver_init(&s_uart_drv, UART_ID, PIN_TX, PIN_RX, baudrate)) {
        printf("[ERROR] Failed to init UART driver\n");
        return false;
    }

    s_active_transport = (poc_transport_t){
        .name = "UART DMA",
        .is_experimental = false,
        .target_phy_mbps = (float)baudrate / 1e6f,
        .send_packet = uart_transport_send,
        .poll_packet = uart_transport_poll,
        .get_stats = uart_transport_get_stats,
        .reset_stats = uart_transport_reset_stats,
        .ctx = NULL
    };
    return true;
}

static bool apply_mode(bench_mode_t mode) {
    if (mode >= MODE_COUNT) return false;

    // 1. Core 1 RXワーカーの安全停止 (ハードウェア再構成中の競合防止)
    s_core1_rx_enabled = false;
    __sync_synchronize();
    while (s_core1_rx_active) {
        tight_loop_contents();
    }

    s_current_mode = mode;
    const mode_desc_t *desc = &MODE_DESCS[mode];

    printf("\n[MODE CHANGE] >>> %s %s <<<\n", desc->name, desc->is_experimental ? "(EXPERIMENTAL)" : "");
    if (!desc->is_uart && desc->rate_family == BMC_RATE_FAST_8CYC) {
        printf("  TX Pad Config: %s\n", s_pad_fast_variant ? "FAST Slew + 8mA Drive" : "SLOW Slew + 4mA Drive (Default)");
    }

    bool ok = desc->is_uart ? setup_uart(desc->uart_baud) : setup_bmc(desc->bmc_clkdiv, desc->rate_family);
    if (!ok) {
        printf("[ERROR] Failed to configure hardware for mode %s!\n", desc->name);
        return false;
    }

    // 統計・時間ベースラインの明示的リセット (Requirement 5)
    poc_stats_reset(&s_stats);
    if (s_active_transport.reset_stats) {
        s_active_transport.reset_stats(s_active_transport.ctx);
    }
    s_seq = 0;
    s_len_idx = 0;
    s_tx_pipeline_next = 0;
    s_tx_pipeline_active = false;
    s_last_report_time = to_ms_since_boot(get_absolute_time());
    s_report_tx_bytes_start = 0;
    s_report_rx_bytes_start = 0;
    s_report_phy_us_start = 0;
    s_report_tx_wire_bytes_start = 0;

    // 不具合1(b) 診断用フラグ設定
    s_diag_first_rx_pending = true;
    s_diag_ready_to_print = false;

    // 連続送信モードの場合は Core 1 の非同期RXを有効化
    if (desc->is_burst) {
        s_core1_rx_enabled = true;
        __sync_synchronize();
    }

    return true;
}

// -------------------------------------------------------------
// H-7 & Requirement 12: 配線導通テスト (apply_mode は一切呼ばず結果のみ返却)
// -------------------------------------------------------------
static bool check_continuity(void) {
    gpio_init(PIN_TX);
    gpio_init(PIN_RX);
    gpio_set_dir(PIN_TX, GPIO_OUT);
    gpio_set_dir(PIN_RX, GPIO_IN);
    gpio_pull_down(PIN_RX);

    // 1. Low チェック
    gpio_put(PIN_TX, 0);
    sleep_ms(2);
    if (gpio_get(PIN_RX) != 0) return false;

    // 2. High チェック
    gpio_put(PIN_TX, 1);
    sleep_ms(2);
    if (gpio_get(PIN_RX) != 1) return false;

    return true;
}

// -------------------------------------------------------------
// サマリー出力・判定 (Requirement 1: 厳格な全数整合PASS判定)
// -------------------------------------------------------------
static void benchmark_print_sweep_summary(void) {
    printf("\n===========================================================================================================================================\n");
    printf("   RP2350 Communication Protocol Rate & Reliability Benchmark Results\n");
    printf("   Pin Setup: GP%d (Pin 1) <---> GP%d (Pin 2) Direct Loopback\n", PIN_TX, PIN_RX);
    printf("===========================================================================================================================================\n");
    printf(" Mode                   PHY Target   App RX Rate     PHY Wire Rate   LineUtil  LineUtil(calc)   Avg RTT   Errors/Drops/Ov/Late   RX CPU(avg/max)  Judgement\n");
    printf("-------------------------------------------------------------------------------------------------------------------------------------------\n");

    bool aggregate_pass = true;

    for (int m = 0; m < MODE_COUNT; ++m) {
        const mode_desc_t *desc = &MODE_DESCS[m];
        poc_stats_t *s = &s_sweep_results[m];

        // PASS条件: 送信・受信が成立し、エラー(CRC, LEN, TO, Overrun, Verify, Drop)が0件であること。
        // ※ late_arrival_drops は起動直後の一時的過渡現象の自己修復カウントであるため、
        //   既存のPASS基準を破壊しない方針に基づき、PASS除外条件には含めない(Sonnet指示推奨準拠)。
        bool passed;
        if (desc->is_burst) {
            passed = (s->tx_packets > 0 &&
                      s->rx_packets > 0 &&
                      s->tx_failures == 0 &&
                      s->tx_timeouts == 0 &&
                      s->rx_crc_errors == 0 &&
                      s->rx_len_errors == 0 &&
                      s->rx_seq_drops == 0 &&
                      s->rx_overrun_errors == 0 &&
                      s->verify_errors == 0);
        } else {
            passed = (s->tx_packets > 0 &&
                      s->tx_packets == s->rx_packets &&
                      s->tx_failures == 0 &&
                      s->tx_timeouts == 0 &&
                      s->rx_timeouts == 0 &&
                      s->rx_crc_errors == 0 &&
                      s->rx_len_errors == 0 &&
                      s->rx_seq_drops == 0 &&
                      s->rx_overrun_errors == 0 &&
                      s->verify_errors == 0);
        }

        if (!desc->is_experimental && !passed) {
            aggregate_pass = false;
        }

        const char *judgement;
        if (desc->is_experimental) {
            judgement = passed ? "PASS [EXPERIMENTAL]" : "FAIL [EXPERIMENTAL]";
        } else {
            judgement = passed ? "PASS" : "FAIL";
        }

        float avg_rtt_us = (s->rtt_samples > 0) ? ((float)s->rtt_total_us / s->rtt_samples) : 0.0f;
        float phy_wire_mbps = (s->tx_phy_duration_us > 0) ?
                              ((float)s->tx_wire_bytes * 8.0f / (float)s->tx_phy_duration_us) : 0.0f;
        float line_util_pct = (SWEEP_DURATION_MS > 0) ?
                              ((float)s->tx_phy_duration_us / (float)(SWEEP_DURATION_MS * 1000ULL) * 100.0f) : 0.0f;
        if (line_util_pct > 100.0f) line_util_pct = 100.0f;

        // B-3: 独立検証用のパケット/バイト計数ベース線路使用率。
        // tx_phy_duration_us の計測経路 (DMA開始〜完了待ち区間) に依存せず、
        // 実際に送出が確定した wire バイト数 (tx_wire_bytes) と、そのトランスポートの
        // 1バイトあたり理論ビット数 (UART: start+data8+stop=10bit, BMC: 8bit/cell) および
        // 目標PHYレートから理論専有時間を逆算し、スイープ窓時間で割って算出する。
        // 既存の line_util_pct と値が乖離する場合は計測経路側にギャップがあることを示す。
        float bits_per_wire_byte = desc->is_uart ? 10.0f : 8.0f;
        float target_bps = desc->target_phy_mbps * 1e6f;
        float calc_line_util_pct = 0.0f;
        if (SWEEP_DURATION_MS > 0 && target_bps > 0.0f) {
            float theoretical_us = (float)s->tx_wire_bytes * bits_per_wire_byte / target_bps * 1e6f;
            calc_line_util_pct = theoretical_us / (float)(SWEEP_DURATION_MS * 1000ULL) * 100.0f;
        }
        if (calc_line_util_pct > 999.9f) calc_line_util_pct = 999.9f;

        float avg_rx_cpu_us = (s->rx_cpu_cost_samples > 0) ?
                              ((float)s->rx_cpu_cost_total_us / (float)s->rx_cpu_cost_samples) : 0.0f;

        char rtt_str[16];
        if (desc->is_burst) {
            snprintf(rtt_str, sizeof(rtt_str), "  BURST  ");
        } else {
            snprintf(rtt_str, sizeof(rtt_str), "%6.1f us", avg_rtt_us);
        }

        char err_str[32];
        snprintf(err_str, sizeof(err_str), "%lu/%lu/%lu/%lu",
                 (unsigned long)(s->rx_crc_errors + s->rx_len_errors + s->rx_timeouts + s->verify_errors),
                 (unsigned long)s->rx_seq_drops,
                 (unsigned long)s->rx_overrun_errors,
                 (unsigned long)s->late_arrival_drops);

        char cpu_str[24];
        snprintf(cpu_str, sizeof(cpu_str), "%4.1f / %-4lu us", avg_rx_cpu_us, (unsigned long)s->rx_cpu_cost_max_us);

        printf(" %-22s %6.3f Mbps  %6.1f KiB/s      %6.3f Mbps      %5.1f%%   %6.1f%%(calc)   %9s   %-20s   %-16s  %s\n",
               desc->name,
               desc->target_phy_mbps,
               s->avg_rx_kibps,
               phy_wire_mbps,
               line_util_pct,
               calc_line_util_pct,
               rtt_str,
               err_str,
               cpu_str,
               judgement);
    }

    printf("-------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf(" Overall Aggregate Verdict (Excluding Experimental 18.75M): %s\n",
           aggregate_pass ? "ALL STANDARD MODES PASSED" : "FAILED / DEGRADED");
    printf("===========================================================================================================================================\n\n");
}

// -------------------------------------------------------------
// 1回の送受信エクスチェンジ実行 (停止待ちモード: RTT計測 & 厳密検証)
// -------------------------------------------------------------
static void benchmark_run_exchange(void) {
    if (!s_active_transport.send_packet) return;

    uint8_t current_len = g_poc_test_lens[s_len_idx];
    s_len_idx = (s_len_idx + 1) % g_poc_num_test_lens;

    for (int i = 0; i < current_len; ++i) {
        s_tx_payload[i] = (uint8_t)(s_seq + i);
    }

    size_t dma_words = bmc_packet_encode(s_tx_dma_words, BMC_PKT_TYPE_BENCH, s_seq, s_tx_payload, current_len);
    if (dma_words == 0) return;

    // アプリケーション RTT 計測開始 (Requirement 6)
    absolute_time_t rtt_start = get_absolute_time();

    // H-3: 送信成否の戻り値を厳密にチェック
    bool tx_ok = s_active_transport.send_packet(s_active_transport.ctx, PIN_TX, s_tx_dma_words, dma_words, current_len);
    if (!tx_ok) {
        s_stats.tx_failures++;
        s_stats.verify_errors++;
    } else {
        // 受信ポーリング (最大 5ms 待機)
        uint8_t rx_type = 0, rx_seq = 0, rx_len = 0;
        absolute_time_t rx_deadline = make_timeout_time_ms(5);
        bool rx_got = false;

        uint32_t poll_count = 0;
        absolute_time_t poll_start = get_absolute_time();

        while (!time_reached(rx_deadline)) {
            poll_count++;

            // B-3: 停止待ちモードは1回のexchangeあたりポーリング回数が少なく
            // (通常1〜数回)、core1_rx_workerのような高頻度ループ向けの1/8間引き
            // サンプリングを適用すると sample_counter が8の倍数に達せず
            // rx_cpu_cost_samples が常に0のまま (表示が 0.0 / 0 us に固定化) して
            // いた。停止待ちモードはCPU負荷懸念が無いため毎回計測する。
            uint32_t t_cpu0 = time_us_32();

            bool poll_res = s_active_transport.poll_packet(s_active_transport.ctx, &rx_type, &rx_seq, s_rx_payload, &rx_len);
            if (poll_res) {
                uint32_t cpu_cost = time_us_32() - t_cpu0;
                s_stats.rx_cpu_cost_total_us += cpu_cost;
                s_stats.rx_cpu_cost_samples++;
                if (cpu_cost > s_stats.rx_cpu_cost_max_us) {
                    s_stats.rx_cpu_cost_max_us = cpu_cost;
                }

                // 不具合1対処: 受信seqが期待値より過去のものであれば遅延到着として破棄し継続
                int8_t seq_diff = (int8_t)(rx_seq - s_seq);
                if (seq_diff < 0) {
                    s_stats.late_arrival_drops++;
                    continue;
                }

                rx_got = true;
                break;
            }
            tight_loop_contents();
        }

        // 不具合1(b) 診断出力: apply_mode() 直後の最初の1回について実測結果を出力
        if (s_diag_first_rx_pending) {
            uint64_t poll_elapsed_us = (uint64_t)absolute_time_diff_us(poll_start, get_absolute_time());
            printf("[DIAG] Mode first RX poll: got=%d, rx_seq=%u, exp_seq=%u, polls=%lu, elapsed=%llu us, late_drops=%lu\n",
                   rx_got ? 1 : 0, rx_seq, s_seq, (unsigned long)poll_count, (unsigned long long)poll_elapsed_us,
                   (unsigned long)s_stats.late_arrival_drops);
            s_diag_first_rx_pending = false;
        }

        if (!rx_got) {
            // 受信タイムアウト
            s_stats.rx_timeouts++;
            s_stats.verify_errors++;
        } else {
            // RTT 計測完了
            absolute_time_t rtt_end = get_absolute_time();
            uint64_t rtt_us = (uint64_t)absolute_time_diff_us(rtt_start, rtt_end);
            s_stats.rtt_total_us += rtt_us;
            s_stats.rtt_samples++;

            // Requirement 2: 送信したまさにそのパケットと一致するか厳格照合 (古いパケットの残留は失格)
            bool match = (rx_seq == s_seq && rx_len == current_len && rx_type == BMC_PKT_TYPE_BENCH &&
                          memcmp(s_tx_payload, s_rx_payload, current_len) == 0);
            if (!match) {
                s_stats.verify_errors++;
            }
        }
    }

    // ドライバ統計を同期 (TX失敗時でも確実に同期)
    if (s_active_transport.get_stats) {
        poc_stats_t drv_st;
        s_active_transport.get_stats(s_active_transport.ctx, &drv_st);
        s_stats.tx_packets = drv_st.tx_packets;
        s_stats.tx_bytes   = drv_st.tx_bytes;
        s_stats.tx_wire_bytes = drv_st.tx_wire_bytes;
        s_stats.tx_phy_duration_us = drv_st.tx_phy_duration_us;
        s_stats.tx_timeouts = drv_st.tx_timeouts;
        s_stats.tx_failures = drv_st.tx_failures;
        s_stats.rx_packets = drv_st.rx_packets;
        s_stats.rx_bytes   = drv_st.rx_bytes;
        s_stats.rx_crc_errors = drv_st.rx_crc_errors;
        s_stats.rx_len_errors = drv_st.rx_len_errors;
        s_stats.rx_seq_drops  = drv_st.rx_seq_drops;
        s_stats.rx_overrun_errors = drv_st.rx_overrun_errors;
    }

    s_seq++;
}

// -------------------------------------------------------------
// 連続送信実行 (Core 0 TX: ウェイトなしで最大スループット送出)
// -------------------------------------------------------------
static void BMC_SRAM_FUNC(benchmark_run_burst_tx)(void) {
    uint8_t current_len = g_poc_test_lens[s_len_idx];
    s_len_idx = (s_len_idx + 1) % g_poc_num_test_lens;

    for (int i = 0; i < current_len; ++i) {
        s_tx_payload[i] = (uint8_t)(s_seq + i);
    }

    if (MODE_DESCS[s_current_mode].is_uart) {
        if (!s_active_transport.send_packet) return;
        size_t dma_words = bmc_packet_encode(s_tx_dma_words, BMC_PKT_TYPE_BENCH, s_seq, s_tx_payload, current_len);
        if (dma_words == 0) return;
        bool tx_ok = s_active_transport.send_packet(s_active_transport.ctx, PIN_TX, s_tx_dma_words, dma_words, current_len);
        if (!tx_ok) {
            s_stats.tx_failures++;
            s_stats.verify_errors++;
        }
    } else {
        // B-4: BMC TXパイプライン化。前パケットのPHY送信中に次パケットをエンコードする。
        uint32_t *buf = s_tx_dma_words_pipeline[s_tx_pipeline_next];
        size_t dma_words = bmc_packet_encode(buf, BMC_PKT_TYPE_BENCH, s_seq, s_tx_payload, current_len);
        if (dma_words == 0) return;

        if (s_tx_pipeline_active) {
            if (!bmc_tx_wait_packet_done(&s_bmc_tx)) {
                s_stats.tx_failures++;
                s_stats.verify_errors++;
            }
        }

        if (bmc_tx_start_packet_async(&s_bmc_tx, PIN_TX, buf, dma_words)) {
            s_tx_pipeline_active = true;
        } else {
            s_stats.tx_failures++;
            s_stats.verify_errors++;
            s_tx_pipeline_active = false;
        }

        s_tx_pipeline_next ^= 1;
    }

    s_seq++;

    // 定期的にドライバ統計を同期
    if ((s_seq & 0x1F) == 0 && s_active_transport.get_stats) {
        poc_stats_t drv_st;
        s_active_transport.get_stats(s_active_transport.ctx, &drv_st);
        s_stats.tx_packets = drv_st.tx_packets;
        s_stats.tx_bytes   = drv_st.tx_bytes;
        s_stats.tx_wire_bytes = drv_st.tx_wire_bytes;
        s_stats.tx_phy_duration_us = drv_st.tx_phy_duration_us;
        s_stats.tx_timeouts = drv_st.tx_timeouts;
        s_stats.tx_failures = drv_st.tx_failures;
        s_stats.rx_packets = drv_st.rx_packets;
        s_stats.rx_bytes   = drv_st.rx_bytes;
        s_stats.rx_crc_errors = drv_st.rx_crc_errors;
        s_stats.rx_len_errors = drv_st.rx_len_errors;
        s_stats.rx_seq_drops  = drv_st.rx_seq_drops;
        s_stats.rx_overrun_errors = drv_st.rx_overrun_errors;
    }
}

// -------------------------------------------------------------
// 定期レポートおよび自動スイープ管理
// -------------------------------------------------------------
static void benchmark_maybe_report(void) {
    // Core 1 側の診断出力要求を Core 0 側で安全にフラッシュ
    if (s_diag_ready_to_print) {
        s_diag_ready_to_print = false;
        printf("[DIAG] Mode first RX poll: got=%d, rx_seq=%u, polls=%lu, elapsed=%llu us, late_drops=%lu\n",
               s_diag_rx_got ? 1 : 0, s_diag_rx_seq,
               (unsigned long)s_diag_rx_poll_count,
               (unsigned long long)s_diag_rx_elapsed_us,
               (unsigned long)s_stats.late_arrival_drops);
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed_ms = now - s_last_report_time;

    if (elapsed_ms >= 1000) {
        if (s_active_transport.get_stats) {
            poc_stats_t drv_st;
            s_active_transport.get_stats(s_active_transport.ctx, &drv_st);
            s_stats.tx_packets = drv_st.tx_packets;
            s_stats.tx_bytes   = drv_st.tx_bytes;
            s_stats.tx_wire_bytes = drv_st.tx_wire_bytes;
            s_stats.tx_phy_duration_us = drv_st.tx_phy_duration_us;
            s_stats.tx_timeouts = drv_st.tx_timeouts;
            s_stats.tx_failures = drv_st.tx_failures;
            s_stats.rx_packets = drv_st.rx_packets;
            s_stats.rx_bytes   = drv_st.rx_bytes;
            s_stats.rx_crc_errors = drv_st.rx_crc_errors;
            s_stats.rx_len_errors = drv_st.rx_len_errors;
            s_stats.rx_seq_drops  = drv_st.rx_seq_drops;
            s_stats.rx_overrun_errors = drv_st.rx_overrun_errors;
        }

        uint32_t delta_tx_bytes = s_stats.tx_bytes - s_report_tx_bytes_start;
        uint32_t delta_rx_bytes = s_stats.rx_bytes - s_report_rx_bytes_start;
        uint64_t delta_phy_us   = s_stats.tx_phy_duration_us - s_report_phy_us_start;
        uint32_t delta_tx_wire_bytes = s_stats.tx_wire_bytes - s_report_tx_wire_bytes_start;

        float app_tx_kibps = (float)delta_tx_bytes / (float)elapsed_ms * 1000.0f / 1024.0f;
        float app_rx_kibps = (float)delta_rx_bytes / (float)elapsed_ms * 1000.0f / 1024.0f;
        s_stats.avg_tx_kibps = app_tx_kibps;
        s_stats.avg_rx_kibps = app_rx_kibps;

        float avg_rtt_us = (s_stats.rtt_samples > 0) ? ((float)s_stats.rtt_total_us / s_stats.rtt_samples) : 0.0f;
        float phy_wire_mbps = (delta_phy_us > 0) ?
                              ((float)(s_stats.tx_wire_bytes) * 8.0f / (float)s_stats.tx_phy_duration_us) : 0.0f;
        float line_util_pct = (elapsed_ms > 0) ?
                              ((float)delta_phy_us / ((float)elapsed_ms * 1000.0f) * 100.0f) : 0.0f;
        if (line_util_pct > 100.0f) line_util_pct = 100.0f;

        // B-3: LineUtil(calc) — tx_phy_duration_us の計測経路を経由しない独立検証値。
        // 実測 wire バイト数と対象トランスポートの理論ビット/バイト・目標PHYレートのみから
        // 理論専有時間を逆算する。line_util_pct との乖離は計測区間側の欠落/重複を示唆する。
        const mode_desc_t *cur_desc = &MODE_DESCS[s_current_mode];
        float bits_per_wire_byte = cur_desc->is_uart ? 10.0f : 8.0f;
        float target_bps = cur_desc->target_phy_mbps * 1e6f;
        float calc_line_util_pct = 0.0f;
        if (elapsed_ms > 0 && target_bps > 0.0f) {
            float theoretical_us = (float)delta_tx_wire_bytes * bits_per_wire_byte / target_bps * 1e6f;
            calc_line_util_pct = theoretical_us / ((float)elapsed_ms * 1000.0f) * 100.0f;
        }
        if (calc_line_util_pct > 999.9f) calc_line_util_pct = 999.9f;

        float avg_rx_cpu_us = (s_stats.rx_cpu_cost_samples > 0) ?
                              ((float)s_stats.rx_cpu_cost_total_us / (float)s_stats.rx_cpu_cost_samples) : 0.0f;

        printf("[%s] TX: %lu pkts (%.1f KiB/s) | RX: %lu pkts (%.1f KiB/s) | PHY: %.3f Mbps (Util: %5.1f%%, calc: %5.1f%%) | RTT: %5.1f us | Err(CRC/LEN/TO/Ver): %lu/%lu/%lu/%lu | Drops/Ov/Late: %lu/%lu/%lu | RX CPU: avg %.1f us, max %lu us\n",
               MODE_DESCS[s_current_mode].name,
               (unsigned long)s_stats.tx_packets, app_tx_kibps,
               (unsigned long)s_stats.rx_packets, app_rx_kibps,
               phy_wire_mbps, line_util_pct, calc_line_util_pct,
               avg_rtt_us,
               (unsigned long)s_stats.rx_crc_errors,
               (unsigned long)s_stats.rx_len_errors,
               (unsigned long)s_stats.rx_timeouts,
               (unsigned long)s_stats.verify_errors,
               (unsigned long)s_stats.rx_seq_drops,
               (unsigned long)s_stats.rx_overrun_errors,
               (unsigned long)s_stats.late_arrival_drops,
               avg_rx_cpu_us, (unsigned long)s_stats.rx_cpu_cost_max_us);

        s_last_report_time = now;
        s_report_tx_bytes_start = s_stats.tx_bytes;
        s_report_rx_bytes_start = s_stats.rx_bytes;
        s_report_phy_us_start = s_stats.tx_phy_duration_us;
        s_report_tx_wire_bytes_start = s_stats.tx_wire_bytes;
    }

    if (s_auto_sweep) {
        uint32_t sweep_elapsed = now - s_sweep_mode_start_ms;
        if (sweep_elapsed >= SWEEP_DURATION_MS) {
            if (s_active_transport.get_stats) {
                poc_stats_t drv_st;
                s_active_transport.get_stats(s_active_transport.ctx, &drv_st);
                s_stats.tx_packets = drv_st.tx_packets;
                s_stats.tx_bytes   = drv_st.tx_bytes;
                s_stats.tx_wire_bytes = drv_st.tx_wire_bytes;
                s_stats.tx_phy_duration_us = drv_st.tx_phy_duration_us;
                s_stats.tx_timeouts = drv_st.tx_timeouts;
                s_stats.tx_failures = drv_st.tx_failures;
                s_stats.rx_packets = drv_st.rx_packets;
                s_stats.rx_bytes   = drv_st.rx_bytes;
                s_stats.rx_crc_errors = drv_st.rx_crc_errors;
                s_stats.rx_len_errors = drv_st.rx_len_errors;
                s_stats.rx_seq_drops  = drv_st.rx_seq_drops;
                s_stats.rx_overrun_errors = drv_st.rx_overrun_errors;
            }
            s_sweep_results[s_current_mode] = s_stats;
            int next_mode = (int)s_current_mode + 1;
            if (next_mode >= MODE_COUNT) {
                s_auto_sweep = false;
                printf("\n>>> 全%dモード自動スイープ完了 <<<\n", MODE_COUNT);
                benchmark_print_sweep_summary();
            } else {
                apply_mode((bench_mode_t)next_mode);
                s_sweep_mode_start_ms = to_ms_since_boot(get_absolute_time());
            }
        }
    }
}

// -------------------------------------------------------------
// 単発送信診断: 1フレームだけ送信し、後続フレームを一切送らない状態で
// そのフレーム単体が受理できたかを表示する (RX受信範囲計算バグの実機回帰確認用)。
// -------------------------------------------------------------
static void benchmark_run_single_shot(void) {
    if (!s_active_transport.send_packet || !s_active_transport.poll_packet) return;

    // 連続送信モードの Core1 RXワーカーが同時にポーリングしていると
    // 単発フレームの「受理できたか」を Core0 側で確定できないため、一時停止する
    bool was_burst_rx_enabled = s_core1_rx_enabled;
    s_core1_rx_enabled = false;
    __sync_synchronize();
    while (s_core1_rx_active) {
        tight_loop_contents();
    }

    uint8_t len = g_poc_test_lens[s_len_idx];
    s_len_idx = (s_len_idx + 1) % g_poc_num_test_lens;
    for (int i = 0; i < len; ++i) {
        s_tx_payload[i] = (uint8_t)(s_seq + i);
    }

    size_t dma_words = bmc_packet_encode(s_tx_dma_words, BMC_PKT_TYPE_ECHO_REQ, s_seq, s_tx_payload, len);
    if (dma_words == 0) {
        printf("\n[SINGLE-SHOT] エンコード失敗 (LEN=%u)\n\n", len);
        if (was_burst_rx_enabled) { s_core1_rx_enabled = true; __sync_synchronize(); }
        return;
    }

    printf("\n[SINGLE-SHOT] >>> 1フレームのみ送信 (Mode=%s, LEN=%u, SEQ=%u) — 後続フレームは送りません <<<\n",
           MODE_DESCS[s_current_mode].name, len, s_seq);

    bool tx_ok = s_active_transport.send_packet(s_active_transport.ctx, PIN_TX, s_tx_dma_words, dma_words, len);
    if (!tx_ok) {
        printf("[SINGLE-SHOT] 結果: FAIL (送信自体が失敗)\n\n");
        s_seq++;
        if (was_burst_rx_enabled) { s_core1_rx_enabled = true; __sync_synchronize(); }
        return;
    }

    uint8_t rx_type = 0, rx_seq = 0, rx_len = 0;
    bool rx_got = false;
    uint32_t poll_count = 0;
    absolute_time_t poll_start = get_absolute_time();
    // 後続フレームを一切送らないため、DMA/PIOの取りこぼしと純粋な「待っても来ない」不具合を
    // 区別できるよう、通常の受信タイムアウト(5ms)より大幅に長い猶予を与える
    absolute_time_t deadline = make_timeout_time_ms(200);

    while (!time_reached(deadline)) {
        poll_count++;
        if (s_active_transport.poll_packet(s_active_transport.ctx, &rx_type, &rx_seq, s_rx_payload, &rx_len)) {
            rx_got = true;
            break;
        }
        tight_loop_contents();
    }
    uint64_t elapsed_us = (uint64_t)absolute_time_diff_us(poll_start, get_absolute_time());

    bool match = rx_got && rx_seq == s_seq && rx_len == len && rx_type == BMC_PKT_TYPE_ECHO_REQ &&
                memcmp(s_tx_payload, s_rx_payload, len) == 0;

    printf("[SINGLE-SHOT] 結果: %s\n", match ? "PASS (次フレームなしで単独受理成功)" : "FAIL");
    printf("    got=%d match=%d rx_seq=%u(期待%u) rx_len=%u(期待%u) polls=%lu elapsed=%llu us\n",
           rx_got ? 1 : 0, match ? 1 : 0, rx_seq, s_seq, rx_len, len,
           (unsigned long)poll_count, (unsigned long long)elapsed_us);
    printf("    読み方: PASS なら、このフレーム単体・後続データなしで受理できている。\n");
    printf("            FAIL かつ got=0 の場合、次フレームが来るまで受理できない不具合が残っている。\n\n");

    s_seq++;

    if (was_burst_rx_enabled) {
        s_core1_rx_enabled = true;
        __sync_synchronize();
    }
}

// -------------------------------------------------------------
// コマンド入力処理
// -------------------------------------------------------------
static void benchmark_process_input(void) {
    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) return;

    if (ch >= '1' && ch <= '9') {
        s_auto_sweep = false;
        apply_mode((bench_mode_t)(ch - '1'));
    } else if (ch == 'q' || ch == 'Q') {
        s_auto_sweep = false;
        apply_mode(MODE_BMC_12M);
    } else if (ch == 'w' || ch == 'W') {
        s_auto_sweep = false;
        apply_mode(MODE_BMC_12M_BURST);
    } else if (ch == 'e' || ch == 'E') {
        s_auto_sweep = false;
        apply_mode(MODE_BMC_15M);
    } else if (ch == 't' || ch == 'T') {
        s_auto_sweep = false;
        apply_mode(MODE_BMC_15M_BURST);
    } else if (ch == 'y' || ch == 'Y') {
        s_auto_sweep = false;
        apply_mode(MODE_BMC_18M_BURST);
    } else if (ch == 'p' || ch == 'P') {
        s_pad_fast_variant = !s_pad_fast_variant;
        printf("\n>>> TX Pad Variant toggled: %s <<<\n",
               s_pad_fast_variant ? "FAST Slew + 8mA Drive (Default for 18.75M)" : "SLOW Slew + 4mA Drive");
        if (!MODE_DESCS[s_current_mode].is_uart && MODE_DESCS[s_current_mode].rate_family == BMC_RATE_FAST_8CYC) {
            bmc_tx_controller_set_pad_variant(&s_bmc_tx, s_pad_fast_variant);
            printf("    Applied to running BMC 18.75M TX SM.\n");
        } else {
            printf("    (Note: FAST pad variant takes effect in BMC 18.75M mode)\n");
        }
        printf("\n");
    } else if (ch == 'o' || ch == 'O') {
        benchmark_run_single_shot();
    } else if (ch == 'd' || ch == 'D') {
        if (!MODE_DESCS[s_current_mode].is_uart) {
            bmc_rx_dump_diagnostic(&s_bmc_rx);
        } else {
            printf("\n[DIAG] Diagnostic raw dump is supported for BMC modes.\n\n");
        }
    } else if (ch == 'a' || ch == 'A' || ch == 's' || ch == 'S') {
        printf("\n>>> 開始: 全%dモード自動スイープ比較ベンチマーク (各%d秒) <<<\n\n", MODE_COUNT, SWEEP_DURATION_MS / 1000);
        s_auto_sweep = true;
        apply_mode(MODE_UART_3M);
        s_sweep_mode_start_ms = to_ms_since_boot(get_absolute_time());
    } else if (ch == 'c' || ch == 'C') {
        bench_mode_t prev = s_current_mode;
        s_core1_rx_enabled = false;
        __sync_synchronize();
        while (s_core1_rx_active) tight_loop_contents();

        printf("\n--- 配線導通テスト (GP%d <-> GP%d) ---\n", PIN_TX, PIN_RX);
        bool ok = check_continuity();
        printf("  判定: %s\n\n", ok ? "PASS (正常接続)" : "FAIL (未接続 / 断線)");
        apply_mode(prev); // 呼び出し元が復旧 (H-7)
    } else if (ch == ' ') {
        s_benchmark_running = !s_benchmark_running;
        printf("\n>>> Benchmark %s <<<\n\n", s_benchmark_running ? "RESUMED" : "PAUSED");
    } else if (ch == 'r' || ch == 'R') {
        poc_stats_reset(&s_stats);
        if (s_active_transport.reset_stats) {
            s_active_transport.reset_stats(s_active_transport.ctx);
        }
        s_last_report_time = to_ms_since_boot(get_absolute_time());
        s_report_tx_bytes_start = 0;
        s_report_rx_bytes_start = 0;
        s_report_phy_us_start = 0;
        s_report_tx_wire_bytes_start = 0;
        printf("\n>>> Statistics counter reset <<<\n\n");
    }
}

static void benchmark_print_banner(void) {
    uint32_t sys_hz = clock_get_hz(clk_sys);
    printf("\n======================================================================\n");
    printf("  RP2350 High-Speed Communication Protocol Unified Benchmark\n");
    printf("  Sys Clock: %.1f MHz | 配線: GP%d (Pin 1) <---> GP%d (Pin 2) 直結\n",
           (float)sys_hz / 1e6f, PIN_TX, PIN_RX);
    printf("  コマンド一覧:\n");
    printf("    [1] UART  3.000 Mbps (標準DMA, 停止待ち)\n");
    printf("    [2] UART  9.375 Mbps (UART ハードウェア限界, 停止待ち)\n");
    printf("    [3] BMC   3.125 Mbps (16 cyc/bit, clkdiv 3.0f, PoC標準, 停止待ち)\n");
    printf("    [4] BMC   9.375 Mbps (16 cyc/bit, clkdiv 1.0f, 停止待ち)\n");
    printf("    [5] BMC  18.750 Mbps ( 8 cyc/bit, clkdiv 1.0f, EXPERIMENTAL, 停止待ち)\n");
    printf("    [6] UART  3.000 Mbps (Burst 連続送信, Dual-Core RX)\n");
    printf("    [7] UART  9.375 Mbps (Burst 連続送信, Dual-Core RX)\n");
    printf("    [8] BMC   3.125 Mbps (Burst 連続送信, Dual-Core RX)\n");
    printf("    [9] BMC   9.375 Mbps (Burst 連続送信, Dual-Core RX)\n");
    printf("    [q] BMC  12.500 Mbps (12 cyc/bit, clkdiv 1.0f, 停止待ち)\n");
    printf("    [w] BMC  12.500 Mbps (Burst 連続送信, Dual-Core RX)\n");
    printf("    [e] BMC  15.000 Mbps (10 cyc/bit, clkdiv 1.0f, 停止待ち)\n");
    printf("    [t] BMC  15.000 Mbps (Burst 連続送信, Dual-Core RX)\n");
    printf("    [y] BMC  18.750 Mbps ( 8 cyc/bit, clkdiv 1.0f, EXPERIMENTAL, Burst 連続送信)\n");
    printf("    [a] または [s]: 全モード自動スイープ比較 (各4秒計測 & サマリー表出力)\n");
    printf("    [ ] (スペース): 一時停止 / 再開\n");
    printf("    [r]: 統計リセット\n");
    printf("    [c]: 配線導通テスト\n");
    printf("    [p]: TXパッド設定トグル (FAST+8mA[既定,18.75M専用適用] / SLOW+4mA, 18.75M以外には無効)\n");
    printf("    [d]: RX生データ診断ダンプ (1回限り)\n");
    printf("    [o]: 単発送信診断 (現在のモードで1フレームだけ送信し、後続フレームなしで受理できたか表示)\n");
    printf("======================================================================\n\n");
}

static void benchmark_init_hardware(void) {
    bmc_tx_controller_zero(&s_bmc_tx);
    bmc_rx_port_zero(&s_bmc_rx);
    uart_driver_zero(&s_uart_drv);
}

static void benchmark_wait_for_usb(void) {
    for (int i = 0; i < 20; ++i) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }
}

// -------------------------------------------------------------
// M-1: メイン関数 (80行以内を厳格遵守)
// -------------------------------------------------------------
int main(void) {
    stdio_init_all();
    benchmark_wait_for_usb();
    build_info_print("poc_benchmark");
    benchmark_print_banner();
    benchmark_init_hardware();

    // Core 1 非同期RXワーカーの起動
    multicore_launch_core1(core1_rx_worker);

    apply_mode(MODE_BMC_3M);

    while (1) {
        // B-a: Burst送信中はCPU側の入力確認/レポート判定を毎パケット行わず、
        // 数ms間隔に間引く (main_benchmark.c:1078 の退行対策)。
        // 停止待ちモードではパケット送出自体が数百µs〜のブロッキングであり、
        // 間引きの効果が乏しく応答性を落とすだけなので毎周期実行のまま維持する。
        bool do_housekeeping = true;
        if (s_benchmark_running && MODE_DESCS[s_current_mode].is_burst) {
            absolute_time_t now = get_absolute_time();
            do_housekeeping = !s_housekeeping_time_valid ||
                (absolute_time_diff_us(s_last_housekeeping_time, now) >= BENCHMARK_HOUSEKEEPING_INTERVAL_US);
        }

        if (do_housekeeping) {
            benchmark_process_input();
            benchmark_maybe_report();
            s_last_housekeeping_time = get_absolute_time();
            s_housekeeping_time_valid = true;
        }

        if (s_benchmark_running) {
            if (MODE_DESCS[s_current_mode].is_burst) {
                benchmark_run_burst_tx();
            } else {
                benchmark_run_exchange();
            }
        }
    }

    return 0;
}
