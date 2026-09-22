#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "uart_driver.h"
#include "common/benchmark_common.h"
#include "common/build_info.h"

#define UART_ID uart0
#define BAUD_RATE 3000000u
#define UART_TX_PIN 0u
#define UART_RX_PIN 1u

typedef struct {
    uart_driver_t driver;
    uint32_t tx_words[BMC_TX_DMA_MAX_WORDS];
    uint8_t tx_payload[BMC_MAX_PAYLOAD_LEN];
    uint8_t rx_payload[BMC_MAX_PAYLOAD_LEN];
    uint8_t seq;
    size_t len_idx;
    uint32_t verify_errors;
    uint32_t last_report_ms;
    uint32_t report_tx_start;
    uint32_t report_rx_start;
} uart_loopback_state_t;

static void wait_for_usb(void) {
    for (int i = 0; i < 20 && !stdio_usb_connected(); ++i) sleep_ms(100);
}

static bool init_loopback(uart_loopback_state_t *s) {
    memset(s, 0, sizeof(*s));
    uart_driver_zero(&s->driver);
    if (!uart_driver_init(&s->driver, UART_ID, UART_TX_PIN, UART_RX_PIN, BAUD_RATE)) return false;
    s->last_report_ms = to_ms_since_boot(get_absolute_time());
    return true;
}

static void run_exchange(uart_loopback_state_t *s) {
    uint8_t len = g_poc_test_lens[s->len_idx];
    s->len_idx = (s->len_idx + 1) % g_poc_num_test_lens;
    for (uint8_t i = 0; i < len; ++i) s->tx_payload[i] = (uint8_t)(s->seq + i);
    size_t words = bmc_packet_encode(s->tx_words, BMC_PKT_TYPE_BENCH, s->seq, s->tx_payload, len);
    const uint8_t *wire = (const uint8_t *)&s->tx_words[1];
    bool tx_ok = words > 0 && uart_tx_send_packet_blocking(
        &s->driver, wire + BMC_PREAMBLE_SYNC_BYTES, bmc_packet_total_bytes(len), len);
    if (!tx_ok) { s->verify_errors++; s->seq++; return; }

    uint8_t rx_type = 0, rx_seq = 0, rx_len = 0;
    bool received = false;
    absolute_time_t deadline = make_timeout_time_ms(5);
    while (!time_reached(deadline)) {
        if (uart_rx_poll_packet(&s->driver, &rx_type, &rx_seq, s->rx_payload, &rx_len)) {
            received = true;
            break;
        }
        tight_loop_contents();
    }
    if (!received || rx_type != BMC_PKT_TYPE_BENCH || rx_seq != s->seq ||
        rx_len != len || memcmp(s->tx_payload, s->rx_payload, len) != 0) s->verify_errors++;
    s->seq++;
}

static void maybe_report(uart_loopback_state_t *s) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = now - s->last_report_ms;
    if (elapsed < 1000) return;
    poc_stats_t st;
    uart_get_stats(&s->driver, &st);
    float tx_rate = (float)(st.tx_bytes - s->report_tx_start) * 1000.0f / elapsed / 1024.0f;
    float rx_rate = (float)(st.rx_bytes - s->report_rx_start) * 1000.0f / elapsed / 1024.0f;
    printf("[UART] TX:%lu %.1f KiB/s RX:%lu %.1f KiB/s CRC:%lu LEN:%lu Drops:%lu Verify:%lu\n",
           (unsigned long)st.tx_packets, tx_rate, (unsigned long)st.rx_packets, rx_rate,
           (unsigned long)st.rx_crc_errors, (unsigned long)st.rx_len_errors,
           (unsigned long)st.rx_seq_drops, (unsigned long)s->verify_errors);
    s->last_report_ms = now;
    s->report_tx_start = st.tx_bytes;
    s->report_rx_start = st.rx_bytes;
}

int main(void) {
    static uart_loopback_state_t state;
    stdio_init_all();
    wait_for_usb();
    build_info_print("poc_uart_loopback");
    printf("RP2350 UART DMA loopback: GP%u TX <-> GP%u RX @ %u bps\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
    if (!init_loopback(&state)) { printf("[ERROR] UART initialization failed\n"); return 1; }
    while (true) { run_exchange(&state); maybe_report(&state); sleep_us(100); }
}
