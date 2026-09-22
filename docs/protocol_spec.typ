#set page(paper: "a4", margin: (x: 1.8cm, y: 2.0cm))
#set text(font: ("Hiragino Sans", "Hiragino Kaku Gothic ProN"), size: 9.5pt)
#set par(justify: true, leading: 0.65em)
#set heading(numbering: "1.1")

#align(center)[
  #text(size: 16pt, weight: "bold")[RP2350 P2P 差動マンチェスタ（BMC）シリアル通信 試作仕様・設計書] \
  #v(2mm)
  #text(size: 10pt, fill: rgb(80, 80, 80))[完全整数分周 3.125 Mbps / 抵抗レス直結 / TX pio0・RX pio1 分離 / 実験的 PoC 設計]
]

#v(4mm)

= システム諸元・概要

*Raspberry Pi Pico 2（RP2350）* 同士を外付け抵抗なしのジャンパ線直結（50cm以内）で接続し、差動マンチェスタ符号（BMC）による低遅延なP2P通信を行う。現行実装では受信（RX）を `pio1`（SM0）で常時リスニング専従とし、送信（TX）を `pio0`（SM0）に配置する。なお、単一ブロック内での動的TX MUX共有による多ポート展開は将来構想（仮説的トポロジ案）である。

#table(
  columns: (1fr, 1.2fr, 2fr),
  inset: 5pt,
  align: (left, left, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*項目*], [*諸元値*], [*備考・分類*],
  [伝送路], [TX, RX, GND (計3線直結)], [50cm ジャンパ線, 外付け抵抗完全排除],
  [変調方式], [Biphase Mark Code (BMC)], [クロック埋め込み自己同期, 極性自動追従],
  [ビットレート], [3.125 Mbps], [公称レート (計算値: $150 "MHz" / (3.0 times 16)$)],
  [システムクロック], [150.0 MHz], [標準動作周波数],
  [PIO分周比], [完全整数分周 `clkdiv = 3.0f`], [PIOクロック 50.0 MHz (1サイクル 20.0 ns, ジッターゼロ)],
  [1ビット幅 ($T_"bit"$)], [非均一: 通常 320.0 ns (16 cyc) / 境界 360.0 ns (18 cyc)], [【ソース実装事実】通常16サイクル、32bitワード境界（OSR補充）のみ18サイクル（+2サイクル伸長）。RXはエッジ同期で吸収],
  [サンプリング点],
  [境界から 240.0 ns (12 cycles)],
  [$0.75 times T_"bit"$ 位置判定 (定常マージン $plus.minus$80 ns, SOP時 -60/+100 ns, 机上計算モデル)],
  [PIOリソース消費], [TX: 14 / 32 words (`pio0`), RX: 17 / 32 words (`pio1`)], [【ソース実装事実】TX/RXを独立PIOブロックに分離。TX空き 18 words, RX空き 15 words],
  [実機計測スループット], [157.4 kB/s (15,251 pkts, CRC Err 0, Drops 0)], [【ユーザー報告実測値】Pico 2 W / RP2350 150MHz, GP0<->GP1直結 (旧シングルレートBMC)。※リファクタリング後ファームウェアは実機検証未実施],
)

= 物理層（PHY）仕様

== 電気的特性およびPAD設定（抵抗レス対策）
外付けダンピング抵抗を排除するため、RP2350の内部GPIO PADレジスタによりスルーレートと駆動電流を制限し、50cm配線上の反射（リンギング）の立ち上がり勾配を緩和する。
（※注意: 旧版では「リンギングを物理的に抑制し誤エッジゼロ」と記されていたが、任意の配線長・浮遊容量・環境下で「誤エッジゼロ」が成立することは実機で確定された物理的事実ではなく、理論的期待に基づく未検証の前提［Unverified assumption］である）。

- *TXピン:* 出力モード / Slew Rate: `GPIO_SLEW_RATE_SLOW` / Drive Strength: `GPIO_DRIVE_STRENGTH_4MA`
- *RXピン:* 入力モード / `PULL_DOWN` 有効（内部プルダウン） / Schmitt Trigger 有効

== 物理ライン状態定義

#table(
  columns: (1.2fr, 1.8fr, 2fr),
  inset: 5pt,
  align: (left, left, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*状態名*], [*電圧・信号特性*], [*システム動作・条件*],
  [未接続 / 切断], [0.0 V (Low固定)], [対向ノード未接続 (RXプルダウンによるLink Down)],
  [接続待機 (Idle)], [3.3 V (High固定)], [TX側がHigh能動ドライブ (Link Up即時送信可能)],
  [パケット伝送 (Active)], [1.56M〜3.125 MHz], [BMC変調パルス (160ns/320ns反転, 正常デコード)],
  [パケット開始 (SOP)], [3.3V $arrow$ 0.0V], [待機Highからの最初の立下りエッジで同期待機解除],
  [パケット終了 (EOP)], [3.3V High維持着地], [極性パディングによりHighで終了、自律High復帰],
  [相手異常 (Fault)], [0.0 V (Low固定)], [エッジ途絶かつLowが100ms継続でLink Down判定],
)

```text
電圧
3.3V                     +-----------------------+   +---+   +---+   +-----------------------+
                         |                       |   |   |   |   |   |                       |
0.0V --------------------+                       +---+   +---+   +---+                       +---
時間 [ 未接続: GND固定 ]   [ 接続待機: High固定 ]    |<─── パケット伝送 ───>|   [ 自律High復帰: 待機 ]   [ 切断 ]
                         ↑                       ↑                   ↑
                     Link Up (安定High)        SOP (立下り)       EOP (IRQ0 / High固定)
```

= PIOプログラム仕様 (`bmc_p2p.pio`)

本実装では、TX を `pio0`（14命令/32ワード、空き 18 ワード）、RX を `pio1`（17命令/32ワード、空き 15 ワード）へと独立ブロックに分離して配置する。

また、TXのビットセル幅は【非均一ビットセル（Non-uniform bit cells）】である。ワード内（OSR内の各ビット）は公称 16 サイクル（前半 8 cyc / 後半 8 cyc）で動作するが、32ビットごとのワード境界において次ワードの `pull block` および `jmp y--, word_loop` の実行により、ワード最終ビットセルのみ 18 サイクル（+2 サイクル伸長）となる。RX 側は各ビットの境界エッジで自律同期するため、この 2 サイクルの伸長は問題なく吸収される。

```pasm
; -----------------------------------------------------------------------------
; RP2350 Differential Manchester (BMC) Transceiver
; 3.125 Mbps @ 150MHz sysclk (clkdiv = 3.0f -> 50MHz PIO Clock, 20ns/cycle)
; Bit width: Non-uniform (Normal cells: 16 cycles, Word-boundary cells: 18 cycles)
; PIO Resource: TX pio0 14/32 words (18 free), RX pio1 17/32 words (15 free)
; -----------------------------------------------------------------------------

.program bmc_tx
; --- TX: ワードカウント式・完全パケット送信器 (14命令) ---
; 先頭ワードとして (送信総ワード数 - 1) を与えることで、
; 途中ワード境界での誤IRQ・High復帰を排除し、パケット送出完了時のみ自律High化・IRQ通知を行う。
.wrap_target
idle:
    set pins, 1             ; 1: 待機時は確実にHigh (3.3V) を能動出力
    pull block              ; 2: パケットヘッダ長ワード取得 (Y = 総データワード数 - 1)
    mov y, osr              ; 3: レジスタYへワードカウンタを格納

word_loop:
    pull block              ; 4: 送信データワード取得 (32bit)

bit_loop:
    mov pins, !pins         ; 5: 境界エッジ反転 (自ピン読み戻し反転) [1cyc]
    out x, 1                ; 6: OSRから1ビット取得 [1cyc]
    jmp !x, is_zero     [5] ; 7: 0なら前半中央反転スキップ (ディレイ5 -> 前半計8cyc)

    ; --- ビット1: 中央反転 (8cyc境界で厳密反転) ---
    mov pins, !pins     [5] ; 8: 中央反転 (ディレイ5 -> 14cyc目まで)
    jmp check_bit           ; 9: 15cyc目

is_zero:
    ; --- ビット0: レベル維持 ---
    nop                 [6] ; 10: 前半レベル維持 (ディレイ6 -> 15cyc目まで)

check_bit:
    jmp !osre, bit_loop     ; 11: OSRに残りがあれば次ビットへ [1cyc] (計16cyc均等)
    jmp y--, word_loop      ; 12: パケット内に次ワードがあれば取得へ

    ; --- 全パケット送出完了 (EOP) ---
    set pins, 1             ; 13: 最終ビット完了: 即座にHigh能動出力
    irq set 0               ; 14: パケット物理送出の完全終了をCPU/DMAへ通知
.wrap


.program bmc_rx
; --- RX: SOP立下り起動・16cyc対応BMC復号器 (17命令) ---
; Autopush: 有効 (32bit)
; 定常サンプリング: 境界から12cyc (jmp 1 + nop[9] 10 + jmp pin 1 = 240ns, マージン±80ns)
; SOP初回サンプリング: 境界から11cyc (nop[9] 10 + jmp pin 1 = 220ns, マージン-60ns/+100ns)
.wrap_target
wait_sop:
    wait 0 pin 0        ; 1: アイドル(High)からの立下りエッジ(SOP)待機

was_low:
    nop             [9] ; 2: 10サイクルディレイ (定常時:境界から11サイクル目到達)
    jmp pin, got_1_from_low ; 3: 中央反転してHighならビット1 (12サイクル目でサンプリング)
got_0_from_low:
    in null, 1          ; 4: ビット0 (ISRへ0シフト)
    wait 1 pin 0        ; 5: 次の境界エッジ (立上がり) 待機
    jmp was_high        ; 6: 次状態へ (1サイクル消費)

got_1_from_low:
    in x, 1             ; 7: ビット1 (ISRへ1シフト)
    wait 0 pin 0        ; 8: 次の境界エッジ (立下り) 待機
    jmp was_low         ; 9: 次状態へ (1サイクル消費)

was_high:
    nop             [9] ; 10: 10サイクルディレイ (定常時:境界から11サイクル目到達)
    jmp pin, got_0_from_high ; 11: High維持ならビット0 (12サイクル目でサンプリング)
got_1_from_high:
    in x, 1             ; 12: ビット1 (ISRへ1シフト)
    wait 1 pin 0        ; 13: 次の境界エッジ (立上がり) 待機
    jmp was_high        ; 14: 次状態へ (1サイクル消費)

got_0_from_high:
    in null, 1          ; 15: ビット0 (ISRへ0シフト)
    wait 0 pin 0        ; 16: 次の境界エッジ (立下り) 待機
    jmp was_low         ; 17: 次状態へ (1サイクル消費)
.wrap
```

= 多ポート化およびTX時分割シェア設計（将来構想）

== リソース配分トポロジ案
受信（RX）は外部からの突発パケットをゼロロスで拾うため常時リスニング（SM専従）とする。送信（TX）は動的に対象ピンへアサインし、非送信時はGPIO MUXをSIO（High出力固定）へ逃がすことで、待機リンクを維持したままSMを共有する構想である。

#table(
  columns: (1.5fr, 1fr, 1.8fr, 1.8fr),
  inset: 5pt,
  align: (left, center, left, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*トポロジ構成*], [*ポート数*], [*SM配分 (1ブロック4SM)*], [*同時送信能力*],
  [A. ブロック完結型 (構想)], [9 ポート], [各ブロック: RX $times 3$, TX $times 1$], [3ポート並列送信 (ブロック毎に1ch)],
  [B. 最大ポート追求型 (構想)], [11 ポート], [PIO0/1: RX $times 4$, PIO2: RX $times 3$, TX $times 1$], [全ポート順次送信 (チップ全体で1ch)],
  [C. 対称全二重型 (構想)], [6 ポート], [各ブロック: RX $times 2$, TX $times 2$], [全6ポート完全独立同時送受信],
)

== TX動的切り替えおよび協調Yield制御シーケンス（構想）

上位層でMutex等の排他制御が完了している前提において、下位ドライバは以下のステート遷移を実行する。

```text
[ 上位タスク ]                   [ ハードウェア / PIO0 ]                   [ スケジューラ ]
      │                                     │                                     │
      ├─ 1. 対象ピンをPIO0に割当 ───────────>│ (GPIO MUX: PIO)                    │
      ├─ 2. DMA起動 (FIFOへデータ投入) ────>│ (BMCパルス送信開始)                  │
      ├─ 3. 完了待ち (IRQ 0 監視)           │                                     │
      │    └─ 未完了なら yield ─────────────────────────────────────────────────>│ (他タスクへCPU譲渡)
      │                                     │ (約345µs 送信実行: 128B)            │
      │                                     │ (送信完了・自律High化・IRQ 0アサート) │
      │<─ 4. 起床 (IRQ 0 検知) ─────────────┤                                     │
      ├─ 5. IRQ 0 フラグクリア              │                                     │
      ├─ 6. ピンをSIO (High固定) へ返却 ────>│ (GPIO MUX: SIO, 3.3V維持)           │
      ▼                                     ▼                                     ▼
```

= パケット・データリンク層フォーマットおよび再同期設計

== ワイヤフォーマット（Wire Format）

初期設計案では「プリアンブル不要」と主張されていたがこれは誤りであり、任意ビット位相のずれ、起動時過渡現象、ノイズによる誤ビット混入から確実・自動的に復帰するため、ワイヤ先頭に2バイトのプリアンブルと2バイトの固有同期ワード（Sync Word）を配置するワイヤフォーマットへ改定された。

#table(
  columns: (1.2fr, 1.2fr, 1fr, 3fr),
  inset: 5pt,
  align: (center, center, center, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*オフセット*], [*フィールド*], [*サイズ*], [*説明・バリデーション*],
  [+0 .. +1], [`PREAMBLE`], [2 Bytes], [クロック引き込み用交流パターン (`0xAAAA` = `10101010_10101010b`)],
  [+2 .. +3], [`SYNC`], [2 Bytes], [固有フレーム同期ワード (`0x93C7`, Big-Endian)],
  [+4], [`LEN`], [1 Byte], [ペイロード実長 ($1 <= N <= 128$)。0 または 129以上は即時破棄],
  [+5], [`TYPE`], [1 Byte], [パケット種別 (`0x01`: Echo Req, `0x02`: Echo Res, `0x10`: Data, `0x20`: Benchmark)],
  [+6], [`SEQ`], [1 Byte], [シーケンス番号 (0〜255 インクリメント、欠落検知用)],
  [+7], [`RESERVED`], [1 Byte], [ヘッダ側32bitアライメント用パディング (`0x00`)],
  [+8 .. +N+7], [`PAYLOAD`], [$N$ Bytes], [転送実データ (最大128バイト)],
  [+N+8 .. +N+9], [`CRC-16`], [2 Bytes], [CRC-16-CCITT (多項式 `0x1021`, 初期値 `0xFFFF`, `LEN`から`PAYLOAD`末尾までを計算)],
  [+N+10 .. +M-1], [`PADDING`], [$P$ Bytes], [32bitワード境界・極性パディング ($1 <= P <= 4$, $P = 4 - ((N + 6) "mod" 4)$)。先行 $P-1$ バイトは `0xFF`、最終バイトは送信全ビット中の「0」の総数が偶数なら `0xFF`、奇数なら `0xFE`（0を1個付加して偶数化）。ワイヤ総長 $M = N + 10 + P$ を4の倍数化],
)

- *32bitワード境界アライメントとパディング長:* ポストシンク長 $(N + 6)$ バイト（ヘッダ4B + ペイロード $N$B + CRC 2B）に対し、パディング長は $P = 4 - ((N + 6) "mod" 4)$（$1 <= P <= 4$ バイト）である（$0 \sim 3$ バイトの `0x00` ではない）。ワイヤ総長 $M = (N + 10 + P)$ は常に4の倍数（$M "mod" 4 = 0$）となり、受信側PIOの32bit autopushで未完了ビットが残存する問題を排除する。
- *偶数反転パリティ補正（極性整合）:* パディング領域のうち先行する $P-1$ バイトは `0xFF`（全ビット1、ゼロ数0）で埋められる。最終パディングバイトは、先頭からの送信全バイト（PREAMBLE、SYNC、ヘッダ、ペイロード、CRC）に含まれるビット「0」の総数が既に偶数であれば `0xFF`、奇数であれば `0xFE`（最下位ビットを0にして偶数化）とする。これによりフレーム全体の「0」の総数が常に偶数となり、BMC変調の数学的性質から最終ビット完了時の物理電位がアイドル電位（High）と確実に一致する。
- *CRC計算対象:* ポストシンクデータ（`+4 (LEN)` から `+N+7 (PAYLOAD末尾)` までの全 $N+4$ バイト）を対象に CRC-16-CCITT（多項式 `0x1021`, 初期値 `0xFFFF`）を計算する。先頭の `PREAMBLE`/`SYNC` および末尾の `PADDING` はCRC対象外である。なお `PREAMBLE` (`0xAAAA`) と `SYNC` (`0x93C7`) はゼロの総数が偶数個（PREAMBLEは8個のゼロ）であり、フレーミング同期用の固定列として極性パリティを乱さない。
- *送信バッファ形式:* TX DMAで送信する際、バッファ先頭（第0ワード）にPIOのワードカウンタ用ヘッダ `(M / 4) - 1`（後続データワード数 $- 1$）を格納し、続けてフレーム実データを転送する。
- *エンディアン:* 32bitワード単位でDMA転送されるが、シリアル線上は先頭バイト `+0 (PREAMBLE MSB)` から順にMSB-firstで送信される。CRC-16は上位バイト（+N+8）、下位バイト（+N+9）の順。

== ビット単位同期探索と任意ビット位相からの再同期（Resynchronization）

- *ビットスライド探索:* 受信ドライバ（`bmc_driver.c`）は、リングバッファ内の受信ビットストリームに対し 1 ビット単位（bit-by-bit）でスライド走査を行い、同期ワード `0x93C7`（16bit）を照合・検出する。
- *任意ビット位相オフセット耐性:* 線路ノイズや過渡状態によりビットサンプリングが 0〜7 ビット任意にずれた場合でも、ドライバはストリーム中から `0x93C7` をビット単位で捕捉してバイト境界を再確立し、次の有効フレームの先頭から即座に正常受信へと復帰する。
- *検証ステータス:* ホスト単体テスト（`test_packet.c`）において、0〜7 ビットの任意オフセット注入、ジッター混入ビットストリーム、および不正フレーム後の次フレーム完全復帰が 100% 検証されている。
- *【重要・実機未検証】:* リファクタリング後の本ファームウェア（PREAMBLE/SYNC bit-by-bit 同期探索）はホストテストで論理検証されたものであり、*実機ハードウェアにおける伝送テストは未実施（NOT hardware-tested）* である。実機での耐ノイズ性・再同期マージンは今後の実測課題である。

= Cドライバ実装リファレンス

== 初期化・ピン設定

```c
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "bmc_p2p.pio.h"

#define BMC_CLKDIV 3.0f

typedef struct {
    PIO pio;
    uint sm_tx;
    uint offset_tx;
    uint dma_tx_chan;
} bmc_tx_shared_t;

// TX共有コントローラの初期化 (PIO0を使用)
void bmc_tx_shared_init(bmc_tx_shared_t *ctx, PIO pio, uint sm) {
    ctx->pio = pio;
    ctx->sm_tx = sm;
    ctx->offset_tx = pio_add_program(pio, &bmc_tx_program);

    // TX DMA設定 (32bit転送, DREQ_PIOx_TX)
    ctx->dma_tx_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ctx->dma_tx_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    dma_channel_configure(ctx->dma_tx_chan, &c, &pio->txf[sm], NULL, 0, false);
}

// ポートTXピンの初期化 (平時SIO High固定、読み戻しイネーブル)
void bmc_tx_pin_init(uint pin) {
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    gpio_put(pin, 1);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_set_input_enabled(pin, true); // PIOのmov pins, !pins による出力読み戻しを許可
}

// ポートRXステートマシンの初期化 (PIO1専従リスニング ＋ RX DMA直結)
void bmc_rx_port_init(PIO pio, uint sm, uint offset_rx, uint pin_rx, uint dma_rx_chan, uint32_t *rx_buf, size_t buf_words) {
    gpio_pull_down(pin_rx);
    gpio_set_input_hysteresis_enabled(pin_rx, true);

    pio_sm_config c = bmc_rx_program_get_default_config(offset_rx);
    sm_config_set_in_pins(&c, pin_rx);
    sm_config_set_jmp_pin(&c, pin_rx);
    sm_config_set_in_shift(&c, false, true, 32); // MSB-first, autopush 32bit
    sm_config_set_clkdiv(&c, BMC_CLKDIV);

    pio_gpio_init(pio, pin_rx);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_rx, 1, false);
    pio_sm_init(pio, sm, offset_rx, &c);
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_x, pio_null)); // X = 0xFFFFFFFF
    pio_sm_set_enabled(pio, sm, true);

    // RX DMA設定: 4ワードFIFOのオーバーランを防ぐためリングバッファへ常時回収
    dma_channel_config dc = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, false));
    dma_channel_configure(dma_rx_chan, &dc, rx_buf, &pio->rxf[sm], buf_words, true);
}
```

== 送信プリミティブ（動的MUX ＋ 協調Yield）

```c
// 送信実行プリミティブ (上位で排他ロック獲得済みであること)
// packet_buf[0] には (データ総ワード数 - 1) を格納し、packet_buf[1..] にフレーム本体を配置すること
void bmc_tx_send_packet_yield(bmc_tx_shared_t *ctx, uint target_tx_pin,
                              const uint32_t *packet_buf, size_t total_dma_words) {
    PIO pio = ctx->pio;
    uint sm = ctx->sm_tx;

    // 1. ピン設定を動的にPIOへ切り替え (INピンも同一ピンにマップして自反転を成立)
    pio_gpio_init(pio, target_tx_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, target_tx_pin, 1, true);

    pio_sm_config c = bmc_tx_program_get_default_config(ctx->offset_tx);
    sm_config_set_set_pins(&c, target_tx_pin, 1);
    sm_config_set_out_pins(&c, target_tx_pin, 1);
    sm_config_set_in_pins(&c, target_tx_pin); // mov pins, !pins 用の自ピン入力マッピング
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_clkdiv(&c, BMC_CLKDIV);
    pio_sm_init(pio, sm, ctx->offset_tx, &c);
    pio_sm_set_enabled(pio, sm, true);

    // 2. 直前のIRQフラグクリア & DMA転送開始 (第0ワード=ワードカウンタ, 第1ワード以降=データ)
    pio_interrupt_clear(pio, 0);
    dma_channel_transfer_from_buffer_now(ctx->dma_tx_chan, packet_buf, total_dma_words);

    // 3. パケット物理出力完了 (最終ビット送出後の IRQ 0) まで協調 yield
    while (!pio_interrupt_get(pio, 0)) {
        shizuku_yield(); // 他タスクへCPU権を譲渡
    }
    pio_interrupt_clear(pio, 0);

    // 4. TX SM停止 & ピン制御を安全にSIO (High固定) へ復帰
    pio_sm_set_enabled(pio, sm, false);
    gpio_put(target_tx_pin, 1);
    gpio_set_function(target_tx_pin, GPIO_FUNC_SIO);
}
```

= リンク死活監視・フェイルセーフデーモン

定期周期（例: 10ms）またはバックグラウンド監視タスクにより、各RXピンの物理レベルをサンプリングする。

```c
#define LINK_DOWN_TIMEOUT_TICKS  10 // 10ms * 10 = 100ms (Low継続で未接続/切断判定)
#define LINK_UP_STABLE_TICKS     2  // 10ms * 2 = 20ms (High継続でリンク確立判定)

typedef struct {
    uint rx_pin;
    uint tx_pin;
    uint32_t low_count;
    uint32_t high_count;
    bool is_link_up;
} bmc_link_monitor_t;

void bmc_link_monitor_tick_10ms(bmc_link_monitor_t *mon) {
    if (!gpio_get(mon->rx_pin)) {
        mon->high_count = 0;
        mon->low_count++;
        if (mon->low_count >= LINK_DOWN_TIMEOUT_TICKS && mon->is_link_up) {
            mon->is_link_up = false;
            gpio_put(mon->tx_pin, 1);
            on_bmc_link_status_changed(mon->rx_pin, false);
        }
    } else {
        mon->low_count = 0;
        mon->high_count++;
        if (!mon->is_link_up && mon->high_count >= LINK_UP_STABLE_TICKS) {
            mon->is_link_up = true;
            gpio_put(mon->tx_pin, 1);
            on_bmc_link_status_changed(mon->rx_pin, true);
        }
    }
}
```

= 設計・仕様の訂正履歴および前提の厳密化（Mandatory Corrections）

本ドキュメントの初版および先行検討において提示された各種の主張・仮説について、実装事実およびハードウェア検証の観点から以下の通り厳密な訂正と分類（認識論的分離）を行う。

== 6大必須訂正事項（Mandatory Corrections）

#table(
  columns: (1.2fr, 1.8fr, 2.2fr),
  inset: 5pt,
  align: (left, left, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*項目*], [*旧主張（Old Claim / 誤認）*], [*訂正された現実（Corrected Reality）*],
  [1. 完全均一ビット幅],
  [【旧主張】TXビットセルは前半8cyc/後半8cyc厳密同期の完全均一16サイクル幅（均等波形）である。],
  [【非均一ビットセル】ワード内のビットは公称16サイクル（8cyc/8cyc）であるが、32bitワード境界（OSR再充填とループ判定）では `pull block` と `jmp y--` の実行によりワード末尾セルが18サイクル（+2サイクル伸長）となる。高速版（`bmc_p2p_fast.pio`）では通常8サイクル、ワード境界10サイクル（+2サイクル）。RX側は境界エッジ同期のためこの伸長を許容・吸収する。],

  [2. $plus.minus$9.83 ns 保証],
  [【旧主張】実効ジッターマージン $plus.minus$9.83 ns が物理的・実機環境において保証される。],
  [【机上モデルであり未確立】$plus.minus$9.83 ns は名目値と対称立ち上がりを仮定した机上計算モデル（Calculation/Model）に過ぎず、実機環境（実デバイス）での保証値としては未確立（Not established）。PAD製造ばらつき、線路容量、非対称遷移、高周波反射は実機特有の未知事項である。],

  [3. 18.75 Mbps PIO/PHY 限界],
  [【旧主張】18.75 Mbps（8cyc/bit @ 150MHz）が抵抗レス直結の物理・PIO命令両面の絶対限界である。],
  [【実験的探索点（EXPERIMENTAL）】18.75 Mbps は現行の1命令/サイクル同期アーキテクチャ下における実験的探索点に過ぎず、チップおよび物理層（PHY）の絶対的限界を確定・確立したものではない。別命令構成やクロック変更による更なる限界探索の余地が残されている。],

  [4. SLOW+4mA 誤エッジゼロ],
  [【旧主張】`GPIO_SLEW_RATE_SLOW` ＋ 4mA 駆動によりリンギング反射波を平滑化し、誤エッジは物理的にゼロ。],
  [【未検証の仮定】スルーレート制限と弱駆動は反射波の尖頭値を抑制するが、任意のジャンパ線長や浮遊インダクタンスにおいて「誤エッジが完全にゼロ」であることは実機証明されておらず、理論的期待・前提（Unverified assumption）である。],

  [5. 量産推奨],
  [【旧主張】9.375 Mbps（または特定レート）が商用・量産推奨レートとして確立されている。],
  [【量産推奨は未確立】本プロジェクトはブレッドボード上のジャンパ線直結による実験的 PoC 実証であり、量産推奨（Production recommendation）は未確立（Not established）。リファクタリング後のファームウェアは実機ハードウェア検証未実施であり、長期環境試験やEMC耐量評価も行われていない。],

  [6. wait_sop への JMP による ISR フラッシュ誤認],
  [【旧主張】`pio_sm_exec(jmp(wait_sop))` により RX ISR の端数ビットを瞬時にフラッシュしリセット可能。],
  [【JMPはPC変更のみ】RP2350 PIO において `jmp` 命令はプログラムカウンタ（PC）を変更するだけであり、ISR の蓄積ビットや入力シフトカウンタをクリアしない。端数ビットとカウンタを真にフラッシュ・クリアするには `mov isr, null`（または SM のリセット・再起動）の実行が必須である。],
)

== 情報の厳密な4区分（認識論的ステータス）

- *(a) ユーザー報告による実機計測値（USER-REPORTED MEASUREMENTS）:*
  - プラットフォーム: Raspberry Pi Pico 2 W / RP2350 @ 150.0 MHz sysclk、同一 GP0 <---> GP1 ジャンパ線直結環境
  - 旧シングルレート BMC (3.125 Mbps): 15,251 pkts, 157.4 kB/s, CRC Err: 0, Drops: 0
  - ハードウェア UART 3.000 Mbps: 11,389 pkts, 162.5 kB/s, CRC Err: 0, Drops: 0, Verify Err: 0
  - ハードウェア UART 9.375 Mbps: 20,339 pkts, 290.1 kB/s, CRC Err: 0, Drops: 0, Verify Err: 0
- *(b) ソースコード／実装から導出された事実（Code-Derived Facts）:*
  - PIO 命令長: 標準 TX 14/32, RX 17/32; 高速 TX 14/32, RX 18/32
  - PIO 配置: TX は `pio0`、RX は `pio1` に完全分離
  - ビット幅: 標準 通常16cyc/境界18cyc; 高速 通常8cyc/境界10cyc
  - ワイヤフォーマット: `PREAMBLE` (`0xAAAA`, 2B) + `SYNC` (`0x93C7`, 2B) + ヘッダ + ペイロード + CRC16 + パディング ($1 <= P <= 4$, 先行 `0xFF`, 末尾 `0xFF`/`0xFE`)
  - 再同期動作: 受信ドライバによるビット単位探索と任意ビットオフセット（0〜7ビット）からの次フレーム自動復帰
  - レジスタ操作: `mov isr, null` による ISR およびシフトカウンタのクリア
- *(c) 計算／モデル（Calculation / Model）:*
  - 伝送線路パラメータ（$t_d approx 2.64 "ns"$, $Z_0 approx 120 Omega$）、名目サンプリングマージン計算
- *(d) 未検証の前提／実機特有の未知事項（Unverified Assumptions / Device-Only Unknowns）:*
  - リファクタリング後ファームウェアの実機ハードウェア動作（ホスト単体テスト合格のみ、実機未検証）
  - 誤エッジゼロ仮説、実機環境におけるジッターマージン保証、量産適格性
