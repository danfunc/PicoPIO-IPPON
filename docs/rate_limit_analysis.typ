#set page(paper: "a4", margin: (x: 1.8cm, y: 1.8cm))
#set text(font: ("Hiragino Sans", "Hiragino Kaku Gothic ProN"), size: 9.0pt)
#set par(justify: true, leading: 0.60em)
#set heading(numbering: "1.1")

#align(center)[
  #text(size: 15pt, weight: "bold")[RP2350 抵抗レスP2P通信における物理層・PIO論理限界レートの徹底探求とアーキテクチャ設計] \
  #v(1.5mm)
  #text(size: 9.5pt, fill: rgb(80, 80, 80))[50cm ジャンパ線直結における分布定数線路解析・PIO限界サイクル圧縮・UART比較・EOP完全解決]
]

#v(2mm)

= エグゼクティブサマリー（限界値・計測値まとめ一覧）

本レポートは、Raspberry Pi Pico 2 / Pico 2 W（RP2350、sysclk 150.0MHz）同士を外付けダンピング抵抗なしのジャンパ線（50cm以内）で直結する環境において、差動マンチェスタ符号（BMC）およびハードウェアUART（PL011）の物理的・論理的限界通信レートを徹底探求・算定したものである。

#table(
  columns: (1.4fr, 1.1fr, 1.1fr, 1.4fr, 2fr),
  inset: 4.5pt,
  align: (left, center, center, center, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(235, 240, 250) } else if y == 3 { rgb(255, 245, 235) } else { none },
  [*プロトコル / モード*], [*分周・設定*], [*物理レート*], [*実効最大スループット*], [*主要な制限要因・分類*],
  [旧 BMC (PoC単一レート)], [`clkdiv = 3.0f` (16cyc)], [3.125 Mbps], [157.4 kB/s (15,251 pkts)], [【ユーザー報告実測値】Pico 2 W / 150MHz, GP0<->GP1直結 (CRC Err 0, Drops 0)],
  [高速 BMC (実験的検討)], [`clkdiv = 2.0f` (8cyc)], [9.375 Mbps], [約 1.17 MB/s (理論計算)], [【計算モデル】UART最大値と同等レート。※量産推奨は未確立・実機未検証],
  [*限界 BMC (EXPERIMENTAL)*], [*`clkdiv = 1.0f` (8cyc)*], [*18.75 Mbps*], [*約 2.34 MB/s (理論計算)*], [*【実験的探索点】特定命令列での探索点であり絶対限界ではない。※実機未検証*],
  [超限界 BMC (TX机上検討)], [`clkdiv = 1.0f` (6cyc)], [25.00 Mbps], [約 3.12 MB/s (理論計算)], [【計算モデル】TX命令は成立するがRXサンプリング困難（未検証）],
  [UART (現行 3.0M)], [150MHz / 50], [3.000 Mbps], [162.5 kB/s (11,389 pkts)], [【ユーザー報告実測値】Pico 2 W / 150MHz, GP0<->GP1 (CRC Err 0, Verify Err 0)],
  [UART (最大 9.375M)], [150MHz / 16], [9.375 Mbps], [290.1 kB/s (20,339 pkts)], [【ユーザー報告実測値】16倍OS固定制約、実機計測で290.1 kB/s (CRC Err 0)],
)

*主たる結論（前提と事実の厳密化）:*
1. *物理層（PHY）の挙動:* 抵抗レス50cm配線では `SLOW` スルーレート（$t_r approx 12 "ns"$）＋ 4mA 駆動を推奨。ただし「誤エッジゼロ」は理論的期待に基づく未検証の前提（Unverified assumption）であり、物理的に完全に誤エッジゼロが保証されているわけではない。
2. *PIO 論理限界の性質:* 8 サイクル/bit（18.75 Mbps、sysclk 150MHz時）は現行命令構成下での「実験的探索点（EXPERIMENTAL）」であり、RP2350 チップや物理層の絶対限界を確定したものではない。また名目計算上の $plus.minus 9.83 "ns"$ ジッターマージンは実機環境での保証値としては未確立（Not established）である。
3. *UART (PL011) との実測比較:* ユーザー報告の実機計測において、UART 3.0Mbps は 162.5 kB/s、UART 9.375Mbps は 290.1 kB/s、旧BMC 3.125Mbps は 157.4 kB/s を達成。リファクタリング後の新ファームウェア（PREAMBLE/SYNC bit-by-bit 同期）は実機検証未実施（NOT hardware-tested）。
4. *EOPファントムビットと再同期:* 送信側パディングによる偶数反転パリティ補正に加え、受信側で PREAMBLE `0xAAAA` ＋ SYNC `0x93C7` によるビット単位探索・任意ビット位相（0〜7bit）からの次フレーム自動再同期を設計。なお、部分ISRのフラッシュには `jmp wait_sop`（PC変更のみ）ではなく `mov isr, null` が必要。

---

= 物理層（PHY）の伝送限界レート

== 伝送線路パラメータと過渡特性解析
50cm のジャンパ線（対地・GND併走、AWG26〜28）を分布定数線路としてモデル化する。

#grid(
  columns: (1fr, 1fr),
  gutter: 10pt,
  [
    - 比誘電率: $epsilon_r approx 2.5$（PVC/ポリエチレン被覆）
    - 伝搬速度: $v_p = c / sqrt(epsilon_r) approx 0.19 "m/ns" = 19 "cm/ns"$
    - 片道伝搬遅延: $t_d = 0.50 "m" / 0.19 "m/ns" approx 2.64 "ns"$
    - 往復伝搬遅延: $t_"RT" = 2 times t_d approx 5.28 "ns"$
  ],
  [
    - 特性インピーダンス: $Z_0 approx 100 "〜" 150 Omega$（代表値 $120 Omega$）
    - 線路浮遊容量: $C_"line" = t_d / Z_0 approx 22 "pF"$
    - 受端入力容量: $C_"pad" approx 5 "〜" 8 "pF"$
    - 総負荷容量: $C_L approx 35 "〜" 60 "pF"$
  ]
)

== 送信ドライバの内部インピーダンスと反射係数
RP2350の内部GPIO PADの3.3V給電時における出力インピーダンス $R_s$ は駆動電流設定に依存する:
- $12 "mA"$: $R_s approx 30 Omega$ $arrow$ 反射係数 $Gamma_S = (30 - 120)/(30 + 120) = -0.60$
- $4 "mA"$: $R_s approx 85 Omega$ $arrow$ 反射係数 $Gamma_S = (85 - 120)/(85 + 120) = -0.17$
受端は高入力インピーダンス（オープン終端）のため、$Gamma_L = +1.0$ である。

== スルーレート `FAST` vs `SLOW` の過渡応答（ラティス解析）

#table(
  columns: (1.2fr, 1.8fr, 1.8fr),
  inset: 5pt,
  align: (left, left, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*項目*], [*FAST スルーレート ($t_r approx 2 "〜" 4 "ns"$)*], [*SLOW スルーレート ($t_r approx 10 "〜" 15 "ns"$)*],
  [成立条件], [$t_r < t_"RT"$（完全な分布定数回路）], [$t_r > 2 times t_"RT"$（集中定数近似が成立）],
  [受端電圧波形], [急峻なステップ状跳ね上がりと深いアンダーシュート], [単調かつ滑らかなランプ状立ち上がり],
  [リンギング],
  [受端で $3.8 "V"$（クランプ）後、反射波で $0.13 "V"$ まで急落],
  [$R_s approx 85 Omega$ による自然ダンピングで反射波が即時減衰],
  [シュミットトリガへの影響],
  [*重大:* $V_"IL" (0.8 "V")$ を下回り、偽エッジ（誤反転）が多発],
  [*安全（モデル上）:* ヒステリシス幅内で単調推移。ただし『誤エッジゼロ』は未検証の前提であり、実機確定値ではない],
  [抵抗レスでの可否], [*使用禁止*（外付けダンピング抵抗 50〜100$Omega$ 必須）], [*実験適合*（50cmジャンパ直結のPoC環境に適合。量産推奨は未確立）],
)

```text
【FAST (12mA) の受端電圧: 偽エッジ発生】          【SLOW (4mA) の受端電圧: 単調増加で安全】
 電圧 [V]                                          電圧 [V]
 3.8 |       +--+                                  3.3 |               +-----------------
     |      /|  | (Over-clamp)                         |              /
 2.0 |...../.|..|............ VIH 閾値            2.0 |............./.... VIH 閾値
 0.8 |..../..|..|............ VIL 閾値            0.8 |....../........... VIL 閾値
 0.1 |   /   |  +---+ (Under-shoot: 偽Low!)            |     /
 0.0 +--+----+-------+-----> 時間                  0.0 +----+------------> 時間
        td  2td 3td                                         tr (12ns)
```

== 物理層の限界パルス幅と限界ビットレート算定
BMC変調信号において最も短いパルス幅は、ビット1の中央反転によって生じる半ビット期間 $T_"half" = T_"bit" / 2$ である。
受端のシュミットトリガが安定してハイ/ローを判定し、アイパターン開口を確保するための条件は:
$ T_"half, min" >= t_r + t_"settle" $
ここで `SLOW` スルーレート（4mA駆動、負荷 50pF）において $t_r approx 12 "ns"$、整定・平坦部マージンとして $t_"settle" approx 8 "ns"$ を仮定すると:
$ T_"half, min" = 12 "ns" + 8 "ns" = 20 "ns" \
T_"bit, min" = 2 times T_"half, min" = 40 "ns" $
したがって、机上計算モデルにおいて物理層（伝送路＋PAD）が許容し得る試算上限は $R_"PHY, max" = 1 / (40 "ns") = 25.0 "Mbps"$ である。
また、実験的探索レートとして $1 / (53.33 "ns") = 18.75 "Mbps"$ が検討されるが、これは現在のPIO命令列における探索動作点であり、物理層やチップの絶対限界を確定したものではない（量産推奨も未確立）。

---

= PIO の論理・命令限界レート

== PIO クロックと分周比の設計（sysclk 150.0 MHz）
RP2350 の PIO クロックを最大（`clkdiv = 1.0f` $arrow$ 150.0 MHz, $tau = 6.667 "ns"$）とした場合のビット圧縮限界を検証する。

#table(
  columns: (1.2fr, 1.2fr, 1fr, 1.5fr, 1.8fr),
  inset: 4.5pt,
  align: (center, center, center, center, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*サイクル/bit*], [*1ビット時間*], [*ビットレート*], [*TX成立性 (命令)*], [*RX成立性・ジッターマージン*],
  [16 cyc/bit], [106.67 ns], [9.375 Mbps], [完全成立 (ディレイ潤沢)], [完全成立 ($plus.minus 26.7 "ns"$ / $plus.minus 4.0 "cyc"$)],
  [12 cyc/bit], [80.00 ns], [12.50 Mbps], [完全成立], [完全成立 ($plus.minus 20.0 "ns"$ / $plus.minus 3.0 "cyc"$)],
  [*8 cyc/bit*], [*53.33 ns*], [*18.75 Mbps*], [*完全成立 (4+4cyc)*], [*成立限界 ($plus.minus 13.3 "ns"$ / $plus.minus 2.0 "cyc"$)*],
  [6 cyc/bit], [40.00 ns], [25.00 Mbps], [完全成立 (3+3cyc)], [論理不成立（中央サンプリングとエッジ衝突）],
  [4 cyc/bit], [26.67 ns], [37.50 Mbps], [不成立（分岐命令不足）], [不成立],
)

== BMC 変調器（TX）の限界サイクル設計と非均一ビットセル

TXは「境界反転（自ピン読み戻し `mov pins, !pins`）」と「ビット1時の中央反転」を行う。
ここで留意すべきソース実装事実として、*TXのビットセル幅は完全均一ではなく非均一（Non-uniform bit cells）* である:
- *標準 BMC (`bmc_p2p.pio`):* ワード内通常ビットは 16 サイクル（320 ns）、32bitワード境界（OSR再充填 `pull block` および `jmp y--`）の最終ビットのみ 18 サイクル（360 ns、+2サイクル伸長）。
- *高速 BMC (`bmc_p2p_fast.pio`):* ワード内通常ビットは 8 サイクル（53.33 ns）、32bitワード境界のみ 10 サイクル（66.67 ns、+2サイクル伸長）。
受信側はビット毎の境界エッジで自律再同期を行うため、この 2 サイクルのワード境界伸長は問題なく吸収される。

150MHzにおいて、以下の通りディレイなしで *6 サイクル/bit* までのTX命令ループを記述可能である（ただし後述の通りRX側が追従不能）。

```pasm
; --- 6 cyc/bit TX bit_loop (25.0 Mbps @ 150MHz, clkdiv = 1.0f) ---
bit_loop:
    mov pins, !pins         ; 1: 境界エッジ反転 (前半 1/3)
    out x, 1                ; 2: OSRから1ビット取得 (前半 2/3)
    jmp !x, is_zero         ; 3: 0なら中央反転スキップ (前半 3/3)
    mov pins, !pins         ; 4: ビット1: 中央エッジ反転 (後半 1/3)
    jmp check_bit           ; 5: (後半 2/3)
is_zero:
    nop                     ; 4-5: ビット0: レベル維持 (後半 1-2/3)
check_bit:
    jmp !osre, bit_loop     ; 6: ループ判定 (後半 3/3: 計6cyc均等)
```
※ 4 cyc/bit の場合、前半 2 サイクル内に「反転」「ビット取得」「分岐」の 3 命令を収めることができず、純粋な命令実行では物理的に成立しない。

== BMC 復号器（RX）の限界サイクル設計と PIO 分離・命令メモリ配分

旧設計では単一ブロック（32ワード）に TX と RX を同居させていたが、現行実装では *TX を `pio0`、RX を `pio1` に独立分離* している。
これにより命令メモリ制約は大幅に緩和され、各ブロックの消費量と空き容量は以下の通り確定している（【ソース実装事実】）:
- *標準 BMC (`bmc_p2p.pio`):* TX 14 / 32 words（空き 18 words, `pio0`）、RX 17 / 32 words（空き 15 words, `pio1`）
- *高速 BMC (`bmc_p2p_fast.pio`):* TX 14 / 32 words（空き 18 words, `pio0`）、RX 18 / 32 words（空き 14 words, `pio1`）

高速 BMC 復号器（`bmc_p2p_fast.pio`）では、8 サイクル/bit（18.75 Mbps、sysclk 150MHz時、clkdiv=1.0f）においてサンプリング点を *Cycle 6* に配置する。

#table(
  columns: (1.5fr, 1.2fr, 1.5fr, 2fr),
  inset: 4.5pt,
  align: (left, center, center, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*タイミングイベント*], [*サイクル*], [*時間 (150MHz時)*], [*タイミングマージン・関係*],
  [ビット境界開始], [Cycle 0], [0.0 ns], [エッジ立上がり/立下り],
  [中央エッジ遷移 (ビット1)], [Cycle 4], [26.67 ns], [中間反転点 (半ビット幅 4 cyc)],
  [*RXサンプリング点*], [*Cycle 6*], [*40.00 ns*], [*中央遷移から +2 cyc (+13.3 ns), 次境界まで -2 cyc (-13.3 ns)*],
  [ビット境界終了], [Cycle 8], [53.33 ns], [次ビット先頭エッジ],
)

```pasm
; --- 8 cyc/bit Fast RX was_low ブロック (Cycle 6 サンプリング, 計18ワード) ---
was_low:
    nop                 [1] ; 1-2: 2サイクルディレイ
    jmp pin, got_1_from_low ; 3: Cycle 6 でピンサンプリング (+2cyc / -2cyc マージン)
got_0_from_low:
    in null, 1              ; 4: ビット0 (Cycle 7)
    wait 1 pin 0            ; 5: 次の境界エッジ (立上がり) 待機 (Cycle 8)
    jmp was_high            ; 6: 次状態へ遷移
got_1_from_low:
    in x, 1                 ; 7: ビット1 (Cycle 7)
    wait 0 pin 0            ; 8: 次の境界エッジ (立下り) 待機 (Cycle 8)
    jmp was_low             ; 9: 次状態へ遷移
```

== サンプリング窓マージンとジッター耐性の定量的評価（机上モデルと実機制約）

8 サイクル/bit（18.75 Mbps, $T_"bit" = 53.33 "ns"$）における名目タイミングバジェット（【机上計算モデル】）:
- 前半期間（中央反転位置）: $t = 26.67 "ns"$（Cycle 4）
- サンプリング位置: $t = 40.00 "ns"$（Cycle 6）
- 次の境界反転位置: $t = 53.33 "ns"$（Cycle 8）
- *理論サンプリングマージン:*
  $ Delta t_"margin, theoretical" = 40.00 "ns" - 26.67 "ns" = plus.minus 13.33 "ns" quad (plus.minus 2.0 "cyc") $
- *ジッター要因の机上見積り:*
  - 配線伝搬遅延の非対称性・温度変動: $Delta t_"line" approx plus.minus 0.5 "ns"$
  - シュミットトリガ入力の閾値ばらつき＋スルーレート起因ジッター: $Delta t_"pad" approx plus.minus 3.0 "ns"$
  - 水晶発振子周波数偏差（$plus.minus 50 "ppm"$）: 累積ドリフト $0.003 "ns"$ 以下（無視可能）
- *名目実効ジッターマージン計算値:*
  $ Delta t_"margin, effective" = 13.33 "ns" - 3.50 "ns" = plus.minus 9.83 "ns" quad (approx plus.minus 1.47 "cyc") $

*(※重要: $plus.minus$9.83 ns 保証の非成立性):*
旧版では上記計算に基づき「$plus.minus 9.83 "ns"$ が実機環境で保証される」と主張されていたが、*この保証値は実機ハードウェアにおいて確立されたものではない（Not established）*。実際のジャンパ配線における高周波リンギング、PAD入力容量ばらつき、基板グラウンドバウンス、立ち上がり・立ち下がりの非対称性は実機特有の未知事項（Device-only unknowns）であり、実測なしに保証することはできない。

また、18.75 Mbps という速度自体も現行の特定命令シーケンス下における「実験的探索点（EXPERIMENTAL）」であり、チップや物理層の絶対限界を確定したものではない。一方、6 サイクル/bit（$T_"bit" = 40.0 "ns"$）ではサンプリングマージンが $plus.minus 1$ サイクル（$6.67 "ns"$）以下となり、`SLOW` スルーレート（$t_r approx 12 "ns"$）の遷移領域と重複するため、現行アーキテクチャでは成立困難である。

---

= ハードウェア UART (PL011) との限界比較

== PL011 UART の限界レートと構造的制約
RP2350 に搭載されている ARM PrimeCell UART (PL011) は、非同期シリアル通信の標準ペリフェラルである。
- *ボーレート生成のアーキテクチャ:*
  PL011 は入力クロック（`UARTCLK` = 通常 150MHz）に対して *16倍固定オーバーサンプリング* を行う仕様となっている。
  $ "BaudRate" = "UARTCLK" / (16 times ("IBRD" + "FBRD" / 64)) $
  したがって、最小分周比（`IBRD = 1, FBRD = 0`）における物理最大ボーレートは以下に固定される:
  $ "BaudRate"_"max" = (150 times 10^6) / (16 times 1.0) = 9.375 "Mbps" $
  150MHz クロック下において、UART は 9.375 Mbps を 1bps たりとも超えることができない。

== プロトコルオーバーヘッドと実効スループット比較
UART（8N1）は、各 8 ビットのデータに対し、1 ビットのスタートビット（Low）と 1 ビットのストップビット（High）が不可欠であり、計 10 ビットを要する。
$ eta_"UART" = 8 / 10 = 80.0% quad (20% "の帯域損失") $
これに対し、BMC は各ビットの境界反転にクロックが埋め込まれているため、フレーミングビットを一切必要としない（自己同期）。
$ eta_"BMC" = 100.0% quad (0% "の帯域損失") $

#table(
  columns: (1.4fr, 1.2fr, 1.2fr, 1.2fr, 1.4fr),
  inset: 5pt,
  align: (left, center, center, center, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*比較項目*], [*UART0 (3.0M)*], [*UART (最大9.375M)*], [*旧BMC (単一3.125M)*], [*BMC (実験的18.75M)*],
  [物理ボーレート], [3.000 Mbps], [9.375 Mbps], [3.125 Mbps], [*18.750 Mbps*],
  [フレーミング効率], [80.0% (8N1)], [80.0% (8N1)], [100.0%], [*100.0%*],
  [理論スループット], [300.0 kB/s], [937.5 kB/s], [390.6 kB/s], [*2,343.8 kB/s (2.34 MB/s)*],
  [実機計測値 / 区分],
  [162.5 kB/s (実測)],
  [290.1 kB/s (実測)],
  [157.4 kB/s (実測)],
  [*実機未検証 (計算のみ)*],
  [実機計測詳細],
  [11,389 pkts \ (Err/Drop 0)],
  [20,339 pkts \ (Err/Drop 0)],
  [15,251 pkts \ (Err/Drop 0)],
  [【未検証】ホスト単体 \ テストのみ通過],
  [チップ内ポート数], [2 ポート], [2 ポート], [TX: pio0, RX: pio1], [TX: pio0, RX: pio1],
  [同期方式], [10bit非同期], [10bit非同期], [毎ビット自己同期], [毎ビット自己同期],
)

*実機計測の留意事項:*
上記の実測値は、Raspberry Pi Pico 2 W / RP2350（150MHz sysclk、同一 GP0 <---> GP1 ジャンパ線直結）において得られた【ユーザー報告実測値】である。
- UART 3.0 Mbps: 162.5 kB/s (11,389 pkts, CRC Err 0, Drops 0, Verify Err 0)
- UART 9.375 Mbps: 290.1 kB/s (20,339 pkts, CRC Err 0, Drops 0, Verify Err 0)
- 旧単一レート BMC: 157.4 kB/s (15,251 pkts, CRC Err 0, Drops 0)
一方で、リファクタリング後の新BMCファームウェア（PREAMBLE/SYNC bit-by-bit 同期、高速18.75Mbps）の数値は机上モデル（Calculation/Model）であり、*実機ハードウェアにおける伝送テストは未実施（NOT hardware-tested）* である。リファクタリング後コードのいかなる値も実測値として混同してはならない。

---

= EOP（パケット末尾）ファントムビット問題の構造的解決

== 問題の構造と発生メカニズムの解明
現行の TX PIO (`bmc_tx`) では、パケット全ワード送出直後に以下の命令を実行して待機 High に復帰する:
```pasm
    set pins, 1             ; 13: 最終ビット完了: 即座にHigh能動出力
    irq set 0               ; 14: パケット物理送出完了をCPUへ通知
```
このとき、*パケットの最終ビットが「Low」電位で終了していた場合*、以下の現象が発生する:
1. `set pins, 1` が実行された瞬間、物理ピンが $0 "V" arrow 3.3 "V"$ へ急峻に立ち上がる。
2. 受信側 PIO (`bmc_rx`) は最終ビットを取り込んだ後、次状態へ進むための境界エッジ待機（`wait 1 pin 0`）でブロックしている。
3. TXの `set pins, 1` による立ち上がり段差を、RXは「次ビットの先頭境界エッジが到来した」と誤認して `wait` を脱出する。
4. RXはアイドルHigh状態をデコードし、*余計なゴミビット（ファントムビット）を ISR へ 1 ビット取り込んでしまう*。
5. RXの Autopush（32bit）のカウンタが 1 ビットずれ、次パケット受信時に全ワード境界が永久に破壊される。

```text
【ファントムビット発生シーケンス】
           最終ビット (後半Lowで終了)        EOP (set pins, 1)
TX Pin:  ----+               +---------------+               +----------------- (アイドルHigh維持)
             |               |               |               |
             +---------------+               +---------------+
             |<-- 半ビット ->|<-- 半ビット ->| ↑
                                             TXが強制High化 (意図せぬ立上がりエッジ!)
RX 動作:                                     ↑
                                             RXの wait 1 pin が誤発火!
                                             ファントムビットをデコードしてISRが1bit破壊!
```

== 解決アプローチの比較検討

#table(
  columns: (1.5fr, 1.2fr, 1.2fr, 2fr),
  inset: 5pt,
  align: (left, center, center, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*解決アプローチ*], [*PIO命令増*], [*CPU負荷*], [*評価・実現性*],
  [A. PIO側タイムアウト検出], [+6〜8 ワード], [ゼロ], [× メモリ残量 1 ワードのため 32 ワード制限を超過し不可],
  [B. 違法シンボル（Break）], [+4〜5 ワード], [ゼロ], [× 同様に命令メモリ超過。ハードウェア成立不能],
  [C. RX SMソフトリセット], [0 ワード], [軽微], [◯ 受信毎にSMを再起動。確実だがパケット間隔の制約が生じる],
  [*D. 偶数反転パリティパディング*], [*0 ワード*], [*極小*], [*◎ 数学的不変条件の成立。理想電位段差ゼロ*],
)

== 【構造的決定打】偶数反転パリティパディング（Polarity-Preserving EOP）の設計

=== 1. BMC 極性遷移の数学的原理
差動マンチェスタ符号（BMC）におけるピン反転回数は厳密に数学付けられる:
- ビット「0」: 境界反転のみ（*1 回反転 / 奇数* $arrow$ 極性が反転する）
- ビット「1」: 境界反転 ＋ 中央反転（*2 回反転 / 偶数* $arrow$ 極性は変化しない）
送信開始前のアイドル電位は必ず High（$1$）である。
したがって、パケット全体の全ビット列に含まれる *ビット「0」の総数が偶数個であれば、最終ビット完了時の物理ピン電位は必ず High（初期状態と同一）になる*。

=== 2. パディングバイトによる極性補正アルゴリズム
パケットフォーマットには、32bit ワードアライメントと極性パリティ調整を兼ねた `PADDING` フィールド（$1 <= P <= 4$ バイト）が存在する。
送信バッファ生成時、パケット末尾のパディングワードの最終バイトを以下の規則で決定する:

```c
// パケット内の '0' ビットのパリティを偶数化し、最終電位を High 固定する
void bmc_packet_finalize_polarity(uint8_t *packet_bytes, size_t total_len_without_pad, size_t total_aligned_len) {
    uint32_t zero_count = 0;
    for (size_t i = 0; i < total_len_without_pad; i++) {
        uint8_t b = packet_bytes[i];
        // 8ビット中の '0' の個数を加算 (__builtin_popcount で高速計算)
        zero_count += (8 - __builtin_popcount(b));
    }
    
    // アライメントパディング領域の先行バイトを 0xFF (全ビット1: 極性変化なし) で初期化
    for (size_t i = total_len_without_pad; i < total_aligned_len - 1; i++) {
        packet_bytes[i] = 0xFF;
    }
    
    // 送信全バイト中の '0' の総数を偶数に整合
    if ((zero_count & 1) == 0) {
        packet_bytes[total_aligned_len - 1] = 0xFF; // 既に偶数のため 0xFF
    } else {
        packet_bytes[total_aligned_len - 1] = 0xFE; // 奇数のため 0xFE (0を1個追加して偶数化)
    }
}
```

=== 3. 物理動作と効果（数学的不変条件と実機制約）
このパリティ補正により、数学的に最終ビット送出完了時のBMC極性不変条件（開始電位High＝終了電位High）が満たされ、TX PIO の `set pins, 1` による理想的電位段差は計算上ゼロとなる。
ただし、これはデジタル論理上の極性整合であり、実際の伝送路上における過渡ノイズ、リンギング、浮遊容量による電位の乱れが『物理的に100%排除』されることを保証するものではない（実機環境特有の未知事項）。

さらに、ノイズ混入や未完了ビット残存に対する防御策の検討において、以下の重要なハードウェア動作の区別に留意する必要がある:

*(※重要: JMP による ISR フラッシュ誤認と MOV ISR, NULL):*
旧版では以下のコードで「RX ISR の端数ビットを瞬時フラッシュできる」とされていた:
```c
// RX ISR の端数ビットを瞬時フラッシュし、次パケットのSOP待機に完全リセット (消費命令 0)
// 【誤り】jmp 命令は PC を変更するのみで、ISR 内容や入力シフトカウンタをクリアしない
pio_sm_exec(pio, sm_rx, pio_encode_jmp(offset_rx + bmc_rx_offset_wait_sop));
```
RP2350 PIO において、`jmp` 命令はプログラムカウンタ（PC）を更新するだけであり、ISR にシフトされた端数ビットデータや入力シフトカウンタ（shift counter）は保持されたままとなる。
端数ビットおよびシフトカウンタを真にゼロクリアするには、以下の通り `mov isr, null`（または SM のリセット・クリア）を実行しなければならない:
```c
// 【正しいリセット】ISR の内容および入力シフトカウンタを完全にゼロクリア
pio_sm_exec(pio, sm_rx, pio_encode_mov(pio_isr, pio_null));
pio_sm_exec(pio, sm_rx, pio_encode_jmp(offset_rx + bmc_rx_offset_wait_sop));
```
現行のリファクタリング設計では、このハードウェア制約を踏まえ、ワイヤ先頭に `PREAMBLE` (`0xAAAA`) および `SYNC` (`0x93C7`) を導入し、受信側ソフトウェアドライバがリングバッファ内でビット単位（bit-by-bit）の同期ワード探索を行うアーキテクチャへと進化させている。これにより、仮にビット位相オフセットが生じた場合でも、次の有効フレーム先頭で自動的にバイト位相が再捕捉される（ホストテストでオフセット 0〜7bit 復帰を検証済み。※実機未検証）。

---

= 総合結論と検証ステータス

1. *物理層の挙動:*
   外付けダンピング抵抗なしの 50cm ジャンパ線環境においては「`SLOW` スルーレート ＋ 4mA 駆動」が有効な設計指針となる。これにより反射波の立ち上がり勾配が緩和されるが、「誤エッジゼロ」は理論モデル上の前提（Unverified assumption）であり、物理的に誤エッジゼロが保証されているわけではない。
2. *レート選定と位置付け:*
   - *実験的検証レート: 3.125 Mbps (`clkdiv = 3.0f`, 16cyc/bit)*
     旧単一レート BMC 実装において実機計測 157.4 kB/s（15,251 pkts, CRC Err 0, Drops 0）を確認済み。
   - *高速検討レート: 9.375 Mbps (`clkdiv = 2.0f`, 8cyc/bit)*
     UART最大レートと同一の物理速度。なお、量産推奨（Production recommendation）は未確立である。
   - *実験的探索レート (EXPERIMENTAL): 18.75 Mbps (`clkdiv = 1.0f`, 8cyc/bit)*
     現行の1命令/サイクル同期構成下における実験的探索点。RP2350 チップや物理層の絶対的限界を確定したものではない。
3. *アーキテクチャ設計:*
   TX を `pio0`（14/32 words）、RX を `pio1`（標準 17/32 words、高速 18/32 words）に分離し、ワイヤフォーマットに PREAMBLE `0xAAAA` ＋ SYNC `0x93C7` による bit-by-bit 同期探索を採用した。なお、リファクタリング後の本ファームウェアは実機検証未実施（NOT hardware-tested）である。

---

= 設計・解析の訂正履歴および前提の厳密化（Mandatory Corrections）

本ドキュメントにおける旧主張と、実装・ハードウェア検証に基づく訂正内容を以下に総括する。

#table(
  columns: (1.2fr, 1.8fr, 2.2fr),
  inset: 5pt,
  align: (left, left, left),
  stroke: 0.5pt + rgb(180, 180, 180),
  fill: (x, y) => if y == 0 { rgb(240, 240, 240) } else { none },
  [*項目*], [*旧主張（Old Claim）*], [*訂正された現実（Corrected Reality）*],
  [1. 完全均一ビット幅],
  [TXビットセルは前半8cyc/後半8cyc厳密同期の完全均一幅である。],
  [【非均一ビットセル】標準版は通常16cyc/ワード境界18cyc、高速版は通常8cyc/ワード境界10cyc（+2サイクル伸長）。32bit境界でのOSR補充処理によるものであり、RXはエッジ同期で吸収する。],

  [2. $plus.minus$9.83 ns 保証],
  [実効ジッターマージン $plus.minus$9.83 ns が物理的・実機環境において保証される。],
  [【机上モデルであり未確立】$plus.minus$9.83 ns は名目値と対称立ち上がりを仮定した机上計算モデル（Calculation/Model）に過ぎず、実機環境（実デバイス）での保証値としては未確立（Not established）。実際の線路反射・PADばらつき等は実機特有の未知事項。],

  [3. 18.75 Mbps PIO/PHY 限界],
  [18.75 Mbps（8cyc/bit）が抵抗レス直結の物理・PIO命令両面の絶対限界である。],
  [【実験的探索点（EXPERIMENTAL）】現行命令列における探索動作点に過ぎず、チップおよび物理層の絶対的限界を確定・確立したものではない。],

  [4. SLOW+4mA 誤エッジゼロ],
  [`GPIO_SLEW_RATE_SLOW` ＋ 4mA 駆動によりリンギング反射波を平滑化し、誤エッジは物理的にゼロ。],
  [【未検証の仮定】スルーレート制限は反射波を抑制するが、「誤エッジゼロ」は理論的期待・前提（Unverified assumption）であり、実機環境で物理的に証明された事実ではない。],

  [5. 量産推奨],
  [9.375 Mbps（または特定レート）が商用・量産推奨レートである。],
  [【量産推奨は未確立】本PoCはブレッドボード上のジャンパ線直結実験であり、量産推奨（Production recommendation）は未確立（Not established）。リファクタリング後のファームウェアは実機ハードウェア検証未実施である。],

  [6. wait_sop への JMP による ISR フラッシュ誤認],
  [`pio_sm_exec(jmp(wait_sop))` により RX ISR の端数ビットを瞬時にフラッシュ可能。],
  [【JMPはPC変更のみ】`jmp` は PC を変更するだけで ISR 内容やシフトカウンタをクリアしない。真のフラッシュには `mov isr, null`（`pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_null))`）が必要。],
)

== 情報の厳密な4区分（認識論的ステータス）

- *(a) ユーザー報告による実機計測値（USER-REPORTED MEASUREMENTS）:*
  - Pico 2 W / RP2350、sysclk 150.0MHz、同一 GP0 <---> GP1 ジャンパ線直結
  - UART 3.000 Mbps: 162.5 kB/s, 11,389 pkts, CRC Err 0, Drops 0, Verify Err 0
  - UART 9.375 Mbps: 290.1 kB/s, 20,339 pkts, CRC Err 0, Drops 0, Verify Err 0
  - 旧単一レート BMC: 157.4 kB/s, 15,251 pkts, CRC Err 0, Drops 0
- *(b) ソースコード／実装から導出された事実（Code-Derived Facts）:*
  - PIO 命令長: 標準 TX 14/32, RX 17/32; 高速 TX 14/32, RX 18/32 (TX `pio0`, RX `pio1` 分離)
  - 非均一ビットセル: 標準 16/18cyc, 高速 8/10cyc
  - 高速 RX サンプリング: Cycle 6 (中央遷移 +2cyc, 次境界 -2cyc)
  - ワイヤフォーマット: PREAMBLE (`0xAAAA`, 2B) + SYNC (`0x93C7`, 2B) + ヘッダ + ペイロード + CRC16 + パディング ($1 <= P <= 4$, 先行 `0xFF`, 末尾 `0xFF`/`0xFE`)
  - `mov isr, null` による ISR およびシフトカウンタのクリア
- *(c) 計算／モデル（Calculation / Model）:*
  - 伝送線路分布定数パラメータ、名目ジッターマージン計算 ($plus.minus 9.83 "ns"$)
- *(d) 未検証の前提／実機特有の未知事項（Unverified Assumptions / Device-Only Unknowns）:*
  - リファクタリング後ファームウェアの実機ハードウェア動作（NOT hardware-tested）
  - 誤エッジゼロ仮説、実機環境におけるジッターマージン保証、量産適格性
