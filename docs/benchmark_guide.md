# RP2350 P2P 通信プロトコル PoC & ベンチマーク

本ディレクトリは、`独自通信プロトコル.typ` に基づく RP2350 差動マンチェスタ（BMC 3.125 Mbps）通信プロトコルおよびハードウェア UART（3.0 Mbps）比較用のスタンドアロン PoC 実装です。

## ディレクトリ構成

- `common/`: 共通ライブラリ
  - `crc16.h` / `crc16.c`: CRC-16-CCITT (多項式 `0x1021`, 初期値 `0xFFFF`, テストベクトル `0x29B1` 検証済み)
  - `packet.h`: パケット構造体、DMA用アライメントエンコード・デコード
- `bmc/`: 独自 BMC プロトコル実装
  - `bmc_p2p.pio`: PIO TX (14命令) + RX (17命令) アセンブリ
  - `bmc_driver.h` / `bmc_driver.c`: 動的 TX MUX、自ピン読み戻し、1024B RX DMA リングバッファ
  - `main_bmc_loopback.c`: BMC ループバックベンチマーク本体
- `uart/`: 比較用ハードウェア UART 実装
  - `main_uart_loopback.c`: UART0 (3.0 Mbps) + TX/RX DMA ループバックベンチマーク本体
- `tests/`: ホストマシン用ユニットテスト
  - `test_packet.c`: CRC16、パケット境界値、改ざん検知テスト
  - `test_packet.c`: CRC16、パケット境界値、任意ビットオフセット(0..7)同期復旧、改ざん検知テスト
  - `test_ring.c`: リングバッファ境界ラップ、オーバーラン保護、カーソル進行テスト

---

## ホスト単体テストの実行（再現手順）

macOS / Linux 等のホスト環境上で、ターゲットハードウェアなしでプロトコル層・リングバッファの完全検証が可能です。
ビルドアーティファクトでリポジトリを汚さないよう、`/tmp` または `.gitignore` された作業ディレクトリを使用します。

```bash
# 1. テストビルド設定 (/tmp/poc-tests を使用)
cmake -S poc/tests -B /tmp/poc-tests -GNinja

# 2. ビルド
cmake --build /tmp/poc-tests

# 3. テスト実行
ctest --test-dir /tmp/poc-tests --output-on-failure
```

### 想定テスト名（Expected Test Names）
- **`test_packet`**: CRC16-CCITT ベクタ検証、任意ビット位相（0〜7ビットオフセット）からの Sync 再同期復元、破損パケット検知
- **`test_ring`**: リングカーソルの境界ラップアラウンド、オーバーラン検知と最新データ維持

※ 生成ディレクトリ（`/tmp/poc-tests`、各ビルドディレクトリ内の `generated/` など）は Git コミット対象外です。

---

## ハードウェア配線 (ループバックテスト)

**BMC版・UART版ともに共通の配線でテスト可能です。ジャンパー線の差し替えは不要です。**

- **GP0 (TX / UART0 TX, Pin 1)** $\longleftrightarrow$ **GP1 (RX / UART0 RX, Pin 2)** をジャンパー線 1 本で直結します。
  - Pico 2 ボード端の隣り合うピン（Pin 1 と Pin 2）をショートさせるだけで準備完了です。
  - 平時は TX ピンが High (3.3V) を能動出力し、RX ピンは内部プルダウン＋シュミットトリガでノイズを防止します。

---

## ビルド方法

ビルド環境: Pico SDK 2.2.0, `arm-none-eabi-gcc`, `cmake`, `ninja`

```bash
cd poc
mkdir -p build && cd build
cmake -DPICO_SDK_PATH=$PICO_SDK_PATH -DPICO_BOARD=pico2 -GNinja ..
ninja
```

ビルド完了後、`build/` 配下に以下のバイナリが生成されます:
- **`poc_benchmark.uf2` (推奨: BMC/UART 動的切り替え対応 統合ベンチマーク)**
- `poc_bmc_loopback.uf2` (BMC 単体ループバック)
- `poc_uart_loopback.uf2` (UART 単体ループバック)

---

## 実行およびモニタリング

1. Raspberry Pi Pico 2 の `BOOTSEL` ボタンを押しながら PC に USB 接続します。
2. マウントされた `RPI-RP2` ドライブに **`poc_benchmark.uf2`** をドラッグ＆ドロップします。
3. シリアルターミナルを開きます:
   ```bash
   minicom -D /dev/tty.usbmodem*
   # または screen /dev/tty.usbmodem* 115200
   ```

### リアルタイム・キーボード操作コマンド
シリアルターミナルを開いた状態で、以下のキーを押すと動作モードが即座に切り替わります：

| キー | 動作 |
|---|---|
| **`b`** または **`1`** | **BMC モード (3.125 Mbps PIO)** に切り替え |
| **`u`** または **`2`** | **UART モード (3.0 Mbps DMA)** に切り替え |
| **Space** | モードをトグル切り替え (BMC $\leftrightarrow$ UART) |
| **`r`** | 統計カウンタをリセット |

### リアルタイム出力画面例
```text
==================================================
  RP2350 Loopback Benchmark: BMC vs UART (Unified)
  Hardware: Connect GP0 (Pin 1) <---> GP1 (Pin 2)
  Commands (Press key in serial terminal):
    [b] Switch to BMC Mode  (3.125 Mbps PIO)
    [u] Switch to UART Mode (3.0 Mbps DMA)
    [ ] (Space) Toggle Mode
    [r] Reset Statistics Counter
==================================================

[INIT] Started in [BMC Mode] (3.125 Mbps)
[BMC (3.125M)] TX:  1182 pkts ( 59.8 kB/s) | RX:  1182 pkts ( 59.8 kB/s) | CRC Err: 0 | Drops: 0 | Verify Err: 0
[BMC (3.125M)] TX:  2365 pkts ( 60.0 kB/s) | RX:  2365 pkts ( 60.0 kB/s) | CRC Err: 0 | Drops: 0 | Verify Err: 0

>>> Switched to [UART Mode] (3.0 Mbps, GP0/GP1) <<<

[UART (3.0M) ] TX:  1041 pkts ( 52.6 kB/s) | RX:  1041 pkts ( 52.6 kB/s) | CRC Err: 0 | Drops: 0 | Verify Err: 0
[UART (3.0M) ] TX:  2082 pkts ( 52.8 kB/s) | RX:  2082 pkts ( 52.8 kB/s) | CRC Err: 0 | Drops: 0 | Verify Err: 0
```

