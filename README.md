# PicoPIO-NIPPON

**Two-wire full-duplex point-to-point link for Raspberry Pi Pico 2 (RP2350) using PIO and Biphase Mark Code (differential Manchester), with firmware transfer (OTA) on top.**

[PicoPIO-IPPON](https://github.com/danfunc/PicoPIO-IPPON)（線 1 本・片方向）を、**線 2 本（+ GND）の全二重**に拡張する派生 PoC です。線 A（GP0→GP1）と線 B（GP2→GP3）をそれぞれ独立した BMC リンクとして同時に使い、片方向 IPPON 相当のモードも残したまま、双方向同時通信と、OTA の応答（READY・欠け一覧など）を別線で返す構成を目指します。

> 状態: **実装途中の PoC**。本 README 時点で実機検証は行っていません（GP0↔GP1・GP2↔GP3 の 2 本ジャンパでの動作確認、実測値の取得はこれから）。IPPON 側の実測値（下表）は本家 PoC のものであり、NIPPON 独自の値ではありません。NIPPON の全二重・OTA 応答分離の実測値は **未計測** です。数値は捏造していません。

## 実測値（Pico 2 W 1 枚、GP0↔GP1 直結、sysclk 150 MHz）

連続送信（Burst）のアプリ実効スループット。全モードでエラー・欠落・リング追越しは 0。

| モード | 線路レート | 実効スループット |
|---|---|---|
| UART（PL011、参考） | 9.375 Mbps | 672.4 KiB/s |
| BMC | 9.375 Mbps | 835.4 KiB/s |
| BMC | 12.5 Mbps | 1,107.0 KiB/s |
| BMC | 15 Mbps | 1,320.0 KiB/s |
| BMC（実験扱い、TX パッド FAST + 8 mA） | 18.75 Mbps | 1,634.9 KiB/s |

- 停止待ち（1 フレーム送って応答を待つ）の往復時間は、18.75 Mbps で 41.3 µs。
- **OTA**: 384 KB のイメージを 64 KB 窓で送り、flash に書いて読み戻し検証まで、1.80 s（18.75 Mbps）〜2.00 s（9.375 Mbps）。CRC 一致、再送 0。1 枚の基板では受信側の flash 書き込み中に送信側も止まるため、リンクと書き込みは重なっていません。
- **flash（W25Q32）**: 4 KB 消去 32.2 ms、64 KB 消去 88.5 ms、4 KB 書き込み 7.23 ms、256 B 書き込み 0.63 ms（書き込み済み領域での計測）。

## しくみ

- **線上フォーマット**: プリアンブル `0xAAAA` + 同期語 `0x93C7` + [LEN][TYPE][SEQ][RSV][PAYLOAD 1〜128 B][CRC16] + パリティパディング（フレーム終端を High にそろえる）+ ポストアンブル 4 B。
- **受信**: PIO が境界エッジに追従してビットを復号し、DMA で 16 KB のリングへ流す。CPU はビット単位で同期語を探すので、**ビット位相がずれても次のフレームで自力回復**する。
- **送信**: PIO の SM を常駐させ、ダブルバッファで次のフレームを符号化しながら送る（線路使用率 95〜99%）。
- 1 ビットあたり 16 / 12 / 10 / 8 PIO サイクルの 4 種類の PIO プログラム（`bmc/`）。TX は pio0、RX は pio1。
- **OTA**（`ota/`）: 64 KB 窓 × 2 面の SRAM ダブルバッファ、XNOR 方式の欠け問い合わせと再送、窓ごとの XIP 読み戻し CRC32、最後にイメージ全体の CRC32。書き込み先はステージング領域だけで、**書いたイメージへの切り替えや再起動はしない**。

詳しい設計は `docs/`（Typst と PDF、IPPON 時点のもの）にあります。

## NIPPON での変更点（現状）

- **ドライバの多重化**: `bmc/bmc_driver.c` の TX 常駐 SM は、送信完了通知を `irq set 0 rel`（相対 IRQ、実効フラグ番号 = SM 番号）に変更し、`bmc_tx_controller_t` ごとに待つ IRQ 番号を分離した。同じ pio0 に 2 つの TX SM を常駐させても完了フラグが衝突しない。GPIO のファンクション選択も `pio_get_funcsel()` を使う形にして pio0/1/2 のどれでも正しく動くようにした。RX 側 (`bmc_rx_port_t`) はもともと構造体インスタンスで完結しており、線ごとに別インスタンスを持てば追加変更なしで多重化できる。
- **OTA 応答喪失からの回復**: `ota_receiver.c` に直近送出 READY のキャッシュを追加し、`ota_sender.c` の WAIT_READY 待ちにタイムアウト再問い合わせ（QUERY 再送）を追加した。将来 OTA の応答（READY 等)を線 B で返す構成にしたとき、その応答パケット自体が失われても送り手が再問い合わせで復帰できる。
- **未実施（実機なし・時間予算の都合で見送り）**: 線 A・線 B を同時に使う全二重バーストの計測ファームウェア、OTA の応答を実際に線 B の物理 BMC フレームに載せて送るファームウェア一式は、まだ書いていない。上記のドライバ側の下地（IRQ 分離、構造体インスタンス化）は用意できているので、次の作業はそれらを組み上げる実体の `.c` ファイルと `CMakeLists.txt` への追加になる。

## ビルド

Pico SDK 2.2.0、`arm-none-eabi-gcc`、CMake、Ninja。

```sh
mkdir build && cd build
cmake -DPICO_SDK_PATH=$PICO_SDK_PATH -DPICO_BOARD=pico2_w -GNinja ..
ninja
```

できるファームウェア:

| ファイル | 内容 |
|---|---|
| `poc_benchmark.uf2` | BMC / UART の計測（全モードの自動スイープ、単発送信、診断ダンプ） |
| `poc_ota_bench.uf2` | BMC 上の OTA 転送の計測（384 KB） |
| `poc_flash_bench.uf2` | flash の消去・書き込み速度の計測（ステージング領域のみ） |
| `poc_bmc_loopback.uf2` / `poc_uart_loopback.uf2` | 単体のループバック |

起動ログの先頭に `[BUILD] <UTC> | git: <sha> | src-sha256: <hash>` が出るので、どのソースから作ったバイナリか確認できます。

**配線**: GP0（TX）と GP1（RX）をジャンパ線 1 本でつなぐだけです。USB シリアル（115200）でキー操作します。操作方法は `docs/benchmark_guide.md` を見てください。

## ホストテスト

パケット、受信フレームの抽出（ビット位相 32 通り × 長さ 1〜128）、リング、DMA カウンタ、flash 領域のガード、OTA 転送（パケット損失 0 / 1 / 10%）のテストがあります。

```sh
cd tests
gcc -std=c11 -Wall -Wextra -I.. -I../common test_packet.c ../common/packet.c ../common/crc16.c -o /tmp/test_packet && /tmp/test_packet
```

ほかのテストも同様です（`tests/CMakeLists.txt` にも登録済み）。

## 未検証・制約

- 別の基板どうし（独立したクロック、長い配線）での試験は未実施。ループバックでは送受信が同じクロックを使うので、位相条件が理想的です。
- OTA の応答（READY や欠けの一覧）は、ループバックでは線を通さず SRAM 上のキューで返しています。2 台構成の戻り経路（同じ線での半二重か、2 本目の線か）は未設計です。
- 18.75 Mbps は TX パッドを FAST スルー + 8 mA にしたときだけ成立します。

## ライセンス

[GPL-3.0-or-later](LICENSE)。

`pico_sdk_import.cmake` は Raspberry Pi の Pico SDK からのコピーで、BSD-3-Clause です。
