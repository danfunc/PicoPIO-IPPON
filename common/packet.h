#ifndef PACKET_H
#define PACKET_H

#include "crc16.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 1. パケット種別定数
// ============================================================================
#define BMC_PKT_TYPE_ECHO_REQ 0x01
#define BMC_PKT_TYPE_ECHO_RES 0x02
#define BMC_PKT_TYPE_DATA     0x10
#define BMC_PKT_TYPE_BENCH    0x20

#define BMC_MAX_PAYLOAD_LEN   128
#define BMC_MIN_PAYLOAD_LEN   1

// ============================================================================
// 2. フレーム構造長定数
// ============================================================================
// パケットヘッダ長 (LEN, TYPE, SEQ, RESERVED)
#define BMC_HEADER_LEN 4
// CRC16 長 (CRC16-CCITT Big-Endian)
#define BMC_CRC_LEN 2

// 最大ポストシンクフレーム長: (4 + 128 + 2 + 3) & ~3 = 136 バイト (34 ワード)
#define BMC_MAX_FRAME_BYTES 136
#define BMC_MAX_FRAME_WORDS (BMC_MAX_FRAME_BYTES / 4)

// ============================================================================
// 3. プリアンブルおよびシンクワード定義
// ============================================================================
// プリアンブル (16bit: 0xAAAA -> '10101010 10101010b', 定常エッジによるクロック引き込み)
#define BMC_PREAMBLE_BYTE_1       0xAA
#define BMC_PREAMBLE_BYTE_2       0xAA

// シンクワード (16bit: 0x93C7 -> '10010011 11000111b', 鋭い自己相関ピーク & プリアンブル交差相関<=7/16)
#define BMC_SYNC_BYTE_1           0x93
#define BMC_SYNC_BYTE_2           0xC7
#define BMC_SYNC_WORD_16          0x93C7

// プリアンブル+シンク長: 4バイト (32bit, 1ワード)
#define BMC_PREAMBLE_SYNC_BYTES   4
#define BMC_PREAMBLE_SYNC_WORDS   (BMC_PREAMBLE_SYNC_BYTES / 4)

// ポストアンブル: RXのISR残留ビット問題対策 (B-3案b)。全1の32bit(4バイト)を追加送信し、
// フレーム末尾のビット位相に関わらず必ず1回のautopush境界を跨がせ、次フレームを待たずに
// 実データがpushされることを保証する。0ビットを含まないためパリティ不変条件は変化しない。
#define BMC_POSTAMBLE_BYTES       4
#define BMC_POSTAMBLE_WORDS       (BMC_POSTAMBLE_BYTES / 4)

// 送信 DMA バッファの最大ワード数:
// ヘッダワード1 + プリアンブル/シンク1 + 最大ポストシンクフレーム34 + ポストアンブル1 = 37ワード
#define BMC_TX_DMA_MAX_WORDS (1 + BMC_PREAMBLE_SYNC_WORDS + BMC_MAX_FRAME_WORDS + BMC_POSTAMBLE_WORDS)
#define BMC_TX_DMA_MAX_BYTES (BMC_TX_DMA_MAX_WORDS * 4)

// ============================================================================
// 4. 名前付きリザルト / エラーコード定数 (M-7)
// ============================================================================
#define BMC_OK                     0
#define BMC_ERR_BUFFER_TOO_SMALL  -1
#define BMC_ERR_INVALID_LEN       -2
#define BMC_ERR_FRAME_INCOMPLETE  -3
#define BMC_ERR_CRC_MISMATCH      -4
#define BMC_ERR_INVALID_ARG       -5
#define BMC_ERR_SYNC_NOT_FOUND    -6

// ============================================================================
// 5. パケット構造体定義
// ============================================================================
#pragma pack(push, 1)
typedef struct {
  uint8_t len;      // ペイロード実長 (1 <= N <= 128)
  uint8_t type;     // パケット種別
  uint8_t seq;      // シーケンス番号 (0-255)
  uint8_t reserved; // 予約フィールド (0x00)
  uint8_t payload[BMC_MAX_PAYLOAD_LEN];
} bmc_packet_t;
#pragma pack(pop)

// ============================================================================
// 6. 関数プロトタイプ宣言 (M-4: 実装は packet.c へ完全分離)
// ============================================================================

/**
 * @brief パケットをTX DMA用バッファにエンコードする
 *
 * @param dma_buf 送信バッファ (最低 BMC_TX_DMA_MAX_WORDS * 4 バイト, 4バイトアライメント)
 * @param type パケット種別
 * @param seq シーケンス番号
 * @param payload ペイロードデータ (payload_len > 0 のとき NULL 不可)
 * @param payload_len ペイロード長 (1..128)
 * @return size_t DMA送信総ワード数 (0 の場合はエラー)
 */
size_t bmc_packet_encode(uint32_t *dma_buf, uint8_t type, uint8_t seq,
                         const uint8_t *payload, uint8_t payload_len);

/**
 * @brief ポストシンクパケットの総バイト長 (32bit境界アライメントおよび極性調整後) を取得する
 *
 * @param payload_len ペイロード長 (1..128)
 * @return size_t ポストシンクフレーム総バイト長 (常に4の倍数)
 */
size_t bmc_packet_total_bytes(uint8_t payload_len);

/**
 * @brief 受信した生フレームからパケットをデコード・検証する (LEN を先頭とするバイト列)
 *
 * @param frame_bytes 受信フレームデータ (LEN を byte[0] とするアライメント済みバイト列)
 * @param max_bytes 受信バッファサイズ
 * @param out_type パケット種別出力先 (NULL可)
 * @param out_seq シーケンス番号出力先 (NULL可)
 * @param out_payload ペイロード出力先 (NULL可, 最低128バイト)
 * @param out_len ペイロード長出力先 (NULL可)
 * @return int 成功時: デコードした総フレームバイト数(>0)、失敗時: 負のエラーコード
 */
int bmc_packet_decode(const uint8_t *frame_bytes, size_t max_bytes,
                      uint8_t *out_type, uint8_t *out_seq,
                      uint8_t *out_payload, uint8_t *out_len);

/**
 * @brief バッファ内の任意ビット範囲から16bitシンクワード (0x93C7) をビット単位で探索する
 *
 * @param buf 探索対象バッファ (バイト配列)
 * @param buf_len_bytes バッファ有効バイト数
 * @param start_bit 探索開始ビット位置 (0-indexed from MSB of buf[0])
 * @param end_bit 探索終了ビット位置 (exclusive)
 * @param out_sync_end_bit シンクワード最終ビット位置 (0-indexed) 出力先
 * @return int 成功時 BMC_OK, 未検出時 BMC_ERR_SYNC_NOT_FOUND, 引数異常時 BMC_ERR_INVALID_ARG
 */
int bmc_find_sync_in_bits(const uint8_t *buf, size_t buf_len_bytes,
                          size_t start_bit, size_t end_bit,
                          size_t *out_sync_end_bit);

/**
 * @brief バッファ内の任意ビットオフセットから指定バイト数のデータを復元・抽出する
 *
 * @param buf 入力バッファ
 * @param buf_len_bytes バッファ有効バイト数
 * @param start_bit 抽出開始ビット位置 (0-indexed from MSB of buf[0])
 * @param num_bytes 抽出するバイト数
 * @param out_bytes 出力バイト配列
 * @return int 成功時 BMC_OK, バッファ終端超過時 BMC_ERR_BUFFER_TOO_SMALL, 引数異常時 BMC_ERR_INVALID_ARG
 */
int bmc_extract_bits_as_bytes(const uint8_t *buf, size_t buf_len_bytes,
                              size_t start_bit, size_t num_bytes,
                              uint8_t *out_bytes);

// ============================================================================
// 7. RX受信範囲計算コア (ハードウェア非依存の純粋関数、ホストテスト対象)
// ============================================================================

// bmc_rx_extract_frame_from_ring() が受理成功時に書き出すデコード済みフレーム
typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[BMC_MAX_PAYLOAD_LEN];
} bmc_rx_frame_t;

typedef enum {
    BMC_RX_EXTRACT_INCOMPLETE = 0,   // 現在の未読データでは判定不能 (フレーム未着、または全く足りない)
    BMC_RX_EXTRACT_SYNC_NOT_FOUND,   // 探索範囲内にシンクワードが見つからなかった
    BMC_RX_EXTRACT_INVALID_LEN,      // 偽シンク: LENフィールドが規定範囲外
    BMC_RX_EXTRACT_CRC_MISMATCH,     // CRC16不一致 (破損フレーム)
    BMC_RX_EXTRACT_FRAME_OK,         // フレーム受理成功
} bmc_rx_extract_status_t;

/**
 * @brief リングバッファ上の tail_bit と unread_bits から、受理可能なフレームを1件抽出する。
 *        ハードウェアレジスタ・DMA・構造体に一切依存しない純粋関数 (ホストテスト対象の核心部)。
 *
 *        バッファ原点 (byte 境界に切り下げた tail_bit) からの座標系で一貫してビット範囲を
 *        計算する: 有効ビット範囲は [tail_bit%8, tail_bit%8 + unread_bits) であり、
 *        コピーすべきバイト数は ceil((tail_bit%8 + unread_bits) / 8) である
 *        (旧実装は unread_bits をそのままバイト境界に切り捨てており、非バイト整列時に
 *        届いている末尾バイトを取りこぼすバグがあった)。
 *
 * @param ring_buf リングバッファ本体
 * @param ring_bytes リングバッファサイズ (バイト単位。2のべき乗であること)
 * @param tail_bit 読み出し開始ビット位置 (リング内の絶対ビット位置。呼出し前に正規化不要、
 *                 関数内部で ring_bytes*8 を法として畳み込む)
 * @param unread_bits tail_bit から数えた未読ビット数
 * @param out_frame 受理成功 (BMC_RX_EXTRACT_FRAME_OK) 時のフレーム格納先 (NULL可)
 * @param out_advance_bits 呼出し側が tail_bit を進めるべきビット数の出力先 (必須, 非NULL)。
 *                         戻り値の種別によらず、呼出し側は常にこの値だけ tail_bit と
 *                         total_bits_read を前進させてよい (0の場合は何もしない)
 * @return bmc_rx_extract_status_t 抽出結果
 */
bmc_rx_extract_status_t bmc_rx_extract_frame_from_ring(
    const uint8_t *ring_buf, size_t ring_bytes,
    size_t tail_bit, uint64_t unread_bits,
    bmc_rx_frame_t *out_frame,
    size_t *out_advance_bits);

// ============================================================================
// 8. DMA 転送語数・累計計算コア (ハードウェア非依存の純粋関数、ホストテスト対象)
// ============================================================================

/**
 * @brief DMA transfer_count (デクリメントカウンタ) と lap_count から
 *        エイリアシングのない真の累計転送語数を算出する純粋関数。
 *
 * RP2350 の DMA TRIGGER_SELF モードでは、転送毎に COUNT がデクリメントされ、
 * 0 に達すると reload_count へ自己再トリガしてリロードされる。
 *
 * 式: total_words = lap_count * reload_count + (reload_count - current_transfer_count)
 *
 * @param lap_count 自己再トリガ (ラップ) が発生した累計回数
 * @param current_transfer_count 現在の transfer_count レジスタ値 (COUNT部, 0..reload_count)
 * @param reload_count 1ラップあたりの転送ワード数 (RELOAD値)
 * @return uint64_t 累計転送ワード数
 */
uint64_t bmc_rx_calc_total_words(uint64_t lap_count, uint32_t current_transfer_count, uint32_t reload_count);

/**
 * @brief 前回のサンプリング点と今回のサンプリング点から、
 *        複数ラップにまたがる転送語数の差分 (delta words) を正しく積算する純粋関数。
 *
 * @param prev_lap_count 前回の lap_count
 * @param prev_transfer_count 前回の transfer_count
 * @param curr_lap_count 今回の lap_count (curr_lap_count >= prev_lap_count)
 * @param curr_transfer_count 今回の transfer_count
 * @param reload_count 1ラップあたりの転送ワード数
 * @return uint64_t サンプリング区間の転送ワード数 (delta)
 */
uint64_t bmc_rx_calc_delta_words(uint64_t prev_lap_count, uint32_t prev_transfer_count,
                                 uint64_t curr_lap_count, uint32_t curr_transfer_count,
                                 uint32_t reload_count);

/**
 * @brief ポーリング時に transfer_count の監視からラップを検知し、
 *        lap_count と累計転送語数を更新する純粋関数。
 *
 * COUNT がカウントダウンするため、curr_transfer_count > last_transfer_count であれば
 * 1回のリロード (ラップ) が発生したと判定する。
 * (前提: ポーリング間隔は reload_count / 2 相当の時間未満であること)
 *
 * @param inout_lap_count lap_count へのポインタ (ラップ検知時にインクリメント)
 * @param inout_last_transfer_count 直前の transfer_count へのポインタ (今回の値で更新)
 * @param curr_transfer_count 今回読み出した transfer_count レジスタ値
 * @param reload_count 1ラップあたりの転送ワード数
 * @return uint64_t 更新後の累計転送ワード数
 */
uint64_t bmc_rx_update_transfer_state(uint64_t *inout_lap_count,
                                      uint32_t *inout_last_transfer_count,
                                      uint32_t curr_transfer_count,
                                      uint32_t reload_count);

#ifdef __cplusplus
}
#endif

#endif // PACKET_H
