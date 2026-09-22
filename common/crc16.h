#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// CRC-16-CCITT (多項式 0x1021, 初期値 0xFFFF)
uint16_t crc16_ccitt(const uint8_t *data, size_t length);

// 差分計算用
uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // CRC16_H

