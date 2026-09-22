#include "ota_protocol.h"
#include <string.h>

/*
 * IEEE 802.3 CRC32 Nibble Table (Polynomial: 0xEDB88320)
 * 64 bytes total, fast execution in SRAM/ROM.
 */
static const uint32_t OTA_CRC32_NIBBLE[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu
};

uint32_t BMC_SRAM_FUNC(ota_crc32_update)(uint32_t crc, const uint8_t *data, size_t length) {
    if (!data) return crc;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        crc = (crc >> 4) ^ OTA_CRC32_NIBBLE[crc & 0x0Fu];
        crc = (crc >> 4) ^ OTA_CRC32_NIBBLE[crc & 0x0Fu];
    }
    return crc;
}

uint32_t BMC_SRAM_FUNC(ota_crc32)(const uint8_t *data, size_t length) {
    uint32_t crc = OTA_CRC32_INIT;
    crc = ota_crc32_update(crc, data, length);
    return crc ^ 0xFFFFFFFFu;
}

uint16_t BMC_SRAM_FUNC(ota_bm_count_missing)(const uint8_t *bm, uint16_t total_chunks) {
    if (!bm || total_chunks == 0) return 0;
    uint16_t missing = 0;
    for (uint16_t i = 0; i < total_chunks; ++i) {
        if (!ota_bm_get(bm, i)) {
            missing++;
        }
    }
    return missing;
}

uint16_t BMC_SRAM_FUNC(ota_bm_find_next_missing)(const uint8_t *bm, uint16_t start_idx, uint16_t total_chunks) {
    if (!bm) return total_chunks;
    for (uint16_t i = start_idx; i < total_chunks; ++i) {
        if (!ota_bm_get(bm, i)) {
            return i;
        }
    }
    return total_chunks;
}
