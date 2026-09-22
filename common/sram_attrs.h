#ifndef SRAM_ATTRS_H
#define SRAM_ATTRS_H

// BMC通信のホットパス(符号化・CRC・受信抽出・RXポーリング・BurstのTXループ)を
// XIP(flash)実行から切り離しSRAM(.time_critical)に配置するための属性マクロ。
// コア1のXIPキャッシュ占有やXIPアクセス競合がコア0側ホットパスの実行時間に
// 影響する可能性を排除する目的 (静的解析による退行原因候補の一つ)。
// ホストテストビルド (PICO_ON_DEVICE 未定義/0) では何もしない恒等マクロとなり、
// 通常のホストコンパイラでもそのままビルドできる。
#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
#include "pico.h"
#define BMC_SRAM_FUNC(func_name) __not_in_flash_func(func_name)
#define BMC_SRAM_RODATA(group) __not_in_flash(group)
#else
#define BMC_SRAM_FUNC(func_name) func_name
#define BMC_SRAM_RODATA(group)
#endif

#endif // SRAM_ATTRS_H
