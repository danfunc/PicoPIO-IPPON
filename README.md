# PicoPIO-IPPON

**English** | [日本語](README.ja.md)

**One-wire point-to-point link for Raspberry Pi Pico 2 (RP2350) using PIO and Biphase Mark Code (differential Manchester), with firmware transfer (OTA) on top.**

A proof of concept of a **single-wire (plus GND) point-to-point link** that sends and receives Biphase Mark Code (BMC, differential Manchester) with the RP2350's PIO. With a plain jumper wire and no external components, it reaches 12.5–18.75 Mbps on real hardware, beyond the hardware UART's ceiling of 9.375 Mbps. On top of the link sits an OTA layer that transfers a firmware image of a few hundred KB into a flash staging area.

"IPPON" (一本) is Japanese for "one line".

> Status: **proof of concept**. All measurements were taken on a single Pico 2 W with GP0 looped back to GP1. Tests between separate boards (with independent clocks) have not been done yet.

## Measured results (one Pico 2 W, GP0↔GP1 jumper, sysclk 150 MHz)

Application-level throughput in continuous (burst) mode. Every mode ran with zero errors, zero drops and zero ring overruns.

| Mode | Line rate | Throughput |
|---|---|---|
| UART (PL011, for reference) | 9.375 Mbps | 672.4 KiB/s |
| BMC | 9.375 Mbps | 835.4 KiB/s |
| BMC | 12.5 Mbps | 1,107.0 KiB/s |
| BMC | 15 Mbps | 1,320.0 KiB/s |
| BMC (experimental, TX pad FAST slew + 8 mA) | 18.75 Mbps | 1,634.9 KiB/s |

- Round-trip time in stop-and-wait mode (send one frame, wait for it to be received): 41.3 µs at 18.75 Mbps.
- **OTA**: sending a 384 KB image in 64 KB windows, writing it to flash and verifying it by read-back takes 1.80 s (18.75 Mbps) to 2.00 s (9.375 Mbps). CRC matched, zero retransmissions. On a single board the sender also stalls while the receiver writes flash, so link time and flash writes do not overlap.
- **Flash (W25Q32)**: 4 KB erase 32.2 ms, 64 KB erase 88.5 ms, 4 KB program 7.23 ms, 256 B program 0.63 ms (measured on previously written regions).

## How it works

- **Wire format**: preamble `0xAAAA` + sync word `0x93C7` + [LEN][TYPE][SEQ][RSV][PAYLOAD 1–128 B][CRC16] + parity padding (so that every frame ends at the High level) + 4-byte postamble.
- **Receive**: the PIO follows boundary edges to decode bits, and DMA streams them into a 16 KB ring. The CPU searches for the sync word bit by bit, so **the receiver recovers on the next frame even after a bit-phase slip**.
- **Transmit**: the PIO state machine stays resident, and a double buffer encodes the next frame while the current one is on the wire (95–99% line utilization).
- Four PIO programs for 16 / 12 / 10 / 8 PIO cycles per bit (`bmc/`). TX runs on pio0 and RX on pio1.
- **OTA** (`ota/`): two 64 KB SRAM windows (double buffer), XNOR-style query for missing packets and retransmission, a per-window XIP read-back CRC32, and a final CRC32 over the whole image. It writes only to the staging area and **never switches to or boots the written image**.

Detailed design documents are in `docs/` (Typst sources and PDFs, in Japanese).

## Build

Pico SDK 2.2.0, `arm-none-eabi-gcc`, CMake and Ninja.

```sh
mkdir build && cd build
cmake -DPICO_SDK_PATH=$PICO_SDK_PATH -DPICO_BOARD=pico2_w -GNinja ..
ninja
```

Firmware images:

| File | Contents |
|---|---|
| `poc_benchmark.uf2` | BMC / UART benchmark (automatic sweep over all modes, single-shot send, diagnostic dump) |
| `poc_ota_bench.uf2` | OTA transfer over BMC (384 KB) |
| `poc_flash_bench.uf2` | Flash erase and program speed (staging area only) |
| `poc_bmc_loopback.uf2` / `poc_uart_loopback.uf2` | Standalone loopback tests |

Every image prints `[BUILD] <UTC> | git: <sha> | src-sha256: <hash>` at the top of its boot log, so you can tell which source a binary was built from.

**Wiring**: connect GP0 (TX) to GP1 (RX) with a single jumper wire. Control it over USB serial (115200). See `docs/benchmark_guide.md` for the key commands.

## Host tests

Tests cover packets, receive-frame extraction (all 32 bit phases × lengths 1–128), the ring buffer, the DMA counter, flash region guards, and OTA transfer (0 / 1 / 10% packet loss).

```sh
cd tests
gcc -std=c11 -Wall -Wextra -I.. -I../common test_packet.c ../common/packet.c ../common/crc16.c -o /tmp/test_packet && /tmp/test_packet
```

The other tests build the same way (they are also listed in `tests/CMakeLists.txt`).

## Not yet verified / limitations

- No tests between separate boards (independent clocks, long wires) yet. In loopback, transmitter and receiver share one clock, so phase conditions are ideal.
- In loopback, OTA replies (READY, lists of missing packets) return to the sender through an in-memory queue instead of the wire. The return path for a two-board setup (half duplex on the same wire, or a second wire) is not designed yet.
- 18.75 Mbps works only with the TX pad set to FAST slew and 8 mA drive.

## License

[GPL-3.0-or-later](LICENSE).

`pico_sdk_import.cmake` is copied from the Raspberry Pi Pico SDK and is licensed under BSD-3-Clause.
