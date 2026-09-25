#ifndef POC_OVERCLOCK_H
#define POC_OVERCLOCK_H

#include <stdbool.h>
#include <stdint.h>

/* Configure RP2350 clk_sys=300MHz, XIP flash=75MHz, clk_peri=150MHz. */
bool poc_configure_300mhz(void);

/* Configure RP2040 clk_sys to freq_mhz (150, 225, 250 MHz, etc.) with
 * appropriate VREG voltage scaling, wait stabilization, and clk_peri update. */
bool poc_configure_rp2040_clock(uint32_t freq_mhz);

/* Configure RP2040 clk_sys=150MHz (default is 125MHz) so BMC/PIO line rates
 * derived from clock_get_hz(clk_sys) match the RP2350 300MHz-side nominal
 * rates once the per-mode clock dividers are halved to compensate. */
bool poc_configure_rp2040_150mhz(void);

#endif
