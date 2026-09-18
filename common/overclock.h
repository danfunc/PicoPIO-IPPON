#ifndef POC_OVERCLOCK_H
#define POC_OVERCLOCK_H

#include <stdbool.h>

/* Configure RP2350 clk_sys=300MHz, XIP flash=75MHz, clk_peri=150MHz. */
bool poc_configure_300mhz(void);

#endif
