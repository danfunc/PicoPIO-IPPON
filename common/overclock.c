#include "overclock.h"

#include "hardware/clocks.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#if PICO_RP2350
#include "hardware/regs/qmi.h"
#include "hardware/structs/qmi.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#endif

#define POC_SYS_CLOCK_KHZ 300000u
#define POC_FLASH_DIV 4u
#define POC_FLASH_RXDELAY 3u

#if PICO_RP2350
static void __no_inline_not_in_flash_func(write_qmi_timing)(uint32_t timing) {
    qmi_hw->m[0].timing = timing;
    (void)qmi_hw->m[0].timing;
    __compiler_memory_barrier();
}
#endif

bool poc_configure_300mhz(void) {
#if !PICO_RP2350
    return false;
#else
    /* Set QMI before raising clk_sys so XIP stays at 75MHz throughout. */
    const uint32_t old_timing = qmi_hw->m[0].timing;
    const uint32_t timing =
        (old_timing & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
        (POC_FLASH_DIV << QMI_M0_TIMING_CLKDIV_LSB) |
        (POC_FLASH_RXDELAY << QMI_M0_TIMING_RXDELAY_LSB);
    uint32_t irq_state = save_and_disable_interrupts();
    write_qmi_timing(timing);
    bool qmi_ok = qmi_hw->m[0].timing == timing;
    restore_interrupts(irq_state);
    if (!qmi_ok) return false;

    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_us(2000);
    if (!set_sys_clock_khz(POC_SYS_CLOCK_KHZ, false) ||
        clock_get_hz(clk_sys) != POC_SYS_CLOCK_KHZ * 1000u) {
        irq_state = save_and_disable_interrupts();
        write_qmi_timing(old_timing);
        restore_interrupts(irq_state);
        return false;
    }

    const uint32_t sys_hz = clock_get_hz(clk_sys);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    sys_hz, sys_hz / 2u);
    return clock_get_hz(clk_peri) == sys_hz / 2u;
#endif
}
