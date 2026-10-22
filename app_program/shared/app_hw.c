/** @file app_hw.c
 *  @brief RP2040 reg-level helpers shared by the subsystem drivers.
 *  See app_hw.h for the ownership statement.
 */

#include "app_hw.h"
#include "app_port.h"

#include <tk/tkernel.h> /* in_w/out_w/set_w/clr_w, DI/EI, RESETS_*, GPIO_*, PWM_* */

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

/* ---- Register map not present in the port's sysdef.h ------------------ */
#define APP_IO_BANK0_INTR0 (IO_BANK0_BASE + 0x0F0U) /* raw, W1C for edges */
#define APP_IO_BANK0_PROC0_INTE0 (IO_BANK0_BASE + 0x100U)
#define APP_IO_BANK0_PROC0_INTF0 (IO_BANK0_BASE + 0x110U)
#define APP_IO_BANK0_PROC0_INTS0 (IO_BANK0_BASE + 0x120U)
#define APP_IO_BANK0_REG_PINS 8U /* pins per 32-bit reg, 4 bits each */

#define APP_WATCHDOG_BASE 0x40058000U
#define APP_WATCHDOG_TICK (APP_WATCHDOG_BASE + 0x2CU)
#define APP_WATCHDOG_TICK_ENABLE (1U << 9U)
#define APP_WATCHDOG_TICK_RUNNING (1U << 10U)
#define APP_WATCHDOG_TICK_CYCLES_XOSC 12U /* 12 MHz XOSC -> 1 us tick */

#define APP_PWM_CSR_EN (1U << 0U)
#define APP_PWM_DIV_INT_SHIFT 4U
#define APP_PWM_DIV_FRAC_MASK 0x0FU
#define APP_PWM_SLICE_STRIDE 0x14U

#define APP_PWM_SLICE_REG(slice, off)                                          \
    (PWM_BASE + ((slice) * APP_PWM_SLICE_STRIDE) + (off))

#define APP_TIMER_RAWL TIMER_TIMERAWL

#define APP_RESET_WAIT_LOOPS 100000U

/* ---- GPIO -------------------------------------------------------------- */

void
app_hw_gpio_input (uint32_t pin, app_gpio_pull_t pull)
{
    uint32_t pad;

    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    pad =
        GPIO_IE | GPIO_SHEMITT | GPIO_DRIVE_4MA; /* input enable, schmitt on */
    if (APP_GPIO_PULL_UP == pull)
    {
        pad |= GPIO_PUE;
    }
    else if (APP_GPIO_PULL_DOWN == pull)
    {
        pad |= GPIO_PDE;
    }
    else
    {
        /* no pull */
    }
    out_w(GPIO_OE_CLR, 1UL << pin);
    out_w(GPIO(pin), pad);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
}

void
app_hw_gpio_output (uint32_t pin, bool level)
{
    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    app_hw_gpio_write(pin,
                      level); /* set the level before enabling the driver */
    out_w(GPIO(pin),
          GPIO_DRIVE_4MA | GPIO_SHEMITT); /* output: IE off, OD off */
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_OE_SET, 1UL << pin);
}

void
app_hw_gpio_write (uint32_t pin, bool level)
{
    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    if (level)
    {
        out_w(GPIO_OUT_SET, 1UL << pin);
    }
    else
    {
        out_w(GPIO_OUT_CLR, 1UL << pin);
    }
}

bool
app_hw_gpio_read (uint32_t pin)
{
    if (pin >= APP_GPIO_COUNT)
    {
        return false;
    }
    return 0U != (in_w(GPIO_IN) & (1UL << pin));
}

/* ---- GPIO interrupts (PROC0 = core 0, which runs every ISR on SMP=0) ---- */

static uint32_t
io_reg_index (uint32_t pin)
{
    return (pin / APP_IO_BANK0_REG_PINS) * 4U; /* byte offset of the pin reg */
}

static uint32_t
io_nibble_shift (uint32_t pin)
{
    return (pin % APP_IO_BANK0_REG_PINS) * 4U;
}

void
app_hw_gpio_irq_enable (uint32_t pin, uint32_t event_mask)
{
    uint32_t const idx   = io_reg_index(pin);
    uint32_t const shift = io_nibble_shift(pin);
    uint32_t const nib   = (event_mask & 0x0FU) << shift;

    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    if (0U == nib)
    {
        clr_w(APP_IO_BANK0_PROC0_INTE0 + idx, 0x0FUL << shift);
        return;
    }
    /* acknowledge stale edge events, then enable */
    out_w(APP_IO_BANK0_INTR0 + idx, (APP_GPIO_IRQ_EDGE_BOTH << shift));
    set_w(APP_IO_BANK0_PROC0_INTE0 + idx, nib);
}

void
app_hw_gpio_irq_force (uint32_t pin, uint32_t event_mask, bool on)
{
    uint32_t const idx   = io_reg_index(pin);
    uint32_t const shift = io_nibble_shift(pin);

    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    if (on)
    {
        set_w(APP_IO_BANK0_PROC0_INTF0 + idx, (event_mask & 0x0FU) << shift);
    }
    else
    {
        clr_w(APP_IO_BANK0_PROC0_INTF0 + idx, (event_mask & 0x0FU) << shift);
    }
}

void
app_hw_gpio_irq_take_all (uint8_t * p_status)
{
    uint32_t reg;
    uint32_t pin;

    for (reg = 0U; reg < 4U; reg++)
    {
        uint32_t const ints = in_w(APP_IO_BANK0_PROC0_INTS0 + (reg * 4U));
        /* W1C only the edge bits that are actually pending; level bits are
           read-only in INTR and clear when the level goes away. */
        uint32_t const edge_bits = ints & 0xCCCCCCCCUL;
        if (0U != edge_bits)
        {
            out_w(APP_IO_BANK0_INTR0 + (reg * 4U), edge_bits);
        }
        for (pin = 0U; pin < APP_IO_BANK0_REG_PINS; pin++)
        {
            uint32_t const gpio = (reg * APP_IO_BANK0_REG_PINS) + pin;
            if (gpio < APP_GPIO_COUNT)
            {
                p_status[gpio] = (uint8_t) ((ints >> (pin * 4U)) & 0x0FU);
            }
        }
    }
}

/* ---- PWM --------------------------------------------------------------- */

void
app_hw_pwm_config_slice (uint32_t slice, uint32_t div_int, uint32_t div_frac,
                         uint32_t top)
{
    uint32_t div;

    if (slice > 7U)
    {
        return;
    }
    if (0U == div_int)
    {
        div_int = 1U;
    }
    div = ((div_int & 0xFFU) << APP_PWM_DIV_INT_SHIFT) |
          (div_frac & APP_PWM_DIV_FRAC_MASK);
    clr_w(APP_PWM_SLICE_REG(slice, PWM_CHx_CSR), APP_PWM_CSR_EN);
    out_w(APP_PWM_SLICE_REG(slice, PWM_CHx_DIV), div);
    out_w(APP_PWM_SLICE_REG(slice, PWM_CHx_TOP), top & 0xFFFFU);
    out_w(APP_PWM_SLICE_REG(slice, PWM_CHx_CC), 0U); /* both channels low */
    out_w(APP_PWM_SLICE_REG(slice, PWM_CHx_CTR), 0U);
}

void
app_hw_pwm_enable (uint32_t slice, bool enable)
{
    if (slice > 7U)
    {
        return;
    }
    if (enable)
    {
        set_w(APP_PWM_SLICE_REG(slice, PWM_CHx_CSR), APP_PWM_CSR_EN);
    }
    else
    {
        clr_w(APP_PWM_SLICE_REG(slice, PWM_CHx_CSR), APP_PWM_CSR_EN);
    }
}

void
app_hw_pwm_set_level (uint32_t pin, uint32_t level)
{
    uint32_t const slice = app_hw_pwm_slice(pin);
    uint32_t const reg   = APP_PWM_SLICE_REG(slice, PWM_CHx_CC);
    uint32_t const cur   = in_w(reg);

    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    level &= 0xFFFFU;
    if (1U == app_hw_pwm_channel(pin))
    {
        out_w(reg, (cur & 0x0000FFFFUL) | (level << 16));
    }
    else
    {
        out_w(reg, (cur & 0xFFFF0000UL) | level);
    }
}

void
app_hw_pwm_pin (uint32_t pin)
{
    if (pin >= APP_GPIO_COUNT)
    {
        return;
    }
    out_w(GPIO(pin), GPIO_DRIVE_4MA | GPIO_SHEMITT); /* output pad, IE off */
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PWM);
}

/* ---- Time -------------------------------------------------------------- */

APP_RAMFUNC uint32_t
app_hw_time_us (void)
{
    return (uint32_t) in_w(APP_TIMER_RAWL);
}

void
app_hw_delay_us (uint32_t us)
{
    uint32_t const start = app_hw_time_us();
    uint32_t       guard = 0U;

    while (((uint32_t) (app_hw_time_us() - start)) < us)
    {
        /* Bounded even if the timer is dead: ~1 loop per 20 ns at 125 MHz
           means 50 iterations per us; cap at 10x the request. */
        guard++;
        if (guard > (us * 500U) + 1000U)
        {
            break;
        }
    }
}

/* ---- Interrupt masking ------------------------------------------------- */

#if TK_SUPPORT_SMP
static T_SPLOCK g_app_hw_lock;
static bool     g_app_hw_lock_init = false;
#endif

uint32_t
app_hw_irq_save (void)
{
#if TK_SUPPORT_SMP
    UINT intsts = 0U;
    if (!g_app_hw_lock_init)
    {
        (void) InitSpinLock(&g_app_hw_lock);
        g_app_hw_lock_init = true;
    }
    (void) ISpinLock(&g_app_hw_lock, &intsts);
    return (uint32_t) intsts;
#else
    UINT intsts;
    DI(intsts); /* disint(): PRIMASK=1, previous returned */
    return (uint32_t) intsts;
#endif
}

void
app_hw_irq_restore (uint32_t saved)
{
#if TK_SUPPORT_SMP
    (void) ISpinUnlock(&g_app_hw_lock, (UINT) saved);
#else
    EI((UINT) saved);
#endif
}

/* ---- Boot -------------------------------------------------------------- */

static bool
release_reset (uint32_t bit)
{
    uint32_t loops = 0U;

    if (0U == (in_w(RESETS_RESET) & bit))
    {
        /* already out of reset (never re-assert: another service may use it) */
        return 0U != (in_w(RESETS_RESET_DONE) & bit);
    }
    clr_w(RESETS_RESET, bit);
    while (0U == (in_w(RESETS_RESET_DONE) & bit))
    {
        loops++;
        if (loops > APP_RESET_WAIT_LOOPS)
        {
            return false;
        }
    }
    return true;
}

uint32_t
app_hw_init (void)
{
    uint32_t ok = 0U;
    uint32_t t0;
    uint32_t t1;
    uint32_t spin;

    if (release_reset(RESETS_RESET_IO_BANK0) &&
        release_reset(RESETS_RESET_PADS_BANK0))
    {
        ok |= APP_HW_OK_IO_RELEASED;
    }
    if (release_reset(RESETS_RESET_PWM))
    {
        ok |= APP_HW_OK_PWM_RELEASED;
    }

    /* The kernel port never releases TIMER nor starts the watchdog tick that
       clocks it (ledger section 2). Writing the same TICK value again is
       harmless if something already did. */
    (void) release_reset(RESETS_RESET_TIMER);
    out_w(APP_WATCHDOG_TICK,
          APP_WATCHDOG_TICK_ENABLE | APP_WATCHDOG_TICK_CYCLES_XOSC);

    /* Prove the microsecond counter advances: wait up to ~1 ms of CPU loops. */
    t0 = app_hw_time_us();
    for (spin = 0U; spin < 50000U; spin++)
    {
        t1 = app_hw_time_us();
        if (((uint32_t) (t1 - t0)) >= 5U)
        {
            ok |= APP_HW_OK_TIMER_RUNS;
            break;
        }
    }
    return ok;
}
