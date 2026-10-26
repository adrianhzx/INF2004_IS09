/** @file drv_line_adc.c
 *  @brief Polled ADC driver implementation. See drv_line_adc.h.
 */

#include "drv_line_adc.h"

#include "../shared/app_hw.h"
#include "../shared/app_port.h"

/* RP2040 ADC registers (datasheet 4.9.6), independent of the kernel driver. */
#define LADC_BASE 0x4004C000U
#define LADC_CS (LADC_BASE + 0x00U)
#define LADC_RESULT (LADC_BASE + 0x04U)
#define LADC_CS_EN (1U << 0U)
#define LADC_CS_START_ONCE (1U << 2U)
#define LADC_CS_READY (1U << 8U)
#define LADC_CS_ERR (1U << 9U)
#define LADC_CS_AINSEL_SHIFT 12U
#define LADC_CS_AINSEL_MASK (0x7U << LADC_CS_AINSEL_SHIFT)
#define LADC_RESULT_MASK 0x0FFFU
#define LADC_POLL_MAX 2000U /* ~40 us at 125 MHz: 20x a conversion */
#define LADC_ERR_SAT 0xFFU

static bool    g_ladc_ready  = false;
static uint8_t g_ladc_errors = 0U;

static void
analog_pin (uint32_t pin)
{
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
    clr_w(GPIO(pin),
          GPIO_IE | GPIO_PUE | GPIO_PDE); /* no digital input, no pulls */
}

bool
drv_line_adc_init (void)
{
    uint32_t loops = 0U;

    analog_pin(APP_PIN_LINE_LEFT);
    analog_pin(APP_PIN_LINE_CENTRE);
    out_w(LADC_CS, LADC_CS_EN); /* reset released at boot (hw_setting.c) */
    while (0U == (in_w(LADC_CS) & LADC_CS_READY))
    {
        loops++;
        if (loops > LADC_POLL_MAX)
        {
            g_ladc_ready = false;
            return false;
        }
    }
    g_ladc_ready = true;
    return true;
}

bool
drv_line_adc_read (drv_line_adc_ch_t channel, uint16_t * p_raw)
{
    uint32_t loops = 0U;
    uint32_t cs    = 0U;

    if (!g_ladc_ready || ((uint32_t) channel > (uint32_t) DRV_LINE_ADC_CENTRE))
    {
        return false;
    }
    out_w(LADC_CS, LADC_CS_EN |
                       (((uint32_t) channel << LADC_CS_AINSEL_SHIFT) &
                        LADC_CS_AINSEL_MASK) |
                       LADC_CS_START_ONCE);
    do
    {
        cs = in_w(LADC_CS);
        loops++;
        if (loops > LADC_POLL_MAX)
        {
            if (g_ladc_errors < LADC_ERR_SAT)
            {
                g_ladc_errors++;
            }
            return false;
        }
    } while (0U == (cs & LADC_CS_READY));

    if (0U != (cs & LADC_CS_ERR))
    {
        if (g_ladc_errors < LADC_ERR_SAT)
        {
            g_ladc_errors++;
        }
        return false;
    }
    *p_raw = (uint16_t) (in_w(LADC_RESULT) & LADC_RESULT_MASK);
    return true;
}

uint8_t
drv_line_adc_errors (void)
{
    return g_ladc_errors;
}
