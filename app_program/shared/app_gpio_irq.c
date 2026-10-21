/** @file app_gpio_irq.c
 *  @brief IO_IRQ_BANK0 dispatcher implementation.
 */

#include "app_gpio_irq.h"

#include "app_hw.h"
#include "app_port.h"

#define APP_GPIO_IRQ_LEVEL                                                     \
    1 /* INTPRI_MAX_EXTINT_PRI: highest external level                         \
       */

typedef struct
{
    app_gpio_irq_cb_t callback;
    uint8_t           events; /* enabled event nibble, 0 = disabled */
} app_gpio_irq_slot_t;

static app_gpio_irq_slot_t g_slots[APP_GPIO_COUNT];
static bool                g_installed      = false;
static volatile uint32_t   g_dispatch_count = 0U;
static volatile uint32_t   g_spurious_count = 0U;

/* Task-independent context. Bounded: at most 30 callbacks per entry. */
static void
app_gpio_bank0_isr (UINT intno)
{
    uint8_t        status[APP_GPIO_COUNT];
    uint32_t const stamp = app_hw_time_us();
    uint32_t       pin;
    bool           any = false;

    (void) intno;
    g_dispatch_count++;
    app_hw_gpio_irq_take_all(status);

    for (pin = 0U; pin < APP_GPIO_COUNT; pin++)
    {
        uint32_t const events = status[pin];
        if (0U != events)
        {
            app_gpio_irq_cb_t const cb = g_slots[pin].callback;
            any                        = true;
            if (NULL != cb)
            {
                cb(pin, events, stamp);
            }
        }
    }
    if (!any)
    {
        g_spurious_count++;
    }
}

int32_t
app_gpio_irq_init (void)
{
    T_DINT   dint;
    ER       er;
    uint32_t pin;

    if (g_installed)
    {
        return E_OK;
    }
    for (pin = 0U; pin < APP_GPIO_COUNT; pin++)
    {
        g_slots[pin].callback = NULL;
        g_slots[pin].events   = 0U;
    }
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) app_gpio_bank0_isr;
    er          = tk_def_int(APP_INTNO_IO_IRQ_BANK0, &dint);
    if (er < E_OK)
    {
        return (int32_t) er;
    }
    EnableInt(APP_INTNO_IO_IRQ_BANK0, APP_GPIO_IRQ_LEVEL);
    g_installed = true;
    return E_OK;
}

bool
app_gpio_irq_attach (uint32_t pin, uint32_t event_mask,
                     app_gpio_irq_cb_t callback)
{
    uint32_t saved;

    if ((pin >= APP_GPIO_COUNT) || (NULL == callback) || !g_installed)
    {
        return false;
    }
    saved = app_hw_irq_save();
    if (NULL != g_slots[pin].callback)
    {
        app_hw_irq_restore(saved);
        return false; /* refuse to overwrite another owner's handler */
    }
    g_slots[pin].callback = callback;
    g_slots[pin].events   = (uint8_t) (event_mask & 0x0FU);
    app_hw_irq_restore(saved);
    app_hw_gpio_irq_enable(pin, event_mask);
    return true;
}

void
app_gpio_irq_disable (uint32_t pin)
{
    if (pin < APP_GPIO_COUNT)
    {
        app_hw_gpio_irq_enable(pin, 0U);
    }
}

void
app_gpio_irq_enable (uint32_t pin)
{
    if ((pin < APP_GPIO_COUNT) && (NULL != g_slots[pin].callback))
    {
        app_hw_gpio_irq_enable(pin, g_slots[pin].events);
    }
}

uint32_t
app_gpio_irq_dispatch_count (void)
{
    return g_dispatch_count;
}

uint32_t
app_gpio_irq_spurious_count (void)
{
    return g_spurious_count;
}
