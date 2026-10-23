/** @file host_port.c
 *  @brief Host stubs for the kernel calls and the hardware layer used by the
 *         shared sources under test.
 */

#include "host_port.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_gpio_irq.h"
#include "app_hw.h"

ID       host_current_tid     = 1;
uint32_t host_flg_set_count   = 0U;
UINT     host_last_flg_ptn    = 0U;
uint32_t host_mbf_send_count  = 0U;
int      host_mbf_full        = 0;
uint32_t host_mbf_last_record = 0U;
uint32_t host_irq_depth       = 0U;
uint32_t host_irq_max_depth   = 0U;
void (*host_irq_hook)(void)   = 0;

#define HOST_FLG_ID 7
#define HOST_CYC_ID 3
#define HOST_TSK_ID 9
#define HOST_RECORD_LEN 4
#define HOST_TICK_US 7U

static uint32_t g_host_time_us = 0U;

ER
tk_set_flg (ID flgid, UINT setptn)
{
    (void) flgid;
    host_flg_set_count++;
    host_last_flg_ptn |= setptn;
    return E_OK;
}

ER
tk_wup_tsk (ID tskid)
{
    (void) tskid;
    return E_OK;
}

ER
tk_snd_mbf (ID mbfid, const void * msg, INT msgsz, TMO tmout)
{
    (void) mbfid;
    (void) tmout;
    if (host_mbf_full)
    {
        return E_TMOUT;
    }
    if (HOST_RECORD_LEN == msgsz)
    {
        memcpy(&host_mbf_last_record, msg, (size_t) HOST_RECORD_LEN);
    }
    host_mbf_send_count++;
    return E_OK;
}

ID
tk_get_tid (void)
{
    return host_current_tid;
}

ID
tk_cre_flg (const T_CFLG * pk)
{
    (void) pk;
    return HOST_FLG_ID;
}

ID
tk_cre_cyc (const T_CCYC * pk)
{
    (void) pk;
    return HOST_CYC_ID;
}

ER
tk_sta_cyc (ID cycid)
{
    (void) cycid;
    return E_OK;
}

ID
tk_cre_tsk (const T_CTSK * pk)
{
    (void) pk;
    return HOST_TSK_ID;
}

ER
tk_sta_tsk (ID tskid, INT stacd)
{
    (void) tskid;
    (void) stacd;
    return E_OK;
}

INT
tm_printf (const UB * fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    (void) vprintf((const char *) fmt, ap);
    va_end(ap);
    return 0;
}

/* ---- app_hw stubs ------------------------------------------------------ */

uint32_t
app_hw_irq_save (void)
{
    host_irq_depth++;
    if (host_irq_depth > host_irq_max_depth)
    {
        host_irq_max_depth = host_irq_depth;
    }
    return host_irq_depth - 1U;
}

void
app_hw_irq_restore (uint32_t saved)
{
    host_irq_depth = saved;
    if ((0U == host_irq_depth) && (0 != host_irq_hook))
    {
        void (*hook)(void) = host_irq_hook;
        host_irq_hook = 0; /* one shot: models the pended ISR running now */
        hook();
    }
}

uint32_t
app_hw_time_us (void)
{
    g_host_time_us += HOST_TICK_US;
    return g_host_time_us;
}

bool
app_hw_gpio_read (uint32_t pin)
{
    (void) pin;
    return true; /* buttons idle (active-low) */
}

void
app_hw_gpio_input (uint32_t pin, app_gpio_pull_t pull)
{
    (void) pin;
    (void) pull;
}

uint32_t host_gpio_in = 0U;

uint32_t
in_w (uint32_t addr)
{
    return (GPIO_IN == addr) ? host_gpio_in : 0U;
}

void
out_w (uint32_t addr, uint32_t data)
{
    (void) addr;
    (void) data;
}

bool
app_gpio_irq_attach (uint32_t pin, uint32_t event_mask,
                     app_gpio_irq_cb_t callback)
{
    (void) pin;
    (void) event_mask;
    return (0 != callback);
}
