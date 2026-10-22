/** @file app_port.h
 *  @brief Single include point for kernel services used by shared code, so
 *         the same sources compile on the target (micro T-Kernel) and in the
 *         host test build (APP_HOST_TEST, see testing/host/host_port.h).
 */

#ifndef APP_PORT_H
#define APP_PORT_H

#ifdef APP_HOST_TEST
#include "host_port.h"
#else
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#endif

/* Code that runs with interrupts masked is placed in SRAM (.data is copied
   from flash to RAM at reset by the pico_rp2040 linker script), so a masked
   window never stalls on an XIP flash cache miss (SH-U04c, Version B R1). */
#ifdef APP_HOST_TEST
#define APP_RAMFUNC
#else
#define APP_RAMFUNC __attribute__((section(".data.app_ramfunc"), noinline))
#endif

#endif /* APP_PORT_H */
