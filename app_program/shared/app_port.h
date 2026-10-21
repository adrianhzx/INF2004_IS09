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

#endif /* APP_PORT_H */
