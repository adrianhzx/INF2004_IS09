/** @file subsystem2_test.c
 *  @brief On-target tests for subsystem 2. Built only with: make TEST=1.
 *         STATUS: placeholder checks only (Phase 2 adds the real tests).
 *         Test IDs: S2-U01, ... (testing/subsystem2/TEST_LOG.md).
 */

#include "subsystem2.h"

#if APP_TEST

#include "../test_report.h"

EXPORT void
subsystem2_test (void)
{
    TEST_INIT();
    TEST_CHECK("S2-U00", E_OK == E_OK,
               "placeholder: subsystem 2 not implemented yet");
    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
