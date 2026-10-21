/** @file subsystem5_test.c
 *  @brief On-target tests for subsystem 5. Built only with: make TEST=1.
 *         STATUS: placeholder checks only (Phase 5 adds the real tests).
 *         Test IDs: S5-U01, ... (testing/subsystem5/TEST_LOG.md).
 */

#include "subsystem5.h"

#if APP_TEST

#include "../test_report.h"

EXPORT void
subsystem5_test (void)
{
    TEST_INIT();
    TEST_CHECK("S5-U00", E_OK == E_OK,
               "placeholder: subsystem 5 not implemented yet");
    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
