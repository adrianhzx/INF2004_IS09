/** @file subsystem4_test.c
 *  @brief On-target tests for subsystem 4. Built only with: make TEST=1.
 *         STATUS: placeholder checks only (Phase 4 adds the real tests).
 *         Test IDs: S4-U01, ... (testing/subsystem4/TEST_LOG.md).
 */

#include "subsystem4.h"

#if APP_TEST

#include "../test_report.h"

EXPORT void
subsystem4_test (void)
{
    TEST_INIT();
    TEST_CHECK("S4-U00", E_OK == E_OK,
               "placeholder: subsystem 4 not implemented yet");
    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
