/** @file subsystem1_test.c
 *  @brief On-target tests for subsystem 1. Built only with: make TEST=1.
 *         STATUS: placeholder checks only (Phase 6 adds the real tests).
 *         Test IDs: S1-U01, ... (testing/subsystem1/TEST_LOG.md).
 */

#include "subsystem1.h"

#if APP_TEST

#include "../test_report.h"

EXPORT void
subsystem1_test (void)
{
    TEST_INIT();
    TEST_CHECK("S1-U00", E_OK == E_OK,
               "placeholder: subsystem 1 not implemented yet");
    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
