/** @file subsystem3_test.c
 *  @brief On-target tests for subsystem 3. Built only with: make TEST=1.
 *         STATUS: placeholder checks only (Phase 3 adds the real tests).
 *         Test IDs: S3-U01, ... (testing/subsystem3/TEST_LOG.md).
 */

#include "subsystem3.h"

#if APP_TEST

#include "../test_report.h"

EXPORT void
subsystem3_test (void)
{
    TEST_INIT();
    TEST_CHECK("S3-U00", E_OK == E_OK,
               "placeholder: subsystem 3 not implemented yet");
    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
