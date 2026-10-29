/** @file integration_test.h
 *  @brief On-target integration run INT-13..INT-20 (make TEST=1): motor
 *         rotations and straight moves, IR line start, servo sweep, ultrasonic
 *         stop at 60 cm and barcode actions on the real car. Logged in
 *         testing/integration/TEST_LOG.md.
 */

#ifndef INTEGRATION_TEST_H
#define INTEGRATION_TEST_H

#include "../app.h"

/** Run INT-13..INT-20. Called from app_main.c after the subsystem tests and
 *  BEFORE app_shared_test() (SH-U08 leaves the system latched in S7). */
IMPORT void integration_test (void);

#endif /* INTEGRATION_TEST_H */
