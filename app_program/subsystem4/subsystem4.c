/** @file subsystem4.c
 *  @brief Subsystem 4 placeholder: creates its kernel objects only.
 *         STATUS: NOT IMPLEMENTED (Phase 4). Motor-safe: no pin written.
 */

#include "subsystem4.h"


EXPORT ER
subsystem4_init (void)
{
    /* PLACEHOLDER: I2C0 / GP4-GP5 untouched until Phase 4. ImuState stays at
       its failsafe value (quality INVALID). */
    return E_OK;
}

EXPORT ER
subsystem4_start (void)
{
    /* PLACEHOLDER: no task yet. The real task is created in Phase 4. */
    return E_OK;
}
