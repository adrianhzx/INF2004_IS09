/** @file subsystem2.h
 *  @brief Subsystem 2 public interface: motion and encoders
 *         (MotionControlTask, priority 2, 10 ms; EncoderIsr on GP0-GP3).
 *         Owner: Buddy 2 S Reuben Kanna 2500695.
 *
 *  Owned objects: MBF_MOTION_CMD (4 x 4-byte records, 32 B with the kernel
 *  header, D-07), PWM slices 4/5 (GP8-GP11), dispatcher callbacks GP0-GP3.
 *  Published: g_motion_status_a/b, g_enc_ticks_l/r (shared/app_motion.h).
 *  Other subsystems include this as "../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID
    mbf_motion_cmd;   /* IF-MC: VehicleControllerTask sends packed commands */
IMPORT ID tid_motion; /* MotionControlTask (CYC_10MS slot MOTION) */

/* Task body and one cycle (the cycle is exposed for the on-target tests). */
IMPORT void     s2_motion_task (INT stacd, void * p_exinf);
IMPORT void     s2_motion_cycle (void);
IMPORT uint32_t s2_motion_cycle_count (void);
IMPORT uint32_t s2_motion_max_cycle_us (void);
IMPORT uint32_t s2_motion_latch_cycles (void);
IMPORT uint8_t  s2_motion_state (void);

#endif /* SUBSYSTEM2_H */
