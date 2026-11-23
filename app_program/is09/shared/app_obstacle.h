/** @file app_obstacle.h
 *  @brief ObstacleProfile (two words, mutex copy) and the echo capture
 *         protocol (owner: Buddy 5, subsystem5). Layout: app_data.h.
 *
 *  ObstacleProfile exceeds one word, so the writer publishes under
 *  MTX_OBSTACLE (TA_INHERIT) and readers copy under the same mutex: two
 *  independent lock-free loads are NOT a coherent read (Rev B rule 3).
 *
 *  Echo capture: EchoIsr is the only writer of g_echo_width_us and of
 *  g_echo_status. The task arms a capture by storing APP_ECHO_ARMED into
 *  g_echo_status (task is the only writer of that value); the ISR stores the
 *  width and then APP_ECHO_DONE (or APP_ECHO_OVERRUN). Each field is one
 *  aligned halfword/byte, so no RMW crosses the ISR/task boundary.
 */

#ifndef APP_OBSTACLE_H
#define APP_OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_pack.h"

/* ---- echo capture (ISR <-> task) ------------------------------------- */
#define APP_ECHO_IDLE 0U
#define APP_ECHO_ARMED 1U   /* task: waiting for rising edge */
#define APP_ECHO_RISING 2U  /* ISR: rising edge seen, waiting for falling */
#define APP_ECHO_DONE 3U    /* ISR: width valid */
#define APP_ECHO_OVERRUN 4U /* ISR: width > 30 ms or sequence error */

extern volatile uint16_t g_echo_width_us; /* 0..30000 */
extern volatile uint8_t  g_echo_status;   /* APP_ECHO_* */

/* ---- ObstacleProfile ---------------------------------------------------- */
extern volatile uint32_t
    g_obstacle_profile[APP_OBSTACLE_WORDS]; /* read/written under MTX_OBSTACLE
                                             */

typedef struct
{
    uint16_t range_cm;       /* 0..511, 511 = sentinel */
    uint8_t  scan_angle_idx; /* 0..36 (5 deg steps) */
    uint8_t  width_cm;       /* 0..63 */
    int8_t   y_min_cm;       /* -64..63 */
    int8_t   y_max_cm;       /* -64..63 */
    uint16_t closest_cm;     /* 0..511 */
    uint8_t  action;         /* app_avoid_action_t */
    uint8_t  scan_state;     /* 0..7 */
    uint8_t  seq;            /* 0..15 profile sequence */
} app_obstacle_profile_t;

static inline bool
app_obstacle_pack (app_obstacle_profile_t const * p_view, uint32_t * p_w0,
                   uint32_t * p_w1)
{
    bool     sat = false;
    uint32_t w0  = 0U;
    uint32_t w1  = 0U;
    w0           = app_pack_set_sat(w0, APP_OP0_RANGE_SHIFT, APP_RANGE_CM_BITS,
                                    p_view->range_cm, &sat);
    w0           = app_pack_set_sat(w0, APP_OP0_ANGLE_SHIFT, APP_OP0_ANGLE_BITS,
                                    p_view->scan_angle_idx, &sat);
    w0           = app_pack_set_sat(w0, APP_OP0_WIDTH_SHIFT, APP_OP0_WIDTH_BITS,
                                    p_view->width_cm, &sat);
    w0 = app_pack_set_s(w0, APP_OP0_YMIN_SHIFT, APP_OP_Y_BITS, p_view->y_min_cm,
                        &sat);
    w1 = app_pack_set_s(w1, APP_OP1_YMAX_SHIFT, APP_OP_Y_BITS, p_view->y_max_cm,
                        &sat);
    w1 = app_pack_set_sat(w1, APP_OP1_CLOSEST_SHIFT, APP_RANGE_CM_BITS,
                          p_view->closest_cm, &sat);
    w1 = app_pack_set_sat(w1, APP_OP1_ACTION_SHIFT, APP_OP1_ACTION_BITS,
                          p_view->action, &sat);
    w1 = app_pack_set_sat(w1, APP_OP1_SCANST_SHIFT, APP_OP1_SCANST_BITS,
                          p_view->scan_state, &sat);
    w1 = app_pack_set(w1, APP_OP1_SEQ_SHIFT, APP_OP1_SEQ_BITS, p_view->seq);
    *p_w0 = w0;
    *p_w1 = w1;
    return sat;
}

static inline void
app_obstacle_unpack (uint32_t w0, uint32_t w1, app_obstacle_profile_t * p_view)
{
    p_view->range_cm =
        (uint16_t) app_pack_get(w0, APP_OP0_RANGE_SHIFT, APP_RANGE_CM_BITS);
    p_view->scan_angle_idx =
        (uint8_t) app_pack_get(w0, APP_OP0_ANGLE_SHIFT, APP_OP0_ANGLE_BITS);
    p_view->width_cm =
        (uint8_t) app_pack_get(w0, APP_OP0_WIDTH_SHIFT, APP_OP0_WIDTH_BITS);
    p_view->y_min_cm =
        (int8_t) app_pack_get_s(w0, APP_OP0_YMIN_SHIFT, APP_OP_Y_BITS);
    p_view->y_max_cm =
        (int8_t) app_pack_get_s(w1, APP_OP1_YMAX_SHIFT, APP_OP_Y_BITS);
    p_view->closest_cm =
        (uint16_t) app_pack_get(w1, APP_OP1_CLOSEST_SHIFT, APP_RANGE_CM_BITS);
    p_view->action =
        (uint8_t) app_pack_get(w1, APP_OP1_ACTION_SHIFT, APP_OP1_ACTION_BITS);
    p_view->scan_state =
        (uint8_t) app_pack_get(w1, APP_OP1_SCANST_SHIFT, APP_OP1_SCANST_BITS);
    p_view->seq =
        (uint8_t) app_pack_get(w1, APP_OP1_SEQ_SHIFT, APP_OP1_SEQ_BITS);
}

/** Echo width (us) to range (cm) with the 511 sentinel. 58 us/cm (HC-SR04). */
static inline uint16_t
app_obstacle_us_to_cm (uint32_t width_us)
{
    uint32_t const cm = width_us / 58U;
    return (cm >= APP_RANGE_CM_SENTINEL) ? (uint16_t) APP_RANGE_CM_SENTINEL
                                         : (uint16_t) cm;
}

#endif /* APP_OBSTACLE_H */
