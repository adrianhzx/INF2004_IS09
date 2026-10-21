/** @file app_line.h
 *  @brief LineState word (owner: Buddy 3, subsystem3). Layout: app_data.h.
 */

#ifndef APP_LINE_H
#define APP_LINE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_pack.h"

/** Published word (writer: LineSensingTask, one aligned store). */
extern volatile uint32_t g_line_state;

typedef struct
{
    int8_t  line_error; /* -100..+100, + = line right of centre */
    int8_t  steer;      /* -100..+100 steering demand */
    bool    on_line;
    uint8_t event; /* app_line_event_t */
    uint8_t seq;   /* 0..15, increments every 10 ms publish */
} app_line_state_t;

static inline bool
app_line_pack (app_line_state_t const * p_view, uint32_t * p_word)
{
    bool     sat = false;
    uint32_t w   = 0U;
    w            = app_pack_set_s(w, APP_LS_ERROR_SHIFT, APP_LS_INT8_BITS,
                                  p_view->line_error, &sat);
    w = app_pack_set_s(w, APP_LS_STEER_SHIFT, APP_LS_INT8_BITS, p_view->steer,
                       &sat);
    w = app_pack_set(w, APP_LS_ON_LINE_SHIFT, 1U, p_view->on_line ? 1U : 0U);
    w = app_pack_set_sat(w, APP_LS_EVENT_SHIFT, APP_LS_EVENT_BITS,
                         p_view->event, &sat);
    w = app_pack_set(w, APP_LS_SEQ_SHIFT, APP_LS_SEQ_BITS, p_view->seq);
    *p_word = w;
    return sat;
}

static inline void
app_line_unpack (uint32_t word, app_line_state_t * p_view)
{
    p_view->line_error =
        (int8_t) app_pack_get_s(word, APP_LS_ERROR_SHIFT, APP_LS_INT8_BITS);
    p_view->steer =
        (int8_t) app_pack_get_s(word, APP_LS_STEER_SHIFT, APP_LS_INT8_BITS);
    p_view->on_line = (0U != app_pack_get(word, APP_LS_ON_LINE_SHIFT, 1U));
    p_view->event =
        (uint8_t) app_pack_get(word, APP_LS_EVENT_SHIFT, APP_LS_EVENT_BITS);
    p_view->seq =
        (uint8_t) app_pack_get(word, APP_LS_SEQ_SHIFT, APP_LS_SEQ_BITS);
}

static inline void
app_line_read (app_line_state_t * p_view)
{
    app_line_unpack(g_line_state, p_view);
}

#endif /* APP_LINE_H */
