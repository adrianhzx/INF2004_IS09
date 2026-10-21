/** @file app_motion.h
 *  @brief MotionStatus words, MBF_MOTION_CMD records and encoder counters
 *         (owner: Buddy 2, subsystem2). Layout authority: app_data.h.
 */

#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_pack.h"

/* ---- Published words (writer: MotionControlTask; one aligned store each) */
extern volatile uint32_t g_motion_status_a; /* speeds, distance, cycle_tag */
extern volatile uint32_t g_motion_status_b; /* state, tag, seqs, dirs, result */

/* ---- Encoder counters (writer: EncoderIsr only; readers take a halfword) */
extern volatile uint16_t g_enc_ticks_l;
extern volatile uint16_t g_enc_ticks_r;

/** Unpacked view (task-private working copy). */
typedef struct
{
    uint8_t speed_l_cm_s; /* 0..127 */
    uint8_t speed_r_cm_s; /* 0..127 */
    int16_t distance_mm;  /* signed run distance */
    uint8_t cycle_tag;    /* 0..3 */
    uint8_t motion_state; /* app_motion_state_t */
    uint8_t done_seq;     /* 0..15 */
    uint8_t active_seq;   /* 0..15 */
    bool    dir_l_reverse;
    bool    dir_r_reverse;
    uint8_t done_result; /* app_motion_result_t */
} app_motion_status_t;

/** Pack both words from a view. Returns true if any field saturated. */
static inline bool
app_motion_pack (app_motion_status_t const * p_view, uint32_t * p_word_a,
                 uint32_t * p_word_b)
{
    bool     sat = false;
    uint32_t a   = 0U;
    uint32_t b   = 0U;

    a = app_pack_set_sat(a, APP_MSA_SPEED_L_SHIFT, APP_MSA_SPEED_BITS,
                         p_view->speed_l_cm_s, &sat);
    a = app_pack_set_sat(a, APP_MSA_SPEED_R_SHIFT, APP_MSA_SPEED_BITS,
                         p_view->speed_r_cm_s, &sat);
    a = app_pack_set_s(a, APP_MSA_DIST_SHIFT, APP_MSA_DIST_BITS,
                       p_view->distance_mm, &sat);
    a = app_pack_set(a, APP_MSA_TAG_SHIFT, APP_MSA_TAG_BITS, p_view->cycle_tag);

    b = app_pack_set_sat(b, APP_MSB_STATE_SHIFT, APP_MSB_STATE_BITS,
                         p_view->motion_state, &sat);
    b = app_pack_set(b, APP_MSB_TAG_SHIFT, APP_MSB_TAG_BITS, p_view->cycle_tag);
    b = app_pack_set(b, APP_MSB_DONE_SHIFT, APP_MSB_DONE_BITS,
                     p_view->done_seq);
    b = app_pack_set(b, APP_MSB_ACTIVE_SHIFT, APP_MSB_ACTIVE_BITS,
                     p_view->active_seq);
    b = app_pack_set(b, APP_MSB_DIR_L_SHIFT, 1U,
                     p_view->dir_l_reverse ? 1U : 0U);
    b = app_pack_set(b, APP_MSB_DIR_R_SHIFT, 1U,
                     p_view->dir_r_reverse ? 1U : 0U);
    b = app_pack_set(b, APP_MSB_RESULT_SHIFT, APP_MSB_RESULT_BITS,
                     p_view->done_result);

    *p_word_a = a;
    *p_word_b = b;
    return sat;
}

/** Unpack two words into a view. */
static inline void
app_motion_unpack (uint32_t word_a, uint32_t word_b,
                   app_motion_status_t * p_view)
{
    p_view->speed_l_cm_s = (uint8_t) app_pack_get(word_a, APP_MSA_SPEED_L_SHIFT,
                                                  APP_MSA_SPEED_BITS);
    p_view->speed_r_cm_s = (uint8_t) app_pack_get(word_a, APP_MSA_SPEED_R_SHIFT,
                                                  APP_MSA_SPEED_BITS);
    p_view->distance_mm =
        (int16_t) app_pack_get_s(word_a, APP_MSA_DIST_SHIFT, APP_MSA_DIST_BITS);
    p_view->cycle_tag =
        (uint8_t) app_pack_get(word_a, APP_MSA_TAG_SHIFT, APP_MSA_TAG_BITS);
    p_view->motion_state =
        (uint8_t) app_pack_get(word_b, APP_MSB_STATE_SHIFT, APP_MSB_STATE_BITS);
    p_view->done_seq =
        (uint8_t) app_pack_get(word_b, APP_MSB_DONE_SHIFT, APP_MSB_DONE_BITS);
    p_view->active_seq = (uint8_t) app_pack_get(word_b, APP_MSB_ACTIVE_SHIFT,
                                                APP_MSB_ACTIVE_BITS);
    p_view->dir_l_reverse =
        (0U != app_pack_get(word_b, APP_MSB_DIR_L_SHIFT, 1U));
    p_view->dir_r_reverse =
        (0U != app_pack_get(word_b, APP_MSB_DIR_R_SHIFT, 1U));
    p_view->done_result = (uint8_t) app_pack_get(word_b, APP_MSB_RESULT_SHIFT,
                                                 APP_MSB_RESULT_BITS);
}

#define APP_MOTION_READ_ATTEMPTS 2U

/** Coherent read of both words (D-04): re-reads once if the tags differ.
 *  Returns false if the pair is still mixed after the retry (caller treats
 *  as stale for this cycle). */
static inline bool
app_motion_read (app_motion_status_t * p_view)
{
    uint32_t a       = 0U;
    uint32_t b       = 0U;
    uint32_t attempt = 0U;

    for (attempt = 0U; attempt < APP_MOTION_READ_ATTEMPTS; attempt++)
    {
        a = g_motion_status_a;
        b = g_motion_status_b;
        if (app_pack_get(a, APP_MSA_TAG_SHIFT, APP_MSA_TAG_BITS) ==
            app_pack_get(b, APP_MSB_TAG_SHIFT, APP_MSB_TAG_BITS))
        {
            app_motion_unpack(a, b, p_view);
            return true;
        }
    }
    app_motion_unpack(g_motion_status_a, g_motion_status_b, p_view);
    return false;
}

/* ---- MBF_MOTION_CMD record ------------------------------------------- */
typedef struct
{
    uint8_t type;       /* app_motion_cmd_type_t */
    uint8_t speed_cm_s; /* 0..127 */
    int16_t param;      /* MOVE: mm, TURN: 0.1 deg, else 0 */
    bool    until_line;
    uint8_t seq; /* 0..15 */
} app_motion_cmd_t;

static inline bool
app_motion_cmd_pack (app_motion_cmd_t const * p_cmd, uint32_t * p_record)
{
    bool     sat = false;
    uint32_t w   = 0U;
    w = app_pack_set_sat(w, APP_MC_TYPE_SHIFT, APP_MC_TYPE_BITS, p_cmd->type,
                         &sat);
    w = app_pack_set_sat(w, APP_MC_SPEED_SHIFT, APP_MC_SPEED_BITS,
                         p_cmd->speed_cm_s, &sat);
    w = app_pack_set_s(w, APP_MC_PARAM_SHIFT, APP_MC_PARAM_BITS, p_cmd->param,
                       &sat);
    w = app_pack_set(w, APP_MC_UNTIL_LINE_SHIFT, 1U,
                     p_cmd->until_line ? 1U : 0U);
    w = app_pack_set(w, APP_MC_SEQ_SHIFT, APP_MC_SEQ_BITS, p_cmd->seq);
    *p_record = w;
    return sat;
}

static inline void
app_motion_cmd_unpack (uint32_t record, app_motion_cmd_t * p_cmd)
{
    p_cmd->type =
        (uint8_t) app_pack_get(record, APP_MC_TYPE_SHIFT, APP_MC_TYPE_BITS);
    p_cmd->speed_cm_s =
        (uint8_t) app_pack_get(record, APP_MC_SPEED_SHIFT, APP_MC_SPEED_BITS);
    p_cmd->param =
        (int16_t) app_pack_get_s(record, APP_MC_PARAM_SHIFT, APP_MC_PARAM_BITS);
    p_cmd->until_line =
        (0U != app_pack_get(record, APP_MC_UNTIL_LINE_SHIFT, 1U));
    p_cmd->seq =
        (uint8_t) app_pack_get(record, APP_MC_SEQ_SHIFT, APP_MC_SEQ_BITS);
}

/** Wrap-safe encoder delta (uint16 subtraction, then sign by direction). */
static inline uint16_t
app_motion_enc_delta (uint16_t now, uint16_t last)
{
    return (uint16_t) (now - last);
}

#endif /* APP_MOTION_H */
