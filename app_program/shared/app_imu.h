/** @file app_imu.h
 *  @brief ImuState word (owner: Buddy 4, subsystem4). Layout: app_data.h.
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_pack.h"

/** Published word (writer: ImuTask, one aligned store). */
extern volatile uint32_t g_imu_state;

typedef struct
{
    int8_t  pitch_half_deg; /* 0.5 deg units, +-63.5 deg */
    uint8_t quality;        /* app_hump_quality_t */
    uint8_t episode_seq;    /* 0..15 */
    uint8_t height_mm;      /* 0..127 current episode estimate */
    uint8_t max_height_mm;  /* 0..127 run maximum */
    uint8_t motion_class;   /* app_motion_class_t */
    bool    hump_active;
} app_imu_state_t;

/** Returns true if height or pitch saturated (caller must then publish
 *  quality DEGRADED/INVALID rather than a clipped number as fact). */
static inline bool
app_imu_pack (app_imu_state_t const * p_view, uint32_t * p_word)
{
    bool     sat = false;
    uint32_t w   = 0U;
    w            = app_pack_set_s(w, APP_IS_PITCH_SHIFT, APP_IS_PITCH_BITS,
                                  p_view->pitch_half_deg, &sat);
    w = app_pack_set_sat(w, APP_IS_QUALITY_SHIFT, APP_IS_QUALITY_BITS,
                         p_view->quality, &sat);
    w = app_pack_set(w, APP_IS_SEQ_SHIFT, APP_IS_SEQ_BITS, p_view->episode_seq);
    w = app_pack_set_sat(w, APP_IS_HEIGHT_SHIFT, APP_IS_HEIGHT_BITS,
                         p_view->height_mm, &sat);
    w = app_pack_set_sat(w, APP_IS_MAXH_SHIFT, APP_IS_MAXH_BITS,
                         p_view->max_height_mm, &sat);
    w = app_pack_set_sat(w, APP_IS_CLASS_SHIFT, APP_IS_CLASS_BITS,
                         p_view->motion_class, &sat);
    w = app_pack_set(w, APP_IS_ACTIVE_SHIFT, 1U, p_view->hump_active ? 1U : 0U);
    *p_word = w;
    return sat;
}

static inline void
app_imu_unpack (uint32_t word, app_imu_state_t * p_view)
{
    p_view->pitch_half_deg =
        (int8_t) app_pack_get_s(word, APP_IS_PITCH_SHIFT, APP_IS_PITCH_BITS);
    p_view->quality =
        (uint8_t) app_pack_get(word, APP_IS_QUALITY_SHIFT, APP_IS_QUALITY_BITS);
    p_view->episode_seq =
        (uint8_t) app_pack_get(word, APP_IS_SEQ_SHIFT, APP_IS_SEQ_BITS);
    p_view->height_mm =
        (uint8_t) app_pack_get(word, APP_IS_HEIGHT_SHIFT, APP_IS_HEIGHT_BITS);
    p_view->max_height_mm =
        (uint8_t) app_pack_get(word, APP_IS_MAXH_SHIFT, APP_IS_MAXH_BITS);
    p_view->motion_class =
        (uint8_t) app_pack_get(word, APP_IS_CLASS_SHIFT, APP_IS_CLASS_BITS);
    p_view->hump_active = (0U != app_pack_get(word, APP_IS_ACTIVE_SHIFT, 1U));
}

static inline void
app_imu_read (app_imu_state_t * p_view)
{
    app_imu_unpack(g_imu_state, p_view);
}

/** Saturating height helper: clamps to 127 mm and reports it, so the caller
 *  can degrade the quality field (Rev B example 3). */
static inline uint8_t
app_imu_height_sat (uint32_t height_mm, bool * p_sat)
{
    return (uint8_t) app_pack_sat_u(height_mm, APP_IS_HEIGHT_BITS, p_sat);
}

#endif /* APP_IMU_H */
