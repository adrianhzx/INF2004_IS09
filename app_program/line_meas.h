/**
 * @file    line_meas.h
 * @brief   How much black each line sensor sees, in mm of travel (19 Nov).
 *
 * One "mark" is everything a sensor passes over while its darkness is above
 * light_pct: the line's edge, a junction bar, a branch, a barcode bar. For
 * each mark it records:
 *   dark_mm   distance travelled with darkness above light_pct
 *   black_mm  ... of that, distance the sensor counted as black (the level
 *             the follower uses: LINE_DARK_PCT with hysteresis)
 *   peak      highest darkness 0..100
 *   rise_mm   distance from the start of the mark until the sensor turned
 *             black (or until the peak if it never did). A bar met head-on
 *             rises in a few mm; drifting onto the line's edge takes 20 mm
 *             or more.
 * The follower uses rise_mm to tell a junction bar or branch reached by one
 * sensor (abrupt) from the line drifting under it (gradual), see
 * line_logic.h. The dashboard shows the last mark and the live one.
 *
 * Pure logic, no hardware access.
 */
#ifndef LINE_MEAS_H
#define LINE_MEAS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float   dark_mm;
    float   black_mm;
    float   rise_mm;
    uint8_t peak;
} line_mark_t;

typedef struct
{
    bool        b_in;          /* inside a mark */
    bool        b_black_prev;
    bool        b_rose;        /* turned black during this mark */
    bool        b_new_black;   /* this step: just turned black */
    line_mark_t cur;           /* the mark in progress */
    line_mark_t last;          /* the last finished mark */
    uint32_t    n;             /* marks finished */
} line_meas_t;

void line_meas_reset(line_meas_t *p_m);
/* one 10 ms step: darkness 0..100, the follower's black level, distance */
void line_meas_step(line_meas_t *p_m, uint8_t n, bool b_black, float ds_mm, uint8_t light_pct);

/* index 0 = LEFT, 1 = CENTRE (the follower's right sensor); written by
   TSK_LINE_BARCODE, read by the telemetry task */
extern line_meas_t g_line_meas[2];

#endif /* LINE_MEAS_H */
