/**
 * @file    obstacle_logic.h
 * @brief   Ultrasonic filtering, obstacle profiling and bypass planning
 *          (DR 6.2 STAGE0..STAGE4, FR9-FR11, NFR8).
 *
 * Angles are in the robot frame: 0 = straight ahead, + = right, - = left.
 * Lateral positions: x = r sin(theta) (+ = right), forward y = r cos(theta).
 *
 * Pure logic: unit-tested on the PC (tests/).
 */
#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"

#define OBS_MAX_POINTS   (64u)

/* ---- STAGE0: NFR8 - a single bad echo must not trigger avoidance ----
 * Median of the last 3 readings; the trigger needs 2 filtered readings in a
 * row below the stop distance. A no-echo reading (< 0) counts as "far". */
typedef struct
{
    float    r[3];
    uint32_t n;
    uint32_t below;
} us_filter_t;

void  usf_reset(us_filter_t *p_f);
float usf_add(us_filter_t *p_f, float cm);           /* returns the filtered value */
bool  usf_trigger(us_filter_t *p_f, float filtered_cm, float stop_cm);

float median3(float a, float b, float c);

/* ---- STAGE1: coarse sweep ---- */
#define COARSE_N  (5u)
extern const float COARSE_DEG[COARSE_N];             /* -60 -30 0 30 60 */
bool  coarse_all_clear(const float *p_cm, float clear_cm);

/* ---- STAGE2/3: fine scan -> profile ---- */
typedef struct
{
    float   deg[OBS_MAX_POINTS];
    float   cm[OBS_MAX_POINTS];                       /* < 0 = no echo */
    uint32_t n;
} scan_t;

/* Builds the profile from the fine scan and the coarse sweep.
   robot_half_mm + margin_mm = how far the robot centre must pass from an edge. */
bool  obstacle_profile(const scan_t *p_fine, const float *p_coarse_cm,
                       float robot_half_mm, float margin_mm, obstacle_profile_t *p_out);

/* 20 Nov: replace a too-wide scan width with the known obstacle width
   (width_cm > 0), keeping the measured centre (offset_cm). The HC-SR04
   beam is much wider than a small box at 20-30 cm: the 15 cm demo box gave
   echoes at every angle from -60 to +60 deg and measured 47-66 cm. The
   clearances beside it change by what each edge moved (unless "open").
   A narrower scan (e.g. one echo point, w 0) is widened the same way.
   Does nothing if width_cm <= 0. */
void  obstacle_set_width(obstacle_profile_t *p_prof, float width_cm);

/* 20 Nov: measure the box edge while driving (OBST_MEASURE).
 * On the diagonal leg the sensor points along the original path. While the
 * sensor is still in front of the box, the nearest point in the (very
 * wide) beam is the foot on the front face, so the range falls in a
 * straight line: r = a - k s (s = leg distance, k = cos of the encoder
 * angle to the path; a from the scan, then from the pings on the box). Once
 * the sensor is past the edge, the nearest point is the corner:
 * r^2 = e^2 + dx^2, e = a - k s. A range EDGE_TOL_MM above the line twice
 * means the edge is passed; each such ping gives the leg distance where
 * the sensor crossed the edge: s - dx / sqrt(1 - k^2) (median kept).
 * Simulated (beam +-45 deg, 3 mm noise, box 400-500 mm from the axle,
 * real pivot 45-58 deg): edge within about +-50 mm, mostly +-30 mm.
 * 20 Nov 20:00 car runs: with a 10 cm box (the sensor is already past its
 * edge after the pivot) and a turned box (its face sends the echo away)
 * the box was never seen along the path; the old code fitted its line to
 * the background (~1.2 m), put the front at 1300 mm and the side leg at
 * 1293 mm. Now only pings within EDGE_GATE_MM of the scan's front count,
 * k is never fitted, and a box not seen in the first EDGE_UNSEEN_N pings
 * gives an assumed edge EDGE_UNSEEN_EXTRA_MM beyond the sensor.
 * 20 Nov car run: at ~45 cm the beam was much narrower and the box simply
 * dropped out of it (a jump to 1.7 m). So a jump of EDGE_JUMP_MM, or no
 * echo, twice in a row also means "past the edge": the crossing is taken
 * half-way between the last ping on the box and the first one past it,
 * and later pings (other objects) are ignored. */
#define EDGE_MAX_PTS     (64u)
#define EDGE_GATE_MM     (200.0f)  /* a ping is the box only if within this of where the scan puts its front */
#define EDGE_TOL_MM      (20.0f)
#define EDGE_USE_MM      (30.0f)   /* only pings this far above the front line give a crossing (precision) */
#define EDGE_JUMP_MM     (80.0f)   /* 20 Nov car run: at ~45 cm the beam is narrow and the box simply
                                      drops out (446 -> 1734 mm): a jump this big above the last range,
                                      or a ping that is not the box, twice = past the edge */
#define EDGE_UNSEEN_N    (3u)      /* the first pings are not the box: already past the edge (narrow box) */
#define EDGE_UNSEEN_EXTRA_MM (50.0f) /* ... then the edge is assumed at most this far beyond the sensor */
typedef struct
{
    float    k;                    /* cos(encoder angle to the path) (20:00: fitting it went wrong) */
    float    a_scan;               /* sensor-to-front along the path at s = 0, from the scan */
    float    a;                    /* the same, from the pings on the box once there are 3 (else a_scan) */
    float    s[EDGE_MAX_PTS];      /* pings on the box: leg mm, range mm */
    float    r[EDGE_MAX_PTS];
    uint32_t n;                    /* pings on the box */
    uint32_t unseen_n;             /* pings at the start that were not the box */
    float    s_first;              /* first ping of the leg */
    uint32_t over_n;
    bool     b_edge;
    bool     b_unseen;             /* the box was never seen along the path: edge assumed */
    float    cross[EDGE_MAX_PTS];
    uint32_t n_cross;
    float    s_ok;                 /* last ping still on the box */
    float    r_ok;
    uint32_t jump_n;
    float    s_jump;               /* first ping of the jump */
    bool     b_done;               /* edge decided: later pings are not used */
} edge_meas_t;

/* heading_deg: the encoder angle to the path; a_scan_mm: where the scan
   puts the box front, as the sensor's distance along the path at s = 0 */
void  edge_reset(edge_meas_t *p_e, float heading_deg, float a_scan_mm);
void  edge_add(edge_meas_t *p_e, float s_mm, float r_mm);    /* r_mm < 0: no echo */
float edge_s_cross(const edge_meas_t *p_e);                  /* median crossing (leg mm), < 0 if none */
float edge_sin(const edge_meas_t *p_e);                      /* sqrt(1 - k^2): sideways mm per leg mm */
float edge_angle_deg(const edge_meas_t *p_e);                /* acos(k): the real diagonal angle */
/* sqrt(r^2 - g^2) (0 if r < g). Here, not in tsk_obstacle.c: <math.h> after
   <tk/tkernel.h> gives two different size_t types on the Pico build. */
float obstacle_leg_of(float r, float g);

/* 20 Nov: TURN_CAL. The car pivots on a straight line; each line sensor
   crosses the line twice per real turn, so a sensor's 1st and 3rd crossing
   (encoder angles, deg, turned so far) are one real 360 deg apart. Returns
   the new track width (mm) = track_mm x encoder deg per real turn / 360,
   from the LEFT and CENTRE onsets (mean of those with 3+ onsets), or < 0
   if neither has 3. */
/* 20 Nov: rejoin pivot rule (rejoin_snap). Pivoting towards the path moves
   the line across the sensor bar from the leading sensor's side to the
   trailing sensor's side. Stop when the leading sensor has been black and is
   white again (black at the start counts), or when the trailing sensor goes
   black after reading white. */
typedef struct
{
    bool b_seen;
    bool b_arm_trail;
} rejoin_watch_t;

void rejoin_watch_reset(rejoin_watch_t *p);
bool rejoin_watch_step(rejoin_watch_t *p, bool b_lead, bool b_trail);

float turn_cal_track_mm(float track_mm, const float *p_on_l, uint32_t n_l,
                        const float *p_on_c, uint32_t n_c);

/* 20 Nov (OBST_MEASURE): how far to drive along the box side before the 45
   deg turn back. The car's near side must clear the box's back corner on
   the 45 deg return: with the axle gap_mm out from the box side, the axle
   must be (half + margin) / sin45 - gap_mm past the back (back = front +
   depth). The 19:52 run used back + gap (+ a 130 mm trail): ~210 mm more
   than needed. All in mm along the path; at least 50 mm. */
float bypass_side_leg_mm(float face_y_mm, float depth_mm, float half_mm, float margin_mm,
                         float gap_mm, float y_now_mm);

/* Sides already tried and found blocked during this avoidance */
#define OBST_BLOCKED_L   (1u)
#define OBST_BLOCKED_R   (2u)

/* Is there room for the car on 'side' ('L' / 'R') of the profiled obstacle? */
bool  obstacle_side_room(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm);

/* The side to pass on, among the sides that have room and are not in
   'blocked' (OBST_BLOCKED_*): the one with MORE clearance; if the two
   clearances are within OBST_SIDE_TIE_CM (e.g. both "no echo") the one
   needing the smaller sideways shift. 0 = none. obstacle_profile() uses it
   with blocked = 0. */
#define OBST_SIDE_TIE_CM (10.0f)
char  obstacle_choose_side(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                           uint32_t blocked);

/* How far (mm) the car must stand off the obstacle's front before the
   first 45 deg leg on 'side', so that leg passes the near corner by at
   least robot_half_mm + margin_mm: e + sqrt(2) (half + margin), e = how far
   the corner is out to that side of the car's centre line. If r_min is
   less, reverse the difference after the scan. */
float obstacle_standoff_mm(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm);

/* Bypass-leg watch: is a ping r_cm at deg (+ right) inside the car's path,
   i.e. within robot_half_mm + side_mm of the centre line and no more than
   ahead_cm in front? (r < 0 = no echo = no.) */
bool  obstacle_in_path(float r_cm, float deg, float robot_half_mm, float side_mm, float ahead_cm);

/* ---- STAGE4: bypass plan ----
 * (1) pivot 45 deg away, (2) forward d1 so the car clears the edge,
 * (3) pivot back 45 deg (parallel to the course), (4) forward d2 past the
 * obstacle, (5) pivot 45 deg towards the line; the caller then drives the
 * diagonal return until the line is found (STAGE5). */
typedef struct
{
    int8_t side;              /* +1 = pass on the right, -1 = left */
    float  turn1_deg;         /* + = right */
    float  d1_mm;
    float  turn2_deg;
    float  d2_mm;
    float  turn3_deg;
    float  shift_mm;          /* lateral offset reached by leg 2 */
    float  return_max_mm;     /* diagonal return limit (DR: 300 mm lateral) */
    float  realign_deg;       /* after the line is found */
} bypass_plan_t;

bool  bypass_plan(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                  float depth_mm, float max_lateral_mm, bypass_plan_t *p_plan);

/* Parallel leg (side-following at 60 deg, the servo's limit): does a ping
   at 60 deg towards the obstacle still hit its side, gap_mm beside the car? */
bool  obstacle_side_seen(float r_cm, float gap_mm);

/* How far to drive on after the side was last seen at 60 deg before the
   45 deg turn back clears its back corner (us_ahead_mm: ultrasonic sensor
   ahead of the axle). */
float obstacle_side_trail_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm);

/* Look-before-turning with the car pivoted 30 deg towards the obstacle and
   the servo at 60 (= square to its side): the side is still there if the
   ping is within the gap (less the 40 mm the pivot moved the sensor) +
   10 cm. If it is not, the back corner is at most ~10 cm ahead of the
   centre: drive on obstacle_side_trail90_mm() before the 45 deg turn. */
bool  obstacle_side_beside(float r90_cm, float gap_mm);
float obstacle_side_trail90_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm);

/* Bypass dead reckoning: advance (x right, y forward) by ds_mm on the mean
   of two headings (deg, + = clockwise). */
void  obstacle_pose_advance(float *p_x, float *p_y, float h_prev_deg, float h_deg, float ds_mm);
float obstacle_sin_deg(float deg);

#endif /* OBSTACLE_LOGIC_H */
