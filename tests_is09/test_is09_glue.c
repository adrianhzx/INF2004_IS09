/**
 * @file    test_is09_glue.c
 * @brief   Host tests for the IS09 adapters of this port
 *          (is09_barcode.c, is09_line.c, is09_scan.c, is09_imu.c) together
 *          with the unchanged IS09 cores. Build and run: make (in this
 *          folder, with a host gcc).
 */
#include <stdio.h>
#include <string.h>

#include "../app_program/is09_glue.h"
#include "../app_program/pickup_logic.h"
#include "../app_program/obstacle_logic.h"
#include <math.h>

static int g_pass;
static int g_fail;

#define CHECK(id, cond)                                                     \
    do                                                                      \
    {                                                                       \
        if (cond) { g_pass++; }                                             \
        else { g_fail++; printf("FAIL %s: %s (line %d)\n", id, #cond, __LINE__); } \
    } while (0)

/* ---- barcode ----------------------------------------------------------- */

static const char *STAR = "nwnnwnwnn";

/* 29 element widths (us) for *X* with pattern of X, narrow n_us, wide 3x */
static int build(const char *p_char_pat, double n_us, double wide_ratio, double ramp, double *p_w)
{
    char pat[30];
    int  k = 0;

    memcpy(&pat[0], STAR, 9);
    pat[9] = 'n';
    memcpy(&pat[10], p_char_pat, 9);
    pat[19] = 'n';
    memcpy(&pat[20], STAR, 9);
    for (k = 0; k < 29; k++)
    {
        double scale = 1.0 + (ramp * (double)k / 28.0);   /* slower or faster across the code */
        p_w[k] = ((pat[k] == 'w') ? wide_ratio : 1.0) * n_us * scale;
    }
    return 29;
}

/* Arms at t, waits quiet, plays the edges, polls every 10 ms. */
static uint32_t g_now;

static bool play(const double *p_w, int n, is09_bc_result_t *p_res)
{
    bool     b_got = false;
    double   t;
    int      k;
    uint32_t end;

    is09_bc_arm(true, g_now);
    t = (double)g_now + 600000.0;            /* 600 ms of white first */
    /* poll while waiting */
    for (uint32_t p = g_now; p < (uint32_t)t; p += 10000u)
    {
        b_got = is09_bc_service(p, p_res) || b_got;
    }
    is09_bc_isr_edge(false, (uint32_t)t);  /* start of the first bar */
    for (k = 0; k < n; k++)
    {
        t += p_w[k];
        is09_bc_isr_edge((k & 1) == 0, (uint32_t)t);
    }
    end = (uint32_t)t + 1000000u;
    for (uint32_t p = (uint32_t)t; p < end; p += 10000u)
    {
        if (is09_bc_service(p, p_res))
        {
            g_now = p;
            return true;
        }
    }
    g_now = end;
    return b_got;
}

static void test_barcode(void)
{
    static const struct { const char *name; double n_us; } SIZES[] =
    {
        { "100%", 37500.0 },     /* 3 mm narrow at 80 mm/s */
        { "75%",  28125.0 },
        { "50%",  18750.0 },
    };
    double           w[29];
    double           rv[29];
    is09_bc_result_t r;
    char             text[300];
    uint32_t         cap0;

    is09_bc_init();
    g_now = 1000u;

    for (unsigned i = 0u; i < 3u; i++)
    {
        build("wnnnnwnnw", SIZES[i].n_us, 3.0, 0.0, w);
        CHECK(SIZES[i].name, play(w, 29, &r));
        CHECK(SIZES[i].name, r.b_done && r.b_letter && (r.letter == 'A') && !r.b_reversed && (r.count == 29u));
        for (int k = 0; k < 29; k++) { rv[28 - k] = w[k]; }
        CHECK("A backwards", play(rv, 29, &r) && r.b_letter && (r.letter == 'A') && r.b_reversed);
    }

    /* the 16 Nov track decode: pat and the IS09 console format */
    build("wnnnnwnnw", 37500.0, 3.0, 0.0, w);
    (void)play(w, 29, &r);
    is09_bc_capture_text(text, sizeof(text));
    CHECK("pat text", strstr(text, "cap=29 ") == text);
    CHECK("pat text", strstr(text, "pat=nwnnwnwnn n wnnnnwnnw n nwnnwnwnn bc=A") != NULL);
    is09_bc_widths_text(text, sizeof(text));
    CHECK("widths text", strstr(text, "(29, ms): b37.4 s112.4 b37.4") != NULL);

    /* letters the IS09 table does not map still come through for nav_map */
    build("wnnnnnnww", 30000.0, 3.0, 0.0, w);  /* K */
    CHECK("K", play(w, 29, &r) && r.b_letter && (r.letter == 'K'));
    build("nnwnnwnnw", 30000.0, 3.0, 0.0, w);  /* B */
    CHECK("B", play(w, 29, &r) && r.b_letter && (r.letter == 'B'));
    build("nnnnwwnnw", 30000.0, 2.2, 0.0, w);  /* D, weak 2.2:1 print */
    CHECK("D 2.2:1", play(w, 29, &r) && r.b_letter && (r.letter == 'D'));

    /* speed change across the code: +60 % slower by the end */
    build("wnwnnwnnn", 25000.0, 3.0, 0.6, w); /* C */
    CHECK("C ramp", play(w, 29, &r) && r.b_letter && (r.letter == 'C'));

    /* 29 equal widths: a result, no letter (a failed read) */
    for (int k = 0; k < 29; k++) { w[k] = 30000.0; }
    CHECK("equal", play(w, 29, &r) && r.b_done && !r.b_letter);

    /* a stray mark (5 widths) is silent but counted as a capture */
    cap0 = is09_bc_capture_count();
    for (int k = 0; k < 5; k++) { w[k] = 20000.0; }
    CHECK("stray silent", !play(w, 5, &r));
    CHECK("stray counted", is09_bc_capture_count() == (cap0 + 1u));

    /* split barcode (3 pieces, as the 16 Nov 100 % log): no letter */
    build("wnnnnwnnw", 37500.0, 3.0, 0.0, w);
    w[14] += 500000.0;                          /* a > T_QUIET gap inside */
    (void)play(w, 29, &r);
    CHECK("split", !r.b_letter);

    /* disarmed: edges ignored */
    is09_bc_arm(false, g_now);
    cap0 = is09_bc_capture_count();
    is09_bc_isr_edge(true, g_now + 500000u);
    is09_bc_isr_edge(false, g_now + 530000u);
    CHECK("disarmed", !is09_bc_service(g_now + 2000000u, &r) && (is09_bc_capture_count() == cap0));

    is09_bc_health_text(text, sizeof(text), true);
    CHECK("health", strstr(text, "pin=1 st=DISARMED edges=") == text);
}

/* ---- line ------------------------------------------------------------ */

static void test_line(void)
{
    is09_line_diag_t d;

    is09_line_init();
    /* white 300, black 3500 (IS09: black reads HIGH) */
    for (int i = 0; i < 20; i++) { is09_line_update(300u, 3500u, 45u, 10u); }
    CHECK("L white", !is09_line_black_left());
    CHECK("C black", is09_line_black_right());
    is09_line_diag(&d);
    CHECK("refs", d.calibrated && (d.ref_white == 300u) && (d.ref_black == 3500u) && (d.n_c == 100u));
    /* left onto the line edge: 45 % = 300 + 0.45 x 3200 = 1740 */
    is09_line_update(1800u, 3500u, 45u, 10u);
    CHECK("L black at 46%", is09_line_black_left());
    is09_line_update(1500u, 3500u, 45u, 10u);   /* 37 %: inside the hysteresis band */
    CHECK("L stays black", is09_line_black_left());
    is09_line_update(1300u, 3500u, 45u, 10u);   /* 31 %: below 35 */
    CHECK("L white again", !is09_line_black_left());
    is09_line_update(1500u, 3500u, 45u, 10u);   /* 37 % from white: not black */
    CHECK("L not black at 37%", !is09_line_black_left());

    /* 20 Nov: LEFT's own black level 25 % (CENTRE keeps 45 %) */
    is09_line_set_left_dark(25u);
    is09_line_update(300u, 300u, 45u, 10u);
    is09_line_update(1300u, 1420u, 45u, 10u);   /* L 31 %, C 35 % */
    CHECK("L black at 31% (own level)", is09_line_black_left());
    CHECK("C not black at 35%", !is09_line_black_right());
    is09_line_update(900u, 300u, 45u, 10u);     /* L 19 %: above 25 - 10 */
    CHECK("L stays black at 19%", is09_line_black_left());
    is09_line_update(700u, 300u, 45u, 10u);     /* L 12 %: white */
    CHECK("L white at 12%", !is09_line_black_left());
    is09_line_set_left_dark(0u);                /* 0 = the shared level again */
    is09_line_update(1300u, 300u, 45u, 10u);
    CHECK("L shared level again", !is09_line_black_left());
}

/* ---- ultrasonic -------------------------------------------------------- */

static void test_scan(void)
{
    uint8_t        plan[IS09_SCAN_MAX];
    uint32_t       n;
    is09_profile_t v;
    bool           b_fault = false;

    CHECK("deg ahead", is09_scan_deg(18u) == 0.0f);
    CHECK("deg right", is09_scan_deg(0u) == 90.0f);
    CHECK("deg left", is09_scan_deg(36u) == -90.0f);
    CHECK("cm none", is09_us_cm(-1.0f) == IS09_RANGE_NONE);
    CHECK("cm", is09_us_cm(23.7f) == 23u);

    /* forward watch: two readings at or inside D_STOP (25 cm), once */
    is09_watch_reset();
    CHECK("watch 1", !is09_watch_step(24u, &b_fault));
    CHECK("watch 2", is09_watch_step(25u, &b_fault));
    CHECK("watch once", !is09_watch_step(20u, &b_fault));
    CHECK("watch rearm", !is09_watch_step(40u, &b_fault) && !is09_watch_step(20u, &b_fault) &&
                         is09_watch_step(20u, &b_fault));
    is09_watch_reset();
    for (int i = 0; i < 5; i++) { (void)is09_watch_step(IS09_RANGE_NONE, &b_fault); }
    CHECK("no echo fault", b_fault);

    /* box 20 cm ahead, slightly right of centre; all else far */
    is09_scan_begin();
    n = is09_scan_coarse_plan(plan, IS09_SCAN_MAX);
    CHECK("coarse 9", (n == 9u) && (plan[0] == 6u) && (plan[8] == 30u));
    for (uint32_t i = 0u; i < n; i++)
    {
        is09_scan_set(plan[i], ((plan[i] >= 15u) && (plan[i] <= 18u)) ? 30u : IS09_RANGE_NONE);
    }
    n = is09_scan_fine_plan(plan, IS09_SCAN_MAX);
    CHECK("fine window", (n > 0u) && (plan[0] == 12u) && (plan[n - 1u] == 21u));
    for (uint32_t i = 0u; i < n; i++)
    {
        is09_scan_set(plan[i], ((plan[i] >= 15u) && (plan[i] <= 18u)) ? 30u : IS09_RANGE_NONE);
    }
    is09_scan_profile(&v);
    CHECK("profile edges right of centre", (v.right_edge_cm > v.left_edge_cm) && (v.right_edge_cm > 0.0f));
    CHECK("profile side LEFT (smaller detour)", v.action == IS09_AVOID_TURN_LEFT);
    CHECK("closest", (v.closest_cm == 30u));

    /* nothing within 45 cm -> CONTINUE */
    is09_scan_begin();
    n = is09_scan_coarse_plan(plan, IS09_SCAN_MAX);
    for (uint32_t i = 0u; i < n; i++) { is09_scan_set(plan[i], 80u); }
    is09_scan_profile(&v);
    CHECK("clear", v.action == IS09_AVOID_CONTINUE);
}

/* ---- IMU ---------------------------------------------------------------- */

static void test_imu(void)
{
    is09_imu_in_t  in;
    is09_imu_out_t out;
    bool           b_coll = false;

    memset(&in, 0, sizeof(in));
    is09_imu_init();
    in.z_mg = 1000;
    in.b_sample_valid = true;
    in.b_motion_fresh = true;
    for (int i = 0; i < 50; i++) { is09_imu_step(&in, &out); }
    CHECK("level pitch", (out.pitch_deg > -1.0f) && (out.pitch_deg < 1.0f));

    /* resting on a slight slope (2 deg nose down): after 0.5 s still it reads 0 */
    in.y_mg = 35;
    in.z_mg = 999;
    for (int i = 0; i < 150; i++) { is09_imu_step(&in, &out); }
    CHECK("zeroed at rest", (out.pitch_deg > -0.6f) && (out.pitch_deg < 0.6f));
    in.y_mg = 0;
    in.z_mg = 1000;
    for (int i = 0; i < 150; i++) { is09_imu_step(&in, &out); }
    CHECK("zero again", (out.pitch_deg > -0.6f) && (out.pitch_deg < 0.6f));
    CHECK("stationary", out.motion_class == IS09_CLASS_STATIONARY);

    /* driving at 8 cm/s */
    in.b_moving = true;
    in.speed_cm_s = 8u;
    in.dist_delta_mm = 1;
    for (int i = 0; i < 100; i++) { is09_imu_step(&in, &out); }
    CHECK("cruising", out.motion_class == IS09_CLASS_CRUISING);

    /* nose up 10 deg for 0.5 s: hump start (forward = -Y, 19 Nov) */
    in.y_mg = -174;
    in.z_mg = 985;
    {
        bool b_start = false;

        for (int i = 0; i < 100; i++) { is09_imu_step(&in, &out); b_start = b_start || out.b_hump_start; }
        CHECK("hump start", b_start && out.b_hump_active && (out.pitch_deg > 5.0f));
    }
    in.y_mg = 0;
    in.z_mg = 1000;
    for (int i = 0; i < 300; i++) { is09_imu_step(&in, &out); }

    /* side tilt 10 deg (X, left up) must NOT read as pitch */
    in.x_mg = 174;
    in.z_mg = 985;
    for (int i = 0; i < 50; i++) { is09_imu_step(&in, &out); }
    CHECK("roll is not pitch", (out.pitch_deg > -1.0f) && (out.pitch_deg < 1.0f));
    in.x_mg = 0;
    in.z_mg = 1000;
    for (int i = 0; i < 50; i++) { is09_imu_step(&in, &out); }

    /* forward spike 3 g for 2 samples while moving: collision */
    in.y_mg = -3000;
    is09_imu_step(&in, &out); b_coll = b_coll || out.b_collision;
    is09_imu_step(&in, &out); b_coll = b_coll || out.b_collision;
    CHECK("collision", b_coll);
    CHECK("class names", strcmp(is09_imu_class_name(IS09_CLASS_IMPACT), "IMPACT") == 0);
}

/* 1 cm triangle ramp under a ~10 cm car: the body tilts ~4 deg up, then
   ~4 deg down. Run with IS09_RUNTIME_THETA (as the firmware). */
static int ramp(float on_deg, float off_deg, int16_t accel_jolt_mg, bool b_comp, bool *p_start)
{
    is09_imu_in_t   in;
    is09_imu_out_t  out;
    is09_imu_diag_t d;
    int             normal = 0;

    memset(&in, 0, sizeof(in));
    is09_imu_init();
    is09_imu_set_thresholds(on_deg, off_deg);
    in.z_mg = 1000;
    in.b_sample_valid = true;
    in.b_motion_fresh = true;
    for (int i = 0; i < 100; i++) { is09_imu_step(&in, &out); }      /* rest zero */
    *p_start = false;
    in.b_moving = true;
    in.speed_cm_s = 8u;
    in.dist_delta_mm = 1;
    /* start-up jolt: the encoders see it too (a_comp) */
    in.y_mg = (int16_t)-accel_jolt_mg;
    in.a_comp_mg = b_comp ? accel_jolt_mg : 0;
    for (int i = 0; i < 20; i++) { is09_imu_step(&in, &out); *p_start = *p_start || out.b_hump_start; }
    in.y_mg = 0;
    in.a_comp_mg = 0;
    for (int i = 0; i < 50; i++) { is09_imu_step(&in, &out); *p_start = *p_start || out.b_hump_start; }
    /* up 4 deg for 0.6 s, down 4 deg for 0.6 s, level */
    for (int i = 0; i < 60; i++) { in.y_mg = -70; in.z_mg = 998; is09_imu_step(&in, &out); normal += (out.b_hump_end && (out.end_quality == IS09_HUMP_Q_NORMAL)); }
    for (int i = 0; i < 60; i++) { in.y_mg = 70;  in.z_mg = 998; is09_imu_step(&in, &out); normal += (out.b_hump_end && (out.end_quality == IS09_HUMP_Q_NORMAL)); }
    for (int i = 0; i < 80; i++) { in.y_mg = 0;   in.z_mg = 1000; is09_imu_step(&in, &out); normal += (out.b_hump_end && (out.end_quality == IS09_HUMP_Q_NORMAL)); }
    is09_imu_diag(&d);
    return normal;
}

static void test_ramp(void)
{
    bool b_start = false;

    CHECK("1 cm ramp missed at IS09's 5 deg", ramp(5.0f, 2.0f, 0, true, &b_start) == 0);
    CHECK("1 cm ramp NORMAL at 3 deg", ramp(3.0f, 1.5f, 0, true, &b_start) == 1);
    (void)ramp(3.0f, 1.5f, 300, false, &b_start);
    CHECK("start jolt NOT compensated: false hump start", b_start);
    CHECK("start jolt compensated: ramp still NORMAL", ramp(3.0f, 1.5f, 300, true, &b_start) == 1);
    CHECK("start jolt compensated: no false start", !b_start);
    {
        is09_imu_diag_t d;

        is09_imu_set_thresholds(3.0f, 1.5f);
        is09_imu_diag(&d);
        CHECK("diag thresholds", (d.on_deg == 3.0f) && (d.off_deg == 1.5f));
        is09_imu_set_thresholds(1.0f, 4.0f);
        is09_imu_diag(&d);
        CHECK("off kept below on", d.off_deg < d.on_deg);
    }
}

/* ---- pick-up detection (pickup_logic.c, 20 Nov) ------------------------- */

/* Runs n 10 ms steps at speed mm_s with the three raw values; returns the
   step (1..n) on which it fired, 0 = never. */
static int pick_run(pickup_ctx_t *p_ctx, const pickup_par_t *p_par, int n,
                    unsigned l, unsigned c, unsigned ir, float mm_s)
{
    int fired = 0;

    for (int i = 1; i <= n; i++)
    {
        if (pickup_step(p_ctx, p_par, l, c, ir, mm_s * 0.01f, 10u))
        {
            if (fired != 0) { return -1; }     /* fired twice */
            fired = i;
        }
    }
    return fired;
}

static void test_pickup(void)
{
    pickup_par_t par = { 3000u, 30.0f, 500u };
    pickup_ctx_t ctx;
    int          k;

    /* worst case black bar: all three high over 18 mm at 60 mm/s (300 ms) */
    pickup_reset(&ctx);
    CHECK("pick-bar", pick_run(&ctx, &par, 30, 4000, 4000, 4000, 60.0f) == 0);
    CHECK("pick-bar-after", pick_run(&ctx, &par, 50, 200, 200, 200, 60.0f) == 0);

    /* on the track the LEFT sensor peaked at 2525: never, however long */
    pickup_reset(&ctx);
    CHECK("pick-left-low", pick_run(&ctx, &par, 500, 2525, 4000, 4000, 80.0f) == 0);

    /* lifted, wheels spinning at 80 mm/s: 30 mm after 38 steps, once */
    pickup_reset(&ctx);
    k = pick_run(&ctx, &par, 200, 3950, 4050, 3990, 80.0f);
    CHECK("pick-spin", (k >= 37) && (k <= 39));

    /* lifted while stopped (e.g. waiting at an obstacle): after 500 ms */
    pickup_reset(&ctx);
    CHECK("pick-still", pick_run(&ctx, &par, 100, 4000, 4000, 4000, 0.0f) == 50);

    /* one report per run, until reset */
    CHECK("pick-once", pick_run(&ctx, &par, 100, 4000, 4000, 4000, 80.0f) == 0);
    pickup_reset(&ctx);
    CHECK("pick-again", pick_run(&ctx, &par, 100, 4000, 4000, 4000, 80.0f) > 0);

    /* a short dip below the threshold starts the count again */
    pickup_reset(&ctx);
    (void)pick_run(&ctx, &par, 30, 4000, 4000, 4000, 80.0f);   /* 24 mm */
    (void)pick_run(&ctx, &par, 1, 4000, 2900, 4000, 80.0f);
    CHECK("pick-dip", pick_run(&ctx, &par, 30, 4000, 4000, 4000, 80.0f) == 0);

    /* PICKUP_ADC = 0: off */
    par.adc = 0u;
    pickup_reset(&ctx);
    CHECK("pick-off", pick_run(&ctx, &par, 300, 4095, 4095, 4095, 80.0f) == 0);
}

/* ---- known obstacle width (obstacle_logic.c, 20 Nov) ------------------- */

/* Scan like the 20 Nov 17:38 car run: the 15 cm box about 26 cm ahead gave
   an echo of 25-30 cm at every angle from -60 to +60 deg. */
static void scan_15cm_box(scan_t *p_s, float *p_coarse)
{
    p_s->n = 0u;
    for (int d = -60; d <= 60; d += 5)
    {
        p_s->deg[p_s->n] = (float)d;
        p_s->cm[p_s->n]  = 26.0f + (float)((d < 0) ? -d : d) / 20.0f;   /* 26..29 cm */
        p_s->n++;
    }
    p_coarse[0] = 25.0f; p_coarse[1] = 27.0f; p_coarse[2] = 26.0f; p_coarse[3] = 26.0f; p_coarse[4] = 30.0f;
}

static void test_obstacle_width(void)
{
    scan_t             s;
    float              coarse[COARSE_N];
    obstacle_profile_t p;
    bypass_plan_t      plan0;
    bypass_plan_t      plan1;
    float              half = 80.0f, margin = 60.0f;
    float              st0;
    float              st1;

    scan_15cm_box(&s, coarse);
    CHECK("obst-profile", obstacle_profile(&s, coarse, half, margin, &p));
    CHECK("obst-scan-too-wide", p.width_cm > 40.0f);                 /* the problem */
    p.side = 'L';
    st0 = obstacle_standoff_mm(&p, 'L', half, margin);
    CHECK("obst-plan-before", bypass_plan(&p, half, margin, 150.0f, 300.0f, &plan0));
    printf("  scan width %.0f cm (edges %.0f / %.0f): shift %.0f mm, diagonal %.0f mm, standoff %.0f mm\n",
           (double)p.width_cm, (double)p.x_l_cm, (double)p.x_r_cm, (double)plan0.shift_mm, (double)plan0.d1_mm, (double)st0);

    obstacle_set_width(&p, 15.0f);
    CHECK("obst-width-15", fabsf(p.width_cm - 15.0f) < 0.01f);
    CHECK("obst-centre-kept", fabsf(((p.x_l_cm + p.x_r_cm) * 0.5f) - p.offset_cm) < 0.01f);
    CHECK("obst-open-stays-open", (p.clear_left_cm >= 200.0f) && (p.clear_right_cm >= 200.0f));
    CHECK("obst-room-both", obstacle_side_room(&p, 'L', half, margin) && obstacle_side_room(&p, 'R', half, margin));
    p.side = 'L';
    st1 = obstacle_standoff_mm(&p, 'L', half, margin);
    CHECK("obst-plan-after", bypass_plan(&p, half, margin, 150.0f, 300.0f, &plan1));
    printf("  with 15 cm: edges %.1f / %.1f: shift %.0f mm, diagonal %.0f mm, standoff %.0f mm\n",
           (double)p.x_l_cm, (double)p.x_r_cm, (double)plan1.shift_mm, (double)plan1.d1_mm, (double)st1);
    CHECK("obst-shift", fabsf(plan1.shift_mm - (75.0f - (p.offset_cm * 10.0f) + half + margin)) < 1.0f);
    CHECK("obst-narrower-plan", (plan1.shift_mm < plan0.shift_mm) && (st1 < st0));

    /* a blocked-looking side next to the box (clrL 14, clrR 1 at 17:43) gets
       the room back that the beam took; a one-point scan (w 0) is widened */
    p.width_cm = 57.0f; p.offset_cm = 0.0f; p.x_l_cm = -29.0f; p.x_r_cm = 28.0f;
    p.clear_left_cm = 14.0f; p.clear_right_cm = 1.0f;
    obstacle_set_width(&p, 15.0f);
    CHECK("obst-clear-grows", (fabsf(p.clear_left_cm - 35.5f) < 0.01f) && (fabsf(p.clear_right_cm - 21.5f) < 0.01f));
    p.width_cm = 0.0f; p.offset_cm = 6.0f; p.x_l_cm = 6.0f; p.x_r_cm = 6.0f; p.clear_left_cm = 200.0f;
    obstacle_set_width(&p, 15.0f);
    CHECK("obst-one-point", (fabsf(p.x_l_cm + 1.5f) < 0.01f) && (fabsf(p.x_r_cm - 13.5f) < 0.01f));
    obstacle_set_width(&p, 0.0f);
    CHECK("obst-off", fabsf(p.width_cm - 15.0f) < 0.01f);
}

/* ---- box edge measured while driving (edge_meas_t, 20 Nov) ------------- */

/* HC-SR04 model fitted to the car: the echo is the nearest box point inside
   a +-45 deg beam (the 15 cm box still echoed at +-60 deg), 3 mm noise. */
static uint32_t g_rng = 1u;
static float rnd_gauss(void)
{
    float u1, u2;

    g_rng = (g_rng * 1103515245u) + 12345u; u1 = ((float)((g_rng >> 8) & 0xFFFFu) + 1.0f) / 65537.0f;
    g_rng = (g_rng * 1103515245u) + 12345u; u2 = ((float)((g_rng >> 8) & 0xFFFFu) + 1.0f) / 65537.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static float sim_echo(float sx, float sy, float axis_deg, float xl, float xr, float yf, float depth)
{
    float best = -1.0f;

    for (int i = 0; i <= 400; i++)
    {
        float x, y;

        if (i <= 300) { x = xl + ((xr - xl) * (float)i / 300.0f); y = yf; }
        else if (i <= 350) { x = xl; y = yf + (depth * (float)(i - 300) / 50.0f); }
        else { x = xr; y = yf + (depth * (float)(i - 350) / 50.0f); }
        {
            float dx = x - sx, dy = y - sy;
            float phi = atan2f(dx, dy) * 57.29578f - axis_deg;

            if ((phi <= 45.0f) && (phi >= -45.0f))
            {
                float r = sqrtf((dx * dx) + (dy * dy));

                if ((best < 0.0f) || (r < best)) { best = r; }
            }
        }
    }
    return best;
}

/* left bypass along a diagonal at h_real deg (encoders think 45). Returns the
   edge error (mm) and the gap between the car's side and the box (mm). */
static int edge_run(float width, float yf, float h_real, uint32_t seed, float *p_err, float *p_gap)
{
    edge_meas_t e;
    float       ua = 80.0f, half = 80.0f, margin = 60.0f;
    float       hr = h_real * 0.01745329f;
    float       xl = -0.5f * width;
    float       target = -1.0f;
    float       s = 0.0f;

    g_rng = seed * 7919u + 1u;
    edge_reset(&e, -45.0f, yf - (ua * 0.7071068f));   /* the scan's front, sensor after a 45 deg pivot */
    for (; s < 900.0f; s += 4.5f)
    {
        float sx = -(s + ua) * sinf(hr), sy = (s + ua) * cosf(hr);
        float r  = sim_echo(sx, sy, h_real - 45.0f, xl, -xl, yf, 150.0f);

        edge_add(&e, s, (r < 0.0f) ? -1.0f : (r + (3.0f * rnd_gauss())));
        if (e.b_edge)
        {
            target = edge_s_cross(&e) + ua + ((half + margin) / edge_sin(&e));
            if (s >= target) { break; }
        }
        if (!e.b_edge && ((e.a - (e.k * s)) < 80.0f)) { return -1; }   /* would block */
    }
    if (target < 0.0f) { return -2; }
    *p_err = (-(edge_s_cross(&e) + ua) * edge_sin(&e)) - xl;
    *p_gap = (xl + (s * sinf(hr))) - half;
    return 0;
}

/* 20 Nov 19:44 car run (telemetry: every second ping): leg mm, range mm.
   The box (15 cm) drops out of the beam at ~70-85 mm (447 -> 1734). The old
   code never found the edge and drove 660 mm until the E-stop. */
static const float LEG_1944[] = { 0.0f, 1627.0f, 1.4f, 1627.0f, 4.8f, 468.0f, 16.1f, 463.0f, 25.5f, 462.0f, 36.2f, 455.0f, 44.6f, 457.0f, 48.5f, 449.0f, 55.8f, 451.0f, 62.2f, 447.0f, 69.9f, 447.0f, 85.4f, 1734.0f, 97.7f, 2040.0f, 104.5f, 3826.0f, 116.1f, 1454.0f, 126.6f, 2996.0f, 136.4f, 1397.0f, 146.0f, 1054.0f, 150.6f, 1289.0f, 160.3f, 1226.0f, 170.0f, 1333.0f, 180.2f, 1263.0f, 189.9f, 1181.0f, 199.6f, 1001.0f, 204.4f, 1279.0f, 213.9f, 1266.0f, 223.4f, 1265.0f, 233.1f, 1211.0f, 243.6f, 1138.0f, 248.4f, 1165.0f, 258.8f, 1143.0f, 268.6f, 1020.0f, 278.7f, 978.0f, 288.7f, 1107.0f, 298.6f, 900.0f, 303.9f, 966.0f, 313.8f, 877.0f, 323.6f, 870.0f, 333.4f, 875.0f, 343.4f, 900.0f, 347.7f, 866.0f, 357.7f, 840.0f, 367.6f, 820.0f, 377.3f, 877.0f, 387.6f, 787.0f, 397.5f, 762.0f, 403.0f, 756.0f, 412.6f, 748.0f, 422.5f, 761.0f, 432.2f, 740.0f, 442.2f, 730.0f, 446.9f, 974.0f, 456.8f, 749.0f, 466.7f, 743.0f, 476.4f, 736.0f, 486.5f, 733.0f, 496.4f, 723.0f, 501.3f, 723.0f, 511.2f, 714.0f, 521.4f, 697.0f, 531.0f, 712.0f, 541.4f, 687.0f, 545.8f, 701.0f, 555.7f, 685.0f, 565.8f, 685.0f, 575.6f, 679.0f, 585.7f, 880.0f, 596.0f, 878.0f, 601.4f, 879.0f, 611.3f, 876.0f, 621.1f, 651.0f, 630.2f, 650.0f, 639.9f, 853.0f, 644.8f, 849.0f, 654.5f, 842.0f, 664.3f, 633.0f };

static void test_edge_car_run(void)
{
    edge_meas_t e;
    uint32_t    n = (uint32_t)(sizeof(LEG_1944) / sizeof(LEG_1944[0])) / 2u;
    float       found_at = -1.0f;

    edge_reset(&e, 45.0f, 500.0f - (80.0f * 0.7071068f));   /* OBST_BOX r=42: front 500 mm */
    for (uint32_t i = 0u; i < n; i++)
    {
        edge_add(&e, LEG_1944[2u * i], LEG_1944[(2u * i) + 1u]);
        if (e.b_edge && (found_at < 0.0f)) { found_at = LEG_1944[2u * i]; }
    }
    CHECK("edge-car-found", e.b_edge && (e.n_cross > 0u));
    CHECK("edge-car-found-early", (found_at > 60.0f) && (found_at < 110.0f));
    CHECK("edge-car-crossing", (edge_s_cross(&e) > 69.0f) && (edge_s_cross(&e) < 86.0f));
    CHECK("edge-car-stops", (edge_s_cross(&e) + 80.0f + (140.0f / edge_sin(&e))) < 400.0f);
    printf("  edge, 19:44 car run: found at %.0f mm, near edge %.0f mm from the line\n", (double)found_at,
           (double)((edge_s_cross(&e) + 80.0f) * edge_sin(&e)));
}

/* 20 Nov 19:52 car run: front 422 mm, axle 145 mm out from the box side at
   y 211 mm. The side leg was 498 mm (+130 mm trail); geometry needs 414. */
static void test_side_leg(void)
{
    float d  = bypass_side_leg_mm(422.0f, 150.0f, 80.0f, 60.0f, 145.0f, 211.0f);
    float ya = 211.0f + d;
    /* on the 45 deg return, the corner's distance from the car's centre line */
    float dist = 0.7071068f * (145.0f + ya - (422.0f + 150.0f));

    printf("  side leg, 19:52 car run: %.0f mm (was 498 + 130), car side passes the back corner by %.0f mm\n",
           (double)d, (double)(dist - 80.0f));
    CHECK("side-leg-shorter", (d > 400.0f) && (d < 430.0f));
    CHECK("side-leg-clears-by-margin", fabsf((dist - 80.0f) - 60.0f) < 1.0f);
    CHECK("side-leg-min", bypass_side_leg_mm(100.0f, 50.0f, 80.0f, 60.0f, 300.0f, 400.0f) == 50.0f);
}

/* 20 Nov 20:00 car runs: a 10 cm box (run 1, OBST_BOX r=33) and a turned
   box (run 2, r=27). The sensor along the path never saw the box (70-360 cm
   background, no echo). The old code put the front at 1300 / 1362 mm and
   the side leg at 1293 / 1156 mm. Now: box not seen -> edge assumed 50 mm
   beyond the sensor, front from the scan. */
static const float LEG_2000_1[] = { 0.0f, 701.0f, 3.8f, 701.0f, 8.7f, -1.0f, 13.4f, -1.0f, 24.0f, -1.0f, 34.0f, 1237.0f, 43.8f, 2403.0f, 53.9f, 2859.0f, 63.6f, -1.0f, 68.9f, -1.0f, 79.0f, -1.0f, 88.9f, 2361.0f, 98.9f, 2347.0f, 108.2f, -1.0f, 112.7f, -1.0f, 123.0f, 2323.0f, 132.8f, 2335.0f, 142.9f, 908.0f, 152.8f, 2287.0f, 162.7f, 704.0f, 168.0f, 919.0f, 177.9f, 2380.0f, 187.9f, 569.0f, 197.7f, -1.0f, 207.4f, -1.0f, 211.9f, 1676.0f, 221.9f, -1.0f, 232.1f, 861.0f, 242.0f, 1530.0f, 251.7f, 1662.0f, 261.5f, 1393.0f, 266.9f, 1360.0f, 276.7f, 1363.0f, 286.9f, 2087.0f, 296.9f, 1431.0f, 301.1f, 1349.0f, 301.1f, 1349.0f };
static const float LEG_2000_2[] = { 0.0f, 1666.0f, 0.0f, 1666.0f, 10.2f, 1291.0f, 20.4f, 1267.0f, 25.2f, 1258.0f, 35.1f, 1266.0f, 44.7f, 1270.0f, 54.5f, -1.0f, 64.4f, 1242.0f, 74.6f, 1764.0f, 79.6f, 1556.0f, 88.8f, 1218.0f, 98.5f, 1219.0f, 108.2f, 1189.0f, 118.0f, 3607.0f, 122.4f, 3439.0f, 131.7f, 3594.0f, 141.7f, 3588.0f, 152.0f, 1534.0f, 162.3f, 1478.0f, 172.7f, 3558.0f, 178.5f, 3553.0f, 188.6f, 3431.0f, 199.1f, 1455.0f, 208.9f, 1492.0f, 218.6f, 3365.0f, 223.2f, 3506.0f, 232.6f, 3416.0f, 243.0f, 1184.0f, 253.1f, 3483.0f, 263.1f, 1727.0f, 273.1f, 2973.0f, 278.5f, 2960.0f, 288.0f, 3445.0f, 297.3f, 1397.0f, 308.0f, 1389.0f, 317.9f, 1108.0f, 322.3f, 1099.0f, 332.0f, 1102.0f, 341.9f, 1089.0f, 351.9f, 1082.0f, 362.0f, 1064.0f, 372.0f, 1055.0f, 377.4f, 1045.0f, 387.3f, 904.0f, 397.1f, 899.0f, 407.1f, 899.0f, 416.9f, 881.0f, 421.6f, 873.0f, 431.5f, 867.0f, 441.4f, 862.0f, 451.6f, 849.0f };

static void edge_unseen_run(const char *p_name, const float *p_leg, uint32_t n, float face_scan)
{
    edge_meas_t e;
    float       target;
    float       d2;

    edge_reset(&e, 45.0f, face_scan - (80.0f * 0.7071068f));
    for (uint32_t i = 0u; i < n; i++)
    {
        edge_add(&e, p_leg[2u * i], p_leg[(2u * i) + 1u]);
    }
    target = edge_s_cross(&e) + 80.0f + (140.0f / edge_sin(&e));
    d2 = bypass_side_leg_mm(face_scan, 150.0f, 80.0f, 60.0f,
                            (target * edge_sin(&e)) - ((edge_s_cross(&e) + 80.0f) * edge_sin(&e)),
                            target * 0.7071068f);
    printf("  %s: box seen %u times, edge assumed %s, diagonal %.0f mm, side leg %.0f mm\n", p_name,
           (unsigned)e.n, e.b_unseen ? "yes" : "no", (double)target, (double)d2);
    CHECK("edge-unseen", e.b_unseen && e.b_edge && (e.n == 0u));
    CHECK("edge-unseen-diagonal", (target > 300.0f) && (target < 400.0f));
    CHECK("edge-unseen-side-leg", d2 < 500.0f);
}

static void test_edge_unseen_runs(void)
{
    edge_unseen_run("20:00 run 1 (10 cm box)", LEG_2000_1, (uint32_t)(sizeof(LEG_2000_1) / sizeof(LEG_2000_1[0])) / 2u, 410.0f);
    edge_unseen_run("20:00 run 2 (turned box)", LEG_2000_2, (uint32_t)(sizeof(LEG_2000_2) / sizeof(LEG_2000_2[0])) / 2u, 350.0f);
}

/* TURN_CAL (20 Nov): sensors 30 mm ahead of the axle, 10 mm either side,
   18 mm line through the axle; the car really turns `scale` x the encoder
   angle. The same onset rule as turn_cal() (black after 5 white samples). */
/* rejoin pivot rule against the logged runs (lead, trail) per frame;
   returns the frame index where the car stops, or -1 */
static int rejoin_run(const bool (*p_s)[2], int n)
{
    rejoin_watch_t w;

    rejoin_watch_reset(&w);
    for (int i = 0; i < n; i++)
    {
        if (rejoin_watch_step(&w, p_s[i][0], p_s[i][1])) { return i; }
    }
    return -1;
}

static void test_rejoin_watch(void)
{
    /* 20:30 right pivot, lead CENTRE / trail LEFT: C black at the start, C
       white, then L black (old code ignored all of it) */
    static const bool r1[][2] = { {1,0}, {1,0}, {0,0}, {0,0}, {0,1}, {0,0} };
    /* 20:32: C black at the start then white, never black again */
    static const bool r2[][2] = { {1,0}, {0,0}, {0,0}, {0,0} };
    /* normal: both white, lead black, lead white */
    static const bool r3[][2] = { {0,0}, {0,0}, {1,0}, {1,0}, {0,0} };
    /* line already between the sensors: trailing goes black */
    static const bool r4[][2] = { {0,0}, {0,0}, {0,1} };
    /* trailing black at the start (handled by the creep): not a stop */
    static const bool r5[][2] = { {0,1}, {0,1}, {0,0} };

    CHECK("rejoin-2030", rejoin_run(r1, 6) == 2);
    CHECK("rejoin-2032", rejoin_run(r2, 4) == 1);
    CHECK("rejoin-normal", rejoin_run(r3, 5) == 4);
    CHECK("rejoin-between", rejoin_run(r4, 3) == 2);
    CHECK("rejoin-trail-start", rejoin_run(r5, 3) == -1);
}

static float turn_cal_sim(float scale, float line_off_mm)
{
    float    on_l[4], on_c[4];
    uint32_t n_l = 0u, n_c = 0u, wl = 0u, wc = 0u;
    bool     pl = true, pc = true;

    for (float te = 0.0f; te <= 720.0f; te += 0.3f)
    {
        float th = -te * scale * 0.01745329f;                 /* left pivot */
        float xl = (-10.0f * cosf(th)) + (30.0f * sinf(th));
        float xc = (10.0f * cosf(th)) + (30.0f * sinf(th));
        bool  bl = fabsf(xl - line_off_mm) < 9.0f;
        bool  bc = fabsf(xc - line_off_mm) < 9.0f;

        if (bl && !pl && (wl >= 5u) && (n_l < 4u)) { on_l[n_l++] = te; }
        if (bc && !pc && (wc >= 5u) && (n_c < 4u)) { on_c[n_c++] = te; }
        wl = bl ? 0u : (wl + 1u); wc = bc ? 0u : (wc + 1u);
        pl = bl; pc = bc;
        if ((n_l >= 3u) && (n_c >= 3u)) { break; }
    }
    return turn_cal_track_mm(140.0f, on_l, n_l, on_c, n_c);
}

static void test_turn_cal(void)
{
    float a = turn_cal_sim(1.5f, 0.0f);
    float b = turn_cal_sim(1.0f, 0.0f);
    float c = turn_cal_sim(1.5f, 8.0f);                       /* axle 8 mm off the line */
    float on[3] = { 30.0f, 150.0f, 270.0f };

    printf("  TURN_CAL sim: real 1.5 x encoder -> %.1f mm (want 93.3); 1.0 x -> %.1f (want 140); axle 8 mm off the line -> %.1f\n",
           (double)a, (double)b, (double)c);
    CHECK("turn-cal-15", fabsf(a - 93.3f) < 2.0f);
    CHECK("turn-cal-10", fabsf(b - 140.0f) < 2.0f);
    CHECK("turn-cal-offset", fabsf(c - 93.3f) < 3.0f);
    CHECK("turn-cal-math", fabsf(turn_cal_track_mm(140.0f, on, 3u, on, 0u) - 93.33f) < 0.1f);
    CHECK("turn-cal-none", turn_cal_track_mm(140.0f, on, 2u, on, 1u) < 0.0f);
}

static void test_edge_measure(void)
{
    static const float W[3] = { 100.0f, 150.0f, 300.0f };
    static const float Y[2] = { 350.0f, 500.0f };
    static const float H[3] = { 45.0f, 52.0f, 58.0f };

    for (int w = 0; w < 3; w++)
    {
        for (int y = 0; y < 2; y++)
        {
            for (int h = 0; h < 3; h++)
            {
                float emin = 1e9f, emax = -1e9f, gmin = 1e9f;
                int   bad = 0;

                for (uint32_t seed = 0u; seed < 8u; seed++)
                {
                    float err, gap;

                    if (edge_run(W[w], Y[y], H[h], seed, &err, &gap) != 0) { bad++; continue; }
                    emin = (err < emin) ? err : emin; emax = (err > emax) ? err : emax;
                    gmin = (gap < gmin) ? gap : gmin;
                }
                printf("  edge: box %3.0f mm, face %3.0f mm, real pivot %2.0f: error %+4.0f..%+4.0f mm, min gap %4.0f mm%s\n",
                       (double)W[w], (double)Y[y], (double)H[h], (double)emin, (double)emax, (double)gmin, bad ? " (some runs blocked)" : "");
                CHECK("edge-no-block", bad == 0);
                CHECK("edge-no-touch", gmin > 0.0f);
                /* boxes >= 150 mm with the pivot within 7 deg of the encoders:
                   edge within 70 mm. A 100 mm box is narrower than where the
                   sensor already is after the pivot: it reads too wide (safe
                   side), up to 100 mm. A 58 deg real pivot: run TURN_TEST. */
                if ((W[w] >= 150.0f) && (H[h] < 55.0f))
                {
                    CHECK("edge-error-70mm", (emin > -70.0f) && (emax < 70.0f));
                }
                if ((W[w] < 150.0f) && (H[h] < 55.0f))
                {
                    CHECK("edge-small-box-safe-side", (emin > -100.0f) && (emax < 80.0f));
                }
            }
        }
    }
}

int main(void)
{
    test_barcode();
    test_line();
    test_scan();
    test_imu();
    test_ramp();
    test_pickup();
    test_obstacle_width();
    test_edge_measure();
    test_edge_car_run();
    test_side_leg();
    test_edge_unseen_runs();
    test_turn_cal();
    test_rejoin_watch();
    printf("is09 glue: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
