/** @file test_line.c
 *  @brief Host tests for subsystem 3: line_core (H-LN-*) and barcode_core
 *         (H-BC-*). Element widths for the barcode tests are generated from
 *         the measured sample geometry (narrow 3 mm, wide 9 mm) at several
 *         speeds, forwards and reversed, with glitches and defects.
 */

#include <stdio.h>
#include <string.h>

#include "app_cal.h"
#include "app_line.h"
#include "barcode_core.h"
#include "host_port.h"
#include "line_core.h"

/* NOLINTBEGIN(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, desc)                                                  \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            g_pass++;                                                          \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            g_fail++;                                                          \
            printf("FAIL %s %s (%s:%d)\n", id, desc, __FILE__, __LINE__);      \
        }                                                                      \
    } while (0)

/* ---- line ------------------------------------------------------------------
 */

#define WHITE 900U
#define BLACK 3100U
#define LEFT_EDGE 1500U /* ~27 % darkness: on-line left reading */

static void
calibrated_core (line_core_t * p_core)
{
    line_core_init(p_core);
    line_core_set_references(p_core, WHITE, BLACK, WHITE, BLACK);
}

static void
test_line_basic (void)
{
    line_core_t core;
    line_out_t  out;
    uint32_t    i = 0U;

    CHECK("H-LN-01", 0U == line_core_normalise(WHITE, WHITE, BLACK),
          "white -> 0");
    CHECK("H-LN-02", 100U == line_core_normalise(BLACK, WHITE, BLACK),
          "black -> 100");
    CHECK("H-LN-03", 50U == line_core_normalise(2000U, WHITE, BLACK),
          "midpoint -> 50");
    CHECK("H-LN-04", 0U == line_core_normalise(100U, WHITE, BLACK),
          "below white clamps 0");
    CHECK("H-LN-05", 100U == line_core_normalise(4095U, WHITE, BLACK),
          "above black clamps 100");
    CHECK("H-LN-06", 0U == line_core_normalise(2000U, 2000U, 2000U),
          "degenerate references -> 0");

    /* boot references (1000/3000) give contrast at once; a flat signal lets
       the running references relax together until the span collapses */
    line_core_init(&core);
    line_core_step(&core, 2000U, 2000U, &out);
    CHECK("H-LN-07a", out.contrast_ok,
          "boot references usable before calibration");
    /* (3000 - 1000 - 400) / DECAY steps x DECAY_CYCLES cycles to collapse */
    for (i = 0U; i < 100000U; i++)
    {
        line_core_step(&core, 2000U, 2000U, &out);
    }
    CHECK("H-LN-07b", !out.contrast_ok,
          "flat signal for ~15 min -> IR_CONTRAST lost (slow relax, D-34)");
    /* D-32: while the contrast is insufficient, noise must not become a
       line: error 0, steer 0, no dark, LOST reported once, no JUNCTION */
    {
        bool quiet = true;
        bool lost  = false;
        line_core_init(&core);
        for (i = 0U; i < 100000U; i++)
        {
            line_core_step(&core, 2000U, 2000U, &out);
            lost = lost || out.evt_lost;
        }
        for (i = 0U; i < 30U; i++)
        {
            line_core_step(&core, (0U == (i & 1U)) ? 1990U : 2010U,
                           (0U == (i & 1U)) ? 2010U : 1990U, &out);
            if ((0 != out.state.line_error) || (0 != out.state.steer) ||
                out.evt_junction || out.evt_found || out.state.on_line)
            {
                quiet = false;
            }
        }
        CHECK("H-LN-07c", quiet && lost && !out.contrast_ok,
              "uncalibrated: noise gives error 0, no junction, off line, LOST "
              "was reported");
    }
    line_core_step(&core, WHITE, WHITE, &out);
    line_core_step(&core, BLACK, BLACK, &out);
    CHECK("H-LN-08", out.contrast_ok,
          "references expand immediately when both shades are seen");

    /* D-34: a car parked on the line (left white, centre black) and driven
       down a lengthy straight keeps its calibration: the shared pair is fed by
       both channels, so neither forgets the shade it never sees */
    line_core_init(&core);
    for (i = 0U; i < 100000U; i++)
    {
        line_core_step(&core, 2000U, 2000U, &out); /* collapse first */
    }
    line_core_step(&core, WHITE, BLACK, &out);
    CHECK("H-LN-08b", out.contrast_ok,
          "one sensor white + one black calibrates both channels at once");
    for (i = 0U; i < 100000U; i++)
    {
        line_core_step(&core, WHITE, BLACK, &out);
    }
    CHECK("H-LN-08c",
          out.contrast_ok && (out.state.line_error == 100) && out.state.on_line,
          "15 min on a straight line: still calibrated, error +100, on line");

    /* on line: centre dark, left on the edge (~27 %) -> error ~ +73 */
    calibrated_core(&core);
    line_core_step(&core, LEFT_EDGE, BLACK, &out);
    CHECK("H-LN-09",
          (out.state.line_error > 60) && (out.state.line_error < 85) &&
              out.state.on_line,
          "centre dark, left on edge: error ~ +73, on_line");
    /* line drifted right: left white, centre black -> +100 */
    line_core_step(&core, WHITE, BLACK, &out);
    CHECK("H-LN-10", (100 == out.state.line_error) && (out.state.steer > 0),
          "line right -> error +100, steer > 0");
    /* line drifted left: left black, centre white -> -100 */
    line_core_step(&core, BLACK, WHITE, &out);
    CHECK("H-LN-11", (-100 == out.state.line_error) && (out.state.steer < 0),
          "line left -> error -100, steer < 0");
    CHECK("H-LN-12", (out.state.steer >= -100) && (out.state.steer <= 100),
          "steer clamped to +-100");
    /* seq advances every step, 4 bits */
    {
        uint8_t const s = out.state.seq;
        line_core_step(&core, BLACK, WHITE, &out);
        CHECK("H-LN-13", out.state.seq == ((s + 1U) & 0x0FU),
              "seq increments mod 16");
    }
}

static void
test_line_events (void)
{
    line_core_t core;
    line_out_t  out;
    uint32_t    i         = 0U;
    uint32_t    junctions = 0U;
    uint32_t    losts     = 0U;
    uint32_t    founds    = 0U;

    calibrated_core(&core);
    /* on line for a while */
    for (i = 0U; i < 20U; i++)
    {
        line_core_step(&core, LEFT_EDGE, BLACK, &out);
    }
    /* both dark: junction after T_JUNC (60 ms = 6 cycles), only once */
    for (i = 0U; i < 30U; i++)
    {
        line_core_step(&core, BLACK, BLACK, &out);
        junctions += out.evt_junction ? 1U : 0U;
        if (out.evt_junction)
        {
            CHECK("H-LN-14", 5U == i,
                  "JUNCTION raised on the 6th both-dark cycle");
            CHECK("H-LN-15", APP_LINE_EVT_JUNCTION == out.state.event,
                  "event field = JUNCTION");
        }
    }
    CHECK("H-LN-16", 1U == junctions,
          "junction reported once while staying dark");
    /* brief both-dark (3 cycles) does not trigger */
    for (i = 0U; i < 10U; i++)
    {
        line_core_step(&core, LEFT_EDGE, BLACK, &out);
    }
    for (i = 0U; i < 3U; i++)
    {
        line_core_step(&core, BLACK, BLACK, &out);
        junctions += out.evt_junction ? 1U : 0U;
    }
    line_core_step(&core, LEFT_EDGE, BLACK, &out);
    CHECK("H-LN-17", 1U == junctions,
          "3-cycle both-dark blip is filtered (hysteresis)");
    /* re-armed: a second real junction triggers again */
    for (i = 0U; i < 10U; i++)
    {
        line_core_step(&core, BLACK, BLACK, &out);
        junctions += out.evt_junction ? 1U : 0U;
    }
    CHECK("H-LN-18", 2U == junctions,
          "junction re-arms after leaving both-dark");

    /* D-35 dead band: both sensors on the line's edges (A7 straddle, ~50 %
       each) for 2 s is neither a junction nor a loss, error 0 */
    {
        uint32_t evts = 0U;
        bool     on   = true;
        bool     zero = true;
        for (i = 0U; i < 200U; i++)
        {
            /* jitter 40..60 % around the edge */
            uint16_t const g = (0U == (i % 3U)) ? 1800U : 2200U;
            line_core_step(&core, g, (0U == (i % 3U)) ? 2200U : 1800U, &out);
            evts += (out.evt_junction || out.evt_lost) ? 1U : 0U;
            on   = on && out.state.on_line;
            zero = zero && (out.state.line_error >= -25) &&
                   (out.state.line_error <= 25);
        }
        CHECK("H-LN-18b", (0U == evts) && on && zero,
              "both sensors on the edges for 2 s: on line, no JUNCTION, no "
              "LOST, small error");
    }

    /* both light (straddle bracket, D-36): on_line false at once (no sensor
       sees the line), error 0 = straight ahead, LOST only after T_LOST */
    {
        uint32_t const lost_cycles = (APP_CAL_T_LOST_MS + 9U) / 10U;
        bool           straight    = true;
        for (i = 0U; i < lost_cycles + 15U; i++)
        {
            line_core_step(&core, WHITE, WHITE, &out);
            losts += out.evt_lost ? 1U : 0U;
            if (out.evt_lost)
            {
                CHECK("H-LN-19", (lost_cycles - 1U) == i,
                      "LOST on the T_LOST-th both-light cycle");
            }
            if (i < (lost_cycles - 1U))
            {
                straight = straight && (0 == out.state.line_error) &&
                           !out.evt_lost && !out.state.on_line;
            }
        }
        CHECK("H-LN-20", straight,
              "before T_LOST: error 0, no LOST, on_line false (no sensor on "
              "the line)");
        CHECK("H-LN-21", (1U == losts) && !out.state.on_line,
              "LOST once, on_line false");
    }
    line_core_step(&core, WHITE, BLACK, &out);
    founds += out.evt_found ? 1U : 0U;
    CHECK("H-LN-22",
          (1U == founds) && out.state.on_line &&
              (APP_LINE_EVT_FOUND == out.state.event),
          "FOUND immediately when a sensor goes dark again");
    line_core_step(&core, WHITE, BLACK, &out);
    CHECK("H-LN-23", !out.evt_found && (APP_LINE_EVT_NONE == out.state.event),
          "FOUND reported once");

    /* packed round trip of a live state */
    {
        uint32_t         w = 0U;
        app_line_state_t v;
        (void) app_line_pack(&out.state, &w);
        app_line_unpack(w, &v);
        CHECK("H-LN-24",
              (v.line_error == out.state.line_error) &&
                  (v.on_line == out.state.on_line) && (v.seq == out.state.seq),
              "LineState word round trip");
    }
    /* raw value above 12 bits is clamped, not wrapped */
    line_core_step(&core, 65535U, 65535U, &out);
    CHECK("H-LN-25", 100U == out.n_left, "raw > 4095 clamps to black");
}

/* ---- barcode ------------------------------------------------------------- */

static char const * const gp_start = "nwnnwnwnn";

/** Build the 29-element width list for '*' X '*' at a given narrow width. */
static uint32_t
build_widths (char const * p_middle, uint32_t narrow_us, uint32_t wide_us,
              uint16_t * p_w, bool reversed)
{
    char     seq[29];
    uint32_t i = 0U;
    for (i = 0U; i < 9U; i++)
    {
        seq[i]       = gp_start[i];
        seq[10U + i] = p_middle[i];
        seq[20U + i] = gp_start[i];
    }
    seq[9U]  = 'n';
    seq[19U] = 'n';
    for (i = 0U; i < 29U; i++)
    {
        uint32_t const w   = ('w' == seq[i]) ? wide_us : narrow_us;
        uint32_t const idx = reversed ? (28U - i) : i;
        p_w[idx] =
            (uint16_t) (w / 8U); /* 8 us units, as the core stores them */
    }
    return 29U;
}

/** Feed widths through the state machine as edges, then the quiet timeout. */
static void
run_capture (barcode_core_t * p_core, uint16_t const * p_w, uint32_t n,
             barcode_out_t * p_final)
{
    barcode_out_t out;
    uint32_t      i = 0U;
    barcode_core_edge(p_core, 500000U,
                      &out); /* first edge after an extended quiet zone */
    CHECK("H-BC-00", !out.result_ready, "no result on the start edge");
    for (i = 0U; i < n; i++)
    {
        barcode_core_edge(p_core, (uint32_t) p_w[i] * 8U, &out);
        if (out.result_ready)
        {
            *p_final = out;
            return;
        }
    }
    barcode_core_quiet(p_core, false, p_final);
}

static void
test_barcode_decode (void)
{
    uint16_t w[64];
    bool     rev = false;
    uint32_t n   = 0U;
    static struct
    {
        char const *         middle;
        app_barcode_result_t expect;
        char                 symbol;
    } const cases[] = {
        {"wnnnnwnnw", APP_BC_LEFT, 'A'},
        {"nnwnnwnnw", APP_BC_RIGHT, 'B'},
        {"wnwnnwnnn", APP_BC_STRAIGHT, 'C'},
        {"nnnnwwnnw", APP_BC_UTURN, 'D'},
        {"nwwnwnnnn", APP_BC_INV_NO_MATCH, 'Z'},
    };
    uint32_t c     = 0U;
    uint32_t speed = 0U;
    /* narrow widths for 3 mm at 50, 20, 10 cm/s: 6, 15, 30 ms */
    uint32_t const narrow_us[3] = {6000U, 15000U, 30000U};

    for (c = 0U; c < 5U; c++)
    {
        for (speed = 0U; speed < 3U; speed++)
        {
            app_barcode_result_t r = APP_BC_NONE;
            n = build_widths(cases[c].middle, narrow_us[speed],
                             narrow_us[speed] * 3U, w, false);
            r = barcode_core_decode(w, n, &rev);
            CHECK("H-BC-01", (r == cases[c].expect) && !rev,
                  "forward decode at 3 speeds");
            n = build_widths(cases[c].middle, narrow_us[speed],
                             narrow_us[speed] * 3U, w, true);
            r = barcode_core_decode(w, n, &rev);
            CHECK("H-BC-02",
                  (r == cases[c].expect) && ((APP_BC_INV_NO_MATCH == r) || rev),
                  "reversed decode (after a U-turn) at 3 speeds");
        }
        CHECK("H-BC-03",
              cases[c].expect == barcode_core_map_char(cases[c].symbol),
              "character map");
    }
    /* +-20 % width noise still decodes (3:1 ratio has margin) */
    {
        uint32_t i = 0U;
        n          = build_widths("nnwnnwnnw", 15000U, 45000U, w, false);
        for (i = 0U; i < n; i++)
        {
            w[i] =
                (uint16_t) ((i & 1U) ? (w[i] * 12U / 10U) : (w[i] * 8U / 10U));
        }
        CHECK("H-BC-04", APP_BC_RIGHT == barcode_core_decode(w, n, &rev),
              "+-20 % width jitter decodes");
    }
    /* defects */
    n = build_widths("wnnnnwnnw", 15000U, 45000U, w, false);
    CHECK("H-BC-05", APP_BC_INV_NO_MATCH == barcode_core_decode(w, 28U, &rev),
          "28 elements: no match");
    w[13] = 45000U / 8U; /* flip a narrow to wide: 4 wide in the character */
    CHECK("H-BC-06", APP_BC_INV_NO_MATCH == barcode_core_decode(w, n, &rev),
          "corrupted element: no match");
    {
        uint32_t i = 0U;
        for (i = 0U; i < n; i++)
        {
            w[i] = 15000U / 8U; /* all equal: no two width classes */
        }
        CHECK("H-BC-07", APP_BC_INV_NO_MATCH == barcode_core_decode(w, n, &rev),
              "no width contrast: no match");
    }
    CHECK("H-BC-08", APP_BC_INV_NO_MATCH == barcode_core_map_char('Q'),
          "unknown character maps to no match");
}

static void
test_barcode_state_machine (void)
{
    barcode_core_t core;
    barcode_out_t  out;
    uint16_t       w[64];
    uint32_t       n = 0U;
    uint32_t       i = 0U;

    barcode_core_init(&core);
    CHECK("H-BC-09", BC_DISARMED == barcode_core_state(&core),
          "boot: DISARMED");
    barcode_core_edge(&core, 500000U, &out);
    barcode_core_edge(&core, 3000U, &out);
    CHECK("H-BC-10",
          (BC_DISARMED == barcode_core_state(&core)) && !out.result_ready,
          "edges ignored while disarmed");

    barcode_core_arm(&core);
    CHECK("H-BC-11", BC_WAIT_QUIET == barcode_core_state(&core),
          "ARM -> WAIT_QUIET");
    /* edges without a preceding quiet zone do not start a capture */
    barcode_core_edge(&core, 3000U, &out);
    CHECK("H-BC-12", BC_WAIT_QUIET == barcode_core_state(&core),
          "brief gap: still waiting for quiet");

    /* full 'C' capture ends by the quiet timeout */
    n = build_widths("wnwnnwnnn", 15000U, 45000U, w, false);
    run_capture(&core, w, n, &out);
    CHECK("H-BC-13",
          out.result_ready && (APP_BC_STRAIGHT == out.result) &&
              (1U == out.seq),
          "'C' captured -> STRAIGHT, seq 1");
    CHECK("H-BC-14", BC_WAIT_QUIET == barcode_core_state(&core),
          "back to WAIT_QUIET after a result");

    /* second code: seq 2; reversed 'B' */
    n = build_widths("nnwnnwnnw", 15000U, 45000U, w, true);
    run_capture(&core, w, n, &out);
    CHECK("H-BC-15",
          out.result_ready && (APP_BC_RIGHT == out.result) && (2U == out.seq),
          "reversed 'B' -> RIGHT, seq 2");

    /* slow crawl (~6.7 cm/s): narrow 45 ms, wide 135 ms. 135 ms would have
       saturated a microsecond uint16_t; in 8 us units it is exact. The floor
       is set by T_QUIET: a wide element >= 150 ms reads as a quiet zone, so
       the slowest decodable speed is 3 mm * 3 / 150 ms = 6 cm/s (TEST_LOG). */
    n = build_widths("nnnnwwnnw", 45000U, 135000U, w, false);
    run_capture(&core, w, n, &out);
    CHECK("H-BC-15b", out.result_ready && (APP_BC_UTURN == out.result),
          "'D' at 6.7 cm/s -> UTURN (no width saturation)");

    /* stray mark: 5 elements then quiet -> silent */
    barcode_core_edge(&core, 500000U, &out);
    for (i = 0U; i < 5U; i++)
    {
        barcode_core_edge(&core, 15000U, &out);
    }
    barcode_core_quiet(&core, false, &out);
    CHECK("H-BC-16",
          !out.result_ready && (1U == core.stats_stray) &&
              (BC_WAIT_QUIET == barcode_core_state(&core)),
          "stray mark discarded silently");

    /* partial code: 22 elements -> INVALID / NO_MATCH reported */
    n = build_widths("wnnnnwnnw", 15000U, 45000U, w, false);
    run_capture(&core, w, 22U, &out);
    CHECK("H-BC-17", out.result_ready && (APP_BC_INV_NO_MATCH == out.result),
          "partial code -> NO_MATCH reported");

    /* glitch inside a wide bar: 45 ms = 20 + 0.1 (glitch) + 24.9 */
    {
        barcode_out_t o2;
        barcode_core_edge(&core, 500000U, &o2);
        n = build_widths("wnnnnwnnw", 15000U, 45000U, w, false);
        for (i = 0U; i < n; i++)
        {
            if (10U == i) /* first element of 'A' is wide: split it with a 100
                             us glitch */
            {
                barcode_core_edge(&core, 20000U, &o2);
                barcode_core_edge(&core, 100U, &o2);
                barcode_core_edge(&core, 24900U, &o2);
            }
            else
            {
                barcode_core_edge(&core, (uint32_t) w[i] * 8U, &o2);
            }
        }
        barcode_core_quiet(&core, false, &o2);
        CHECK("H-BC-18", o2.result_ready && (APP_BC_LEFT == o2.result),
              "100 us glitch merged: 'A' still LEFT");
    }

    /* a long gap mid-capture ends the capture (decode) and starts a new one */
    barcode_core_edge(&core, 500000U, &out);
    for (i = 0U; i < 25U; i++)
    {
        barcode_core_edge(&core, 15000U, &out);
    }
    barcode_core_edge(&core, 400000U,
                      &out); /* gap: previous 25-element capture decodes */
    CHECK("H-BC-19",
          out.result_ready && (APP_BC_INV_NO_MATCH == out.result) &&
              (BC_CAPTURING == barcode_core_state(&core)),
          "gap ends the capture (NO_MATCH) and a new capture starts");
    barcode_core_quiet(&core, false, &out);

    /* ring overflow notice -> INV_OVERFLOW + fault, ring reset by the task */
    barcode_core_edge(&core, 500000U, &out);
    barcode_core_edge(&core, 15000U, &out);
    barcode_core_quiet(&core, true, &out);
    CHECK("H-BC-20",
          out.result_ready && (APP_BC_INV_OVERFLOW == out.result) &&
              out.overflow_fault,
          "overflow during capture -> INV_OVERFLOW + fault");

    /* element overflow: more than 64 elements */
    barcode_core_edge(&core, 500000U, &out);
    for (i = 0U; i < 70U; i++)
    {
        barcode_core_edge(&core, 15000U, &out);
        if (out.result_ready)
        {
            break;
        }
    }
    CHECK("H-BC-21",
          out.result_ready && (APP_BC_INV_OVERFLOW == out.result) && (64U == i),
          "65th element -> INV_OVERFLOW");

    /* seq wraps at 16 and is never 0 after the first result */
    for (i = 0U; i < 20U; i++)
    {
        n = build_widths("wnnnnwnnw", 15000U, 45000U, w, false);
        run_capture(&core, w, n, &out);
    }
    CHECK("H-BC-22", out.result_ready && (out.seq < 16U),
          "seq stays in 4 bits");

    /* disarm clears everything */
    barcode_core_disarm(&core);
    barcode_core_quiet(&core, true, &out);
    CHECK("H-BC-23",
          (BC_DISARMED == barcode_core_state(&core)) && !out.result_ready,
          "disarmed quiet/overflow: nothing");
}

int
main (void)
{
    test_line_basic();
    test_line_events();
    test_barcode_decode();
    test_barcode_state_machine();
    printf("line/barcode host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
/* NOLINTEND(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
