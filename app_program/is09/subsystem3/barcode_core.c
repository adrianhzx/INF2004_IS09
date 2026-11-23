/** @file barcode_core.c
 *  @brief Pure Code 39 barcode decoder. See barcode_core.h.
 */

#include "barcode_core.h"

#include "../shared/app_cal.h"

#define BC_CHAR_ELEMENTS 9U
#define BC_GAP1_INDEX 9U
#define BC_CHAR2_INDEX 10U
#define BC_GAP2_INDEX 19U
#define BC_CHAR3_INDEX 20U
#define BC_Q8 256U
#define BC_SEQ_MASK 0x0FU
#define BC_HALF 2U
#define BC_WIDE 'w'
#define BC_NARROW 'n'
#define BC_T_QUIET_US (APP_CAL_T_QUIET_MS * 1000U)
#define BC_WIDTH_SAT                                                           \
    0xFFFFU /* 8 us units: 524 ms, beyond T_QUIET so never reached */
#define BC_US_PER_UNIT                                                         \
    8U /* stored widths (u16, 524 ms); edge deltas arrive in us */
/* A quiet zone must be shorter than the largest edge delta the ring can
   carry, or WAIT_QUIET could never see one (10 Nov). */
#if ((APP_CAL_T_QUIET_MS * 1000U) >=                                           \
     (APP_EDGE_DELTA_MASK * APP_EDGE_DELTA_UNIT_US))
#error "APP_CAL_T_QUIET_MS must be below the EdgeRing delta range"
#endif

/* Code 39 element patterns, bars and spaces interleaved b s b s b s b s b.
   '*' and 'A' were read directly off the supplied sample pages; B, C, D are
   the standard table in the same convention (bars 01001/0100, 11000/0100,
   00101/0100). 'Z' (second sample) is kept as a decoder sanity vector. */
typedef struct
{
    char                 symbol;
    char const *         pattern; /* 9 chars of 'n'/'w' */
    app_barcode_result_t result;
} bc_symbol_t;

static bc_symbol_t const g_bc_symbols[] = {
    {'A', "wnnnnwnnw", APP_BC_LEFT},         {'B', "nnwnnwnnw", APP_BC_RIGHT},
    {'C', "wnwnnwnnn", APP_BC_STRAIGHT},     {'D', "nnnnwwnnw", APP_BC_UTURN},
    {'Z', "nwwnwnnnn", APP_BC_INV_NO_MATCH}, /* known symbol, not a command */
};
#define BC_SYMBOL_COUNT (sizeof(g_bc_symbols) / sizeof(g_bc_symbols[0]))

static char const g_bc_start_stop[] = "nwnnwnwnn"; /* '*' as measured */

app_barcode_result_t
barcode_core_map_char (char code39_char)
{
    uint32_t i = 0U;
    for (i = 0U; i < BC_SYMBOL_COUNT; i++)
    {
        if (g_bc_symbols[i].symbol == code39_char)
        {
            return g_bc_symbols[i].result;
        }
    }
    return APP_BC_INV_NO_MATCH;
}

static bool
pattern_equals (char const * p_pattern, char const * p_ref)
{
    uint32_t i = 0U;
    for (i = 0U; i < BC_CHAR_ELEMENTS; i++)
    {
        if (p_pattern[i] != p_ref[i])
        {
            return false;
        }
    }
    return true;
}

static app_barcode_result_t
decode_pattern (char const * p_pat)
{
    uint32_t i = 0U;

    if (!pattern_equals(&p_pat[0], g_bc_start_stop) ||
        !pattern_equals(&p_pat[BC_CHAR3_INDEX], g_bc_start_stop))
    {
        return APP_BC_INV_NO_MATCH;
    }
    if ((BC_NARROW != p_pat[BC_GAP1_INDEX]) ||
        (BC_NARROW != p_pat[BC_GAP2_INDEX]))
    {
        return APP_BC_INV_NO_MATCH;
    }
    for (i = 0U; i < BC_SYMBOL_COUNT; i++)
    {
        if (pattern_equals(&p_pat[BC_CHAR2_INDEX], g_bc_symbols[i].pattern))
        {
            return g_bc_symbols[i].result;
        }
    }
    return APP_BC_INV_NO_MATCH;
}

/* Per-character classification (10 Nov): every Code 39 character, the
   '*' start/stop included, has exactly 3 wide elements out of 9. Taking the
   3 widest of each character as wide makes the read independent of the
   print size (50 / 75 / 100 % A4) and of speed changes across the barcode,
   which a single threshold over all 29 widths is not: a narrow bar crossed
   slowly can be longer than a wide bar crossed fast. */
#define BC_CHAR_WIDE_COUNT 3U
#define BC_CHAR_COUNT 3U
#define BC_FIRST 0U  /* start '*' */
#define BC_MIDDLE 1U /* the data character */
#define BC_LAST 2U   /* stop '*' */

static uint32_t const g_bc_char_start[BC_CHAR_COUNT] = {0U, BC_CHAR2_INDEX,
                                                        BC_CHAR3_INDEX};

/** Classify one 9-element character; returns the wide / narrow midpoint, or
 *  0 when the widths do not split into 3 clearly wider and 6 narrower. */
static uint32_t
classify_char (uint16_t const * p_w, char * p_pat)
{
    bool     is_wide[BC_CHAR_ELEMENTS];
    uint32_t i         = 0U;
    uint32_t k         = 0U;
    uint32_t min_wide  = BC_WIDTH_SAT;
    uint32_t max_wide  = 0U;
    uint32_t min_narr  = BC_WIDTH_SAT;
    uint32_t max_narr  = 0U;
    uint32_t threshold = 0U;

    for (i = 0U; i < BC_CHAR_ELEMENTS; i++)
    {
        is_wide[i] = false;
    }
    for (k = 0U; k < BC_CHAR_WIDE_COUNT; k++)
    {
        uint32_t best = BC_CHAR_ELEMENTS; /* none yet */
        for (i = 0U; i < BC_CHAR_ELEMENTS; i++)
        {
            if (!is_wide[i] &&
                ((BC_CHAR_ELEMENTS == best) || (p_w[i] > p_w[best])))
            {
                best = i;
            }
        }
        is_wide[best] = true;
    }
    for (i = 0U; i < BC_CHAR_ELEMENTS; i++)
    {
        uint32_t const v = p_w[i];
        if (is_wide[i])
        {
            min_wide = (v < min_wide) ? v : min_wide;
            max_wide = (v > max_wide) ? v : max_wide;
        }
        else
        {
            min_narr = (v < min_narr) ? v : min_narr;
            max_narr = (v > max_narr) ? v : max_narr;
        }
        p_pat[i] = is_wide[i] ? BC_WIDE : BC_NARROW;
    }
    /* the narrowest wide must clear the widest narrow by RATIO_MIN, and the
       spread inside one character must stay plausible (RATIO_MAX) */
    if ((0U == min_narr) ||
        ((min_wide * BC_Q8) < (max_narr * APP_CAL_BC_RATIO_MIN_Q8)) ||
        ((max_wide * BC_Q8) > (min_narr * APP_CAL_BC_RATIO_MAX_Q8)))
    {
        return 0U;
    }
    threshold = (max_narr + min_wide) / BC_HALF;
    return (0U == threshold) ? 1U : threshold;
}

/** Classify all 29 widths in the order given; false when a character is
 *  ambiguous. The two gaps are judged against their neighbours. */
static bool
classify_all (uint16_t const * p_w, char * p_pat)
{
    uint32_t thr[BC_CHAR_COUNT];
    uint32_t c = 0U;

    for (c = 0U; c < BC_CHAR_COUNT; c++)
    {
        thr[c] =
            classify_char(&p_w[g_bc_char_start[c]], &p_pat[g_bc_char_start[c]]);
        if (0U == thr[c])
        {
            return false;
        }
    }
    p_pat[BC_GAP1_INDEX] = ((uint32_t) p_w[BC_GAP1_INDEX] >
                            ((thr[BC_FIRST] + thr[BC_MIDDLE]) / BC_HALF))
                               ? BC_WIDE
                               : BC_NARROW;
    p_pat[BC_GAP2_INDEX] = ((uint32_t) p_w[BC_GAP2_INDEX] >
                            ((thr[BC_MIDDLE] + thr[BC_LAST]) / BC_HALF))
                               ? BC_WIDE
                               : BC_NARROW;
    return true;
}

app_barcode_result_t
barcode_core_decode_pattern (uint16_t const * p_width, uint32_t count,
                             bool * p_reversed, char * p_pattern)
{
    uint16_t             rev_w[APP_CAL_BC_ELEMENTS];
    char                 rev_pat[APP_CAL_BC_ELEMENTS];
    uint32_t             i   = 0U;
    app_barcode_result_t res = APP_BC_INV_NO_MATCH;

    *p_reversed = false;
    for (i = 0U; i < APP_CAL_BC_ELEMENTS; i++)
    {
        p_pattern[i] = '?';
    }
    if (APP_CAL_BC_ELEMENTS != count)
    {
        return APP_BC_INV_NO_MATCH;
    }
    if (!classify_all(p_width, p_pattern))
    {
        return APP_BC_INV_NO_MATCH; /* no clear 3-of-9 split somewhere */
    }
    res = decode_pattern(p_pattern);
    if (APP_BC_INV_NO_MATCH == res)
    {
        /* read backwards (after a U-turn): classify the mirrored list, the
           character groups land on the same positions */
        for (i = 0U; i < count; i++)
        {
            rev_w[count - 1U - i] = p_width[i];
        }
        if (classify_all(rev_w, rev_pat))
        {
            res = decode_pattern(rev_pat);
            if (APP_BC_INV_NO_MATCH != res)
            {
                *p_reversed = true;
            }
        }
    }
    return res;
}

app_barcode_result_t
barcode_core_decode (uint16_t const * p_width, uint32_t count,
                     bool * p_reversed)
{
    char pattern[APP_CAL_BC_ELEMENTS];
    return barcode_core_decode_pattern(p_width, count, p_reversed, pattern);
}

/* ---- state machine ---------------------------------------------------------
 */

void
barcode_core_init (barcode_core_t * p_core)
{
    p_core->state           = (uint8_t) BC_DISARMED;
    p_core->count           = 0U;
    p_core->merge_pending   = false;
    p_core->seq             = 0U;
    p_core->stats_decoded   = 0U;
    p_core->stats_invalid   = 0U;
    p_core->stats_stray     = 0U;
    p_core->last_count      = 0U;
    p_core->last_w_min      = 0U;
    p_core->last_w_max      = 0U;
    p_core->last_pattern[0] = '-';
    p_core->last_reversed   = false;
    p_core->last_dump_seq   = 0U;
#if defined(APP_TEST) && APP_TEST
    p_core->dbg_seq   = 0U;
    p_core->dbg_count = 0U;
#endif
}

void
barcode_core_arm (barcode_core_t * p_core)
{
    p_core->state         = (uint8_t) BC_WAIT_QUIET;
    p_core->count         = 0U;
    p_core->merge_pending = false;
}

void
barcode_core_disarm (barcode_core_t * p_core)
{
    p_core->state = (uint8_t) BC_DISARMED;
    p_core->count = 0U;
}

bc_state_t
barcode_core_state (barcode_core_t const * p_core)
{
    return (bc_state_t) p_core->state;
}

static void
clear_out (barcode_out_t * p_out)
{
    p_out->result_ready   = false;
    p_out->result         = APP_BC_NONE;
    p_out->seq            = 0U;
    p_out->overflow_fault = false;
}

static uint16_t
to_units (uint32_t delta_us)
{
    uint32_t const u = (delta_us + (BC_US_PER_UNIT / 2U)) / BC_US_PER_UNIT;
    return (u > BC_WIDTH_SAT) ? (uint16_t) BC_WIDTH_SAT : (uint16_t) u;
}

static void
append_to_last (barcode_core_t * p_core, uint32_t delta_us)
{
    uint32_t const sum = (uint32_t) p_core->width[p_core->count - 1U] +
                         (uint32_t) to_units(delta_us);
    p_core->width[p_core->count - 1U] =
        (sum > BC_WIDTH_SAT) ? (uint16_t) BC_WIDTH_SAT : (uint16_t) sum;
}

static void
publish (barcode_core_t * p_core, barcode_out_t * p_out,
         app_barcode_result_t result)
{
    p_core->seq         = (uint8_t) ((p_core->seq + 1U) & BC_SEQ_MASK);
    p_out->result_ready = true;
    p_out->result       = result;
    p_out->seq          = p_core->seq;
    if ((APP_BC_INV_NO_MATCH == result) || (APP_BC_INV_OVERFLOW == result))
    {
        p_core->stats_invalid++;
    }
    else
    {
        p_core->stats_decoded++;
    }
    p_core->state = (uint8_t) BC_WAIT_QUIET;
    p_core->count = 0U;
}

void
barcode_core_edge (barcode_core_t * p_core, uint32_t delta_us,
                   barcode_out_t * p_out)
{
    clear_out(p_out);
    switch ((bc_state_t) p_core->state)
    {
        case BC_DISARMED:
            break;
        case BC_WAIT_QUIET:
            if (delta_us >= BC_T_QUIET_US)
            {
                p_core->state =
                    (uint8_t) BC_CAPTURING; /* first bar starts now */
                p_core->count         = 0U;
                p_core->merge_pending = false;
            }
            break;
        case BC_CAPTURING:
            if (delta_us >= BC_T_QUIET_US)
            {
                /* a gap ended the previous capture before the timeout fired */
                barcode_core_quiet(p_core, false, p_out);
                p_core->state         = (uint8_t) BC_CAPTURING;
                p_core->count         = 0U;
                p_core->merge_pending = false;
            }
            else if (p_core->merge_pending)
            {
                /* continuation of the element that a glitch interrupted */
                append_to_last(p_core, delta_us);
                p_core->merge_pending = false;
            }
            else if (delta_us < APP_CAL_T_GLITCH_US)
            {
                /* glitch: fold it into the previous element; the element that
                   follows has the same colour as that one, so merge it too */
                if (p_core->count > 0U)
                {
                    append_to_last(p_core, delta_us);
                    p_core->merge_pending = true;
                }
            }
            else if (p_core->count < BC_MAX_ELEMENTS)
            {
                p_core->width[p_core->count] = to_units(delta_us);
                p_core->count++;
            }
            else
            {
                p_out->overflow_fault = true;
                publish(p_core, p_out, APP_BC_INV_OVERFLOW);
            }
            break;
        case BC_DECODING:
        default:
            p_core->state = (uint8_t) BC_WAIT_QUIET;
            break;
    }
}

/** Capture summary for telemetry: how many widths, narrowest, widest. */
static void
record_last (barcode_core_t * p_core)
{
    uint32_t k     = 0U;
    uint16_t w_min = BC_WIDTH_SAT;
    uint16_t w_max = 0U;

    for (k = 0U; (k < p_core->count) && (k < BC_MAX_ELEMENTS); k++)
    {
        if (p_core->width[k] < w_min)
        {
            w_min = p_core->width[k];
        }
        if (p_core->width[k] > w_max)
        {
            w_max = p_core->width[k];
        }
    }
    p_core->last_dump_seq++; /* odd: copy in progress */
    for (k = 0U; (k < p_core->count) && (k < BC_MAX_ELEMENTS); k++)
    {
        p_core->last_width[k] = p_core->width[k];
    }
    p_core->last_count      = p_core->count;
    p_core->last_w_min      = (0U == p_core->count) ? 0U : w_min;
    p_core->last_w_max      = w_max;
    p_core->last_pattern[0] = '-'; /* filled in only when 29 widths decode */
    p_core->last_reversed   = false;
    p_core->last_dump_seq++; /* even: complete */
}

void
barcode_core_quiet (barcode_core_t * p_core, bool overflow,
                    barcode_out_t * p_out)
{
    clear_out(p_out);
    if ((uint8_t) BC_CAPTURING != p_core->state)
    {
        if (overflow && ((uint8_t) BC_DISARMED != p_core->state))
        {
            p_out->overflow_fault = true;
        }
        return;
    }
    p_core->merge_pending = false;
    record_last(p_core);
#if defined(APP_TEST) && APP_TEST
    {
        uint32_t k = 0U;
        p_core->dbg_seq++; /* odd: copy in progress */
        p_core->dbg_count = p_core->count;
        for (k = 0U; (k < p_core->count) && (k < BC_MAX_ELEMENTS); k++)
        {
            p_core->dbg_width[k] = p_core->width[k];
        }
        p_core->dbg_seq++; /* even: copy complete */
    }
#endif
    if (overflow)
    {
        p_out->overflow_fault = true;
        publish(p_core, p_out, APP_BC_INV_OVERFLOW);
    }
    else if (p_core->count < APP_CAL_BC_N_MIN)
    {
        p_core->stats_stray++;
        p_core->state = (uint8_t) BC_WAIT_QUIET; /* stray mark, silent */
        p_core->count = 0U;
    }
    else
    {
        bool                       reversed = false;
        app_barcode_result_t const res      = barcode_core_decode_pattern(
            p_core->width, p_core->count, &reversed, p_core->last_pattern);
        p_core->last_reversed = reversed;
        publish(p_core, p_out, res);
    }
}
