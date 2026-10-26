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
#define BC_US_PER_UNIT 8U /* same unit as app_edge_pack (NFR28) */

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

app_barcode_result_t
barcode_core_decode (uint16_t const * p_width, uint32_t count,
                     bool * p_reversed)
{
    char                 pattern[APP_CAL_BC_ELEMENTS];
    char                 reversed[APP_CAL_BC_ELEMENTS];
    uint32_t             w_min     = BC_WIDTH_SAT;
    uint32_t             w_max     = 0U;
    uint32_t             threshold = 0U;
    uint32_t             ratio_q8  = 0U;
    uint32_t             i         = 0U;
    app_barcode_result_t res       = APP_BC_INV_NO_MATCH;

    *p_reversed = false;
    if (APP_CAL_BC_ELEMENTS != count)
    {
        return APP_BC_INV_NO_MATCH;
    }
    for (i = 0U; i < count; i++)
    {
        if (p_width[i] < w_min)
        {
            w_min = p_width[i];
        }
        if (p_width[i] > w_max)
        {
            w_max = p_width[i];
        }
    }
    if (0U == w_min)
    {
        return APP_BC_INV_NO_MATCH;
    }
    ratio_q8 = (w_max * BC_Q8) / w_min;
    if ((ratio_q8 < APP_CAL_BC_RATIO_MIN_Q8) ||
        (ratio_q8 > APP_CAL_BC_RATIO_MAX_Q8))
    {
        return APP_BC_INV_NO_MATCH; /* no two clear width classes */
    }
    threshold = (w_min + w_max) / BC_HALF;
    for (i = 0U; i < count; i++)
    {
        pattern[i] = (p_width[i] > threshold) ? BC_WIDE : BC_NARROW;
        reversed[count - 1U - i] = pattern[i];
    }
    res = decode_pattern(pattern);
    if (APP_BC_INV_NO_MATCH == res)
    {
        res = decode_pattern(reversed);
        if (APP_BC_INV_NO_MATCH != res)
        {
            *p_reversed = true;
        }
    }
    return res;
}

/* ---- state machine ---------------------------------------------------------
 */

void
barcode_core_init (barcode_core_t * p_core)
{
    p_core->state         = (uint8_t) BC_DISARMED;
    p_core->count         = 0U;
    p_core->merge_pending = false;
    p_core->seq           = 0U;
    p_core->stats_decoded = 0U;
    p_core->stats_invalid = 0U;
    p_core->stats_stray   = 0U;
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
        app_barcode_result_t const res =
            barcode_core_decode(p_core->width, p_core->count, &reversed);
        publish(p_core, p_out, res);
    }
}
