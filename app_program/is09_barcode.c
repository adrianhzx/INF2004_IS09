/**
 * @file    is09_barcode.c
 * @brief   IS09 barcode reader inside this software (see is09_glue.h).
 *
 * Same chain as IS09 subsystem 3 (task_barcode.c), with the decode task
 * replaced by a 10 ms poll from TSK_LINE_BARCODE:
 *   GP7 edge (GPIO interrupt) -> time since the previous edge, packed into
 *   the IS09 EdgeRing (16 us units) -> barcode_core_edge() per edge ->
 *   barcode_core_quiet() once no edge has arrived for APP_CAL_T_QUIET_MS
 *   (the quiet zone) -> 29 widths classified per character (3 widest of 9
 *   are wide), read forwards or backwards.
 *
 * barcode_core maps only A/B/C/D to directions. The base planner maps any
 * letter (nav_map.c), so the letter itself is read from the 29 classified
 * elements barcode_core leaves in last_pattern: start '*', narrow gap, the
 * character, narrow gap, stop '*', forwards or backwards, looked up in the
 * full Code 39 table (code39.c). barcode_core itself is unchanged.
 */
#include "is09_glue.h"
#include "code39.h"
#include "is09/shared/app_barcode.h"
#include "is09/subsystem3/barcode_core.h"

#define BC_DRAIN_MAX     (APP_EDGE_RING_ENTRIES)
#define BC_T_QUIET_US    (APP_CAL_T_QUIET_MS * 1000u)
#define BC_UNITS_PER_MS  (125u)       /* barcode_core stores widths in 8 us units */
#define BC_UNITS_PER_TENTH (13u)      /* 12.5 rounded, as the IS09 console */
#define BC_CHAR_LEN      (9u)
#define BC_GAP1          (9u)
#define BC_CHAR2         (10u)
#define BC_GAP2          (19u)
#define BC_CHAR3         (20u)

/* The IS09 globals declared in app_barcode.h */
app_edge_ring_t  g_edge_ring;
volatile uint8_t g_barcode_byte;

static barcode_core_t    g_bc;
static volatile bool     g_armed;
static volatile uint32_t g_last_edge_us;     /* ISR-private while armed */
static volatile uint32_t g_edges;
static uint32_t          g_results;

static const char STAR[BC_CHAR_LEN + 1u] = "nwnnwnwnn";

/* ------------------------------------------------------------------------ */

void is09_bc_init(void)
{
    g_armed = false;
    barcode_core_init(&g_bc);
    app_edge_ring_reset(&g_edge_ring);
    g_barcode_byte = 0u;
    g_edges   = 0u;
    g_results = 0u;
}

/* Interrupt context: one difference, one ring push (IS09 barcode_edge_isr). */
void is09_bc_isr_edge(bool b_level, uint32_t stamp_us)
{
    uint32_t delta;
    bool     b_sat = false;

    if (!g_armed)
    {
        return;
    }
    delta = stamp_us - g_last_edge_us;
    g_last_edge_us = stamp_us;
    g_edges++;
    (void)app_edge_ring_push(&g_edge_ring, app_edge_pack(b_level, delta, &b_sat));
}

void is09_bc_arm(bool b_arm, uint32_t now_us)
{
    g_armed = false;                          /* the ISR ignores edges from here */
    app_edge_ring_reset(&g_edge_ring);
    if (b_arm)
    {
        g_last_edge_us = now_us;
        barcode_core_arm(&g_bc);
        g_armed = true;
    }
    else
    {
        barcode_core_disarm(&g_bc);
    }
}

/* ------------------------------------------------------------------------ */

static bool same9(const char *p_a, const char *p_b)
{
    for (uint32_t i = 0u; i < BC_CHAR_LEN; i++)
    {
        if (p_a[i] != p_b[i])
        {
            return false;
        }
    }
    return true;
}

static char lookup9(const char *p_pat)
{
    uint32_t w[BC_CHAR_LEN];

    for (uint32_t i = 0u; i < BC_CHAR_LEN; i++)
    {
        if ((p_pat[i] != 'n') && (p_pat[i] != 'w'))
        {
            return 0;                         /* '?': not classified */
        }
        w[i] = (p_pat[i] == 'w') ? 3u : 1u;
    }
    return code39_char(w);
}

/* '*' gap X gap '*' in the order given; the letter, or 0. */
static char frame_letter(const char *p_pat)
{
    char c;

    if (!same9(&p_pat[0], STAR) || !same9(&p_pat[BC_CHAR3], STAR) ||
        (p_pat[BC_GAP1] != 'n') || (p_pat[BC_GAP2] != 'n'))
    {
        return 0;
    }
    c = lookup9(&p_pat[BC_CHAR2]);
    return (c == '*') ? 0 : c;
}

/* The letter in the last 29-element capture, forwards or backwards. */
static char letter_of_last(bool *p_rev)
{
    char rev[APP_CAL_BC_ELEMENTS];
    char c;

    *p_rev = false;
    if (g_bc.last_count != APP_CAL_BC_ELEMENTS)
    {
        return 0;
    }
    c = frame_letter(g_bc.last_pattern);
    if (c != 0)
    {
        return c;
    }
    /* backwards: each character is classified on its own 9 widths, so the
       mirrored pattern is the pattern of the mirrored widths */
    for (uint32_t i = 0u; i < APP_CAL_BC_ELEMENTS; i++)
    {
        rev[APP_CAL_BC_ELEMENTS - 1u - i] = g_bc.last_pattern[i];
    }
    c = frame_letter(rev);
    *p_rev = (c != 0);
    return c;
}

static bool take_result(const barcode_out_t *p_out, is09_bc_result_t *p_res)
{
    if (p_out->overflow_fault)
    {
        app_edge_ring_reset(&g_edge_ring);
        p_res->b_overflow = true;
    }
    if (!p_out->result_ready)
    {
        return false;
    }
    g_barcode_byte = app_barcode_pack(p_out->result, p_out->seq);
    g_results++;
    p_res->b_done = true;
    p_res->count  = g_bc.last_count;
    p_res->letter = letter_of_last(&p_res->b_reversed);
    p_res->b_letter = (p_res->letter != 0);
    return true;
}

bool is09_bc_service(uint32_t now_us, is09_bc_result_t *p_res)
{
    uint16_t      entry = 0u;
    uint32_t      n = 0u;
    barcode_out_t out;
    bool          b_got = false;

    p_res->b_done     = false;
    p_res->b_letter   = false;
    p_res->letter     = 0;
    p_res->b_reversed = false;
    p_res->b_overflow = false;
    p_res->count      = 0u;

    /* one result per call: a second one waits in the ring for the next 10 ms */
    while (!b_got && (n < BC_DRAIN_MAX) && app_edge_ring_pop(&g_edge_ring, &entry))
    {
        n++;
        barcode_core_edge(&g_bc, app_edge_delta_us(entry), &out);
        b_got = take_result(&out, p_res);
    }
    if (!b_got && (g_edge_ring.overflow != 0u))
    {
        g_edge_ring.overflow = 0u;            /* consumer clears after noticing */
        barcode_core_quiet(&g_bc, true, &out);
        b_got = take_result(&out, p_res);
    }
    /* quiet zone: capturing, nothing left in the ring, no edge for T_QUIET */
    if (!b_got && g_armed && (barcode_core_state(&g_bc) == BC_CAPTURING) &&
        (app_edge_ring_count(&g_edge_ring) == 0u) &&
        ((uint32_t)(now_us - g_last_edge_us) >= BC_T_QUIET_US))
    {
        barcode_core_quiet(&g_bc, false, &out);
        b_got = take_result(&out, p_res);
    }
    return b_got;
}

/* ------------------------------------------------------------------------ */
/* Diagnostics text (IS09 subsystem1.c serial format)                        */
/* ------------------------------------------------------------------------ */

typedef struct
{
    char    *p;
    uint32_t size;
    uint32_t len;
} txt_t;

static void t_char(txt_t *p_t, char c)
{
    if ((p_t->len + 1u) < p_t->size)
    {
        p_t->p[p_t->len] = c;
        p_t->len++;
        p_t->p[p_t->len] = '\0';
    }
}

static void t_str(txt_t *p_t, const char *p_s)
{
    while (*p_s != '\0')
    {
        t_char(p_t, *p_s);
        p_s++;
    }
}

static void t_uint(txt_t *p_t, uint32_t v)
{
    char     d[10];
    uint32_t n = 0u;

    do
    {
        d[n] = (char)('0' + (char)(v % 10u));
        n++;
        v /= 10u;
    } while ((v != 0u) && (n < sizeof(d)));
    while (n > 0u)
    {
        n--;
        t_char(p_t, d[n]);
    }
}

/* 8 us units -> "X.Y" ms */
static void t_ms(txt_t *p_t, uint32_t units)
{
    t_uint(p_t, units / BC_UNITS_PER_MS);
    t_char(p_t, '.');
    t_uint(p_t, (units % BC_UNITS_PER_MS) / BC_UNITS_PER_TENTH);
}

static void t_init(txt_t *p_t, char *p_buf, uint32_t size)
{
    p_t->p    = p_buf;
    p_t->size = size;
    p_t->len  = 0u;
    if (size > 0u)
    {
        p_buf[0] = '\0';
    }
}

uint32_t is09_bc_capture_count(void)
{
    return g_bc.last_dump_seq / 2u;
}

void is09_bc_capture_text(char *p_buf, uint32_t size)
{
    txt_t t;
    bool  b_rev = false;
    char  c;

    t_init(&t, p_buf, size);
    t_str(&t, "cap=");
    t_uint(&t, g_bc.last_count);
    t_str(&t, " n=");
    t_ms(&t, g_bc.last_w_min);
    t_str(&t, "ms w=");
    t_ms(&t, g_bc.last_w_max);
    t_str(&t, "ms stray=");
    t_uint(&t, g_bc.stats_stray);
    t_str(&t, " pat=");
    if ((g_bc.last_count != APP_CAL_BC_ELEMENTS) || (g_bc.last_pattern[0] == '-'))
    {
        t_char(&t, '-');                      /* not a 29-width capture */
    }
    else
    {
        for (uint32_t i = 0u; i < APP_CAL_BC_ELEMENTS; i++)
        {
            if ((i == BC_GAP1) || (i == BC_CHAR2) || (i == BC_GAP2) || (i == BC_CHAR3))
            {
                t_char(&t, ' ');
            }
            t_char(&t, g_bc.last_pattern[i]);
        }
    }
    c = letter_of_last(&b_rev);
    if (c != 0)
    {
        t_str(&t, " bc=");
        t_char(&t, c);
        if (b_rev)
        {
            t_str(&t, " R");
        }
    }
}

void is09_bc_widths_text(char *p_buf, uint32_t size)
{
    txt_t    t;
    uint32_t s1 = g_bc.last_dump_seq;
    uint32_t n  = g_bc.last_count;

    t_init(&t, p_buf, size);
    t_str(&t, "widths #");
    t_uint(&t, s1 / 2u);
    t_str(&t, " (");
    t_uint(&t, n);
    t_str(&t, ", ms):");
    for (uint32_t k = 0u; (k < n) && (k < BC_MAX_ELEMENTS); k++)
    {
        t_str(&t, ((k & 1u) == 0u) ? " b" : " s");
        t_ms(&t, g_bc.last_width[k]);
    }
}

void is09_bc_health_text(char *p_buf, uint32_t size, bool b_pin)
{
    static const char * const NAMES[4] = { "DISARMED", "WAIT_QUIET", "CAPTURING", "DECODING" };
    txt_t t;

    t_init(&t, p_buf, size);
    t_str(&t, "pin=");
    t_uint(&t, b_pin ? 1u : 0u);
    t_str(&t, " st=");
    t_str(&t, NAMES[(uint32_t)barcode_core_state(&g_bc) & 3u]);
    t_str(&t, " edges=");
    t_uint(&t, g_edges);
    t_str(&t, " results=");
    t_uint(&t, g_results);
}
