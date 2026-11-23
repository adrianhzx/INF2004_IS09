/**
 * @file    hal_isr.c
 * @brief   Interrupt drivers (Pico SDK side, see hal.h), ported to the
 *          IS09 car's wiring (19 Nov).
 *
 * GPIO (IO_BANK0, IRQ 13): one dispatcher for every edge interrupt, as agreed
 * in DR Section 4 - wheel encoders (4x quadrature), HC-SR04 Echo (pulse
 * timing) and the IS09 barcode sensor DO on GP7: every edge goes with its
 * time stamp to is09_bc_isr_edge() (IS09 BarcodeEdgeIsr). The IS09 car has
 * no IMU INT1 wire.
 *
 * TIMER alarm 3 (IRQ 3), 1 kHz: every tick samples the barcode sensor AO
 * on ADC2 (GP28, BARCODE_AO = 1) and, as the base driver did, finds
 * black/white edges with hysteresis (or, BARCODE_DETECT = 1, queues every
 * sample with the wheel counts for bc_detect.c), so barcode widths are
 * measured in distance. Then one of the two IS09 analog line sensors (ADC0
 * LEFT, ADC1 CENTRE, alternately: 500 Hz each), summed for hal_line_adc().
 * The ADC has a single user. VBAT sensing shares GP28 and is not possible
 * on the IS09 car.
 */
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/timer.h"

#include "hal.h"
#include "hal_internal.h"
#include "is09_glue.h"
#include "pins.h"
#include "robot_config.h"

#define SAMPLE_US        (1000u)
#define BC_RING          (64u)          /* power of 2 */
#define RAW_RING         (256u)         /* power of 2: 256 ms of samples */

#if VBAT_ENABLED
#error "VBAT sensing uses GP28, which is the barcode AO on the IS09 car"
#endif

/* ---- encoders ---- */
static volatile int32_t  g_enc[2];      /* raw 4x counts, wheel 0 = right */
static volatile uint32_t g_odo;
static volatile uint32_t g_enc_err;
static uint8_t           g_enc_prev[2];
static volatile bool     g_enc_rev[2] = { (ENC_R_REVERSED != 0), (ENC_L_REVERSED != 0) };

/* quadrature table: index = (prev << 2) | now, value = step (0 = none/invalid) */
static const int8_t QUAD[16] =
{
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

/* ---- ultrasonic ---- */
static volatile uint64_t g_echo_rise_us;
static volatile uint32_t g_echo_us;
static volatile bool     g_echo_done;

/* ---- barcode sampling (base driver) ---- */
static volatile uint32_t g_adc_last;              /* last barcode AO sample */
static volatile uint32_t g_adc_min = 0xFFFFu;     /* range since hal_ir_adc_range() */
static volatile uint32_t g_adc_max;
static volatile bool     g_bc_enabled;
static volatile bool     g_bc_black_high = true;
static volatile uint32_t g_bc_thr_black = 3000u;   /* crossing this way -> black */
static volatile uint32_t g_bc_thr_white = 1000u;   /* crossing this way -> white */
static bool              g_bc_black;
static hal_bc_edge_t     g_bc_ring[BC_RING];
static volatile uint32_t g_bc_head;                /* written by the ISR */
static volatile uint32_t g_bc_tail;                /* written by the reader */
static volatile uint32_t g_bc_overruns;
static volatile bool     g_raw_enabled;
static int32_t           g_raw_er[RAW_RING];
static int32_t           g_raw_el[RAW_RING];
static uint16_t          g_raw_adc[RAW_RING];
static volatile uint32_t g_raw_head;               /* written by the ISR */
static volatile uint32_t g_raw_tail;               /* written by the reader */

/* ---- line sensors (1 kHz, alternately) ---- */
static volatile uint32_t g_line_sum[2];           /* [0] LEFT, [1] CENTRE, since hal_line_adc() */
static volatile uint32_t g_line_n[2];
static volatile uint32_t g_line_last[2];
static uint32_t          g_line_ch;               /* channel sampled next */
static uint32_t          g_alarm_target;
static uint32_t          g_sample_n;

/* ------------------------------------------------------------------------ */

static uint8_t enc_state(uint32_t pin_a, uint32_t pin_b)
{
    return (uint8_t)(((gpio_get(pin_a) ? 1u : 0u) << 1) | (gpio_get(pin_b) ? 1u : 0u));
}

static void enc_update(uint32_t wheel, uint32_t pin_a, uint32_t pin_b)
{
    uint8_t now  = enc_state(pin_a, pin_b);
    uint8_t prev = g_enc_prev[wheel];
    int8_t  step = QUAD[(prev << 2) | now];

    if (step != 0)
    {
        g_enc[wheel] += step;
        g_odo++;
    }
    else if (now != prev)
    {
        g_enc_err++;                          /* both channels changed: a missed edge */
    }
    else
    {
        /* bounce back to the same state */
    }
    g_enc_prev[wheel] = now;
}

void hal_irq_enable_sources(void)
{
    const uint32_t both = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    g_enc_prev[HAL_WHEEL_RIGHT] = enc_state(PIN_ENC_R_A, PIN_ENC_R_B);
    g_enc_prev[HAL_WHEEL_LEFT]  = enc_state(PIN_ENC_L_A, PIN_ENC_L_B);

    gpio_set_irq_enabled(PIN_ENC_R_A, both, true);
    gpio_set_irq_enabled(PIN_ENC_R_B, both, true);
    gpio_set_irq_enabled(PIN_ENC_L_A, both, true);
    gpio_set_irq_enabled(PIN_ENC_L_B, both, true);
    gpio_set_irq_enabled(PIN_US_ECHO, both, true);
#if !BARCODE_AO
    gpio_set_irq_enabled(PIN_BARCODE_DO, both, true);   /* is09_bc_isr_edge() ignores edges while disarmed */
#endif

    /* 1 kHz sampling on TIMER alarm 3 */
    g_alarm_target = timer_hw->timerawl + SAMPLE_US;
    timer_hw->alarm[3] = g_alarm_target;
    hw_set_bits(&timer_hw->inte, 1u << 3);
}

/* Reads the edge events of one pin (4 bits) from the latched status. */
static uint32_t pin_events(const uint32_t *p_ints, uint32_t pin)
{
    return (p_ints[pin >> 3] >> (4u * (pin & 7u))) & 0xFu;
}

void hal_gpio_isr(uint32_t intno)
{
    uint32_t       ints[4];
    uint32_t const stamp_us = timer_hw->timerawl;   /* one stamp per entry, as the IS09 dispatcher */

    (void)intno;
    for (uint32_t i = 0u; i < 4u; i++)
    {
        ints[i] = io_bank0_hw->proc0_irq_ctrl.ints[i];
        io_bank0_hw->intr[i] = ints[i];       /* acknowledge the edge events */
    }

    if ((pin_events(ints, PIN_ENC_R_A) | pin_events(ints, PIN_ENC_R_B)) != 0u)
    {
        enc_update(HAL_WHEEL_RIGHT, PIN_ENC_R_A, PIN_ENC_R_B);
    }
    if ((pin_events(ints, PIN_ENC_L_A) | pin_events(ints, PIN_ENC_L_B)) != 0u)
    {
        enc_update(HAL_WHEEL_LEFT, PIN_ENC_L_A, PIN_ENC_L_B);
    }
    if (pin_events(ints, PIN_US_ECHO) != 0u)
    {
        uint64_t now = hal_time_us();

        if (gpio_get(PIN_US_ECHO))
        {
            g_echo_rise_us = now;
        }
        else if ((g_echo_rise_us != 0u) && !g_echo_done)
        {
            g_echo_us   = (uint32_t)(now - g_echo_rise_us);
            g_echo_done = true;
        }
        else
        {
            /* stray falling edge */
        }
    }
    if (pin_events(ints, PIN_BARCODE_DO) != 0u)
    {
        is09_bc_isr_edge(gpio_get(PIN_BARCODE_DO), stamp_us);
    }
}

int32_t hal_enc_count(uint32_t wheel)
{
    int32_t c;

    wheel = (wheel == HAL_WHEEL_RIGHT) ? HAL_WHEEL_RIGHT : HAL_WHEEL_LEFT;
    c = g_enc[wheel];
    return g_enc_rev[wheel] ? -c : c;
}

void hal_enc_set_reversed(bool b_right, bool b_left)
{
    g_enc_rev[HAL_WHEEL_RIGHT] = b_right;
    g_enc_rev[HAL_WHEEL_LEFT]  = b_left;
}

uint32_t hal_odometer(void)
{
    return g_odo;
}

uint32_t hal_enc_errors(void)
{
    return g_enc_err;
}

/* ---- ultrasonic ---- */

void hal_us_arm(void)
{
    g_echo_done    = false;
    g_echo_rise_us = 0u;
}

float hal_us_read_cm(void)
{
    if (!g_echo_done)
    {
        return -1.0f;                         /* no echo (nothing within ~4 m) */
    }
    if (g_echo_us > 25000u)
    {
        return -1.0f;
    }
    return (float)g_echo_us / 58.0f;
}

/* ---- line-sensor sampling (1 kHz) ---- */

void hal_barcode_config(uint32_t thr_black, uint32_t thr_white, bool b_black_high, bool b_enable)
{
    g_bc_enabled    = false;
    g_bc_thr_black  = thr_black;
    g_bc_thr_white  = thr_white;
    g_bc_black_high = b_black_high;
    g_bc_black      = false;
    g_bc_tail       = g_bc_head;
    g_bc_enabled    = b_enable;
}

/* the base 1 kHz barcode step on one AO sample (unchanged logic) */
static void barcode_sample(uint32_t v)
{
    g_adc_last = v;
    if (v < g_adc_min)
    {
        g_adc_min = v;
    }
    if (v > g_adc_max)
    {
        g_adc_max = v;
    }

    if (g_raw_enabled)
    {
        uint32_t head = g_raw_head;

        if ((head - g_raw_tail) < RAW_RING)
        {
            g_raw_er[head & (RAW_RING - 1u)]  = g_enc_rev[HAL_WHEEL_RIGHT] ? -g_enc[HAL_WHEEL_RIGHT] : g_enc[HAL_WHEEL_RIGHT];
            g_raw_el[head & (RAW_RING - 1u)]  = g_enc_rev[HAL_WHEEL_LEFT] ? -g_enc[HAL_WHEEL_LEFT] : g_enc[HAL_WHEEL_LEFT];
            g_raw_adc[head & (RAW_RING - 1u)] = (uint16_t)v;
            g_raw_head = head + 1u;
        }
        else
        {
            g_bc_overruns++;
        }
    }
    else if (g_bc_enabled)
    {
        bool b_black = g_bc_black;

        if (g_bc_black_high)
        {
            b_black = g_bc_black ? (v > g_bc_thr_white) : (v >= g_bc_thr_black);
        }
        else
        {
            b_black = g_bc_black ? (v < g_bc_thr_white) : (v <= g_bc_thr_black);
        }
        if (b_black != g_bc_black)
        {
            uint32_t head = g_bc_head;

            g_bc_black = b_black;
            if ((head - g_bc_tail) < BC_RING)
            {
                g_bc_ring[head & (BC_RING - 1u)].odo     = g_odo;
                g_bc_ring[head & (BC_RING - 1u)].t_ms    = (uint32_t)(hal_time_us() / 1000u);
                g_bc_ring[head & (BC_RING - 1u)].b_black = b_black;
                g_bc_head = head + 1u;
            }
            else
            {
                g_bc_overruns++;
            }
        }
    }
    else
    {
        /* sampling only (range for the dashboard) */
    }
}

void hal_timer_isr(uint32_t intno)
{
    uint32_t v;
    uint32_t ch;

    (void)intno;
    timer_hw->intr = 1u << 3;                     /* INTR is write-1-to-clear */

    /* next sample, without drift; resynchronise if we fell behind */
    g_alarm_target += SAMPLE_US;
    if ((int32_t)(g_alarm_target - timer_hw->timerawl) <= 0)
    {
        g_alarm_target = timer_hw->timerawl + SAMPLE_US;
    }
    timer_hw->alarm[3] = g_alarm_target;

    /* barcode AO first, every tick (two conversions per tick: ~4 us) */
    adc_select_input(ADC_CH_BARCODE);
    barcode_sample(adc_read());

    /* one line sensor */
    ch = g_line_ch;
    adc_select_input((ch == 0u) ? ADC_CH_LINE_LEFT : ADC_CH_LINE_CENTRE);
    v = adc_read();
    g_line_ch = 1u - ch;
    g_line_last[ch] = v;
    g_line_sum[ch] += v;
    g_line_n[ch]++;
    g_sample_n++;
}

/* Mean of each channel since the last call (the last sample if none came).
   Called from TSK_LINE_BARCODE only; the interrupt is masked for the copy. */
void hal_line_adc(uint32_t *p_left, uint32_t *p_centre)
{
    uint32_t sum[2];
    uint32_t n[2];
    uint32_t saved = save_and_disable_interrupts();

    for (uint32_t i = 0u; i < 2u; i++)
    {
        sum[i] = g_line_sum[i];
        n[i]   = g_line_n[i];
        g_line_sum[i] = 0u;
        g_line_n[i]   = 0u;
    }
    restore_interrupts(saved);
    *p_left   = (n[0] > 0u) ? (sum[0] / n[0]) : g_line_last[0];
    *p_centre = (n[1] > 0u) ? (sum[1] / n[1]) : g_line_last[1];
}

bool hal_barcode_pop(hal_bc_edge_t *p_edge)
{
    uint32_t tail = g_bc_tail;

    if (tail == g_bc_head)
    {
        return false;
    }
    *p_edge   = g_bc_ring[tail & (BC_RING - 1u)];
    g_bc_tail = tail + 1u;
    return true;
}

void hal_barcode_raw(bool b_enable)
{
    g_raw_enabled = false;
    g_raw_tail    = g_raw_head;
    if (b_enable)
    {
        g_bc_enabled = false;                     /* no threshold edges in raw mode */
    }
    g_raw_enabled = b_enable;
}

bool hal_barcode_pop_sample(int32_t *p_enc_r, int32_t *p_enc_l, uint32_t *p_adc)
{
    uint32_t tail = g_raw_tail;

    if (tail == g_raw_head)
    {
        return false;
    }
    *p_enc_r = g_raw_er[tail & (RAW_RING - 1u)];
    *p_enc_l = g_raw_el[tail & (RAW_RING - 1u)];
    *p_adc = g_raw_adc[tail & (RAW_RING - 1u)];
    g_raw_tail = tail + 1u;
    return true;
}

uint32_t hal_barcode_overruns(void)
{
    return g_bc_overruns;
}

uint32_t hal_ir_adc(void)
{
    return g_adc_last;
}

void hal_ir_adc_range(uint32_t *p_min, uint32_t *p_max)
{
    uint32_t mn = g_adc_min;
    uint32_t mx = g_adc_max;

    g_adc_min = 0xFFFFu;                          /* a sample in between is lost at worst */
    g_adc_max = 0u;
    *p_min = (mn > 4095u) ? g_adc_last : mn;
    *p_max = (mx == 0u) ? g_adc_last : mx;
}

uint32_t hal_vbat_mv(void)
{
    return 0u;                                    /* GP28 is the barcode AO */
}
