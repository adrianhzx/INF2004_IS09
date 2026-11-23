/** @file app_barcode.h
 *  @brief EdgeRing (ISR producer / task consumer) and the barcode result byte
 *         (owner: Buddy 3, subsystem3). Layout: app_data.h.
 *
 *  EdgeRing concurrency: single producer (BarcodeEdgeIsr) writes entries and
 *  then the head index; single consumer (BarcodeDecodeTask) reads entries and
 *  then advances the tail. Each index is a byte (single-copy atomic). The
 *  producer never touches tail, the consumer never touches head, so there is
 *  no read-modify-write across the boundary. The ring is full when
 *  (head + 1) % N == tail; the producer then drops the edge and sets the
 *  overflow byte (set by ISR, cleared by the task after reporting).
 */

#ifndef APP_BARCODE_H
#define APP_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_pack.h"

typedef struct
{
    volatile uint16_t entries[APP_EDGE_RING_ENTRIES];
    volatile uint8_t  head;     /* ISR writes */
    volatile uint8_t  tail;     /* task writes */
    volatile uint8_t  overflow; /* ISR sets 1, task clears */
    volatile uint8_t  reserved;
} app_edge_ring_t;

extern app_edge_ring_t g_edge_ring;

/** Published result byte (writer: BarcodeDecodeTask). */
extern volatile uint8_t g_barcode_byte;

/** Encode one entry. delta_us is saturated to 15 bits of 8 us units; the
 *  return value's bit 15 is the level. *p_sat reports saturation. */
static inline uint16_t
app_edge_pack (bool level, uint32_t delta_us, bool * p_sat)
{
    uint32_t units = delta_us / APP_EDGE_DELTA_UNIT_US;
    if (units > APP_EDGE_DELTA_MASK)
    {
        units = APP_EDGE_DELTA_MASK;
        if (NULL != p_sat)
        {
            *p_sat = true;
        }
    }
    return (uint16_t) ((level ? (1U << APP_EDGE_LEVEL_SHIFT) : 0U) | units);
}

static inline bool
app_edge_level (uint16_t entry)
{
    return 0U != ((uint32_t) entry & (1U << APP_EDGE_LEVEL_SHIFT));
}

static inline uint32_t
app_edge_delta_us (uint16_t entry)
{
    return ((uint32_t) entry & APP_EDGE_DELTA_MASK) * APP_EDGE_DELTA_UNIT_US;
}

/** Producer side (ISR). Returns false and sets overflow if full. */
static inline bool
app_edge_ring_push (app_edge_ring_t * p_ring, uint16_t entry)
{
    uint8_t const head = p_ring->head;
    uint8_t const next = (uint8_t) ((head + 1U) % APP_EDGE_RING_ENTRIES);
    if (next == p_ring->tail)
    {
        p_ring->overflow = 1U;
        return false;
    }
    p_ring->entries[head] = entry;
    p_ring->head          = next; /* publish after the entry is stored */
    return true;
}

/** Consumer side (task). Returns false when empty. */
static inline bool
app_edge_ring_pop (app_edge_ring_t * p_ring, uint16_t * p_entry)
{
    uint8_t const tail = p_ring->tail;
    if (tail == p_ring->head)
    {
        return false;
    }
    *p_entry     = p_ring->entries[tail];
    p_ring->tail = (uint8_t) ((tail + 1U) % APP_EDGE_RING_ENTRIES);
    return true;
}

static inline uint32_t
app_edge_ring_count (app_edge_ring_t const * p_ring)
{
    return ((uint32_t) p_ring->head + APP_EDGE_RING_ENTRIES - p_ring->tail) %
           APP_EDGE_RING_ENTRIES;
}

/** Consumer-only reset (ISR must be disarmed first). */
static inline void
app_edge_ring_reset (app_edge_ring_t * p_ring)
{
    p_ring->tail     = p_ring->head;
    p_ring->overflow = 0U;
}

/* ---- barcode result byte ---------------------------------------------- */
static inline uint8_t
app_barcode_pack (app_barcode_result_t result, uint8_t seq)
{
    uint32_t b = 0U;
    b          = app_pack_set(b, APP_BC_RESULT_SHIFT, APP_BC_RESULT_BITS,
                              (uint32_t) result);
    b          = app_pack_set(b, APP_BC_SEQ_SHIFT, APP_BC_SEQ_BITS, seq);
    return (uint8_t) b;
}

static inline app_barcode_result_t
app_barcode_result (uint8_t byte)
{
    return (app_barcode_result_t) app_pack_get(byte, APP_BC_RESULT_SHIFT,
                                               APP_BC_RESULT_BITS);
}

static inline uint8_t
app_barcode_seq (uint8_t byte)
{
    return (uint8_t) app_pack_get(byte, APP_BC_SEQ_SHIFT, APP_BC_SEQ_BITS);
}

#endif /* APP_BARCODE_H */
