/** @file app_pack.h
 *  @brief Mask/shift field helpers used for every shared or wire field.
 *
 *  All arithmetic is on unsigned types (BARR-C 5.3.b: no bitwise operators on
 *  signed operands; no implementation-defined right shift of a negative
 *  value).  Signed fields are converted with explicit two's-complement
 *  arithmetic and range checks.  Every "pack" helper saturates and reports
 *  whether saturation happened so the caller can raise a quality or fault
 *  indication instead of pretending the value was in range (rule 6).
 *
 *  Header-only by design (DEV-3): these must inline into 10 ms loops.
 */

#ifndef APP_PACK_H
#define APP_PACK_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h" /* NULL, APP_STATIC_ASSERT */

#define APP_PACK_WORD_BITS 32U
#define APP_PACK_WORD_MASK 0xFFFFFFFFU
#define APP_PACK_BYTE_MASK 0xFFU
#define APP_PACK_BYTE_BITS 8U
#define APP_PACK_SHIFT_16 16U
#define APP_PACK_SHIFT_24 24U
#define APP_PACK_U8_MAX 255U
#define APP_PACK_B2 2U /* byte indices in LE helpers */
#define APP_PACK_B3 3U

/** Mask of @p bits low bits (bits must be 1..32). */
static inline uint32_t
app_pack_mask (uint32_t bits)
{
    return (bits >= APP_PACK_WORD_BITS) ? APP_PACK_WORD_MASK
                                        : ((1UL << bits) - 1UL);
}

/** Insert an unsigned field. Caller guarantees value fits (use *_sat). */
static inline uint32_t
app_pack_set (uint32_t word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t const mask = app_pack_mask(bits) << shift;
    return (word & ~mask) | ((value << shift) & mask);
}

/** Extract an unsigned field. */
static inline uint32_t
app_pack_get (uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & app_pack_mask(bits);
}

/** Saturate an unsigned value to @p bits. Sets *p_sat when clipped. */
static inline uint32_t
app_pack_sat_u (uint32_t value, uint32_t bits, bool * p_sat)
{
    uint32_t const max = app_pack_mask(bits);
    if (value > max)
    {
        if (NULL != p_sat)
        {
            *p_sat = true;
        }
        return max;
    }
    return value;
}

/** Saturate a signed value to a @p bits two's-complement field. */
static inline int32_t
app_pack_sat_s (int32_t value, uint32_t bits, bool * p_sat)
{
    /* bits is 2..32; the range is -(2^(bits-1)) .. 2^(bits-1)-1 */
    int32_t const max = (int32_t) (app_pack_mask(
        bits - 1U)); /* 2^(b-1)-1, fits int32 for b<=32 */
    int32_t const min = -max - 1;
    if (value > max)
    {
        if (NULL != p_sat)
        {
            *p_sat = true;
        }
        return max;
    }
    if (value < min)
    {
        if (NULL != p_sat)
        {
            *p_sat = true;
        }
        return min;
    }
    return value;
}

/** Two's-complement encode a signed value (already in range) into @p bits
 *  using unsigned arithmetic only. */
static inline uint32_t
app_pack_s_to_u (int32_t value, uint32_t bits)
{
    uint32_t const mask = app_pack_mask(bits);
    /* C99 6.3.1.3p2: conversion of a negative value to an unsigned type is
       defined as value + 2^32, i.e. the two's-complement bit pattern. The
       caller has already range-checked value to bits, so masking keeps
       exactly the low `bits` bits of that pattern. */
    uint32_t const raw = (uint32_t) value;
    return raw & mask;
}

/** Decode a @p bits two's-complement field with explicit sign extension. */
static inline int32_t
app_pack_u_to_s (uint32_t raw, uint32_t bits)
{
    uint32_t const mask     = app_pack_mask(bits);
    uint32_t const sign_bit = 1UL << (bits - 1U);
    uint32_t const field    = raw & mask;
    if (0U != (field & sign_bit))
    {
        /* negative: value = field - 2^bits, computed without signed overflow */
        uint32_t const magnitude = (mask - field) + 1U; /* 1 .. 2^(bits-1) */
        return -(int32_t) (magnitude - 1U) - 1;         /* -(magnitude) */
    }
    return (int32_t) field; /* fits: field <= 2^(bits-1)-1 */
}

/** Insert a signed field with saturation. */
static inline uint32_t
app_pack_set_s (uint32_t word, uint32_t shift, uint32_t bits, int32_t value,
                bool * p_sat)
{
    int32_t const clipped = app_pack_sat_s(value, bits, p_sat);
    return app_pack_set(word, shift, bits, app_pack_s_to_u(clipped, bits));
}

/** Extract a signed field. */
static inline int32_t
app_pack_get_s (uint32_t word, uint32_t shift, uint32_t bits)
{
    return app_pack_u_to_s(app_pack_get(word, shift, bits), bits);
}

/** Insert an unsigned field with saturation. */
static inline uint32_t
app_pack_set_sat (uint32_t word, uint32_t shift, uint32_t bits, uint32_t value,
                  bool * p_sat)
{
    return app_pack_set(word, shift, bits, app_pack_sat_u(value, bits, p_sat));
}

/* ---- little-endian byte helpers for wire frames ------------------------ */

static inline void
app_pack_le16 (uint8_t * p_dst, uint16_t value)
{
    uint32_t const v = (uint32_t) value;
    p_dst[0]         = (uint8_t) (v & APP_PACK_BYTE_MASK);
    p_dst[1] = (uint8_t) ((v >> APP_PACK_BYTE_BITS) & APP_PACK_BYTE_MASK);
}

static inline void
app_pack_le32 (uint8_t * p_dst, uint32_t value)
{
    p_dst[0] = (uint8_t) (value & APP_PACK_BYTE_MASK);
    p_dst[1] = (uint8_t) ((value >> APP_PACK_BYTE_BITS) & APP_PACK_BYTE_MASK);
    p_dst[APP_PACK_B2] =
        (uint8_t) ((value >> APP_PACK_SHIFT_16) & APP_PACK_BYTE_MASK);
    p_dst[APP_PACK_B3] =
        (uint8_t) ((value >> APP_PACK_SHIFT_24) & APP_PACK_BYTE_MASK);
}

static inline uint16_t
app_unpack_le16 (uint8_t const * p_src)
{
    return (uint16_t) ((uint32_t) p_src[0] |
                       ((uint32_t) p_src[1] << APP_PACK_BYTE_BITS));
}

static inline uint32_t
app_unpack_le32 (uint8_t const * p_src)
{
    return (uint32_t) p_src[0] | ((uint32_t) p_src[1] << APP_PACK_BYTE_BITS) |
           ((uint32_t) p_src[APP_PACK_B2] << APP_PACK_SHIFT_16) |
           ((uint32_t) p_src[APP_PACK_B3] << APP_PACK_SHIFT_24);
}

/** Saturating 8-bit increment (dropped-event counters). */
static inline uint8_t
app_pack_inc_sat8 (uint8_t value)
{
    return (value < APP_PACK_U8_MAX) ? (uint8_t) (value + 1U)
                                     : (uint8_t) APP_PACK_U8_MAX;
}

#endif /* APP_PACK_H */
