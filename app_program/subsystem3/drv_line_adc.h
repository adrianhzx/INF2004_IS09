/** @file drv_line_adc.h
 *  @brief Polled one-shot ADC driver for the two line sensors (ADC0 = GP26
 *         left, ADC1 = GP27 centre). Owner: LineSensingTask only (the kernel
 *         ADC device driver is switched off in config/config_device.h so the
 *         ADC block has exactly one owner, NFR23).
 *
 *  A conversion takes 96 ADC clocks = 2 us at 48 MHz; both channels are read
 *  back to back inside the 10 ms task with a bounded poll.
 */

#ifndef DRV_LINE_ADC_H
#define DRV_LINE_ADC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DRV_LINE_ADC_LEFT   = 0, /* ADC0, GP26 */
    DRV_LINE_ADC_CENTRE = 1  /* ADC1, GP27 */
} drv_line_adc_ch_t;

/** Enable the ADC block, configure GP26/GP27 as analogue inputs (digital
 *  input and pulls off). Returns false if the block never reports READY. */
bool drv_line_adc_init (void);

/** One 12-bit conversion. Returns false on a conversion error or timeout;
 *  *p_raw is then left unchanged. */
bool drv_line_adc_read (drv_line_adc_ch_t channel, uint16_t * p_raw);

/** Conversion error counter (ERR bit or timeout), saturating. */
uint8_t drv_line_adc_errors (void);

#endif /* DRV_LINE_ADC_H */
