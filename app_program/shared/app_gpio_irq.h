/** @file app_gpio_irq.h
 *  @brief The single IO_IRQ_BANK0 dispatcher (Rev B "GpioIrqDispatcher").
 *
 *  Exactly one interrupt handler is registered for IO_IRQ_BANK0 with
 *  tk_def_int(TA_HLNG). It reads PROC0_INTS, acknowledges the pending edge
 *  events once, timestamps the batch with the 1 MHz timer and calls each
 *  registered pin callback in pin order. Callbacks run in task-independent
 *  context: capture, store, signal (tk_set_flg / tk_wup_tsk / tk_snd_mbf
 *  TMO_POL) only - never wait, never printf.
 *
 *  Owners of the callbacks: GP0-3 Motion (EncoderIsr), GP7 Barcode
 *  (BarcodeEdgeIsr), GP17 Scan (EchoIsr), GP20/21 Controller (ButtonIsr).
 */

#ifndef APP_GPIO_IRQ_H
#define APP_GPIO_IRQ_H

#include <stdbool.h>
#include <stdint.h>

/** Pin callback: @p events is the APP_GPIO_IRQ_* nibble that fired,
 *  @p stamp_us the timer value read once at dispatcher entry. */
typedef void (*app_gpio_irq_cb_t)(uint32_t pin, uint32_t events,
                                  uint32_t stamp_us);

/** Install the dispatcher on IO_IRQ_BANK0 (idempotent). Must run before any
 *  app_gpio_irq_attach(); returns E_OK or a kernel error. */
int32_t app_gpio_irq_init (void);

/** Attach a callback and enable @p event_mask on @p pin. A pin has at most
 *  one callback (second attach to the same pin is rejected, returns false),
 *  so two subsystems cannot silently overwrite each other's handler. */
bool app_gpio_irq_attach (uint32_t pin, uint32_t event_mask,
                          app_gpio_irq_cb_t callback);

/** Disable the pin's interrupt events (callback stays registered). */
void app_gpio_irq_disable (uint32_t pin);
/** Re-enable previously attached events. */
void app_gpio_irq_enable (uint32_t pin);

/** Diagnostics for the A3 spike / self-test. */
uint32_t app_gpio_irq_dispatch_count (void);
uint32_t app_gpio_irq_spurious_count (void);

#endif /* APP_GPIO_IRQ_H */
