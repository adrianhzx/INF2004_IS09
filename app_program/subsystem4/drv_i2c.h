/** @file drv_i2c.h
 *  @brief Task-owned polled I2C0 master on GP4 (SDA) / GP5 (SCL) for the
 *         LSM303DLHC. The kernel I2C device driver stays OFF
 *         (DEVCNF_USE_IIC 0); this driver programs the DW_apb_i2c block
 *         directly. Every wait is bounded by a microsecond deadline, so a
 *         stuck bus or an absent sensor costs at most DRV_I2C_TIMEOUT_US and
 *         returns false instead of blocking ImuTask. Owner: Buddy 4.
 */

#ifndef DRV_I2C_H
#define DRV_I2C_H

#include <stdbool.h>
#include <stdint.h>

#define DRV_I2C_TIMEOUT_US                                                     \
    2000U /* per transaction (7-byte read ~250 us at 400 kHz) */

/** Reset and configure I2C0 for APP_CAL_IMU_I2C_HZ; routes GP4/GP5.
 *  Returns false if the block never left reset. Safe to call again. */
bool drv_i2c_init (void);

/** Write one register: START addr W, reg, value, STOP. */
bool drv_i2c_write_reg (uint8_t addr7, uint8_t reg, uint8_t value);

/** Read @p n bytes from @p reg (caller sets any auto-increment bit):
 *  START addr W, reg, RESTART addr R, n bytes, STOP. */
bool drv_i2c_read_regs (uint8_t addr7, uint8_t reg, uint8_t * p_buf,
                        uint32_t n);

/** Diagnostics: transactions aborted (NACK/arbitration) and timed out. */
uint32_t drv_i2c_aborts (void);
uint32_t drv_i2c_timeouts (void);

#endif /* DRV_I2C_H */
