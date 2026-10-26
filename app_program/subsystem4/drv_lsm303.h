/** @file drv_lsm303.h
 *  @brief LSM303DLHC accelerometer (I2C 0x19) in normal/high-resolution mode
 *         at 100 Hz, +-2 g, block data update: one 7-byte burst per cycle
 *         (STATUS_REG_A + X/Y/Z). The magnetometer (0x1E) is only touched by
 *         the identity check. Owner: Buddy 4.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdbool.h>
#include <stdint.h>

#define LSM303_ADDR_ACC 0x19U
#define LSM303_ADDR_MAG 0x1EU

typedef struct
{
    int16_t x_mg; /* 1 mg/LSB after the 12-bit alignment */
    int16_t y_mg;
    int16_t z_mg;
    bool    fresh; /* ZYXDA: a new sample since the previous read */
} lsm303_sample_t;

/** Configure CTRL_REG1_A/CTRL_REG4_A and read them back. */
bool drv_lsm303_init (void);

/** Read STATUS + 6 data bytes; false on any bus error. */
bool drv_lsm303_read (lsm303_sample_t * p_s);

/** Identity check without WHO_AM_I: CTRL_REG1_A reads back the written
 *  configuration and the magnetometer IRA_REG_M reads 'H' (0x48). */
bool drv_lsm303_identify (uint8_t * p_ctrl1, uint8_t * p_ira);

#endif /* DRV_LSM303_H */
