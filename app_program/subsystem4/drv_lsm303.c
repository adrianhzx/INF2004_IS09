/** @file drv_lsm303.c
 *  @brief LSM303DLHC accelerometer access. See drv_lsm303.h.
 *         Register values from the datasheet (DocID018771 Rev 2, section 7).
 */

#include "drv_lsm303.h"

#include "drv_i2c.h"

#define LSM_CTRL_REG1_A 0x20U
#define LSM_CTRL_REG4_A 0x23U
#define LSM_STATUS_REG_A 0x27U
#define LSM_IRA_REG_M 0x0AU
#define LSM_AUTO_INC 0x80U /* sub-address MSB: address increments per byte */

/* ODR 100 Hz (0101), normal mode (LPen 0), X/Y/Z enabled */
#define LSM_CTRL1_100HZ_XYZ 0x57U
/* BDU 1, little endian, FS +-2 g, HR 1 */
#define LSM_CTRL4_BDU_HR_2G 0x88U
#define LSM_IRA_EXPECTED 0x48U /* 'H' */
#define LSM_STATUS_ZYXDA 0x08U

#define LSM_BURST_BYTES 7U
#define LSM_IDX_STATUS 0U
#define LSM_IDX_XL 1U
#define LSM_IDX_XH 2U
#define LSM_IDX_YL 3U
#define LSM_IDX_YH 4U
#define LSM_IDX_ZL 5U
#define LSM_IDX_ZH 6U
#define LSM_BYTE_SHIFT 8U
#define LSM_ALIGN_SHIFT 4 /* 12-bit left-justified -> 1 mg/LSB at +-2 g HR */
#define LSM_SIGN_BIT 0x8000U
#define LSM_TWO_POW_16 65536

static int16_t
to_mg (uint8_t lo, uint8_t hi)
{
    uint32_t const raw = ((uint32_t) hi << LSM_BYTE_SHIFT) | (uint32_t) lo;
    int32_t const  s = (raw >= LSM_SIGN_BIT) ? ((int32_t) raw - LSM_TWO_POW_16)
                                             : (int32_t) raw;
    return (int16_t) (s / (1 << LSM_ALIGN_SHIFT)); /* 12-bit left-justified ->
                                                      mg, sign kept */
}

bool
drv_lsm303_init (void)
{
    uint8_t ctrl1 = 0U;

    if (!drv_i2c_write_reg(LSM303_ADDR_ACC, LSM_CTRL_REG1_A,
                           LSM_CTRL1_100HZ_XYZ))
    {
        return false;
    }
    if (!drv_i2c_write_reg(LSM303_ADDR_ACC, LSM_CTRL_REG4_A,
                           LSM_CTRL4_BDU_HR_2G))
    {
        return false;
    }
    if (!drv_i2c_read_regs(LSM303_ADDR_ACC, LSM_CTRL_REG1_A, &ctrl1, 1U))
    {
        return false;
    }
    return LSM_CTRL1_100HZ_XYZ == ctrl1;
}

bool
drv_lsm303_read (lsm303_sample_t * p_s)
{
    uint8_t buf[LSM_BURST_BYTES];

    if (!drv_i2c_read_regs(LSM303_ADDR_ACC,
                           (uint8_t) (LSM_STATUS_REG_A | LSM_AUTO_INC), buf,
                           LSM_BURST_BYTES))
    {
        return false;
    }
    p_s->fresh = (0U != (buf[LSM_IDX_STATUS] & LSM_STATUS_ZYXDA));
    p_s->x_mg  = to_mg(buf[LSM_IDX_XL], buf[LSM_IDX_XH]);
    p_s->y_mg  = to_mg(buf[LSM_IDX_YL], buf[LSM_IDX_YH]);
    p_s->z_mg  = to_mg(buf[LSM_IDX_ZL], buf[LSM_IDX_ZH]);
    return true;
}

bool
drv_lsm303_identify (uint8_t * p_ctrl1, uint8_t * p_ira)
{
    bool ok = drv_i2c_read_regs(LSM303_ADDR_ACC, LSM_CTRL_REG1_A, p_ctrl1, 1U);
    ok = drv_i2c_read_regs(LSM303_ADDR_MAG, LSM_IRA_REG_M, p_ira, 1U) && ok;
    return ok && (LSM_CTRL1_100HZ_XYZ == *p_ctrl1) &&
           (LSM_IRA_EXPECTED == *p_ira);
}
