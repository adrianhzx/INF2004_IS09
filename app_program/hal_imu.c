/**
 * @file    hal_imu.c
 * @brief   GY-511 accelerometer on I2C0 GP4/GP5 (Pico SDK side, see hal.h),
 *          set up as the IS09 driver does (subsystem4/drv_lsm303.c).
 *
 * IS09 car (19 Nov port): the accelerometer is read for IS09 imu_core,
 * which expects milli-g and was tuned with the IS09 settings: LSM303DLHC
 * at 0x19, CTRL_REG1_A = 0x57 (100 Hz, X/Y/Z on), CTRL_REG4_A = 0x88 (BDU,
 * +-2 g, high resolution: 1 mg per LSB after the 4-bit shift). There is no
 * INT1 wire, so collisions come from imu_core's spike test only.
 *
 * Two chips are sold as "GY-511" (base notes, kept): if the DLHC does not
 * answer, an LSM303D at 0x1E / 0x1D is set to 100 Hz, BDU, +-2 g
 * (0.061 mg/LSB, 16 bit), so imu_core still gets milli-g.
 * Every transfer has a 2 ms timeout (DR: i2c_read_blocking_until).
 */
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/structs/timer.h"

#include "hal.h"
#include "pins.h"

#define IMU_I2C           (i2c0)
#define I2C_TIMEOUT_US    (2000u)

/* LSM303DLHC accelerometer */
#define CTRL_REG1_A       (0x20u)
#define CTRL_REG2_A       (0x21u)
#define CTRL_REG3_A       (0x22u)
#define CTRL_REG4_A       (0x23u)
#define CTRL_REG5_A       (0x24u)
#define OUT_X_L_A         (0x28u)
#define AUTO_INC          (0x80u)

/* LSM303D (same OUT_X_L_A = 0x28, auto-increment = MSB of the register) */
#define D_WHO_AM_I        (0x0Fu)
#define D_WHO_AM_I_VALUE  (0x49u)
#define D_CTRL0           (0x1Fu)
#define D_CTRL1           (0x20u)
#define D_CTRL2           (0x21u)
#define D_CTRL3           (0x22u)
#define D_CTRL7           (0x26u)
#define D_ADDR_SA0_LOW    (0x1Eu)
#define D_ADDR_SA0_HIGH   (0x1Du)
#define D_MG_PER_LSB_X1000 (61)        /* +-2 g: 0.061 mg/LSB */

static uint8_t       g_addr = IMU_ACCEL_ADDR;
static hal_imu_chip_t g_chip = HAL_IMU_NONE;

static absolute_time_t deadline(void)
{
    return from_us_since_boot(hal_time_us() + I2C_TIMEOUT_US);
}

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    return i2c_write_blocking_until(IMU_I2C, g_addr, buf, 2u, false, deadline()) == 2;
}

static bool reg_read(uint8_t reg, uint8_t *p_buf, uint32_t len)
{
    if (i2c_write_blocking_until(IMU_I2C, g_addr, &reg, 1u, true, deadline()) != 1)
    {
        return false;
    }
    return i2c_read_blocking_until(IMU_I2C, g_addr, p_buf, len, false, deadline()) == (int)len;
}

static void i2c_pins(void)
{
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

static bool init_dlhc(void)
{
    uint8_t check = 0u;

    g_addr = IMU_ACCEL_ADDR;
    if (!reg_write(CTRL_REG1_A, 0x57u))          /* 100 Hz, X/Y/Z on */
    {
        return false;
    }
    if (!reg_read(CTRL_REG1_A, &check, 1u) || (check != 0x57u))
    {
        return false;                             /* not an LSM303DLHC accelerometer */
    }
    return reg_write(CTRL_REG4_A, 0x88u) &&      /* IS09: BDU, +-2 g, high resolution */
           reg_write(CTRL_REG3_A, 0x00u);        /* nothing on INT1 (no wire) */
}

static bool init_lsm303d(uint8_t addr)
{
    uint8_t who = 0u;
    uint8_t check = 0u;

    g_addr = addr;
    if (!reg_read(D_WHO_AM_I, &who, 1u) || (who != D_WHO_AM_I_VALUE))
    {
        return false;
    }
    if (!reg_write(D_CTRL0, 0x00u) ||            /* no FIFO, no high-pass */
        !reg_write(D_CTRL1, 0x6Fu) ||            /* AODR 0110 = 100 Hz, BDU, Z/Y/X on */
        !reg_write(D_CTRL2, 0x40u) ||            /* ABW 01 = 194 Hz anti-alias, AFS 000 = +-2 g */
        !reg_write(D_CTRL3, 0x00u) ||            /* nothing routed to INT1 */
        !reg_write(D_CTRL7, 0x02u))              /* magnetometer power-down (not used) */
    {
        return false;
    }
    return reg_read(D_CTRL1, &check, 1u) && (check == 0x6Fu);
}

bool hal_imu_init(void)
{
    i2c_pins();
    if (init_dlhc())
    {
        g_chip = HAL_IMU_LSM303DLHC;
        return true;
    }
    if (init_lsm303d(D_ADDR_SA0_LOW) || init_lsm303d(D_ADDR_SA0_HIGH))
    {
        g_chip = HAL_IMU_LSM303D;
        return true;
    }
    g_chip = HAL_IMU_NONE;
    g_addr = IMU_ACCEL_ADDR;
    return false;
}

hal_imu_chip_t hal_imu_chip(void)
{
    return g_chip;
}

const char *hal_imu_chip_name(void)
{
    return (g_chip == HAL_IMU_LSM303DLHC) ? "LSM303DLHC at 0x19" :
           (g_chip == HAL_IMU_LSM303D) ? ((g_addr == D_ADDR_SA0_LOW) ? "LSM303D at 0x1E" : "LSM303D at 0x1D") :
           "none";
}

bool hal_imu_int1_used(void)
{
    return false;                                 /* no INT1 wire on the IS09 car */
}

static int16_t clamp16(int32_t v)
{
    return (int16_t)((v > 32767) ? 32767 : ((v < -32768) ? -32768 : v));
}

bool hal_imu_accel_mg(int16_t *p_x, int16_t *p_y, int16_t *p_z)
{
    uint8_t raw[6];

    if (!reg_read(OUT_X_L_A | AUTO_INC, raw, 6u))
    {
        return false;
    }
    if (g_chip == HAL_IMU_LSM303D)
    {
        /* 16 bit, 0.061 mg/LSB at +-2 g */
        *p_x = clamp16(((int32_t)(int16_t)(((uint16_t)raw[1] << 8) | raw[0]) * D_MG_PER_LSB_X1000) / 1000);
        *p_y = clamp16(((int32_t)(int16_t)(((uint16_t)raw[3] << 8) | raw[2]) * D_MG_PER_LSB_X1000) / 1000);
        *p_z = clamp16(((int32_t)(int16_t)(((uint16_t)raw[5] << 8) | raw[4]) * D_MG_PER_LSB_X1000) / 1000);
        return true;
    }
    /* DLHC: 12-bit left-justified, 1 mg/LSB at +-2 g high resolution
       (IS09 drv_lsm303.c to_mg(): divide by 16, sign kept) */
    *p_x = (int16_t)((int32_t)(int16_t)(((uint16_t)raw[1] << 8) | raw[0]) / 16);
    *p_y = (int16_t)((int32_t)(int16_t)(((uint16_t)raw[3] << 8) | raw[2]) / 16);
    *p_z = (int16_t)((int32_t)(int16_t)(((uint16_t)raw[5] << 8) | raw[4]) / 16);
    return true;
}

bool hal_imu_int1_src(uint8_t *p_src)
{
    *p_src = 0u;                                  /* no INT1 on the IS09 car */
    return true;
}

bool hal_imu_int1_level(void)
{
    return false;
}

void hal_i2c_recover(void)
{
    /* DR 6.3 FAULT: I2C_BUS_RECOVERY - clock SCL nine times to free a slave
       that is holding SDA low, then re-initialise the peripheral */
    i2c_deinit(IMU_I2C);
    gpio_init(PIN_I2C_SCL);
    gpio_init(PIN_I2C_SDA);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    gpio_pull_up(PIN_I2C_SDA);
    for (uint32_t i = 0u; i < 9u; i++)
    {
        uint64_t t;

        gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
        gpio_put(PIN_I2C_SCL, false);
        t = hal_time_us() + 5u;
        while (hal_time_us() < t)
        {
            /* 5 us low */
        }
        gpio_set_dir(PIN_I2C_SCL, GPIO_IN);       /* released: pulled high */
        gpio_pull_up(PIN_I2C_SCL);
        t = hal_time_us() + 5u;
        while (hal_time_us() < t)
        {
            /* 5 us high */
        }
    }
    i2c_pins();
}
