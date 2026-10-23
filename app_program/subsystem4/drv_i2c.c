/** @file drv_i2c.c
 *  @brief Polled I2C0 master (DW_apb_i2c, RP2040 datasheet 4.3). See drv_i2c.h.
 *         Register addresses are written out here because the kernel's
 *         device-driver headers are private to the (disabled) kernel driver.
 */

#include "drv_i2c.h"

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_port.h"

#define I2C0_BASE_ADDR 0x40044000UL
#define I2C_CON (I2C0_BASE_ADDR + 0x00UL)
#define I2C_TAR (I2C0_BASE_ADDR + 0x04UL)
#define I2C_DATA_CMD (I2C0_BASE_ADDR + 0x10UL)
#define I2C_SS_SCL_HCNT (I2C0_BASE_ADDR + 0x14UL)
#define I2C_SS_SCL_LCNT (I2C0_BASE_ADDR + 0x18UL)
#define I2C_FS_SCL_HCNT (I2C0_BASE_ADDR + 0x1CUL)
#define I2C_FS_SCL_LCNT (I2C0_BASE_ADDR + 0x20UL)
#define I2C_INTR_MASK (I2C0_BASE_ADDR + 0x30UL)
#define I2C_RAW_INTR_STAT (I2C0_BASE_ADDR + 0x34UL)
#define I2C_RX_TL (I2C0_BASE_ADDR + 0x38UL)
#define I2C_TX_TL (I2C0_BASE_ADDR + 0x3CUL)
#define I2C_CLR_INTR (I2C0_BASE_ADDR + 0x40UL)
#define I2C_CLR_TX_ABRT (I2C0_BASE_ADDR + 0x54UL)
#define I2C_CLR_STOP_DET (I2C0_BASE_ADDR + 0x60UL)
#define I2C_ENABLE (I2C0_BASE_ADDR + 0x6CUL)
#define I2C_STATUS (I2C0_BASE_ADDR + 0x70UL)
#define I2C_SDA_HOLD (I2C0_BASE_ADDR + 0x7CUL)
#define I2C_TX_ABRT_SOURCE (I2C0_BASE_ADDR + 0x80UL)
#define I2C_ENABLE_STATUS (I2C0_BASE_ADDR + 0x9CUL)
#define I2C_FS_SPKLEN (I2C0_BASE_ADDR + 0xA0UL)

#define I2C_CON_MASTER (1UL << 0U)
#define I2C_CON_SPEED_STD (1UL << 1U)
#define I2C_CON_SPEED_FAST (2UL << 1U)
#define I2C_CON_RESTART_EN (1UL << 5U)
#define I2C_CON_SLAVE_DISABLE (1UL << 6U)
#define I2C_CON_TX_EMPTY_CTRL (1UL << 8U)
#define I2C_CMD_READ (1UL << 8U)
#define I2C_CMD_STOP (1UL << 9U)
#define I2C_CMD_RESTART (1UL << 10U)
#define I2C_STATUS_TFNF (1UL << 1U)
#define I2C_STATUS_RFNE (1UL << 3U)
#define I2C_INTR_TX_ABRT (1UL << 6U)
#define I2C_INTR_STOP_DET (1UL << 9U)
#define I2C_ENABLE_ON 1UL
#define I2C_ENABLE_OFF 0UL
#define I2C_ADDR7_MASK 0x7FU
#define I2C_BYTE_MASK 0xFFUL

/* SCL timing as the Pico SDK derives it from clk_sys = 125 MHz. */
#define I2C_CLK_SYS_HZ 125000000UL
#define I2C_PERIOD                                                             \
    ((I2C_CLK_SYS_HZ + (APP_CAL_IMU_I2C_HZ / 2UL)) / APP_CAL_IMU_I2C_HZ)
#define I2C_LCNT ((I2C_PERIOD * 3UL) / 5UL)
#define I2C_HCNT (I2C_PERIOD - I2C_LCNT)
#define I2C_SPKLEN ((I2C_LCNT < 16UL) ? 1UL : (I2C_LCNT / 16UL))
#define I2C_SDA_TX_HOLD (((I2C_CLK_SYS_HZ * 3UL) / 10000000UL) + 1UL)
#define I2C_FAST_LIMIT_HZ 100000UL
#define I2C_RESETS_I2C0 (1UL << 3U)
#define I2C_ENABLE_WAIT_LOOPS 1000U

static uint32_t g_i2c_aborts   = 0U;
static uint32_t g_i2c_timeouts = 0U;

static bool
deadline_passed (uint32_t t0)
{
    return ((uint32_t) (app_hw_time_us() - t0)) > DRV_I2C_TIMEOUT_US;
}

static bool
aborted (void)
{
    if (0U != (in_w(I2C_RAW_INTR_STAT) & I2C_INTR_TX_ABRT))
    {
        (void) in_w(I2C_TX_ABRT_SOURCE);
        (void) in_w(I2C_CLR_TX_ABRT); /* reading clears */
        g_i2c_aborts++;
        return true;
    }
    return false;
}

static bool
wait_status (uint32_t bit, uint32_t t0)
{
    while (0U == (in_w(I2C_STATUS) & bit))
    {
        if (aborted())
        {
            return false;
        }
        if (deadline_passed(t0))
        {
            g_i2c_timeouts++;
            return false;
        }
    }
    return true;
}

static bool
wait_stop (uint32_t t0)
{
    while (0U == (in_w(I2C_RAW_INTR_STAT) & I2C_INTR_STOP_DET))
    {
        if (aborted())
        {
            return false;
        }
        if (deadline_passed(t0))
        {
            g_i2c_timeouts++;
            return false;
        }
    }
    (void) in_w(I2C_CLR_STOP_DET);
    return !aborted();
}

static void
set_enable (bool on)
{
    uint32_t loops = 0U;
    out_w(I2C_ENABLE, on ? I2C_ENABLE_ON : I2C_ENABLE_OFF);
    while ((0U != (in_w(I2C_ENABLE_STATUS) & I2C_ENABLE_ON)) != on)
    {
        loops++;
        if (loops > I2C_ENABLE_WAIT_LOOPS)
        {
            break;
        }
    }
}

bool
drv_i2c_init (void)
{
    if (!app_hw_release_reset(I2C_RESETS_I2C0))
    {
        return false;
    }
    set_enable(false);
    out_w(I2C_CON,
          I2C_CON_MASTER | I2C_CON_SLAVE_DISABLE | I2C_CON_RESTART_EN |
              I2C_CON_TX_EMPTY_CTRL |
              ((APP_CAL_IMU_I2C_HZ > I2C_FAST_LIMIT_HZ) ? I2C_CON_SPEED_FAST
                                                        : I2C_CON_SPEED_STD));
    out_w(I2C_TX_TL, 0U);
    out_w(I2C_RX_TL, 0U);
    out_w(I2C_INTR_MASK, 0U); /* polled: no interrupt line used */
    out_w(I2C_SS_SCL_HCNT, I2C_HCNT);
    out_w(I2C_SS_SCL_LCNT, I2C_LCNT);
    out_w(I2C_FS_SCL_HCNT, I2C_HCNT);
    out_w(I2C_FS_SCL_LCNT, I2C_LCNT);
    out_w(I2C_FS_SPKLEN, I2C_SPKLEN);
    out_w(I2C_SDA_HOLD, I2C_SDA_TX_HOLD);
    (void) in_w(I2C_CLR_INTR);

    app_hw_gpio_function(APP_PIN_IMU_SDA, GPIO_CTRL_FUNCSEL_I2C, true);
    app_hw_gpio_function(APP_PIN_IMU_SCL, GPIO_CTRL_FUNCSEL_I2C, true);
    set_enable(true);
    return true;
}

static void
set_target (uint8_t addr7)
{
    set_enable(false); /* TAR may only change while disabled */
    out_w(I2C_TAR, (uint32_t) addr7 & I2C_ADDR7_MASK);
    set_enable(true);
}

bool
drv_i2c_write_reg (uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint32_t const t0 = app_hw_time_us();

    set_target(addr7);
    (void) in_w(I2C_CLR_INTR);
    if (!wait_status(I2C_STATUS_TFNF, t0))
    {
        return false;
    }
    out_w(I2C_DATA_CMD, (uint32_t) reg);
    if (!wait_status(I2C_STATUS_TFNF, t0))
    {
        return false;
    }
    out_w(I2C_DATA_CMD, (uint32_t) value | I2C_CMD_STOP);
    return wait_stop(t0);
}

bool
drv_i2c_read_regs (uint8_t addr7, uint8_t reg, uint8_t * p_buf, uint32_t n)
{
    uint32_t const t0 = app_hw_time_us();
    uint32_t       i  = 0U;

    if (0U == n)
    {
        return false;
    }
    set_target(addr7);
    (void) in_w(I2C_CLR_INTR);
    if (!wait_status(I2C_STATUS_TFNF, t0))
    {
        return false;
    }
    out_w(I2C_DATA_CMD, (uint32_t) reg); /* sub-address, no STOP */
    for (i = 0U; i < n; i++)
    {
        uint32_t cmd = I2C_CMD_READ;
        if (0U == i)
        {
            cmd |= I2C_CMD_RESTART;
        }
        if (i == (n - 1U))
        {
            cmd |= I2C_CMD_STOP;
        }
        if (!wait_status(I2C_STATUS_TFNF, t0))
        {
            return false;
        }
        out_w(I2C_DATA_CMD, cmd);
        if (!wait_status(I2C_STATUS_RFNE, t0))
        {
            return false;
        }
        p_buf[i] = (uint8_t) (in_w(I2C_DATA_CMD) & I2C_BYTE_MASK);
    }
    return wait_stop(t0);
}

uint32_t
drv_i2c_aborts (void)
{
    return g_i2c_aborts;
}

uint32_t
drv_i2c_timeouts (void)
{
    return g_i2c_timeouts;
}
