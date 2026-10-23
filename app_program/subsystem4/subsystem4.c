/** @file subsystem4.c
 *  @brief Subsystem 4: brings up I2C0 and the LSM303DLHC, starts ImuTask.
 *         A missing or silent sensor is NOT a boot failure: the task keeps
 *         publishing quality INVALID with IMU_FAULT set and retries.
 */

#include "subsystem4.h"

#include "../shared/app_safety.h"
#include "drv_i2c.h"

EXPORT ID tid_imu = 0;

EXPORT ER
subsystem4_init (void)
{
    if (!drv_i2c_init())
    {
        return E_OBJ; /* I2C0 never left reset: silicon/clock fault */
    }
    if (!s4_imu_sensor_init())
    {
        app_fault_set(
            APP_FAULT_IMU_FAULT); /* diagnostic; task retries every 500 ms */
    }
    return E_OK;
}

EXPORT ER
subsystem4_start (void)
{
    tid_imu = app_start_task_stack((FP) s4_imu_task, APP_PRI_IMU,
                                   (SZ) APP_STACK_IMU, "ImuTask");
    if (tid_imu < E_OK)
    {
        return tid_imu;
    }
    app_ipc_register_cyclic_task(APP_CYC_SLOT_IMU, (int32_t) tid_imu);
    return E_OK;
}
