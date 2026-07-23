/**
 * @file        gimbal_device.c
 * @author      JerryZheng
 * @brief       D36A步进驱动器与MT6816编码器设备层实现
 * @date        2026-07-23
 */

#include "gimbal_device.h"
#include "bsp_gimbal.h"
#include "mt6816.h"
#include "mt6816_config.h"

/**
 * @brief 将设备层轴编号转换为BSP轴编号
 * @param axis 云台设备轴
 * @return BSP_GimbalAxis_t BSP硬件轴编号
 */
static BSP_GimbalAxis_t GIMBAL_DeviceToBspAxis(
    GIMBAL_DeviceAxis_t axis)
{
    return (axis == GIMBAL_DEVICE_AXIS_YAW) ?
        BSP_GIMBAL_AXIS_YAW : BSP_GIMBAL_AXIS_PITCH;
}

/** @copydoc GIMBAL_DeviceInit */
void GIMBAL_DeviceInit(void)
{
    BSP_GimbalInit();
    MT6816_Init();
}

/** @copydoc GIMBAL_DeviceUpdate */
void GIMBAL_DeviceUpdate(void)
{
    MT6816_Update();
}

/** @copydoc GIMBAL_DeviceSetEnabled */
void GIMBAL_DeviceSetEnabled(
    GIMBAL_DeviceAxis_t axis, bool enable)
{
    BSP_GimbalSetEnabled(GIMBAL_DeviceToBspAxis(axis), enable);
}

/** @copydoc GIMBAL_DeviceSetStep */
void GIMBAL_DeviceSetStep(GIMBAL_DeviceAxis_t axis,
    bool positive, uint32_t frequency_hz)
{
    BSP_GimbalSetStep(
        GIMBAL_DeviceToBspAxis(axis), positive, frequency_hz);
}

/** @copydoc GIMBAL_DeviceStop */
void GIMBAL_DeviceStop(GIMBAL_DeviceAxis_t axis)
{
    BSP_GimbalStop(GIMBAL_DeviceToBspAxis(axis));
}

/** @copydoc GIMBAL_DeviceGetEncoderCount */
int32_t GIMBAL_DeviceGetEncoderCount(GIMBAL_DeviceAxis_t axis)
{
    return MT6816_GetMultiTurnCount(
        (axis == GIMBAL_DEVICE_AXIS_YAW) ?
            MT6816_AXIS_YAW : MT6816_AXIS_PITCH);
}

/** @copydoc GIMBAL_DeviceResetEncoder */
void GIMBAL_DeviceResetEncoder(GIMBAL_DeviceAxis_t axis)
{
    MT6816_ResetUserPosition(
        (axis == GIMBAL_DEVICE_AXIS_YAW) ?
            MT6816_AXIS_YAW : MT6816_AXIS_PITCH);
}

/** @copydoc GIMBAL_DeviceIsFeedbackReady */
bool GIMBAL_DeviceIsFeedbackReady(void)
{
#if MT6816_REQUIRE_PWM_SYNC_BEFORE_ENABLE
    return MT6816_IsSynchronized(MT6816_AXIS_YAW) &&
        MT6816_IsSynchronized(MT6816_AXIS_PITCH);
#else
    return true;
#endif
}

/** @copydoc GIMBAL_DeviceIsDirectionBlocked */
bool GIMBAL_DeviceIsDirectionBlocked(
    GIMBAL_DeviceAxis_t axis, bool positive)
{
    if (axis == GIMBAL_DEVICE_AXIS_YAW)
    {
        return positive ? BSP_GimbalIsYawLimitRightActive() :
                          BSP_GimbalIsYawLimitLeftActive();
    }
    return positive ? BSP_GimbalIsPitchLimitDownActive() :
                      BSP_GimbalIsPitchLimitUpActive();
}
