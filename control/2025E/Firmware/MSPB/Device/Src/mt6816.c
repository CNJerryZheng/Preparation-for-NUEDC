/**
 * @file        mt6816.c
 * @author      JerryZheng
 * @brief       MT6816 A/B、PWM、Z 六线融合设备驱动实现
 * @date        2026-07-23
 */

#include "mt6816.h"
#include "mt6816_config.h"
#include "bsp_gimbal.h"

/**
 * @brief 单只 MT6816 的融合状态
 */
typedef struct
{
    BSP_GimbalAxis_t bsp_axis;      /**< 对应的 BSP 物理轴。 */
    int32_t sensor_offset;          /**< A/B 原始计数到传感器坐标的偏移。 */
    int32_t user_offset;            /**< 传感器坐标到用户坐标的偏移。 */
    uint32_t last_pwm_sequence;     /**< 上次处理的 PWM 捕获序号。 */
    uint32_t last_index_sequence;   /**< 上次处理的 Z 脉冲序号。 */
    uint16_t pwm_angle_code;        /**< 最近一次 12 位绝对角度码。 */
    uint32_t pwm_high_ticks;        /**< 最近一次 PWM 高电平计数。 */
    uint32_t pwm_period_ticks;      /**< 最近一次 PWM 周期计数。 */
    bool pwm_valid;                 /**< PWM 捕获当前有效。 */
    bool synchronized;              /**< 已使用 PWM 建立初始绝对位置。 */
} MT6816_State_t;

/** @brief Yaw 编码器状态。 */
static MT6816_State_t s_yaw;
/** @brief Pitch 编码器状态。 */
static MT6816_State_t s_pitch;
/** @brief Yaw MT6816 的 Live Watch 实时状态。 */
volatile MT6816_Status_t g_mt6816_yaw_status;
/** @brief Pitch MT6816 的 Live Watch 实时状态。 */
volatile MT6816_Status_t g_mt6816_pitch_status;

/**
 * @brief 获取指定轴的内部状态
 * @param axis 编码器轴
 * @return MT6816_State_t* 对应状态地址
 */
static MT6816_State_t *MT6816_GetState(MT6816_Axis_t axis)
{
    return (axis == MT6816_AXIS_YAW) ? &s_yaw : &s_pitch;
}

/**
 * @brief 获取指定轴的机械零点 PWM 码值
 * @param axis 编码器轴
 * @return uint16_t 12 位零点码值
 */
static uint16_t MT6816_GetZeroCode(MT6816_Axis_t axis)
{
    return (axis == MT6816_AXIS_YAW) ?
        (uint16_t)MT6816_YAW_ZERO_PWM_CODE :
        (uint16_t)MT6816_PITCH_ZERO_PWM_CODE;
}

/**
 * @brief 获取指定轴 PWM 方向修正
 * @param axis 编码器轴
 * @return int32_t 1 或 -1
 */
static int32_t MT6816_GetPwmDirectionSign(MT6816_Axis_t axis)
{
    return (axis == MT6816_AXIS_YAW) ?
        MT6816_YAW_PWM_DIRECTION_SIGN :
        MT6816_PITCH_PWM_DIRECTION_SIGN;
}

/**
 * @brief 将单圈计数折返到负半圈至正半圈范围
 * @param count 任意单圈或多圈计数
 * @return int32_t 范围为 -2000～1999 的单圈计数
 */
static int32_t MT6816_WrapSingleTurn(int32_t count)
{
    count %= MT6816_AB_COUNTS_PER_REV;
    if (count >= (MT6816_AB_COUNTS_PER_REV / 2))
    {
        count -= MT6816_AB_COUNTS_PER_REV;
    }
    else if (count < -(MT6816_AB_COUNTS_PER_REV / 2))
    {
        count += MT6816_AB_COUNTS_PER_REV;
    }
    return count;
}

/**
 * @brief 将计数取整到最近的整圈位置
 * @param count 当前传感器坐标
 * @return int32_t 最近的 4000 整数倍
 */
static int32_t MT6816_RoundToNearestTurn(int32_t count)
{
    if (count >= 0)
    {
        return ((count + (MT6816_AB_COUNTS_PER_REV / 2)) /
            MT6816_AB_COUNTS_PER_REV) * MT6816_AB_COUNTS_PER_REV;
    }
    return -(((-count + (MT6816_AB_COUNTS_PER_REV / 2)) /
        MT6816_AB_COUNTS_PER_REV) * MT6816_AB_COUNTS_PER_REV);
}

/**
 * @brief 判断最近 PWM 绝对角度是否位于 MT6816 Z 零位附近
 * @param state 编码器内部状态
 * @return true 可以接受本次 Z 相校准
 * @return false PWM 无效或 Z 脉冲疑似干扰
 */
static bool MT6816_IsIndexPhaseValid(const MT6816_State_t *state)
{
    uint32_t distance_to_zero;

    if (!state->pwm_valid)
    {
        return false;
    }

    distance_to_zero = state->pwm_angle_code;
    if (distance_to_zero >
        (MT6816_PWM_ANGLE_CODES_PER_REV / 2U))
    {
        distance_to_zero =
            MT6816_PWM_ANGLE_CODES_PER_REV - distance_to_zero;
    }
    return distance_to_zero <= MT6816_Z_VALID_WINDOW_PWM_CODES;
}

/**
 * @brief 根据 MT6816 官方 PWM 帧格式还原 12 位绝对角度
 * @param capture BSP 原始捕获值
 * @param angle_code 用于接收 0～4095 角度码
 * @return true 捕获值符合 MT6816 帧周期和固定保护位
 * @return false 捕获值不合法
 */
static bool MT6816_DecodePwm(const BSP_GimbalPwmCapture_t *capture,
    uint16_t *angle_code)
{
    uint32_t high_clocks;

    if ((capture == 0) || (angle_code == 0) || !capture->valid ||
        (capture->period_ticks < MT6816_PWM_MIN_PERIOD_TICKS) ||
        (capture->period_ticks > MT6816_PWM_MAX_PERIOD_TICKS) ||
        (capture->high_ticks >= capture->period_ticks))
    {
        return false;
    }

    high_clocks =
        ((capture->high_ticks * MT6816_PWM_FRAME_CLOCKS) +
            (capture->period_ticks / 2U)) /
        capture->period_ticks;
    if ((high_clocks < MT6816_PWM_START_HIGH_CLOCKS) ||
        (high_clocks >=
            (MT6816_PWM_START_HIGH_CLOCKS +
             MT6816_PWM_ANGLE_CODES_PER_REV)))
    {
        return false;
    }

    *angle_code =
        (uint16_t)(high_clocks - MT6816_PWM_START_HIGH_CLOCKS);
    return true;
}

/**
 * @brief 将 PWM 绝对码转换为相对机械零点的 A/B 单圈计数
 * @param axis 编码器轴
 * @param angle_code 12 位绝对角度码
 * @return int32_t 折返后的单圈计数
 */
static int32_t MT6816_AbsoluteCodeToCount(
    MT6816_Axis_t axis, uint16_t angle_code)
{
    const int32_t angle_count = (int32_t)(
        (((uint32_t)angle_code * (uint32_t)MT6816_AB_COUNTS_PER_REV) +
            (MT6816_PWM_ANGLE_CODES_PER_REV / 2U)) /
        MT6816_PWM_ANGLE_CODES_PER_REV);
    const int32_t zero_count = (int32_t)(
        (((uint32_t)MT6816_GetZeroCode(axis) *
            (uint32_t)MT6816_AB_COUNTS_PER_REV) +
            (MT6816_PWM_ANGLE_CODES_PER_REV / 2U)) /
        MT6816_PWM_ANGLE_CODES_PER_REV);

    return MT6816_WrapSingleTurn(
        (angle_count - zero_count) * MT6816_GetPwmDirectionSign(axis));
}

/**
 * @brief 更新一只编码器的绝对对齐和 Z 相校准
 * @param axis 编码器轴
 * @param state 编码器内部状态
 */
static void MT6816_UpdateOne(
    MT6816_Axis_t axis, MT6816_State_t *state)
{
    BSP_GimbalPwmCapture_t capture;
    uint32_t index_sequence = 0U;
    int32_t index_raw_count = 0;
    uint16_t angle_code;

    (void)BSP_GimbalGetPwmCapture(state->bsp_axis, &capture);
    if (capture.sequence != state->last_pwm_sequence)
    {
        state->last_pwm_sequence = capture.sequence;
        state->pwm_high_ticks = capture.high_ticks;
        state->pwm_period_ticks = capture.period_ticks;
        state->pwm_valid = MT6816_DecodePwm(&capture, &angle_code);
        if (state->pwm_valid)
        {
            const int32_t raw_count =
                BSP_GimbalGetEncoderCount(state->bsp_axis);

            state->pwm_angle_code = angle_code;
            if (!state->synchronized)
            {
                state->sensor_offset =
                    MT6816_AbsoluteCodeToCount(axis, angle_code) - raw_count;
                state->synchronized = true;
            }
        }
    }

    if (BSP_GimbalGetIndexEvent(state->bsp_axis,
            &index_sequence, &index_raw_count) &&
        (index_sequence != state->last_index_sequence))
    {
        state->last_index_sequence = index_sequence;
        if (state->synchronized && MT6816_IsIndexPhaseValid(state))
        {
            const int32_t sensor_count_at_index =
                index_raw_count + state->sensor_offset;
            const int32_t index_phase_count =
                MT6816_AbsoluteCodeToCount(axis, 0U);
            const int32_t expected_index_count =
                MT6816_RoundToNearestTurn(
                    sensor_count_at_index - index_phase_count) +
                index_phase_count;

            state->sensor_offset +=
                expected_index_count - sensor_count_at_index;
        }
    }
}

/**
 * @brief 发布一只编码器的 Live Watch 状态
 * @param axis 编码器轴
 * @param state 编码器内部状态
 * @param status 对应的全局监视结构体
 */
static void MT6816_PublishStatus(MT6816_Axis_t axis,
    MT6816_State_t *state, volatile MT6816_Status_t *status)
{
    status->multi_turn_count = MT6816_GetMultiTurnCount(axis);
    status->pwm_angle_code = state->pwm_angle_code;
    status->pwm_high_ticks = state->pwm_high_ticks;
    status->pwm_period_ticks = state->pwm_period_ticks;
    status->pwm_frame_count = state->last_pwm_sequence;
    status->index_count = state->last_index_sequence;
    status->pwm_valid = state->pwm_valid;
    status->synchronized = state->synchronized;
}

/** @copydoc MT6816_Init */
void MT6816_Init(void)
{
    s_yaw = (MT6816_State_t) {
        BSP_GIMBAL_AXIS_YAW, 0, 0, 0U, 0U, 0U, 0U, 0U, false, false
    };
    s_pitch = (MT6816_State_t) {
        BSP_GIMBAL_AXIS_PITCH, 0, 0, 0U, 0U, 0U, 0U, 0U, false, false
    };
    g_mt6816_yaw_status = (MT6816_Status_t) {
        0, 0U, 0U, 0U, 0U, 0U, false, false
    };
    g_mt6816_pitch_status = (MT6816_Status_t) {
        0, 0U, 0U, 0U, 0U, 0U, false, false
    };
}

/** @copydoc MT6816_Update */
void MT6816_Update(void)
{
    MT6816_UpdateOne(MT6816_AXIS_YAW, &s_yaw);
    MT6816_UpdateOne(MT6816_AXIS_PITCH, &s_pitch);
    MT6816_PublishStatus(
        MT6816_AXIS_YAW, &s_yaw, &g_mt6816_yaw_status);
    MT6816_PublishStatus(
        MT6816_AXIS_PITCH, &s_pitch, &g_mt6816_pitch_status);
}

/** @copydoc MT6816_GetMultiTurnCount */
int32_t MT6816_GetMultiTurnCount(MT6816_Axis_t axis)
{
    MT6816_State_t *state = MT6816_GetState(axis);

    return BSP_GimbalGetEncoderCount(state->bsp_axis) +
        state->sensor_offset + state->user_offset;
}

/** @copydoc MT6816_ResetUserPosition */
void MT6816_ResetUserPosition(MT6816_Axis_t axis)
{
    MT6816_State_t *state = MT6816_GetState(axis);
    const int32_t sensor_count =
        BSP_GimbalGetEncoderCount(state->bsp_axis) +
        state->sensor_offset;

    state->user_offset = -sensor_count;
}

/** @copydoc MT6816_IsSynchronized */
bool MT6816_IsSynchronized(MT6816_Axis_t axis)
{
    return MT6816_GetState(axis)->synchronized;
}

/** @copydoc MT6816_GetStatus */
void MT6816_GetStatus(MT6816_Axis_t axis, MT6816_Status_t *status)
{
    MT6816_State_t *state;

    if (status == 0)
    {
        return;
    }

    state = MT6816_GetState(axis);
    status->multi_turn_count = MT6816_GetMultiTurnCount(axis);
    status->pwm_angle_code = state->pwm_angle_code;
    status->pwm_high_ticks = state->pwm_high_ticks;
    status->pwm_period_ticks = state->pwm_period_ticks;
    status->pwm_frame_count = state->last_pwm_sequence;
    status->index_count = state->last_index_sequence;
    status->pwm_valid = state->pwm_valid;
    status->synchronized = state->synchronized;
}
