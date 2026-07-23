/**
 * @file        gimbal_axis.c
 * @author      JerryZheng
 * @brief       云台编码器位置闭环控制实现
 * @date        2026-07-23
 */

#include "gimbal_axis.h"
#include "gimbal_axis_config.h"
#include "gimbal_device.h"

/**
 * @brief 单轴位置闭环运行状态
 */
typedef struct
{
    // 定义本步骤需要的局部数据并完成初始化。
    GIMBAL_DeviceAxis_t axis;  /**< 对应云台设备轴。 */
    float target;              /**< 目标编码器计数。 */
    int32_t position;          /**< 当前编码器计数。 */
    int32_t last_position;     /**< 上一控制周期编码器计数。 */
    float velocity_cps;        /**< 编码器速度，单位计数/秒。 */
    float integral;            /**< 位置误差积分。 */
    float command_step_hz;     /**< 当前有符号STEP频率。 */
    float kp;                  /**< 位置比例系数。 */
    float ki;                  /**< 位置积分系数。 */
    float kd;                  /**< 编码器速度阻尼系数。 */
    float minimum_position;    /**< 软件最小位置。 */
    float maximum_position;    /**< 软件最大位置。 */
} GIMBAL_AxisState_t;

/** @brief Yaw闭环状态。 */
static GIMBAL_AxisState_t s_yaw;
/** @brief Pitch闭环状态。 */
static GIMBAL_AxisState_t s_pitch;
/** @brief 视觉目标是否有效。 */
static bool s_vision_valid = false;
/** @brief 距离最近有效视觉帧的时间。 */
static uint32_t s_vision_age_ms = GIMBAL_VISION_TIMEOUT_MS;
/** @brief 两轴 MT6816 是否已经完成上电绝对位置同步。 */
static bool s_feedback_ready = false;
/** @brief 姿态跟随开始时的 Yaw 编码器位置。 */
static float s_follow_yaw_origin = 0.0f;
/** @brief 姿态跟随开始时的 Pitch 编码器位置。 */
static float s_follow_pitch_origin = 0.0f;
/** @brief 是否已经建立姿态跟随编码器零点。 */
static bool s_follow_origin_valid = false;

/**
 * @brief 将浮点数限制在指定范围
 * @param value 原始数值
 * @param minimum 最小值
 * @param maximum 最大值
 * @return float 限幅结果
 */
static float GIMBAL_Clamp(float value, float minimum, float maximum)
{
    // 判断 value > maximum 是否成立，并选择对应处理分支。
    if (value > maximum)
    {
        // 返回本次计算或查询得到的结果。
        return maximum;
    }
    // 判断 value < minimum 是否成立，并选择对应处理分支。
    if (value < minimum)
    {
        // 返回本次计算或查询得到的结果。
        return minimum;
    }
    // 返回本次计算或查询得到的结果。
    return value;
}

/**
 * @brief 计算浮点数绝对值
 * @param value 原始数值
 * @return float 非负绝对值
 */
static float GIMBAL_Abs(float value)
{
    // 返回本次计算或查询得到的结果。
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 以加速度限制更新有符号STEP频率
 * @param current_hz 当前有符号频率
 * @param target_hz 目标有符号频率
 * @return float 本周期允许的有符号频率
 */
static float GIMBAL_SlewStepFrequency(
    float current_hz, float target_hz, uint32_t elapsed_ms)
{
    // 更新 maximum_change 对应的本步骤的运行数据。
    const float maximum_change =
        GIMBAL_STEP_FREQUENCY_SLEW_HZ_PER_MS * (float)elapsed_ms;

    // 判断 target_hz > current_hz + maximum_change 是否成立，并选择对应处理分支。
    if (target_hz > current_hz + maximum_change)
    {
        // 返回本次计算或查询得到的结果。
        return current_hz + maximum_change;
    }
    // 判断 target_hz < current_hz - maximum_change 是否成立，并选择对应处理分支。
    if (target_hz < current_hz - maximum_change)
    {
        // 返回本次计算或查询得到的结果。
        return current_hz - maximum_change;
    }
    // 返回本次计算或查询得到的结果。
    return target_hz;
}

/**
 * @brief 更新一个云台轴的编码器位置闭环
 * @param axis 单轴状态
 * @param elapsed_ms 距离上次更新实际经过的毫秒数
 */
static void GIMBAL_UpdateOneAxis(
    GIMBAL_AxisState_t *axis, uint32_t elapsed_ms)
{
    // 定义本步骤需要的局部数据并完成初始化。
    float error;
    // 定义本步骤需要的局部数据并完成初始化。
    float requested_hz;
    // 定义本步骤需要的局部数据并完成初始化。
    float output_hz;
    // 定义本步骤需要的局部数据并完成初始化。
    float velocity_alpha;
    // 定义本步骤需要的局部数据并完成初始化。
    bool positive;

    // 读取最新多圈位置，并对离散差分速度进行一阶低通滤波。
    axis->position = GIMBAL_DeviceGetEncoderCount(axis->axis);
    // 更新 velocity_alpha 对应的速度状态。
    velocity_alpha = (float)elapsed_ms /
        (GIMBAL_VELOCITY_FILTER_TIME_CONSTANT_MS + (float)elapsed_ms);
    // 根据位置差分和低通系数更新当前编码器速度反馈。
    axis->velocity_cps =
        ((1.0f - velocity_alpha) * axis->velocity_cps) +
        (velocity_alpha * (float)(axis->position - axis->last_position) *
            (1000.0f / (float)elapsed_ms));
    // 保存本周期位置，供下一控制周期计算位置差分。
    axis->last_position = axis->position;
    // 更新 error 对应的本步骤的运行数据。
    error = axis->target - (float)axis->position;

    // 进入位置死区后逐步降频至零，避免目标附近反复换向抖动。
    if (GIMBAL_Abs(error) <= GIMBAL_POSITION_DEADBAND_COUNTS)
    {
        // 更新或清除积分状态，避免历史误差继续影响输出。
        axis->integral = 0.0f;
        // 调用 GIMBAL_SlewStepFrequency，完成当前步骤的业务处理。
        axis->command_step_hz =
            GIMBAL_SlewStepFrequency(
                axis->command_step_hz, 0.0f, elapsed_ms);
    }
    // 前述条件均不成立时执行备用处理。
    else
    {
        // 死区外执行带积分限幅和速度阻尼的位置闭环。
        axis->integral = GIMBAL_Clamp(axis->integral +
                (error * (float)elapsed_ms * 0.001f),
            -GIMBAL_POSITION_INTEGRAL_LIMIT,
            GIMBAL_POSITION_INTEGRAL_LIMIT);
        // 更新 requested_hz 对应的本步骤的运行数据。
        requested_hz = (axis->kp * error) +
            (axis->ki * axis->integral) -
            (axis->kd * axis->velocity_cps);
        // 调用 GIMBAL_Clamp，完成当前步骤的业务处理。
        requested_hz = GIMBAL_Clamp(requested_hz,
            -GIMBAL_MAX_STEP_FREQUENCY_HZ,
            GIMBAL_MAX_STEP_FREQUENCY_HZ);
        // D36A 在过低频率下可能无法可靠起步，因此补偿到最低运行频率。
        if ((requested_hz > 0.0f) &&
            (requested_hz < GIMBAL_MIN_STEP_FREQUENCY_HZ))
        {
            // 更新 requested_hz 对应的本步骤的运行数据。
            requested_hz = GIMBAL_MIN_STEP_FREQUENCY_HZ;
        }
        // 前一条件不成立时继续判断当前条件。
        else if ((requested_hz < 0.0f) &&
            (requested_hz > -GIMBAL_MIN_STEP_FREQUENCY_HZ))
        {
            // 更新 requested_hz 对应的本步骤的运行数据。
            requested_hz = -GIMBAL_MIN_STEP_FREQUENCY_HZ;
        }
        // 调用 GIMBAL_SlewStepFrequency，完成当前步骤的业务处理。
        axis->command_step_hz =
            GIMBAL_SlewStepFrequency(
                axis->command_step_hz, requested_hz, elapsed_ms);
    }

    // 更新 positive 对应的本步骤的运行数据。
    positive = axis->command_step_hz >= 0.0f;
    // 只封锁朝已触发限位的方向，反方向仍可退出限位。
    if (GIMBAL_DeviceIsDirectionBlocked(axis->axis, positive))
    {
        // 更新当前有符号STEP频率指令。
        axis->command_step_hz = 0.0f;
        // 更新或清除积分状态，避免历史误差继续影响输出。
        axis->integral = 0.0f;
        // 调用 GIMBAL_DeviceStop，停止运动或保持当前安全状态。
        GIMBAL_DeviceStop(axis->axis);
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 调用 GIMBAL_Abs，完成当前步骤的业务处理。
    output_hz = GIMBAL_Abs(axis->command_step_hz);
    // 斜坡减速阶段低于最低频率时，根据位置误差决定停止或维持最低频率。
    if (output_hz < GIMBAL_MIN_STEP_FREQUENCY_HZ)
    {
        // 根据当前控制误差选择对应处理方式。
        if (GIMBAL_Abs(error) <= GIMBAL_POSITION_DEADBAND_COUNTS)
        {
            // 更新当前有符号STEP频率指令。
            axis->command_step_hz = 0.0f;
            // 调用 GIMBAL_DeviceStop，停止运动或保持当前安全状态。
            GIMBAL_DeviceStop(axis->axis);
            // 返回本次计算或查询得到的结果。
            return;
        }
        // 更新 output_hz 对应的本步骤的运行数据。
        output_hz = GIMBAL_MIN_STEP_FREQUENCY_HZ;
    }

    // 调用 GIMBAL_DeviceSetStep，更新或发送对应数据。
    GIMBAL_DeviceSetStep(
        axis->axis, positive, (uint32_t)output_hz);
}

/** @copydoc GIMBAL_AxisInit */
void GIMBAL_AxisInit(void)
{
    // 调用 GIMBAL_DeviceInit，初始化对应模块或运行状态。
    GIMBAL_DeviceInit();

    s_yaw = (GIMBAL_AxisState_t) {
        GIMBAL_DEVICE_AXIS_YAW, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f,
        GIMBAL_YAW_POSITION_KP, GIMBAL_YAW_POSITION_KI,
        GIMBAL_YAW_VELOCITY_KD, GIMBAL_YAW_MIN_POSITION_COUNTS,
        GIMBAL_YAW_MAX_POSITION_COUNTS
    };
    s_pitch = (GIMBAL_AxisState_t) {
        GIMBAL_DEVICE_AXIS_PITCH, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f,
        GIMBAL_PITCH_POSITION_KP, GIMBAL_PITCH_POSITION_KI,
        GIMBAL_PITCH_VELOCITY_KD, GIMBAL_PITCH_MIN_POSITION_COUNTS,
        GIMBAL_PITCH_MAX_POSITION_COUNTS
    };

    /*
     * 上电后先保持驱动器失能，收到两路 MT6816 PWM 绝对角度并完成
     * A/B 初值对齐后，才允许闭环发出 STEP 脉冲。
     */
    GIMBAL_DeviceSetEnabled(GIMBAL_DEVICE_AXIS_YAW, false);
    // 调用 GIMBAL_DeviceSetEnabled，更新或发送对应数据。
    GIMBAL_DeviceSetEnabled(GIMBAL_DEVICE_AXIS_PITCH, false);
    // 更新 s_feedback_ready 对应的有效或使能状态。
    s_feedback_ready = false;
    // 更新 s_follow_origin_valid 对应的有效或使能状态。
    s_follow_origin_valid = false;
    // 更新 s_vision_valid 对应的有效或使能状态。
    s_vision_valid = false;
    // 更新 s_vision_age_ms 对应的时间状态。
    s_vision_age_ms = GIMBAL_VISION_TIMEOUT_MS;
}

/** @copydoc GIMBAL_AxisSetVisionTarget */
void GIMBAL_AxisSetVisionTarget(
    bool valid, int16_t laser_x, int16_t laser_y)
{
    // 目标失效时立即保持当前位置，避免继续追踪旧坐标。
    if (!valid)
    {
        // 更新 s_vision_valid 对应的有效或使能状态。
        s_vision_valid = false;
        // 更新 s_yaw.target 对应的目标值。
        s_yaw.target = (float)s_yaw.position;
        // 更新 s_pitch.target 对应的目标值。
        s_pitch.target = (float)s_pitch.position;
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 将图像坐标偏差转换为相对当前位置的编码器目标。
    s_yaw.target = GIMBAL_Clamp(
        (float)s_yaw.position +
            ((float)(laser_x - 320) * GIMBAL_YAW_COUNTS_PER_PIXEL *
                GIMBAL_YAW_VISION_SIGN),
        s_yaw.minimum_position, s_yaw.maximum_position);
    // 调用 GIMBAL_Clamp，完成当前步骤的业务处理。
    s_pitch.target = GIMBAL_Clamp(
        (float)s_pitch.position +
            ((float)(laser_y - 180) * GIMBAL_PITCH_COUNTS_PER_PIXEL *
                GIMBAL_PITCH_VISION_SIGN),
        s_pitch.minimum_position, s_pitch.maximum_position);
    // 更新 s_vision_valid 对应的有效或使能状态。
    s_vision_valid = true;
    // 更新 s_vision_age_ms 对应的时间状态。
    s_vision_age_ms = 0U;
}

/** @copydoc GIMBAL_AxisBeginAngleFollow */
bool GIMBAL_AxisBeginAngleFollow(void)
{
    // 绝对位置尚未同步时不能建立可靠的相对跟随零点。
    if (!s_feedback_ready)
    {
        // 返回失败状态，表示当前条件尚未满足。
        return false;
    }

    // 记录当前两轴位置，使 WT901 后续角度变化成为相对目标。
    s_follow_yaw_origin = (float)s_yaw.position;
    // 更新 s_follow_pitch_origin 对应的本步骤的运行数据。
    s_follow_pitch_origin = (float)s_pitch.position;
    // 更新 s_yaw.target 对应的目标值。
    s_yaw.target = s_follow_yaw_origin;
    // 更新 s_pitch.target 对应的目标值。
    s_pitch.target = s_follow_pitch_origin;
    // 更新 s_yaw.integral 对应的积分状态。
    s_yaw.integral = 0.0f;
    // 更新 s_pitch.integral 对应的积分状态。
    s_pitch.integral = 0.0f;
    // 更新 s_vision_valid 对应的有效或使能状态。
    s_vision_valid = false;
    // 更新 s_follow_origin_valid 对应的有效或使能状态。
    s_follow_origin_valid = true;
    // 返回成功状态，表示本次处理已经完成。
    return true;
}

/** @copydoc GIMBAL_AxisSetRelativeAngleTarget */
void GIMBAL_AxisSetRelativeAngleTarget(
    float yaw_delta_deg, float pitch_delta_deg)
{
    // 只接受已完成反馈同步且已建立跟随零点的角度目标。
    if (!s_feedback_ready || !s_follow_origin_valid)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 将相对角度换算为编码器计数，并限制在当前软件行程内。
    s_yaw.target = GIMBAL_Clamp(
        s_follow_yaw_origin +
            (yaw_delta_deg * GIMBAL_ENCODER_COUNTS_PER_DEGREE),
        s_yaw.minimum_position, s_yaw.maximum_position);
    // 调用 GIMBAL_Clamp，完成当前步骤的业务处理。
    s_pitch.target = GIMBAL_Clamp(
        s_follow_pitch_origin +
            (pitch_delta_deg * GIMBAL_ENCODER_COUNTS_PER_DEGREE),
        s_pitch.minimum_position, s_pitch.maximum_position);
    // 更新 s_vision_valid 对应的有效或使能状态。
    s_vision_valid = false;
}

/** @copydoc GIMBAL_AxisHoldCurrentPosition */
void GIMBAL_AxisHoldCurrentPosition(void)
{
    // 用当前反馈覆盖目标，同时清除积分和跟随零点，防止恢复时突然运动。
    s_yaw.target = (float)s_yaw.position;
    // 更新 s_pitch.target 对应的目标值。
    s_pitch.target = (float)s_pitch.position;
    // 更新 s_yaw.integral 对应的积分状态。
    s_yaw.integral = 0.0f;
    // 更新 s_pitch.integral 对应的积分状态。
    s_pitch.integral = 0.0f;
    // 更新 s_follow_origin_valid 对应的有效或使能状态。
    s_follow_origin_valid = false;
    // 更新 s_vision_valid 对应的有效或使能状态。
    s_vision_valid = false;
}

/** @copydoc GIMBAL_AxisIsReady */
bool GIMBAL_AxisIsReady(void)
{
    // 返回本次计算或查询得到的结果。
    return s_feedback_ready;
}

/** @copydoc GIMBAL_AxisUpdate */
void GIMBAL_AxisUpdate(uint32_t elapsed_ms)
{
    // 无有效节拍时不更新速度差分，避免除零和重复控制。
    if (elapsed_ms == 0U)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 调用 GIMBAL_DeviceUpdate，更新并处理对应业务数据。
    GIMBAL_DeviceUpdate();
    // 上电阶段等待两轴 PWM 绝对角度完成 A/B 多圈位置对齐。
    if (!s_feedback_ready)
    {
        // 判断当前数据或反馈状态是否已经有效。
        if (!GIMBAL_DeviceIsFeedbackReady())
        {
            // 任一轴尚未同步时保持两轴停止，避免以错误位置闭环。
            GIMBAL_DeviceStop(GIMBAL_DEVICE_AXIS_YAW);
            // 调用 GIMBAL_DeviceStop，停止运动或保持当前安全状态。
            GIMBAL_DeviceStop(GIMBAL_DEVICE_AXIS_PITCH);
            // 返回本次计算或查询得到的结果。
            return;
        }

        // 以同步完成时的位置初始化闭环，避免使能瞬间产生位置阶跃。
        s_yaw.position =
            GIMBAL_DeviceGetEncoderCount(GIMBAL_DEVICE_AXIS_YAW);
        // 调用 GIMBAL_DeviceGetEncoderCount，读取当前反馈或状态。
        s_pitch.position =
            GIMBAL_DeviceGetEncoderCount(GIMBAL_DEVICE_AXIS_PITCH);
        // 更新 s_yaw.last_position 对应的位置状态。
        s_yaw.last_position = s_yaw.position;
        // 更新 s_pitch.last_position 对应的位置状态。
        s_pitch.last_position = s_pitch.position;
        // 更新 s_yaw.target 对应的目标值。
        s_yaw.target = (float)s_yaw.position;
        // 更新 s_pitch.target 对应的目标值。
        s_pitch.target = (float)s_pitch.position;
        // 更新 s_yaw.velocity_cps 对应的速度状态。
        s_yaw.velocity_cps = 0.0f;
        // 更新 s_pitch.velocity_cps 对应的速度状态。
        s_pitch.velocity_cps = 0.0f;
        // 更新 s_yaw.integral 对应的积分状态。
        s_yaw.integral = 0.0f;
        // 更新 s_pitch.integral 对应的积分状态。
        s_pitch.integral = 0.0f;
        // 更新 s_yaw.command_step_hz 对应的本步骤的运行数据。
        s_yaw.command_step_hz = 0.0f;
        // 更新 s_pitch.command_step_hz 对应的本步骤的运行数据。
        s_pitch.command_step_hz = 0.0f;
        // 调用 GIMBAL_DeviceSetEnabled，更新或发送对应数据。
        GIMBAL_DeviceSetEnabled(GIMBAL_DEVICE_AXIS_YAW, true);
        // 调用 GIMBAL_DeviceSetEnabled，更新或发送对应数据。
        GIMBAL_DeviceSetEnabled(GIMBAL_DEVICE_AXIS_PITCH, true);
        // 更新 s_feedback_ready 对应的有效或使能状态。
        s_feedback_ready = true;
    }

    // 视觉目标使用独立超时计时，长时间无新帧时自动保持当前位置。
    if (s_vision_age_ms <= (UINT32_MAX - elapsed_ms))
    {
        // 更新 s_vision_age_ms 对应的时间状态。
        s_vision_age_ms += elapsed_ms;
    }
    // 前述条件均不成立时执行备用处理。
    else
    {
        // 更新 s_vision_age_ms 对应的时间状态。
        s_vision_age_ms = UINT32_MAX;
    }
    // 判断当前数据或反馈状态是否已经有效。
    if (s_vision_valid && (s_vision_age_ms > GIMBAL_VISION_TIMEOUT_MS))
    {
        // 更新 s_vision_valid 对应的有效或使能状态。
        s_vision_valid = false;
        // 更新 s_yaw.target 对应的目标值。
        s_yaw.target = (float)s_yaw.position;
        // 更新 s_pitch.target 对应的目标值。
        s_pitch.target = (float)s_pitch.position;
    }

    // 两轴使用相同控制周期分别完成位置闭环。
    GIMBAL_UpdateOneAxis(&s_yaw, elapsed_ms);
    // 调用 GIMBAL_UpdateOneAxis，更新并处理对应业务数据。
    GIMBAL_UpdateOneAxis(&s_pitch, elapsed_ms);
}

/** @copydoc GIMBAL_AxisGetTelemetry */
void GIMBAL_AxisGetTelemetry(GIMBAL_AxisTelemetry_t *telemetry)
{
    // 检查相关输入、计数或对象状态是否有效。
    if (telemetry == 0)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 更新 telemetry->yaw_position 的编码器位置状态。
    telemetry->yaw_position = s_yaw.position;
    // 更新 telemetry->pitch_position 的编码器位置状态。
    telemetry->pitch_position = s_pitch.position;
    // 更新 telemetry->yaw_target 的控制目标值。
    telemetry->yaw_target = s_yaw.target;
    // 更新 telemetry->pitch_target 的控制目标值。
    telemetry->pitch_target = s_pitch.target;
    // 更新 telemetry->yaw_step_hz，保存当前语句的计算结果。
    telemetry->yaw_step_hz = s_yaw.command_step_hz;
    // 更新 telemetry->pitch_step_hz，保存当前语句的计算结果。
    telemetry->pitch_step_hz = s_pitch.command_step_hz;
    // 更新 telemetry->vision_valid 对应的状态标志。
    telemetry->vision_valid = s_vision_valid;
}
