/**
 * @file        chassis_control.c
 * @author      JerryZheng
 * @brief       差速底盘占空比控制实现
 * @date        2026-07-22
 */

#include "chassis_control.h"
#include "line_control.h"
#include "mg513x.h"
#include "wheel_speed_control.h"

static int16_t s_left_target_percent = 0;
static int16_t s_right_target_percent = 0;
static bool s_brake_requested = false;
static bool s_line_follow_enabled = false;
static bool s_speed_closed_loop_enabled = false;

/**
 * @brief 将占空比限制在电机允许范围内
 * @param duty_percent 原始有符号占空比
 * @return int16_t 限幅后的有符号占空比
 */
static int16_t CHASSIS_ClampDutyPercent(int16_t duty_percent)
{
    if (duty_percent > 100)
    {
        return 100;
    }
    if (duty_percent < -100)
    {
        return -100;
    }
    return duty_percent;
}

/**
 * @brief 初始化差速底盘控制状态
 */
void CHASSIS_ControlInit(void)
{
    s_left_target_percent = 0;
    s_right_target_percent = 0;
    s_brake_requested = false;
    s_line_follow_enabled = false;
    s_speed_closed_loop_enabled = false;
    LINE_ControlReset();
    MG513X_Init();
    WHEEL_SpeedControlInit();
    MG513X_SetEnabled(false);
}

/**
 * @brief 设置底盘电机驱动总使能
 * @param enable true 允许输出，false 关闭输出
 */
void CHASSIS_SetEnabled(bool enable)
{
    MG513X_SetEnabled(enable);
}

/**
 * @brief 设置循迹速度外环是否接管左右轮速度目标
 * @param enable true 启用循迹，false 关闭循迹
 */
void CHASSIS_SetLineFollowEnabled(bool enable)
{
    s_line_follow_enabled = enable;
    LINE_ControlReset();
    if (enable)
    {
        s_speed_closed_loop_enabled = false;
        WHEEL_SpeedControlSetEnabled(true);
    }
    else
    {
        WHEEL_SpeedControlSetEnabled(false);
        CHASSIS_Stop(false);
    }
}

/**
 * @brief 查询循迹控制是否启用
 * @retval bool 当前循迹使能状态
 */
bool CHASSIS_IsLineFollowEnabled(void)
{
    return s_line_follow_enabled;
}

/**
 * @brief 设置双轮速度闭环是否接管电机输出
 * @param enable true 启用速度 PID，false 关闭速度 PID
 */
void CHASSIS_SetSpeedClosedLoopEnabled(bool enable)
{
    s_speed_closed_loop_enabled = enable;
    WHEEL_SpeedControlSetEnabled(enable);
    if (enable)
    {
        s_line_follow_enabled = false;
        LINE_ControlReset();
    }
    else
    {
        CHASSIS_Stop(false);
    }
}

/**
 * @brief 查询双轮速度闭环是否启用
 * @retval bool 当前速度闭环使能状态
 */
bool CHASSIS_IsSpeedClosedLoopEnabled(void)
{
    return s_speed_closed_loop_enabled;
}

/**
 * @brief 设置双轮速度闭环目标值
 * @param left_cps 左轮目标速度，单位为霍尔计数/秒
 * @param right_cps 右轮目标速度，单位为霍尔计数/秒
 */
void CHASSIS_SetWheelSpeedTarget(float left_cps, float right_cps)
{
    WHEEL_SpeedControlSetTarget(left_cps, right_cps);
}

/**
 * @brief 在线设置双轮速度闭环 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void CHASSIS_SetWheelSpeedPid(float kp, float ki, float kd)
{
    WHEEL_SpeedControlSetPid(kp, ki, kd);
}

/**
 * @brief 在线设置双轮速度前馈系数
 * @param gain 前馈系数，单位为占空比百分数/(霍尔计数/秒)
 */
void CHASSIS_SetWheelSpeedFeedforward(float gain)
{
    WHEEL_SpeedControlSetFeedforward(gain);
}

/**
 * @brief 设置左右轮有符号 PWM 占空比
 * @param left_percent 左轮占空比，范围 -100~100
 * @param right_percent 右轮占空比，范围 -100~100
 */
void CHASSIS_SetWheelDutyPercent(
    int16_t left_percent, int16_t right_percent)
{
    s_left_target_percent = CHASSIS_ClampDutyPercent(left_percent);
    s_right_target_percent = CHASSIS_ClampDutyPercent(right_percent);
    s_brake_requested = false;
}

/**
 * @brief 停止底盘
 * @param brake true 使用短路制动，false 使用滑行停止
 */
void CHASSIS_Stop(bool brake)
{
    s_left_target_percent = 0;
    s_right_target_percent = 0;
    s_brake_requested = brake;
}

/**
 * @brief 获取当前左右轮目标占空比
 * @param left_percent 左轮目标占空比输出地址，可传入空指针
 * @param right_percent 右轮目标占空比输出地址，可传入空指针
 */
void CHASSIS_GetWheelDutyPercent(
    int16_t *left_percent, int16_t *right_percent)
{
    if (left_percent != 0)
    {
        *left_percent = s_left_target_percent;
    }
    if (right_percent != 0)
    {
        *right_percent = s_right_target_percent;
    }
}

/**
 * @brief 在 10 ms 控制节拍中更新底盘电机输出
 * @param line 当前循迹结果
 */
void CHASSIS_ControlUpdate(
    const LINE_Result_t *line, uint32_t elapsed_ticks)
{
    if (s_line_follow_enabled)
    {
        float left_target_cps;
        float right_target_cps;

        LINE_ControlCalculate(line, elapsed_ticks,
            &left_target_cps, &right_target_cps);
        WHEEL_SpeedControlSetTarget(left_target_cps, right_target_cps);
    }

    WHEEL_SpeedControlUpdate(elapsed_ticks);

    if (s_speed_closed_loop_enabled || s_line_follow_enabled)
    {
        WHEEL_SpeedControlGetOutput(
            &s_left_target_percent, &s_right_target_percent);
        s_brake_requested = false;
    }

    if (s_brake_requested)
    {
        MG513X_Brake(MG513X_MOTOR_LEFT);
        MG513X_Brake(MG513X_MOTOR_RIGHT);
        return;
    }

    MG513X_SetDutyPercent(MG513X_MOTOR_LEFT, s_left_target_percent);
    MG513X_SetDutyPercent(MG513X_MOTOR_RIGHT, s_right_target_percent);
}
