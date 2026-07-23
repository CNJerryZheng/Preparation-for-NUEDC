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

// 更新 s_left_target_percent 对应的目标值。
static int16_t s_left_target_percent = 0;
// 更新 s_right_target_percent 对应的目标值。
static int16_t s_right_target_percent = 0;
// 更新 s_brake_requested 对应的本步骤的运行数据。
static bool s_brake_requested = false;
// 更新 s_line_follow_enabled 对应的有效或使能状态。
static bool s_line_follow_enabled = false;
// 更新 s_speed_closed_loop_enabled 对应的速度状态。
static bool s_speed_closed_loop_enabled = false;

/**
 * @brief 将占空比限制在电机允许范围内
 * @param duty_percent 原始有符号占空比
 * @return int16_t 限幅后的有符号占空比
 */
static int16_t CHASSIS_ClampDutyPercent(int16_t duty_percent)
{
    // 判断 duty_percent > 100 是否成立，并选择对应处理分支。
    if (duty_percent > 100)
    {
        // 返回本次计算或查询得到的结果。
        return 100;
    }
    // 判断 duty_percent < -100 是否成立，并选择对应处理分支。
    if (duty_percent < -100)
    {
        // 返回本次计算或查询得到的结果。
        return -100;
    }
    // 返回本次计算或查询得到的结果。
    return duty_percent;
}

/**
 * @brief 初始化差速底盘控制状态
 */
void CHASSIS_ControlInit(void)
{
    // 清空所有控制模式和目标后，再初始化电机、循迹与轮速闭环。
    s_left_target_percent = 0;
    // 更新 s_right_target_percent 对应的目标值。
    s_right_target_percent = 0;
    // 更新 s_brake_requested 对应的本步骤的运行数据。
    s_brake_requested = false;
    // 更新 s_line_follow_enabled 对应的有效或使能状态。
    s_line_follow_enabled = false;
    // 更新 s_speed_closed_loop_enabled 对应的速度状态。
    s_speed_closed_loop_enabled = false;
    // 调用 LINE_ControlReset，复位对应模块的历史状态。
    LINE_ControlReset();
    // 调用 MG513X_Init，初始化对应模块或运行状态。
    MG513X_Init();
    // 调用 WHEEL_SpeedControlInit，初始化对应模块或运行状态。
    WHEEL_SpeedControlInit();
    // 调用 MG513X_SetEnabled，更新或发送对应数据。
    MG513X_SetEnabled(false);
}

/**
 * @brief 设置底盘电机驱动总使能
 * @param enable true 允许输出，false 关闭输出
 */
void CHASSIS_SetEnabled(bool enable)
{
    // 调用 MG513X_SetEnabled，更新或发送对应数据。
    MG513X_SetEnabled(enable);
}

/**
 * @brief 设置循迹速度外环是否接管左右轮速度目标
 * @param enable true 启用循迹，false 关闭循迹
 */
void CHASSIS_SetLineFollowEnabled(bool enable)
{
    // 更新 s_line_follow_enabled 对应的有效或使能状态。
    s_line_follow_enabled = enable;
    // 模式切换时清除循迹历史，避免沿用旧的误差和速度斜坡。
    LINE_ControlReset();
    // 判断对应业务功能当前是否处于使能状态。
    if (enable)
    {
        // 循迹外环输出轮速目标，因此启用轮速内环并退出独立调速模式。
        s_speed_closed_loop_enabled = false;
        // 调用 WHEEL_SpeedControlSetEnabled，更新或发送对应数据。
        WHEEL_SpeedControlSetEnabled(true);
    }
    // 前述条件均不成立时执行备用处理。
    else
    {
        // 关闭循迹时同步关闭轮速内环并释放当前电机目标。
        WHEEL_SpeedControlSetEnabled(false);
        // 调用 CHASSIS_Stop，停止运动或保持当前安全状态。
        CHASSIS_Stop(false);
    }
}

/**
 * @brief 查询循迹控制是否启用
 * @retval bool 当前循迹使能状态
 */
bool CHASSIS_IsLineFollowEnabled(void)
{
    // 返回本次计算或查询得到的结果。
    return s_line_follow_enabled;
}

/**
 * @brief 设置双轮速度闭环是否接管电机输出
 * @param enable true 启用速度 PID，false 关闭速度 PID
 */
void CHASSIS_SetSpeedClosedLoopEnabled(bool enable)
{
    // 更新 s_speed_closed_loop_enabled 对应的速度状态。
    s_speed_closed_loop_enabled = enable;
    // 调用 WHEEL_SpeedControlSetEnabled，更新或发送对应数据。
    WHEEL_SpeedControlSetEnabled(enable);
    // 判断对应业务功能当前是否处于使能状态。
    if (enable)
    {
        // 独立轮速调试与循迹模式互斥，防止两个模块同时修改目标。
        s_line_follow_enabled = false;
        // 调用 LINE_ControlReset，复位对应模块的历史状态。
        LINE_ControlReset();
    }
    // 前述条件均不成立时执行备用处理。
    else
    {
        // 调用 CHASSIS_Stop，停止运动或保持当前安全状态。
        CHASSIS_Stop(false);
    }
}

/**
 * @brief 查询双轮速度闭环是否启用
 * @retval bool 当前速度闭环使能状态
 */
bool CHASSIS_IsSpeedClosedLoopEnabled(void)
{
    // 返回本次计算或查询得到的结果。
    return s_speed_closed_loop_enabled;
}

/**
 * @brief 设置双轮速度闭环目标值
 * @param left_cps 左轮目标速度，单位为霍尔计数/秒
 * @param right_cps 右轮目标速度，单位为霍尔计数/秒
 */
void CHASSIS_SetWheelSpeedTarget(float left_cps, float right_cps)
{
    // 调用 WHEEL_SpeedControlSetTarget，更新或发送对应数据。
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
    // 调用 WHEEL_SpeedControlSetPid，更新或发送对应数据。
    WHEEL_SpeedControlSetPid(kp, ki, kd);
}

/**
 * @brief 在线设置双轮速度前馈系数
 * @param gain 前馈系数，单位为占空比百分数/(霍尔计数/秒)
 */
void CHASSIS_SetWheelSpeedFeedforward(float gain)
{
    // 调用 WHEEL_SpeedControlSetFeedforward，更新或发送对应数据。
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
    // 调用 CHASSIS_ClampDutyPercent，完成当前步骤的业务处理。
    s_left_target_percent = CHASSIS_ClampDutyPercent(left_percent);
    // 调用 CHASSIS_ClampDutyPercent，完成当前步骤的业务处理。
    s_right_target_percent = CHASSIS_ClampDutyPercent(right_percent);
    // 更新 s_brake_requested 对应的本步骤的运行数据。
    s_brake_requested = false;
}

/**
 * @brief 停止底盘
 * @param brake true 使用短路制动，false 使用滑行停止
 */
void CHASSIS_Stop(bool brake)
{
    // 更新 s_left_target_percent 对应的目标值。
    s_left_target_percent = 0;
    // 更新 s_right_target_percent 对应的目标值。
    s_right_target_percent = 0;
    // 更新 s_brake_requested 对应的本步骤的运行数据。
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
    // 检查相关输入、计数或对象状态是否有效。
    if (left_percent != 0)
    {
        *left_percent = s_left_target_percent;
    }
    // 检查相关输入、计数或对象状态是否有效。
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
    // 循迹模式由循迹外环计算双轮速度目标，再交给速度内环。
    if (s_line_follow_enabled)
    {
        // 定义本步骤需要的局部数据并完成初始化。
        float left_target_cps;
        // 定义本步骤需要的局部数据并完成初始化。
        float right_target_cps;

        // 调用 LINE_ControlCalculate，更新并处理对应业务数据。
        LINE_ControlCalculate(line, elapsed_ticks,
            &left_target_cps, &right_target_cps);
        // 调用 WHEEL_SpeedControlSetTarget，更新或发送对应数据。
        WHEEL_SpeedControlSetTarget(left_target_cps, right_target_cps);
    }

    // 每个有效节拍都更新测速和PID内部状态。
    WHEEL_SpeedControlUpdate(elapsed_ticks);

    // 任一速度闭环模式生效时，用PID输出覆盖手动占空比目标。
    if (s_speed_closed_loop_enabled || s_line_follow_enabled)
    {
        // 调用 WHEEL_SpeedControlGetOutput，读取当前反馈或状态。
        WHEEL_SpeedControlGetOutput(
            &s_left_target_percent, &s_right_target_percent);
        // 更新 s_brake_requested 对应的本步骤的运行数据。
        s_brake_requested = false;
    }

    // 制动请求优先于普通占空比输出。
    if (s_brake_requested)
    {
        // 调用 MG513X_Brake，停止运动或保持当前安全状态。
        MG513X_Brake(MG513X_MOTOR_LEFT);
        // 调用 MG513X_Brake，停止运动或保持当前安全状态。
        MG513X_Brake(MG513X_MOTOR_RIGHT);
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 调用 MG513X_SetDutyPercent，更新或发送对应数据。
    MG513X_SetDutyPercent(MG513X_MOTOR_LEFT, s_left_target_percent);
    // 调用 MG513X_SetDutyPercent，更新或发送对应数据。
    MG513X_SetDutyPercent(MG513X_MOTOR_RIGHT, s_right_target_percent);
}
