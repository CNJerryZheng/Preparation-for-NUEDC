/**
 * @file        chassis_task.c
 * @author      JerryZheng
 * @brief       底盘周期业务服务
 * @date        2026-07-21
 */

#include "chassis_task.h"
#include "bsp_gpio.h"
#include "bsp_timer.h"
#include "chassis_control.h"
#include "linetrack.h"

#define CHASSIS_BUTTON_DEBOUNCE_TICKS (3U)

// 更新 g_line_raw 对应的本步骤的运行数据。
volatile uint8_t g_line_raw = 0xFFU;
// 更新 g_line_position 对应的位置状态。
volatile float g_line_position = 0.0f;
// 更新 g_line_state 对应的状态机状态。
volatile LINE_State_t g_line_state = State_Unknown;
// 更新 g_line_left_duty_percent 对应的本步骤的运行数据。
volatile int16_t g_line_left_duty_percent = 0;
// 更新 g_line_right_duty_percent 对应的本步骤的运行数据。
volatile int16_t g_line_right_duty_percent = 0;
// 更新 g_line_follow_enabled 对应的有效或使能状态。
volatile bool g_line_follow_enabled = false;

// 更新 s_button_candidate 对应的本步骤的运行数据。
static bool s_button_candidate = false;
// 更新 s_button_stable 对应的本步骤的运行数据。
static bool s_button_stable = false;
// 更新 s_button_stable_ticks 对应的本步骤的运行数据。
static uint8_t s_button_stable_ticks = 0U;

/**
 * @brief 对 PA18 板载按键消抖，并在稳定按下时切换循迹状态
 */
static void CHASSIS_TaskProcessButton(void)
{
    // 调用 BSP_GPIO_IsUserButtonPressed，完成当前步骤的业务处理。
    const bool pressed = BSP_GPIO_IsUserButtonPressed();

    // 原始电平变化后重新开始连续稳定计数。
    if (pressed != s_button_candidate)
    {
        // 更新 s_button_candidate 对应的本步骤的运行数据。
        s_button_candidate = pressed;
        // 更新 s_button_stable_ticks 对应的本步骤的运行数据。
        s_button_stable_ticks = 1U;
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 候选状态尚未稳定到设定节拍数时暂不改变业务状态。
    if (s_button_stable_ticks < CHASSIS_BUTTON_DEBOUNCE_TICKS)
    {
        // 更新对应的累计计数或遍历位置。
        ++s_button_stable_ticks;
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 稳定状态未发生变化时无需重复切换循迹使能。
    if (s_button_stable == s_button_candidate)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 更新 s_button_stable 对应的本步骤的运行数据。
    s_button_stable = s_button_candidate;
    // 仅在稳定按下沿切换一次，松开动作只更新消抖状态。
    if (s_button_stable)
    {
        // 更新 g_line_follow_enabled 对应的有效或使能状态。
        g_line_follow_enabled = !g_line_follow_enabled;
        // 调用 CHASSIS_SetLineFollowEnabled，更新或发送对应数据。
        CHASSIS_SetLineFollowEnabled(g_line_follow_enabled);
        // 调用 CHASSIS_SetEnabled，更新或发送对应数据。
        CHASSIS_SetEnabled(g_line_follow_enabled);
    }
}

/**
 * @brief 初始化底盘周期业务
 */
void CHASSIS_TaskInit(void)
{
    // 上电默认进入循迹模式，按键可在运行中切换启停。
    CHASSIS_ControlInit();
    // 更新 g_line_follow_enabled 对应的有效或使能状态。
    g_line_follow_enabled = true;
    // 调用 CHASSIS_SetLineFollowEnabled，更新或发送对应数据。
    CHASSIS_SetLineFollowEnabled(true);
    // 调用 CHASSIS_SetEnabled，更新或发送对应数据。
    CHASSIS_SetEnabled(true);
    // 调用 BSP_GPIO_IsUserButtonPressed，完成当前步骤的业务处理。
    s_button_candidate = BSP_GPIO_IsUserButtonPressed();
    // 更新 s_button_stable 对应的本步骤的运行数据。
    s_button_stable = s_button_candidate;
    // 更新 s_button_stable_ticks 对应的本步骤的运行数据。
    s_button_stable_ticks = CHASSIS_BUTTON_DEBOUNCE_TICKS;
}

/**
 * @brief 在 10ms 节拍到达时处理循迹与底盘控制业务
 */
void CHASSIS_TaskProcess(void)
{
    // 调用 BSP_Timer_TakeChassisTicks，读取当前反馈或状态。
    const uint32_t elapsed_ticks = BSP_Timer_TakeChassisTicks();
    // 定义本步骤需要的局部数据并完成初始化。
    int16_t left_percent;
    // 定义本步骤需要的局部数据并完成初始化。
    int16_t right_percent;

    // 没有新的10ms控制节拍时不重复读取和计算。
    if (elapsed_ticks == 0U)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }
    // 先获取传感器结果和按键状态，再执行本周期底盘控制。
    LINE_Result_t line = LINE_Process();
    // 调用 CHASSIS_TaskProcessButton，更新并处理对应业务数据。
    CHASSIS_TaskProcessButton();
    // 调用 CHASSIS_ControlUpdate，更新并处理对应业务数据。
    CHASSIS_ControlUpdate(&line, elapsed_ticks);

    // 将本周期关键状态镜像到全局变量供 Live Watch 观察。
    g_line_raw = line.raw;
    // 更新 g_line_position 对应的位置状态。
    g_line_position = line.position;
    // 更新 g_line_state 对应的状态机状态。
    g_line_state = line.state;
    // 调用 CHASSIS_GetWheelDutyPercent，读取当前反馈或状态。
    CHASSIS_GetWheelDutyPercent(&left_percent, &right_percent);
    // 更新 g_line_left_duty_percent 对应的本步骤的运行数据。
    g_line_left_duty_percent = left_percent;
    // 更新 g_line_right_duty_percent 对应的本步骤的运行数据。
    g_line_right_duty_percent = right_percent;
}
