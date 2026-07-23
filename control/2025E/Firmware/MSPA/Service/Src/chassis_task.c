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

volatile uint8_t g_line_raw = 0xFFU;
volatile float g_line_position = 0.0f;
volatile LINE_State_t g_line_state = State_Unknown;
volatile int16_t g_line_left_duty_percent = 0;
volatile int16_t g_line_right_duty_percent = 0;
volatile bool g_line_follow_enabled = false;

static bool s_button_candidate = false;
static bool s_button_stable = false;
static uint8_t s_button_stable_ticks = 0U;

/**
 * @brief 对 PA18 板载按键消抖，并在稳定按下时切换循迹状态
 */
static void CHASSIS_TaskProcessButton(void)
{
    const bool pressed = BSP_GPIO_IsUserButtonPressed();

    // 原始电平变化后重新开始连续稳定计数。
    if (pressed != s_button_candidate)
    {
        s_button_candidate = pressed;
        s_button_stable_ticks = 1U;
        return;
    }

    // 候选状态尚未稳定到设定节拍数时暂不改变业务状态。
    if (s_button_stable_ticks < CHASSIS_BUTTON_DEBOUNCE_TICKS)
    {
        ++s_button_stable_ticks;
        return;
    }

    // 稳定状态未发生变化时无需重复切换循迹使能。
    if (s_button_stable == s_button_candidate)
    {
        return;
    }

    s_button_stable = s_button_candidate;
    // 仅在稳定按下沿切换一次，松开动作只更新消抖状态。
    if (s_button_stable)
    {
        g_line_follow_enabled = !g_line_follow_enabled;
        CHASSIS_SetLineFollowEnabled(g_line_follow_enabled);
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
    g_line_follow_enabled = true;
    CHASSIS_SetLineFollowEnabled(true);
    CHASSIS_SetEnabled(true);
    s_button_candidate = BSP_GPIO_IsUserButtonPressed();
    s_button_stable = s_button_candidate;
    s_button_stable_ticks = CHASSIS_BUTTON_DEBOUNCE_TICKS;
}

/**
 * @brief 在 10ms 节拍到达时处理循迹与底盘控制业务
 */
void CHASSIS_TaskProcess(void)
{
    const uint32_t elapsed_ticks = BSP_Timer_TakeChassisTicks();
    int16_t left_percent;
    int16_t right_percent;

    // 没有新的10ms控制节拍时不重复读取和计算。
    if (elapsed_ticks == 0U)
    {
        return;
    }
    // 先获取传感器结果和按键状态，再执行本周期底盘控制。
    LINE_Result_t line = LINE_Process();
    CHASSIS_TaskProcessButton();
    CHASSIS_ControlUpdate(&line, elapsed_ticks);

    // 将本周期关键状态镜像到全局变量供 Live Watch 观察。
    g_line_raw = line.raw;
    g_line_position = line.position;
    g_line_state = line.state;
    CHASSIS_GetWheelDutyPercent(&left_percent, &right_percent);
    g_line_left_duty_percent = left_percent;
    g_line_right_duty_percent = right_percent;
}
