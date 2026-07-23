/**
 * @file        line_control.h
 * @author      JerryZheng
 * @brief       八路循迹差速控制算法接口
 * @date        2026-07-22
 */

#pragma once

#include <stdint.h>
#include "linetrack.h"

/**
 * @brief 根据八路循迹结果计算左右轮闭环速度目标
 * @param line 当前循迹结果
 * @param elapsed_ticks 距离上次计算实际经过的 10ms 节拍数
 * @param left_cps 左轮有符号速度目标输出地址，单位为等效 x1 霍尔计数/秒
 * @param right_cps 右轮有符号速度目标输出地址，单位为等效 x1 霍尔计数/秒
 * @note 正常状态采用比例控制；全黑时直行，短暂异常或未知状态保持上一帧输出。
 */
void LINE_ControlCalculate(const LINE_Result_t *line, uint32_t elapsed_ticks,
    float *left_cps, float *right_cps);

/**
 * @brief 清除循迹控制的历史输出
 * @note 下一次计算会从 0 平滑过渡到配置的循迹速度。
 */
void LINE_ControlReset(void);
