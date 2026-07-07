#ifndef COLREGS_DECISION_H
#define COLREGS_DECISION_H

#include "ais_sync.h"
#include "collision_risk.h"

// 定义避让动作宏
#define ACTION_STRAIGHT         0   // 直行 (保向保速)
#define ACTION_TURN_PORT        1   // 左转
#define ACTION_TURN_STARBOARD   2   // 右转

/**
 * @brief 基于 COLREGs 判断会遇局面并输出避让动作
 * * @param params    [in] 时间对齐后的两船相对运动参数
 * @param riskLevel [in] 前置模块计算出的碰撞危险程度
 * @return int      返回避让动作 (0-直行, 1-左转, 2-右转)
 */
int decideEvasiveAction(const RelativeMotionParams* params, int riskLevel);

#endif // COLREGS_DECISION_H