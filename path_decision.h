#ifndef PATH_DECISION_H
#define PATH_DECISION_H

#include "ais_sync.h"
#include "colregs_decision.h"

// 定义避让策略输出结构体
typedef struct {
    int methodUsed;          // 使用的方法: 0-无操作, 1-GA算法, 2-VO算法
    int actionDirection;     // 转向方向: 1-左转, 2-右转
    
    // GA 算法输出结果 (仅当 methodUsed == 1 时有效)
    float ga_t_delay;        // 直航时间/延迟执行时间 (秒)
    float ga_turn_angle;     // 转向角度 (0-180度)
    float ga_t_avoid;        // 避让持续时间 (秒)
    float ga_return_angle;   // 复航转向角度 (0-180度)
    
    // VO 算法输出结果 (仅当 methodUsed == 2 时有效)
    float vo_turn_angle;     // 速度调整角度 (0-180度)
} AvoidancePath;

/**
 * @brief 核心决策规划模块：计算最终的避碰路径或转向角
 * * @param params          时间对齐后的两船相对运动参数
 * @param riskLevel       危险程度 (0-无, 1-小, 2-大)
 * @param actionDirection COLREGs 建议的行动方向 (0-直行, 1-左转, 2-右转)
 * @param D_safe          安全距离阈值 (米)
 * @return AvoidancePath  规划出的避碰参数
 */
AvoidancePath planAvoidancePath(const RelativeMotionParams* params, int riskLevel, int actionDirection, double D_safe);

#endif // PATH_DECISION_H