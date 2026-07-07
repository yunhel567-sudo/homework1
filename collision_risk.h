#ifndef COLLISION_RISK_H
#define COLLISION_RISK_H

#include "ais_sync.h"

// 定义碰撞风险等级宏
#define RISK_LEVEL_NONE 0   // 无风险
#define RISK_LEVEL_LOW  1   // 风险小
#define RISK_LEVEL_HIGH 2   // 风险大

/**
 * @brief 计算 DCPA, TCPA 并评估碰撞风险
 * * @param params   [in]  时间对齐后的两船相对运动参数
 * @param outDcpa  [out] 输出计算得到的 DCPA (单位：米)。如果不需要可传 NULL
 * @param outTcpa  [out] 输出计算得到的 TCPA (单位：秒)。如果不需要可传 NULL
 * @return int     返回碰撞风险等级 (0-无风险, 1-风险小, 2-风险大)
 */
int assessCollisionRisk(const RelativeMotionParams* params, double* outDcpa, double* outTcpa);

#endif // COLLISION_RISK_H