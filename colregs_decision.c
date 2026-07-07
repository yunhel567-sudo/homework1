#include "colregs_decision.h"

int decideEvasiveAction(const RelativeMotionParams* params, int riskLevel) {
    // 0. 如果无碰撞危险，首要原则就是保向保速
    if (riskLevel == RISK_LEVEL_NONE) {
        return ACTION_STRAIGHT;
    }

    // 1. 计算相对方位角 (Relative Bearing)
    // 相对方位 = 目标真方位 - 自身航向
    double RB = params->trueBearing - params->ownHeading;
    
    // 将相对方位规范化到 0 ~ 360 度范围内
    while (RB < 0.0) {
        RB += 360.0;
    }
    while (RB >= 360.0) {
        RB -= 360.0;
    }

    // 2. 计算航向差
    double headingDiff = params->targetHeading - params->ownHeading;
    
    // 规范化到 0 ~ 360 度范围内
    while (headingDiff < 0.0) {
        headingDiff += 360.0;
    }
    while (headingDiff >= 360.0) {
        headingDiff -= 360.0;
    }

    // ==========================================
    // 3. 核心 COLREGs 局面分类树
    // ==========================================

    // 情况 A: 对遇局面 (Head-on)
    // 目标在正前方狭窄扇区 (±10度)，且航向基本相反 (误差允许±20度)
    if ((RB <= 10.0 || RB >= 350.0) && (headingDiff >= 160.0 && headingDiff <= 200.0)) {
        return ACTION_TURN_STARBOARD; // 对遇需向右转向
    }

    // 情况 B: 追越局面 (Overtaking) - 本船追越他船
    // 航向基本一致(差值<45度)，目标在前方，且本船速大于他船
    else if ((RB < 45.0 || RB > 315.0) && (headingDiff <= 45.0 || headingDiff >= 315.0) && (params->ownSpeed_ms > params->targetSpeed_ms)) {
        // 追赶右前方的船只时，选择向左打舵从其左舷超船，避免横越船艏
        if (RB <= 45.0) {
            return ACTION_TURN_PORT;      // 左转
        } else {
            return ACTION_TURN_STARBOARD; // 右转
        }
    }

    // 情况 C: 交叉相遇 - 目标在右舷 (Crossing - Give-way)
    // 相对方位在 10度 到 112.5度 之间，本船为让路船
    else if (RB > 10.0 && RB <= 112.5) {
        return ACTION_TURN_STARBOARD; // 应当右转，从他船尾部绕过
    }

    // 情况 D: 交叉相遇/被追越 - 目标在左舷或后方 (Crossing - Stand-on)
    // 目标船在左舷 (112.5度 到 247.5度) 或被追越，本船为直航船
    else {
        // 作为直航船，规则要求首选保向保速
        return ACTION_STRAIGHT; 
    }
}