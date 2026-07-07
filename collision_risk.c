#include "collision_risk.h"
#include <math.h>

#define PI 3.14159265358979323846
#define DEG2RAD(x) ((x) * PI / 180.0)

// 定义危险判定阈值 (可以根据水域和实际需求调整)
#define DCPA_HIGH_RISK 1852.0    // 高风险距离阈值：1海里 = 1852米
#define DCPA_LOW_RISK  3704.0    // 低风险距离阈值：2海里 = 3704米
#define TCPA_HIGH_RISK 900.0     // 高风险时间阈值：15分钟 = 900秒
#define TCPA_LOW_RISK  1800.0    // 低风险时间阈值：30分钟 = 1800秒

int assessCollisionRisk(const RelativeMotionParams* params, double* outDcpa, double* outTcpa) {
    // 1. 将极坐标形式的相对位置，转换为自身船坐标系下的 X/Y 坐标 (米)
    // 航海坐标系：正北为 0 度 (Y轴)，顺时针增加。
    double tar_px = params->distance * sin(DEG2RAD(params->trueBearing));
    double tar_py = params->distance * cos(DEG2RAD(params->trueBearing));

    // 2. 将两船的对地航速分解为 X 和 Y 方向的速度分量 (米/秒)
    double own_vx = params->ownSpeed_ms * sin(DEG2RAD(params->ownHeading));
    double own_vy = params->ownSpeed_ms * cos(DEG2RAD(params->ownHeading));

    double tar_vx = params->targetSpeed_ms * sin(DEG2RAD(params->targetHeading));
    double tar_vy = params->targetSpeed_ms * cos(DEG2RAD(params->targetHeading));

    // 3. 计算相对速度 (目标船相对于自身船的速度)
    double rel_vx = tar_vx - own_vx;
    double rel_vy = tar_vy - own_vy;

    // 计算相对速度的平方和
    double v_rel_sq = rel_vx * rel_vx + rel_vy * rel_vy;

    double tcpa = 0.0;
    double dcpa = params->distance; // 默认 DCPA 为当前距离

    // 4. 计算 TCPA 和 DCPA
    // 如果 v_rel_sq 接近于 0，说明两船相对静止，TCPA 设为 0，DCPA 就是当前距离
    if (v_rel_sq > 0.000001) {
        // TCPA 公式 (单位：秒)
        tcpa = -(tar_px * rel_vx + tar_py * rel_vy) / v_rel_sq;
        
        // 预测在 TCPA 时刻，目标船相对于自身船的位置
        double cpa_x = tar_px + rel_vx * tcpa;
        double cpa_y = tar_py + rel_vy * tcpa;
        
        // DCPA 公式 (单位：米)
        dcpa = sqrt(cpa_x * cpa_x + cpa_y * cpa_y);
    }

    // 将结果写入输出参数
    *outDcpa = dcpa;
    *outTcpa = tcpa;

    // 5. 危险等级判定
    // 如果 TCPA < 0，说明两船已经相互错过，正在逐渐远离，判为无风险
    if (tcpa < 0) {
        return RISK_LEVEL_NONE;
    }

    // 根据设定的阈值进行逻辑判断
    // 满足高风险阈值 (距离近 且 时间短)
    if (dcpa <= DCPA_HIGH_RISK && tcpa <= TCPA_HIGH_RISK) {
        return RISK_LEVEL_HIGH;
    } 
    // 满足低风险阈值
    else if (dcpa <= DCPA_LOW_RISK && tcpa <= TCPA_LOW_RISK) {
        return RISK_LEVEL_LOW;
    }

    // 其它情况判为无风险
    return RISK_LEVEL_NONE;
}