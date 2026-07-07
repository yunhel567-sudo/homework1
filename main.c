#include <stdio.h>
#include "ais_sync.h"
#include "collision_risk.h"
#include "colregs_decision.h"
#include "path_decision.h"

// 全局状态变量：是否在避碰进程 (0-否, 1-是)
int is_avoiding = 0;

/**
 * @brief 执行单次导航周期
 * @param ownShip    自身船数据
 * @param targetShip 目标船数据
 * @param D_safe     安全距离阈值 (米)
 */
void run_navigation_cycle(const char* scenario_name, const AisData* ownShip, const AisData* targetShip, double D_safe) {
    printf("========== Scenario: %s ==========\n", scenario_name);

    // 1. 判断是否已经在避碰进程中
    if (is_avoiding == 1) {
        printf("Status: In collision avoidance process. Exiting current cycle.\n\n");
        return; // 退出
    }

    // 2. 时间与空间坐标对齐
    RelativeMotionParams params;
    processAndAlignData(ownShip, targetShip, &params);

    // 3. 危险程度评估
    double dcpa = 0.0, tcpa = 0.0;
    int riskLevel = assessCollisionRisk(&params, &dcpa, &tcpa);

    printf("Assessment: DCPA = %.2f m, TCPA = %.2f s\n", dcpa, tcpa);

    // 4. 无风险处理
    if (riskLevel == RISK_LEVEL_NONE) {
        printf("Status: No collision risk. Safe to proceed.\n\n");
        return; // 无风险直接退出
    }

    // 5. 有风险处理：修改避碰状态标识
    is_avoiding = 1;
    printf("Status: Risk detected (Level %d)! Setting 'is_avoiding' to 1.\n", riskLevel);

    // 6. COLREGs 避碰规则决策 (确定行动方向)
    int actionDir = decideEvasiveAction(&params, riskLevel);
    
    const char* actionStr = "Straight";
    if (actionDir == ACTION_TURN_PORT) actionStr = "Turn Port (Left)";
    else if (actionDir == ACTION_TURN_STARBOARD) actionStr = "Turn Starboard (Right)";
    printf("COLREGs Decision: %s\n", actionStr);

    // 7. 具体路径/角度规划 (GA 或 VO)
    AvoidancePath path = planAvoidancePath(&params, riskLevel, actionDir, D_safe);

    if (path.methodUsed == 1) {
        printf("Path Planning: GA Algorithm (Low Risk)\n");
        printf(" -> Delay Time      : %.2f s\n", path.ga_t_delay);
        printf(" -> Turn Angle      : %.2f deg\n", path.ga_turn_angle);
        printf(" -> Avoid Duration  : %.2f s\n", path.ga_t_avoid);
        printf(" -> Return Angle    : %.2f deg\n", path.ga_return_angle);
    } else if (path.methodUsed == 2) {
        printf("Path Planning: VO Algorithm (High Risk / Emergency)\n");
        printf(" -> Adjusted Heading: %.2f deg\n", path.vo_turn_angle);
    } else {
        printf("Path Planning: No action required.\n");
    }
    
    printf("\n");
}

int main() {
    // 统一定义安全距离 (2海里)
    double D_safe = 3704.0; 
    long long current_time = 1000; // 统一基准时间

    // =======================================================
    // 场景 1: 无风险 (No Risk)
    // 自身船向正北航行，目标船在东边，且向正东航行 (两船逐渐远离)
    // =======================================================
    AisData own1 = {1, 0, 15.0, 110.0, 20.0, 0.0, 0, 0, 0, 0, current_time};
    AisData tar1 = {2, 0, 15.0, 110.1, 20.0, 90.0, 90, 0, 0, 0, current_time};
    is_avoiding = 0; // 重置状态
    run_navigation_cycle("No Risk", &own1, &tar1, D_safe);


    // =======================================================
    // 场景 2: 小风险 (Low Risk) -> 触发 GA 算法
    // 对遇局面：目标船在正北约 9 海里处向南行驶。
    // TCPA 约 18 分钟 (处于 15~30分钟之间)，DCPA = 0，触发小风险。
    // =======================================================
    AisData own2 = {1, 0, 15.0, 110.0, 20.0, 0.0, 0, 0, 0, 0, current_time};
    AisData tar2 = {3, 0, 15.0, 110.0, 20.15, 180.0, 180, 0, 0, 0, current_time};
    is_avoiding = 0; // 重置状态
    run_navigation_cycle("Low Risk (GA)", &own2, &tar2, D_safe);


    // =======================================================
    // 场景 3: 验证 is_avoiding 的拦截机制
    // 在场景 2 执行完后，is_avoiding 已经被置为 1，此时再次调用应直接退出
    // =======================================================
    run_navigation_cycle("Check 'is_avoiding' Status", &own2, &tar2, D_safe);


    // =======================================================
    // 场景 4: 大风险 (High Risk) -> 触发 VO 算法
    // 对遇局面：目标船距离极近，在正北约 3 海里处向南行驶。
    // TCPA 约 6 分钟 (小于 15 分钟的紧急阈值)，触发大风险。
    // =======================================================
    AisData own3 = {1, 0, 15.0, 110.0, 20.0, 0.0, 0, 0, 0, 0, current_time};
    AisData tar3 = {4, 0, 15.0, 110.0, 20.05, 180.0, 180, 0, 0, 0, current_time};
    is_avoiding = 0; // 重置状态 (模拟避让结束后的新航次)
    run_navigation_cycle("High Risk (VO)", &own3, &tar3, D_safe);

    return 0;
}