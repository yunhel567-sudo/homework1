#include "path_decision.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

#define PI 3.14159265358979323846
#define DEG2RAD(x) ((x) * PI / 180.0)

// === GA 算法超参数 ===
#define GA_POP_SIZE 100       // 种群大小 (从 30 增大到 100，增加基因多样性)
#define GA_GENERATIONS 150    // 进化代数 (从 40 增大到 150，保证充分收敛)
#define GA_CROSS_RATE 0.7     // 交叉概率
#define GA_MUT_RATE 0.1       // 突变概率
#define SIM_STEP 10.0         // 模拟步长 (秒)

typedef struct {
    float t_delay;      // 直航时间 (0~600秒)
    float turn_angle;   // 转向角 (0~180度)
    float t_avoid;      // 避让持续时间 (0~3600秒)
    float return_angle; // 复航转向角 (0~180度)
    float fitness;      // 适应度
} Chromosome;

// === 辅助函数：生成范围内的随机浮点数 ===
float randFloat(float min, float max) {
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

// === GA：适应度评估 (推演航线) ===
void evaluateFitness(Chromosome* ind, const RelativeMotionParams* params, int actionDir, double D_safe) {
    double own_x = 0.0, own_y = 0.0;
    // 目标船的初始直角坐标
    double tar_x = params->distance * sin(DEG2RAD(params->trueBearing));
    double tar_y = params->distance * cos(DEG2RAD(params->trueBearing));
    
    // 目标船速度分量 (恒定)
    double tar_vx = params->targetSpeed_ms * sin(DEG2RAD(params->targetHeading));
    double tar_vy = params->targetSpeed_ms * cos(DEG2RAD(params->targetHeading));
    
    double min_dist = 999999.0;
    float current_time = 0.0;
    float total_sim_time = ind->t_delay + ind->t_avoid + 600.0; // 模拟到复航后一段时间
    
    // 确定转向符号: 左转(-1), 右转(+1)
    float dir_sign = (actionDir == ACTION_TURN_PORT) ? -1.0 : 1.0;
    
    // 步进模拟推演
    while (current_time < total_sim_time) {
        float current_heading = params->ownHeading;
        
        // 判定当前所处阶段并调整航向
        if (current_time >= ind->t_delay && current_time < (ind->t_delay + ind->t_avoid)) {
            current_heading += dir_sign * ind->turn_angle; // 第一阶段转向
        } else if (current_time >= (ind->t_delay + ind->t_avoid)) {
            current_heading += dir_sign * ind->turn_angle - dir_sign * ind->return_angle; // 尝试复航
        }
        
        // 更新本船位置
        own_x += params->ownSpeed_ms * sin(DEG2RAD(current_heading)) * SIM_STEP;
        own_y += params->ownSpeed_ms * cos(DEG2RAD(current_heading)) * SIM_STEP;
        
        // 更新目标船位置
        tar_x += tar_vx * SIM_STEP;
        tar_y += tar_vy * SIM_STEP;
        
        // 计算当前距离
        double dx = tar_x - own_x;
        double dy = tar_y - own_y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < min_dist) min_dist = dist;
        
        current_time += SIM_STEP;
    }
    
    // 计算适应度 (安全性 + 经济性)
    if (min_dist < D_safe) {
        // 如果侵入安全圆，施加严重惩罚
        ind->fitness = min_dist - 10000.0; 
    } else {
        // 安全情况下，追求经济性：打舵越小、避让时间越短越好
        ind->fitness = 1000.0 - (0.1 * ind->t_avoid) - (2.0 * ind->turn_angle);
    }
}

// === 大风险：VO 速度障碍法 ===
float calculateVOAngle(const RelativeMotionParams* params, int actionDir, double D_safe) {
    // 目标船相对初始坐标
    double px = params->distance * sin(DEG2RAD(params->trueBearing));
    double py = params->distance * cos(DEG2RAD(params->trueBearing));
    
    float dir_sign = (actionDir == ACTION_TURN_PORT) ? -1.0 : 1.0;
    
    // 以 1 度为步长，搜索最小避让角 (0 到 180度)
    for (int angle = 0; angle <= 180; angle++) {
        float test_heading = params->ownHeading + dir_sign * angle;
        
        // 测试航向下的本船速度分量
        double test_vx = params->ownSpeed_ms * sin(DEG2RAD(test_heading));
        double test_vy = params->ownSpeed_ms * cos(DEG2RAD(test_heading));
        
        // 测试航向下的相对速度分量 Vr
        double vrx = test_vx - (params->targetSpeed_ms * sin(DEG2RAD(params->targetHeading)));
        double vry = test_vy - (params->targetSpeed_ms * cos(DEG2RAD(params->targetHeading)));
        
        // 相对速度的平方
        double vr_sq = vrx * vrx + vry * vry;
        if (vr_sq < 0.001) continue;
        
        // 计算到达 CPA 的时间 (基于本船不动，目标船以 -Vr 运动的模型)
        // 相当于 target_pos 在相对速度场中的投影
        double tcpa = (px * vrx + py * vry) / vr_sq;
        
        // 如果 tcpa < 0，说明两船正在远离，VO 锥解除！
        if (tcpa < 0) return (float)angle;
        
        // 计算 TCPA 时刻的最小距离平方
        double d_cpa_sq = (px*px + py*py) - (tcpa * tcpa * vr_sq);
        
        // 如果最小距离大于安全距离，说明新速度矢量已经移出 VO 锥！
        if (d_cpa_sq > (D_safe * D_safe)) {
            return (float)angle;
        }
    }
    return 180.0; // 如果找不到，被迫做最大幅度转向
}

// === 主接口函数 ===
AvoidancePath planAvoidancePath(const RelativeMotionParams* params, int riskLevel, int actionDir, double D_safe) {
    AvoidancePath path = {0};
    path.actionDirection = actionDir;
    
    // 初始化随机数种子用于 GA
    static int rand_seeded = 0;
    if (!rand_seeded) { srand((unsigned int)time(NULL)); rand_seeded = 1; }
    
    if (riskLevel == RISK_LEVEL_NONE || actionDir == ACTION_STRAIGHT) {
        path.methodUsed = 0;
        return path;
    }
    
    if (riskLevel == RISK_LEVEL_HIGH) {
        // --- 采用 VO 算法 ---
        path.methodUsed = 2;
        path.vo_turn_angle = calculateVOAngle(params, actionDir, D_safe);
        return path;
    }
    
    if (riskLevel == RISK_LEVEL_LOW) {
        // --- 采用 GA 算法 ---
        path.methodUsed = 1;
        Chromosome pop[GA_POP_SIZE];
        Chromosome new_pop[GA_POP_SIZE];
        Chromosome best_ind = {0, 0, 0, 0, -999999.0};
        
        // 1. 初始化种群
        for (int i = 0; i < GA_POP_SIZE; i++) {
            pop[i].t_delay = randFloat(0.0, 600.0);
            pop[i].turn_angle = randFloat(10.0, 90.0); // 初始尽量在合理区间找
            pop[i].t_avoid = randFloat(300.0, 1800.0);
            pop[i].return_angle = randFloat(10.0, 90.0);
            evaluateFitness(&pop[i], params, actionDir, D_safe);
            if (pop[i].fitness > best_ind.fitness) best_ind = pop[i];
        }
        
        // 2. 进化迭代
        for (int gen = 0; gen < GA_GENERATIONS; gen++) {
            for (int i = 0; i < GA_POP_SIZE; i++) {
                // 锦标赛选择 (选2个取最优)
                int idx1 = rand() % GA_POP_SIZE;
                int idx2 = rand() % GA_POP_SIZE;
                Chromosome p1 = (pop[idx1].fitness > pop[idx2].fitness) ? pop[idx1] : pop[idx2];
                
                int idx3 = rand() % GA_POP_SIZE;
                int idx4 = rand() % GA_POP_SIZE;
                Chromosome p2 = (pop[idx3].fitness > pop[idx4].fitness) ? pop[idx3] : pop[idx4];
                
                // 交叉 (Crossover)
                Chromosome child = p1;
                if (randFloat(0, 1) < GA_CROSS_RATE) {
                    child.t_avoid = p2.t_avoid;
                    child.return_angle = p2.return_angle;
                }
                
                // 突变 (Mutation)
                if (randFloat(0, 1) < GA_MUT_RATE) {
                    child.turn_angle += randFloat(-10.0, 10.0);
                    if (child.turn_angle < 0) child.turn_angle = 0;
                    if (child.turn_angle > 180) child.turn_angle = 180;
                }
                
                // 评估新个体
                evaluateFitness(&child, params, actionDir, D_safe);
                new_pop[i] = child;
                
                // 记录全局最优
                if (child.fitness > best_ind.fitness) {
                    best_ind = child;
                }
            }
            // 更新种群
            for (int i = 0; i < GA_POP_SIZE; i++) pop[i] = new_pop[i];
        }
        
        // 输出最优基因
        path.ga_t_delay = best_ind.t_delay;
        path.ga_turn_angle = best_ind.turn_angle;
        path.ga_t_avoid = best_ind.t_avoid;
        path.ga_return_angle = best_ind.return_angle;
        return path;
    }
    
    return path;
}