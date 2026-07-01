clear; clc; close all;

%% 1. 参数设置
pop_size = 100;      % 初始样本（种群）大小
max_gen = 100;       % 最大迭代代数
pc = 0.8;            % 交叉概率 (Crossover Probability)
pm = 0.1;            % 变异概率 (Mutation Probability)
num_vars = 2;        % 变量维度 (x, y)
lb = [-3, -3];       % 变量取值下界
ub = [3, 3];         % 变量取值上界

%% 2. 初始化种群 (实数编码)
% 随机生成 100 个在 [-3, 3] 范围内的初始样本点
pop = zeros(pop_size, num_vars);
for i = 1:num_vars
    pop(:, i) = lb(i) + (ub(i) - lb(i)) * rand(pop_size, 1);
end

%% 3. 数据记录准备
best_fitness_history = zeros(max_gen, 1); % 记录历代最优适应度
mean_fitness_history = zeros(max_gen, 1); % 记录历代平均适应度
global_best_x = zeros(1, num_vars);       % 全局最优解坐标
global_best_fitness = -inf;               % 全局最优适应度值

%% 4. 遗传算法主循环
for gen = 1:max_gen
    % --- 步骤 A: 计算适应度 ---
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size
        fitness(i) = myFitnessFun(pop(i,:));
    end
    
    % 记录当前代的最优值
    [current_best_fit, best_idx] = max(fitness);
    if current_best_fit > global_best_fitness
        global_best_fitness = current_best_fit;
        global_best_x = pop(best_idx, :);
    end
    
    best_fitness_history(gen) = global_best_fitness;
    mean_fitness_history(gen) = mean(fitness);
    
    % --- 步骤 B: 轮盘赌选择 (Selection) ---
    % 将适应度平移至非负区间，防止轮盘赌出现负概率
    min_fit = min(fitness);
    if min_fit < 0
        fit_shift = fitness - min_fit + 1e-5;
    else
        fit_shift = fitness + 1e-5;
    end
    prob = fit_shift / sum(fit_shift);
    cum_prob = cumsum(prob);
    
    new_pop = zeros(pop_size, num_vars);
    for i = 1:pop_size
        r = rand();
        idx = find(cum_prob >= r, 1, 'first');
        new_pop(i, :) = pop(idx, :);
    end
    pop = new_pop; % 更新种群
    
    % --- 步骤 C: 算术交叉 (Crossover) ---
    for i = 1:2:pop_size-1
        if rand() < pc
            alpha = rand();
            temp1 = alpha * pop(i,:) + (1-alpha) * pop(i+1,:);
            temp2 = (1-alpha) * pop(i,:) + alpha * pop(i+1,:);
            pop(i,:) = temp1;
            pop(i+1,:) = temp2;
        end
    end
    
    % --- 步骤 D: 高斯变异 (Mutation) ---
    for i = 1:pop_size
        if rand() < pm
            mut_point = randi([1, num_vars]); % 随机选择变异维度
            % 添加随迭代次数衰减的高斯扰动（实现前期大范围探索，后期精细收敛）
            delta = randn() * (ub(mut_point) - lb(mut_point)) * (1 - gen/max_gen) * 0.1;
            pop(i, mut_point) = pop(i, mut_point) + delta;
            
            % 边界检查，防止越界
            pop(i, mut_point) = max(lb(mut_point), min(ub(mut_point), pop(i, mut_point)));
        end
    end
end

%% 5. 结果输出
fprintf('===== 遗传算法求解完成 =====\n');
fprintf('寻优获得的最优解: x = %.4f, y = %.4f\n', global_best_x(1), global_best_x(2));
fprintf('目标函数的最大值: %.4f\n', global_best_fitness);

%% 6. 绘图展示
figure('Name', '遗传算法优化结果', 'Position', [100, 100, 900, 400]);

% 图1：收敛情况曲线
subplot(1,2,1);
plot(1:max_gen, best_fitness_history, 'r-', 'LineWidth', 2); hold on;
plot(1:max_gen, mean_fitness_history, 'b--', 'LineWidth', 1.5);
xlabel('迭代次数 (Generations)');
ylabel('适应度值 (Fitness)');
title('遗传算法收敛曲线');
legend('历代最优适应度', '历代平均适应度', 'Location', 'southeast');
grid on;

% 图2：目标函数3D曲面与最优解位置
subplot(1,2,2);
[X, Y] = meshgrid(linspace(lb(1), ub(1), 100), linspace(lb(2), ub(2), 100));
Z = zeros(size(X));
for i = 1:size(X,1)
    for j = 1:size(X,2)
        Z(i,j) = myFitnessFun([X(i,j), Y(i,j)]);
    end
end
% 绘制3D彩色曲面
surf(X, Y, Z, 'EdgeColor', 'none'); hold on;
colormap(parula);
alpha(0.85);

% 在曲面上标出找到的全局最优解
best_z = myFitnessFun(global_best_x);
plot3(global_best_x(1), global_best_x(2), best_z, 'r*', 'MarkerSize', 15, 'LineWidth', 2);

xlabel('变量 x'); 
ylabel('变量 y'); 
zlabel('适应度值 F(x,y)');
title('目标函数三维曲面及最优解');
legend('自拟函数曲面', 'GA找到的最优解', 'Location', 'northeast');
view(-35, 45); % 调整观看视角

%% 7. 自拟适应性函数 (目标函数)
function f = myFitnessFun(x)
    % 这里自拟了一个典型的多峰函数（类似Peaks函数）
    % 具有多个局部极值点，能够很好地测试遗传算法跳出局部最优的能力
    x1 = x(1);
    x2 = x(2);
    
    term1 = 3 * (1 - x1)^2 * exp(-(x1^2) - (x2 + 1)^2);
    term2 = 10 * (x1/5 - x1^3 - x2^5) * exp(-x1^2 - x2^2);
    term3 = 1/3 * exp(-(x1 + 1)^2 - x2^2);
    
    f = term1 - term2 - term3; % 目标是求该函数的最大值
end