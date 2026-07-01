% 多目标遗传算法 (Multi-Objective GA) - 基于帕累托支配等级
clear; clc; close all;

%% 1. 参数设置
pop_size = 100;      % 种群大小
max_gen = 100;       % 最大迭代代数
pc = 0.8;            % 交叉概率
pm = 0.1;            % 变异概率
num_vars = 2;        % 变量维度 (x, y)
lb = [-3, -3];       % 变量取值下界
ub = [3, 3];         % 变量取值上界

%% 2. 初始化种群 (实数编码)
pop = zeros(pop_size, num_vars);
for i = 1:num_vars
    pop(:, i) = lb(i) + (ub(i) - lb(i)) * rand(pop_size, 1);
end

%% 3. 数据记录准备 (帕累托档案)
archive_pop = []; % 记录帕累托最优解的坐标
archive_F = [];   % 记录帕累托最优解的目标函数值

%% 4. 遗传算法主循环
for gen = 1:max_gen
    % --- 步骤 A: 计算多目标适应度 ---
    F = zeros(pop_size, 2); % 存储两个目标函数的值
    for i = 1:pop_size
        F(i, :) = myMultiObjFun(pop(i,:));
    end
    
    % --- 步骤 B: 帕累托支配检查与适应度分配 ---
    dom_count = zeros(pop_size, 1); % 记录每个个体被支配的次数
    for i = 1:pop_size
        for j = 1:pop_size
            if i ~= j
                % 判断个体 j 是否支配个体 i (求最大值)
                % 条件: j 的所有目标 >= i 的所有目标，且至少有一个目标严格 > i
                if (F(j,1) >= F(i,1) && F(j,2) >= F(i,2)) && (F(j,1) > F(i,1) || F(j,2) > F(i,2))
                    dom_count(i) = dom_count(i) + 1;
                end
            end
        end
    end
    
    % 基于被支配次数计算标量适应度 (非支配解 fitness = 1)
    fitness = 1 ./ (1 + dom_count);
    
    % --- 步骤 C: 更新全局帕累托档案 ---
    % 将当前代的非支配解加入候选档案
    non_dominated_idx = find(dom_count == 0);
    temp_archive_pop = [archive_pop; pop(non_dominated_idx, :)];
    temp_archive_F = [archive_F; F(non_dominated_idx, :)];
    
    % 对合并后的候选档案再次进行非支配筛选，剔除劣解
    n_archive = size(temp_archive_F, 1);
    archive_dom_count = zeros(n_archive, 1);
    for i = 1:n_archive
        for j = 1:n_archive
            if i ~= j
                if (temp_archive_F(j,1) >= temp_archive_F(i,1) && temp_archive_F(j,2) >= temp_archive_F(i,2)) && ...
                   (temp_archive_F(j,1) > temp_archive_F(i,1) || temp_archive_F(j,2) > temp_archive_F(i,2))
                    archive_dom_count(i) = archive_dom_count(i) + 1;
                    break; % 只要被一个支配就判定为劣解
                end
            end
        end
    end
    
    % 保留纯粹的全局帕累托前沿
    keep_idx = (archive_dom_count == 0);
    
    % 为了防止档案过大影响计算速度，使用 unique 去重
    [~, unique_idx] = unique(round(temp_archive_F(keep_idx, :), 4), 'rows');
    valid_keep = find(keep_idx);
    final_keep = valid_keep(unique_idx);
    
    archive_pop = temp_archive_pop(final_keep, :);
    archive_F = temp_archive_F(final_keep, :);
    
    % --- 步骤 D: 轮盘赌选择 ---
    prob = fitness / sum(fitness);
    cum_prob = cumsum(prob);
    new_pop = zeros(pop_size, num_vars);
    for i = 1:pop_size
        r = rand();
        idx = find(cum_prob >= r, 1, 'first');
        new_pop(i, :) = pop(idx, :);
    end
    pop = new_pop;
    
    % --- 步骤 E: 算术交叉 ---
    for i = 1:2:pop_size-1
        if rand() < pc
            alpha = rand();
            temp1 = alpha * pop(i,:) + (1-alpha) * pop(i+1,:);
            temp2 = (1-alpha) * pop(i,:) + alpha * pop(i+1,:);
            pop(i,:) = temp1;
            pop(i+1,:) = temp2;
        end
    end
    
    % --- 步骤 F: 高斯变异 ---
    for i = 1:pop_size
        if rand() < pm
            mut_point = randi([1, num_vars]);
            delta = randn() * (ub(mut_point) - lb(mut_point)) * (1 - gen/max_gen) * 0.1;
            pop(i, mut_point) = pop(i, mut_point) + delta;
            pop(i, mut_point) = max(lb(mut_point), min(ub(mut_point), pop(i, mut_point)));
        end
    end
end

%% 5. 结果展示与绘图
fprintf('===== 多目标优化完成 =====\n');
fprintf('共找到 %d 个帕累托最优解 (Pareto Front)\n', size(archive_F, 1));

% 按照目标 1 的数值大小对档案排序，方便画出平滑的前沿曲线
[~, sort_idx] = sort(archive_F(:, 1));
archive_F = archive_F(sort_idx, :);
archive_pop = archive_pop(sort_idx, :);

figure('Name', '多目标帕累托遗传算法', 'Position', [100, 100, 1000, 450]);

% 图1：目标空间 (Objective Space) 中的帕累托前沿
subplot(1, 2, 1);
% 画出最后一代的所有解作为背景对比
plot(F(:, 1), F(:, 2), 'ko', 'MarkerFaceColor', [0.8 0.8 0.8], 'MarkerEdgeColor', 'none', 'DisplayName', '最后一代种群');
hold on;
% 画出全局帕累托前沿
plot(archive_F(:, 1), archive_F(:, 2), 'r.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', '帕累托前沿 (Pareto Front)');
xlabel('目标函数 1: f_1(x,y)');
ylabel('目标函数 2: f_2(x,y)');
title('目标空间 (Objective Space) 映射');
legend('Location', 'best');
grid on;

% 图2：决策空间 (Decision Space) 中的帕累托最优集
subplot(1, 2, 2);
plot(archive_pop(:, 1), archive_pop(:, 2), 'b.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', '帕累托最优解集 (Pareto Set)');
xlabel('变量 x');
ylabel('变量 y');
title('决策空间 (Decision Space) 分布');
xlim(lb(1:2)); ylim(lb(1:2));
legend('Location', 'best');
grid on;

%% 6. 自拟多目标函数
function F = myMultiObjFun(x)
    % 目标是同时“最大化”这两个函数
    % f1 的最优解在 (0, 0)，最大值为 0
    % f2 的最优解在 (2, 2)，最大值为 0
    % 这两个目标存在冲突：靠近 (0,0) 则 f1 大而 f2 小；靠近 (2,2) 则 f2 大而 f1 小。
    
    x1 = x(1);
    x2 = x(2);
    
    f1 = -(x1^2 + x2^2); 
    f2 = -((x1 - 2)^2 + (x2 - 2)^2);
    
    F = [f1, f2]; 
end