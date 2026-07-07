% avoidance_visualization.m
% 基于 C 语言程序输出结果的船舶局部避碰全过程可视化
% 数据来源：C语言避碰系统运行日志

clc; clear; close all;

% =========================================================
% 全局公共参数配置
% =========================================================
v_knots = 15.0;                         % 航速(节)
v_ms = v_knots * 0.5144444;             % 航速(米/秒) = 7.7167 m/s
D_safe = 3704.0;                        % 安全距离: 2海里 = 3704米

% 创建画布
figure('Name', '船舶局部避碰决策规划轨迹可视化', 'Position', [100, 100, 1200, 600]);

% =========================================================
% 场景一：小风险 (GA算法规划)
% =========================================================
% 从 C 程序输出提取数据：
% TCPA = 1081.94 s, 相对速度为 2 * v_ms
% 初始距离 = TCPA * 相对速度
init_dist_ga = 1081.94 * (2 * v_ms); 
ga_t_delay = 13.04;                     % 直航延迟时间 (s)
ga_turn_angle = 34.39;                  % 避让转向角 (度)
ga_t_avoid = 428.18;                    % 避让持续时间 (s)
ga_return_angle = 14.46;                % 复航转向角 (度)

t_total_ga = 2200;                      % 【修改】增加总模拟时间至2200秒，保证能完整画出复航后轨迹
dt = 1;                                 % 步长 (s)

% 预分配内存
x_own1 = zeros(1, t_total_ga); y_own1 = zeros(1, t_total_ga);
x_tar1 = zeros(1, t_total_ga); y_tar1 = zeros(1, t_total_ga);
tar1_start_y = init_dist_ga; 

% 初始位置
x_own1(1) = 0; y_own1(1) = 0;
x_tar1(1) = 0; y_tar1(1) = tar1_start_y;

min_dist_ga = inf;
min_idx_ga = 1;

for i = 2:t_total_ga
    t = i * dt;
    
    % 本船航向控制
    if t < ga_t_delay
        own_heading = 0; % 阶段1：直航延迟
    elseif t < (ga_t_delay + ga_t_avoid)
        own_heading = ga_turn_angle; % 阶段2：向右转向避让
    else
        % 【修改】阶段3：复航恢复。向左打舵（负角度）驶回原航线
        if x_own1(i-1) > 0 
            own_heading = -ga_return_angle; % 仍在原航线右侧，继续向左回归
        else
            own_heading = 0; % 阶段4：已回到原航线(X=0)，摆正航向继续直行
            x_own1(i-1) = 0; % 修正微小误差
        end
    end
    
    % 目标船航向始终为 180 度 (正南)
    tar_heading = 180;
    
    % 更新坐标
    x_own1(i) = x_own1(i-1) + v_ms * sind(own_heading) * dt;
    y_own1(i) = y_own1(i-1) + v_ms * cosd(own_heading) * dt;
    
    x_tar1(i) = x_tar1(i-1) + v_ms * sind(tar_heading) * dt;
    y_tar1(i) = y_tar1(i-1) + v_ms * cosd(tar_heading) * dt;
    
    % 记录最小距离
    current_dist = sqrt((x_own1(i) - x_tar1(i))^2 + (y_own1(i) - y_tar1(i))^2);
    if current_dist < min_dist_ga
        min_dist_ga = current_dist;
        min_idx_ga = i;
    end
end

% 绘制 GA 场景
subplot(1, 2, 1);
hold on; grid on; axis equal;

% 绘制原计划航线 (参考线)
h_ref1 = plot([0 0], [0 max(y_own1)], 'b:', 'LineWidth', 1.2);

% 绘制实际航线
h_own1 = plot(x_own1, y_own1, 'b-', 'LineWidth', 2);
h_tar1 = plot(x_tar1, y_tar1, 'r-', 'LineWidth', 2);

% 绘制初始点
plot(x_own1(1), y_own1(1), 'b^', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
plot(x_tar1(1), y_tar1(1), 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 绘制最小距离点连线及安全圆
h_cpa1 = plot([x_own1(min_idx_ga), x_tar1(min_idx_ga)], [y_own1(min_idx_ga), y_tar1(min_idx_ga)], 'k--', 'LineWidth', 1.5);
theta = linspace(0, 2*pi, 100);
h_safe1 = plot(x_own1(min_idx_ga) + D_safe*cos(theta), y_own1(min_idx_ga) + D_safe*sin(theta), 'g--', 'LineWidth', 1.5);

title('场景：小风险 (GA 遗传算法规划)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X (米)'); ylabel('Y (米)');
legend([h_own1, h_tar1, h_ref1, h_cpa1, h_safe1], ...
    {'本船实际轨迹', '目标船实际轨迹', '本船原计划航线', 'CPA最小距离', '安全保护圈'}, 'Location', 'best');

% 动态放置文本框
xl = xlim; yl = ylim;
text(xl(1) + (xl(2)-xl(1))*0.05, yl(2) - (yl(2)-yl(1))*0.05, ...
    sprintf('实际最小距离: %.1f m\n要求安全距离: %.1f m\nGA转向角: %.1f°', min_dist_ga, D_safe, ga_turn_angle), ...
    'BackgroundColor', 'w', 'EdgeColor', 'k', 'VerticalAlignment', 'top');


% =========================================================
% 场景二：大风险 (VO算法避让)
% =========================================================
% 从 C 程序输出提取数据：
% TCPA = 360.65 s
init_dist_vo = 360.65 * (2 * v_ms); 
vo_turn_angle = 84.00;                  % VO 计算出的极限转向角 (度)

t_total_vo = 1200;                      % 【修改】增加紧急避让总模拟时间至1200秒，观察避让后延伸的距离

% 预分配内存
x_own2 = zeros(1, t_total_vo); y_own2 = zeros(1, t_total_vo);
x_tar2 = zeros(1, t_total_vo); y_tar2 = zeros(1, t_total_vo);
tar2_start_y = init_dist_vo;

% 初始位置
x_own2(1) = 0; y_own2(1) = 0;
x_tar2(1) = 0; y_tar2(1) = tar2_start_y;

min_dist_vo = inf;
min_idx_vo = 1;

for i = 2:t_total_vo
    t = i * dt;
    
    % 本船航向控制 (VO：立即执行大幅度右转，并保持以脱离危险)
    own_heading = vo_turn_angle; 
    
    % 目标船航向始终为 180 度 (正南)
    tar_heading = 180;
    
    % 更新坐标
    x_own2(i) = x_own2(i-1) + v_ms * sind(own_heading) * dt;
    y_own2(i) = y_own2(i-1) + v_ms * cosd(own_heading) * dt;
    
    x_tar2(i) = x_tar2(i-1) + v_ms * sind(tar_heading) * dt;
    y_tar2(i) = y_tar2(i-1) + v_ms * cosd(tar_heading) * dt;
    
    % 记录最小距离
    current_dist = sqrt((x_own2(i) - x_tar2(i))^2 + (y_own2(i) - y_tar2(i))^2);
    if current_dist < min_dist_vo
        min_dist_vo = current_dist;
        min_idx_vo = i;
    end
end

% 绘制 VO 场景
subplot(1, 2, 2);
hold on; grid on; axis equal;

% 绘制原计划航线 (参考线)
h_ref2 = plot([0 0], [0 max(y_own2)], 'b:', 'LineWidth', 1.2);

% 绘制实际航线
h_own2 = plot(x_own2, y_own2, 'b-', 'LineWidth', 2);
h_tar2 = plot(x_tar2, y_tar2, 'r-', 'LineWidth', 2);

% 绘制初始点
plot(x_own2(1), y_own2(1), 'b^', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
plot(x_tar2(1), y_tar2(1), 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 绘制最小距离点连线及安全圆
h_cpa2 = plot([x_own2(min_idx_vo), x_tar2(min_idx_vo)], [y_own2(min_idx_vo), y_tar2(min_idx_vo)], 'k--', 'LineWidth', 1.5);
h_safe2 = plot(x_own2(min_idx_vo) + D_safe*cos(theta), y_own2(min_idx_vo) + D_safe*sin(theta), 'g--', 'LineWidth', 1.5);

title('场景：大风险紧急避让 (VO 速度障碍法)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X (米)'); ylabel('Y (米)');
legend([h_own2, h_tar2, h_ref2, h_cpa2, h_safe2], ...
    {'本船实际轨迹', '目标船实际轨迹', '本船原计划航线', 'CPA最小距离', '安全保护圈'}, 'Location', 'best');

% 动态放置文本框
xl = xlim; yl = ylim;
text(xl(1) + (xl(2)-xl(1))*0.05, yl(2) - (yl(2)-yl(1))*0.05, ...
    sprintf('实际最小距离: %.1f m\n要求安全距离: %.1f m\nVO紧急转向角: %.1f°', min_dist_vo, D_safe, vo_turn_angle), ...
    'BackgroundColor', 'w', 'EdgeColor', 'k', 'VerticalAlignment', 'top');

% 调整布局
sgtitle('C 语言避碰系统决策推演验证 (延长模拟时间)', 'FontSize', 16, 'FontWeight', 'bold');