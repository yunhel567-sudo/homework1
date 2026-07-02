% verify_cpa.m - 船舶碰撞危险判断验证与轨迹可视化

% === 1. 设置安全阈值 (参考大洋航行标准) ===
D_safe = 2.0; % 安全距离阈值：2.0海里
T_safe = 1.0; % 安全时间阈值：1.0小时 (60分钟)

% 创建绘图窗口
figure('Name', 'AIS数据 CPA 碰撞危险评估与轨迹可视化', 'Position', [100, 100, 1200, 550]);

% =========================================================
% === 2. 场景一：存在碰撞危险 (Dangerous) ===
% =========================================================
% 本船：位于原点，向东北(045°)航行，航速15节
ownShip1 = struct('x', 0, 'y', 0, 'v', 15, 'heading', 45);
% 他船：位于正东10海里处，向西北西(300°)航行，航速15节
targetShip1 = struct('x', 10, 'y', 0, 'v', 15, 'heading', 300);

% 调用未修改的函数进行判断
[dcpa1, tcpa1, isDanger1] = calculateCollisionRisk(ownShip1, targetShip1, D_safe, T_safe);

% 绘图：场景一
subplot(1, 2, 1);
plotScenario(ownShip1, targetShip1, dcpa1, tcpa1, isDanger1, D_safe, '场景一：存在碰撞危险 (交叉相遇)');


% =========================================================
% === 3. 场景二：安全 / 无危险 (Safe) ===
% =========================================================
% 本船：位于原点，向正北(000°)航行，航速15节
ownShip2 = struct('x', 0, 'y', 0, 'v', 15, 'heading', 0);
% 他船：位于左前方(X=-5, Y=10)处，向正东(090°)横越，航速15节
targetShip2 = struct('x', -5, 'y', 10, 'v', 15, 'heading', 90);

% 调用未修改的函数进行判断
[dcpa2, tcpa2, isDanger2] = calculateCollisionRisk(ownShip2, targetShip2, D_safe, T_safe);

% 绘图：场景二
subplot(1, 2, 2);
plotScenario(ownShip2, targetShip2, dcpa2, tcpa2, isDanger2, D_safe, '场景二：安全 (距离宽裕，无紧迫危险)');


% =========================================================
% 辅助绘图函数 (局部函数，支持直接写在脚本末尾)
% =========================================================
function plotScenario(own, tar, dcpa, tcpa, isDanger, D_safe, titleStr)
    hold on; grid on; axis equal;
    
    % 设置模拟演化的总时间 (比TCPA稍微长一点，方便看清会遇后的轨迹)
    t_max = max(tcpa * 1.5, 1.0); 
    
    % 1. 计算航速的 X(东) 和 Y(北) 向量
    vx_own = own.v * sind(own.heading);
    vy_own = own.v * cosd(own.heading);
    vx_tar = tar.v * sind(tar.heading);
    vy_tar = tar.v * cosd(tar.heading);
    
    % 2. 绘制预期航线轨迹 (虚线)
    plot([own.x, own.x + vx_own*t_max], [own.y, own.y + vy_own*t_max], 'b-.', 'LineWidth', 1);
    plot([tar.x, tar.x + vx_tar*t_max], [tar.y, tar.y + vy_tar*t_max], 'r-.', 'LineWidth', 1);
    
    % 3. 绘制 t=0 时刻的初始位置 (三角形代表船首向)
    plot(own.x, own.y, '^b', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot(tar.x, tar.y, '^r', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(own.x, own.y-1, '本船(初始)', 'Color', 'b', 'HorizontalAlignment', 'center');
    text(tar.x, tar.y-1, '他船(初始)', 'Color', 'r', 'HorizontalAlignment', 'center');
    
    % 4. 如果 TCPA 有效，绘制 CPA (最近会遇点) 相关视觉元素
    if tcpa > 0
        % 计算在 TCPA 时刻两船的具体坐标
        cpa_own_x = own.x + vx_own * tcpa;
        cpa_own_y = own.y + vy_own * tcpa;
        cpa_tar_x = tar.x + vx_tar * tcpa;
        cpa_tar_y = tar.y + vy_tar * tcpa;
        
        % 标记 CPA 位置 (圆点)
        plot(cpa_own_x, cpa_own_y, 'ob', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        plot(cpa_tar_x, cpa_tar_y, 'or', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        
        % 用粗黑虚线连接两个 CPA 点，直观展示 DCPA 距离
        plot([cpa_own_x, cpa_tar_x], [cpa_own_y, cpa_tar_y], 'k--', 'LineWidth', 1.5);
        mid_x = (cpa_own_x + cpa_tar_x)/2;
        mid_y = (cpa_own_y + cpa_tar_y)/2;
        text(mid_x, mid_y+0.8, sprintf('DCPA=%.2fnm', dcpa), 'FontWeight', 'bold');
        
        % 以本船 CPA 点为圆心，绘制安全距离 (D_safe) 保护圈
        theta = linspace(0, 2*pi, 100);
        circle_x = cpa_own_x + D_safe * cos(theta);
        circle_y = cpa_own_y + D_safe * sin(theta);
        plot(circle_x, circle_y, 'g-', 'LineWidth', 1.5);
    end
    
    % 5. 完善图表外观
    title(titleStr, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('X (正东方向 / 海里)');
    ylabel('Y (正北方向 / 海里)');
    
    % 6. 提取状态并显示在图表左上角
    if isDanger
        statusColor = 'r';
        statusText = '评估结果: 【危险】 (DCPA侵入安全圈)';
    else
        statusColor = [0, 0.5, 0]; % 深绿色
        statusText = '评估结果: 【安全】 (DCPA宽裕)';
    end
    
    infoStr = {
        sprintf('DCPA: %.2f 海里', dcpa),
        sprintf('TCPA: %.2f 小时 (%.0f 分钟)', tcpa, tcpa*60),
        statusText
    };
    
    % 自动获取坐标轴范围以放置文本框
    xl = xlim; yl = ylim;
    text(xl(1) + (xl(2)-xl(1))*0.05, yl(2) - (yl(2)-yl(1))*0.1, infoStr, ...
        'EdgeColor', 'k', 'BackgroundColor', 'w', 'Color', statusColor, ...
        'FontWeight', 'bold', 'FontSize', 10);
end