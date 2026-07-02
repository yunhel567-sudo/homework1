% 4种经典COLREGs会遇局面航线可视化

% === 1. 场景数据定义 ===
% 场景1: 对遇局面 (Head-on)
scenarios(1).own = struct('x', 0, 'y', 0, 'v', 15, 'heading', 0);
scenarios(1).tar = struct('x', 0, 'y', 10, 'v', 15, 'heading', 180);
scenarios(1).name = '场景1: 对遇局面 (Head-on)';

% 场景2: 交叉相遇 - 目标在右舷 (让路船)
scenarios(2).own = struct('x', 0, 'y', 0, 'v', 15, 'heading', 0);
scenarios(2).tar = struct('x', 10, 'y', 10, 'v', 15, 'heading', 270);
scenarios(2).name = '场景2: 交叉相遇 (本船让路)';

% 场景3: 交叉相遇 - 目标在左舷 (直航船)
scenarios(3).own = struct('x', 0, 'y', 0, 'v', 15, 'heading', 0);
scenarios(3).tar = struct('x', -10, 'y', 10, 'v', 15, 'heading', 90);
scenarios(3).name = '场景3: 交叉相遇 (本船直航)';

% 场景4: 追越局面 (Overtaking) - 本船速度快，追赶前方慢船
scenarios(4).own = struct('x', 0, 'y', 0, 'v', 15, 'heading', 0);
scenarios(4).tar = struct('x', 1, 'y', 5, 'v', 8, 'heading', 0);
scenarios(4).name = '场景4: 本船追越他船';

% === 2. 创建绘图窗口 ===
figure('Name', 'COLREGs 经典会遇局面与避碰决策', 'Position', [100, 100, 1000, 800]);

% === 3. 循环遍历并绘制四个场景 ===
for i = 1:4
    own = scenarios(i).own;
    tar = scenarios(i).tar;
    
    % 为了专门演示避碰规则，这里我们直接假定这些预设场景都已触发危险(isDangerous=true)
    isDangerous = true; 
    
    % 调用局面判断模块获取决策结果
    [situation, action] = decideEvasiveAction(own, tar, isDangerous);
    
    % 激活对应的子图
    subplot(2, 2, i);
    hold on; grid on; axis equal;
    
    % 计算两船在X(东)和Y(北)方向的速度分量
    vx_own = own.v * sind(own.heading);
    vy_own = own.v * cosd(own.heading);
    vx_tar = tar.v * sind(tar.heading);
    vy_tar = tar.v * cosd(tar.heading);
    
    % 设置模拟推演时间为 1.0 小时，绘制预期航迹虚线
    t_sim = 1.0; 
    plot([own.x, own.x + vx_own*t_sim], [own.y, own.y + vy_own*t_sim], 'b--', 'LineWidth', 1.5);
    plot([tar.x, tar.x + vx_tar*t_sim], [tar.y, tar.y + vy_tar*t_sim], 'r--', 'LineWidth', 1.5);
    
    % 绘制初始位置 (用三角形代表船首方向)
    plot(own.x, own.y, '^b', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(tar.x, tar.y, '^r', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % 标注本船与他船
    text(own.x, own.y - 1.5, '本船', 'Color', 'b', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    text(tar.x, tar.y - 1.5, '他船', 'Color', 'r', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    
    % 图表美化与坐标轴设置
    title(scenarios(i).name, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('X (正东 / 海里)');
    ylabel('Y (正北 / 海里)');
    
    % 动态计算合适的坐标轴范围，留出边缘空白
    all_x = [own.x, own.x + vx_own*t_sim, tar.x, tar.x + vx_tar*t_sim];
    all_y = [own.y, own.y + vy_own*t_sim, tar.y, tar.y + vy_tar*t_sim];
    xlim([min(all_x)-3, max(all_x)+3]);
    ylim([min(all_y)-3, max(all_y)+3]);
    
    % 在图表左上角创建一个文本框，显示判定局面和避让动作
    infoText = {
        sprintf('判定局面: %s', situation),
        sprintf('避让动作: 【 %s 】', action)
    };
    
    % 根据动作改变文本框背景色以作区分 (右转/左转用浅黄色，直行用浅绿色)
    if strcmp(action, '直行')
        bgColor = '#E8F5E9'; % 浅绿色
    else
        bgColor = '#FFF9C4'; % 浅黄色
    end
    
    xl = xlim; yl = ylim;
    text(xl(1) + (xl(2)-xl(1))*0.05, yl(2) - (yl(2)-yl(1))*0.1, infoText, ...
        'EdgeColor', 'k', 'BackgroundColor', bgColor, 'FontSize', 10, 'FontWeight', 'bold');
end