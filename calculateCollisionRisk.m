function [DCPA, TCPA, isDangerous] = calculateCollisionRisk(ownShip, targetShip, D_safe, T_safe)
    % calculateCollisionRisk 计算两船的DCPA和TCPA并判断碰撞危险
    %
    % 输入参数:
    %   ownShip    - 本船数据结构体，包含字段: x, y (海里/米), v (航速, 节/米每秒), heading (航向, 0-360度)
    %   targetShip - 他船数据结构体，字段同上
    %   D_safe     - 安全距离阈值 (单位与x,y保持一致)
    %   T_safe     - 安全时间阈值 (单位与速度换算后的时间保持一致，通常为小时或分钟)
    %
    % 输出参数:
    %   DCPA       - 最近会遇距离
    %   TCPA       - 到达最近会遇点的时间
    %   isDangerous- 布尔值，true表示有碰撞危险，false表示安全

    % 1. 计算两船在X(东)和Y(北)方向的速度分量
    % 注意：航海中航向是以正北为0度，顺时针方向，因此用sind计算X分量，cosd计算Y分量
    v1_x = ownShip.v * sind(ownShip.heading);
    v1_y = ownShip.v * cosd(ownShip.heading);
    
    v2_x = targetShip.v * sind(targetShip.heading);
    v2_y = targetShip.v * cosd(targetShip.heading);
    
    % 2. 计算相对位置和相对速度 (他船相对于本船)
    dx = targetShip.x - ownShip.x;
    dy = targetShip.y - ownShip.y;
    
    dv_x = v2_x - v1_x;
    dv_y = v2_y - v1_y;
    
    % 相对速度的平方和
    v_rel_sq = dv_x^2 + dv_y^2;
    
    % 3. 计算 TCPA (Time to Closest Point of Approach)
    if v_rel_sq == 0
        % 两船相对静止
        TCPA = 0;
        DCPA = sqrt(dx^2 + dy^2);
    else
        TCPA = -(dx * dv_x + dy * dv_y) / v_rel_sq;
        
        % 4. 计算 DCPA (Distance at Closest Point of Approach)
        % 预测在TCPA时刻两船的位置差
        x_cpa = dx + dv_x * TCPA;
        y_cpa = dy + dv_y * TCPA;
        DCPA = sqrt(x_cpa^2 + y_cpa^2);
    end
    
    % 5. 危险判断逻辑
    % 条件1：DCPA 小于设定的安全距离
    % 条件2：TCPA 大于 0 (表示会在未来相遇，而不是已经错过) 且 小于设定的安全时间
    if (DCPA <= D_safe) && (TCPA >= 0) && (TCPA <= T_safe)
        isDangerous = true;
    else
        isDangerous = false;
    end
end