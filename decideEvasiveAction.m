function [situation, action] = decideEvasiveAction(ownShip, targetShip, isDangerous)
    % decideEvasiveAction 基于简化版 COLREGs 规则判断会遇局面并给出动作
    %
    % 输入:
    %   ownShip, targetShip - 包含 x, y, v, heading 的结构体
    %   isDangerous         - 布尔值，由 CPA 模块计算得出
    % 输出:
    %   situation           - 字符串，当前判定的局面名称
    %   action              - 字符串，决策动作：'直行' | '左转' | '右转'

    % 0. 如果没有碰撞危险，直接保向保速
    if ~isDangerous
        situation = '安全驶过 (无紧迫危险)';
        action = '直行';
        return;
    end
    
    % 1. 计算真方位角 (True Bearing)
    dx = targetShip.x - ownShip.x;
    dy = targetShip.y - ownShip.y;
    
    % 注意：航海中以Y轴为正北，X轴为正东，因此用 atan2d(dx, dy)
    TB = atan2d(dx, dy); 
    if TB < 0
        TB = TB + 360;
    end
    
    % 2. 计算相对方位角 (Relative Bearing)
    % 范围 0~360度：0为正前，90为右正横，270为左正横
    RB = mod(TB - ownShip.heading, 360);
    
    % 3. 计算航向差
    headingDiff = mod(targetShip.heading - ownShip.heading, 360);
    
    % ==========================================
    % 4. 核心 COLREGs 局面分类树
    % ==========================================
    
    % 情况 A: 对遇局面 (Rule 14)
    % 目标在正前方狭窄扇区，且航向基本相反 (误差允许±20度)
    if (RB <= 10 || RB >= 350) && (headingDiff >= 160 && headingDiff <= 200)
        situation = '对遇局面 (Head-on)';
        action = '右转';
        
    % 情况 B: 追越局面 (Rule 13) - 本船追他船
    % 航向基本一致(差值<45度)，目标在前方，且本船速大于他船
    elseif (RB < 45 || RB > 315) && (headingDiff <= 45 || headingDiff >= 315) && (ownShip.v > targetShip.v)
        situation = '本船追越他船 (Overtaking)';
        % 追赶右前方的船只时，选择向左打舵从其左舷超船，避免横越船艏
        if RB <= 45
            action = '左转'; 
        else
            action = '右转';
        end
        
    % 情况 C: 交叉相遇 - 目标在右舷 (Rule 15 - 让路船)
    % 相对方位在 10度 到 112.5度 之间
    elseif (RB > 10 && RB <= 112.5)
        situation = '交叉相遇 - 目标在右舷 (让路船)';
        action = '右转'; 
        
    % 情况 D: 交叉相遇/被追越 - 目标在左舷或正后方 (Rule 17 - 直航船)
    % 作为直航船，首要义务是保向保速
    else
        situation = '目标在左舷/后方 (直航船)';
        action = '直行';
    end
end