% 动态窗口法（DWA）
function [nextPos, dynamicObstacles] = DWA(currentPos, goal, obstacles, dynamicObstacles, robotRadius, mapSize)
    % 评分函数：目标距离、障碍物距离、速度
    bestScore = -inf;
    bestPos = currentPos;
    
    for v = 0.1:0.1:1 % 线速度
        for w = -pi/4:pi/16:pi/4 % 角速度
            % 模拟下一步位置
            nextPos = currentPos + [v * cos(w), v * sin(w)];
            
            % 计算评分
            goalDist = -norm(nextPos - goal);
            obsDist = min(vecnorm(dynamicObstacles - nextPos, 2, 2));
            score = goalDist + obsDist;
            
            if score > bestScore
                bestScore = score;
                bestPos = nextPos;
            end
        end
    end
    
    nextPos = bestPos;
    dynamicObstacles = updateObstacles(dynamicObstacles);
end