function [nextPos, dynamicObstacles] = ImprovedDWA(currentPos, goal, obstacles, dynamicObstacles, robotRadius, mapSize)
    bestScore = -inf;
    bestPos = currentPos;
    maxSpeed = 1.5; % 最大速度
    maxAngle = pi / 4; % 最大转角

    for v = 0.1:0.2:maxSpeed
        for w = -maxAngle:maxAngle/8:maxAngle
            % 模拟下一步位置
            nextPosSim = currentPos + [v * cos(w), v * sin(w)];
            goalDist = -norm(nextPosSim - goal); % 趋近目标评分
            obsDist = min(vecnorm(obstacles - nextPosSim, 2, 2)); % 避障评分
            speedScore = v; % 保持速度评分
            
            % 综合评分
            score = goalDist + obsDist + speedScore * 0.5;
            if obsDist > robotRadius && score > bestScore % 避障有效
                bestScore = score;
                bestPos = nextPosSim;
            end
        end
    end

    nextPos = bestPos;
    dynamicObstacles = updateObstacles(dynamicObstacles);
end
