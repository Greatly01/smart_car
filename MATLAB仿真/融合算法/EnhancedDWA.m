function [nextPos, dynamicObstacles] = EnhancedDWA(currentPos, goal, obstacles, dynamicObstacles, robotRadius, mapSize)
    bestScore = -inf;
    bestPos = currentPos;
    maxSpeed = 2.0; % 增大最大速度
    maxAngle = pi / 4;

    for v = 0.1:0.2:maxSpeed
        for w = -maxAngle:maxAngle/8:maxAngle
            nextPosSim = currentPos + [v * cos(w), v * sin(w)];
            goalDist = -norm(nextPosSim - goal); % 趋近目标
            obsDist = min(vecnorm(obstacles - nextPosSim, 2, 2)); % 避开障碍物
            predictionPenalty = -norm(nextPosSim - predictDynamicObstacles(dynamicObstacles)); % 预测动态障碍物位置
            score = goalDist + obsDist * 0.5 + predictionPenalty * 0.2;

            if obsDist > robotRadius && score > bestScore
                bestScore = score;
                bestPos = nextPosSim;
            end
        end
    end

    nextPos = bestPos;
    dynamicObstacles = updateObstacles(dynamicObstacles);
end
