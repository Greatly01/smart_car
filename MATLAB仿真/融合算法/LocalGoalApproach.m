function nextPos = LocalGoalApproach(currentPos, goal, obstacles, robotRadius)
    % 当距离目标较近时，直接逼近目标
    direction = goal - currentPos;
    stepSize = min(norm(direction), robotRadius * 1.5);
    direction = direction / norm(direction) * stepSize;
    nextPos = currentPos + direction;

    % 碰撞检测并调整
    if ~isPathSafe(currentPos, nextPos, obstacles, robotRadius)
        % 如果与障碍物碰撞，调整方向
        nextPos = avoidLocalObstacle(currentPos, nextPos, obstacles, robotRadius);
    end
end
