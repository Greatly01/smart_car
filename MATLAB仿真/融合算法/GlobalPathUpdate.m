function nextPos = GlobalPathUpdate(currentPos, goal, obstacles, robotRadius, dynamicObstacles)
    % 使用全局路径规划（带目标偏向和动态步长调整）
    globalPath = ImprovedRRTDynamic(currentPos, goal, obstacles, robotRadius);
    if ~isempty(globalPath)
        nextPos = globalPath(2, :); % 获取下一步
    else
        % RRT 失败时回退到局部控制
        nextPos = LocalGoalApproach(currentPos, goal, obstacles, robotRadius);
    end
end
