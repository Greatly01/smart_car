function useLocalControl = shouldSwitchToLocalControl(currentPos, goal)
    % 距离阈值
    switchDistanceThreshold = 10; % 单位
    % 判断是否切换到局部控制
    distanceToGoal = norm(currentPos - goal);
    useLocalControl = distanceToGoal < switchDistanceThreshold;
end
