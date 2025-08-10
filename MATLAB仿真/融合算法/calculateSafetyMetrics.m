function distList = calculateSafetyMetrics(path, obstacles)
    distList = []; % 存储每点到障碍物的最小距离
    for i = 1:size(path, 1)
        dists = vecnorm(obstacles - path(i, :), 2, 2); % 距离障碍物
        distList = [distList; min(dists)]; %#ok<AGROW>
    end
end
