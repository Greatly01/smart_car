function [minDist, distList] = calculatePathSafety(path, obstacles)
    distList = []; % 存储每个点到障碍物的最小距离
    for i = 1:size(path, 1)
        dists = vecnorm(obstacles - path(i, :), 2, 2); % 计算到所有障碍物的距离
        distList = [distList; min(dists)]; %#ok<AGROW>
    end
    minDist = min(distList); % 路径最小安全距离
end
