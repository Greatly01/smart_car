function safe = isPathSafe(point1, point2, obstacles, robotRadius)
    % 初始化路径安全性为 true
    safe = true;

    % 采样路径上的点
    numSamples = 20; % 插值点数量
    for t = linspace(0, 1, numSamples)
        % 插值计算路径上的点
        interpPoint = point1 + t * (point2 - point1);
        
        % 计算路径点到所有障碍物的距离
        dists = vecnorm(obstacles - interpPoint, 2, 2);
        
        % 如果距离小于机器人半径，判定为碰撞
        if min(dists) < robotRadius
            safe = false;
            return;
        end
    end
end
