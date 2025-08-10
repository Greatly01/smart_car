function smoothedPath = smoothOptimizedPath(path, obstacles, robotRadius)
    % 输入:
    % path: 原始路径 (Nx2 矩阵，每行是路径上的一个点)
    % obstacles: 障碍物位置 (Mx2 矩阵)
    % robotRadius: 机器人半径
    %
    % 输出:
    % smoothedPath: 平滑后的路径

    if size(path, 1) < 3
        % 如果路径点过少，直接返回原始路径
        smoothedPath = path;
        return;
    end

    % 样条插值
    t = 1:size(path, 1);
    tFine = linspace(1, size(path, 1), 10 * size(path, 1)); % 更高分辨率
    xFine = interp1(t, path(:, 1), tFine, 'spline');
    yFine = interp1(t, path(:, 2), tFine, 'spline');
    interpolatedPath = [xFine', yFine'];

    % 碰撞检测与调整
    smoothedPath = [];
    smoothedPath = [smoothedPath; interpolatedPath(1, :)]; % 添加起点
    for i = 2:size(interpolatedPath, 1)
        if ~isPathSafe(smoothedPath(end, :), interpolatedPath(i, :), obstacles, robotRadius)
            % 如果检测到碰撞，尝试插入新点
            adjustedPoint = avoidLocalObstacle(smoothedPath(end, :), interpolatedPath(i, :), obstacles, robotRadius);
            smoothedPath = [smoothedPath; adjustedPoint]; %#ok<AGROW>
        else
            smoothedPath = [smoothedPath; interpolatedPath(i, :)]; %#ok<AGROW>
        end
    end
end
