function evaluateAndVisualizeResults1(path, obstacles, pathLength, pathSmoothness, minDistToObstacles, turnCount)
    % 输入:
    % path: 路径点 (Nx2 矩阵)
    % obstacles: 障碍物位置 (Mx2 矩阵)
    % pathLength: 路径总长度
    % pathSmoothness: 路径平滑度（曲率变化总和）
    % minDistToObstacles: 路径最近点到障碍物的最小距离
    % turnCount: 路径转向次数

    % 1. 路径图
    figure;
    subplot(2, 2, 1);
    plot(path(:, 1), path(:, 2), '-o');
    title('Planned Path');
    xlabel('X');
    ylabel('Y');
    hold on;
    scatter(obstacles(:, 1), obstacles(:, 2), 80, 'k', 'filled');
    legend(sprintf('Length: %.2f', pathLength));
    grid on;

    % 2. 曲率变化图
    [~, curvature] = calculatePathSmoothness(path);
    subplot(2, 2, 2);
    plot(2:(size(path, 1) - 1), curvature, '-o');
    title('Path Curvature');
    xlabel('Segment');
    ylabel('Curvature (rad)');
    legend(sprintf('Smoothness: %.2f', pathSmoothness));
    grid on;

    % 3. 路径安全性图
    distList = calculateSafetyMetrics(path, obstacles);
    subplot(2, 2, 3);
    plot(1:size(path, 1), distList, '-o');
    title('Path Safety (Distance to Obstacles)');
    xlabel('Path Point');
    ylabel('Distance to Nearest Obstacle');
    legend(sprintf('Min Dist: %.2f', minDistToObstacles));
    grid on;

    % 4. 转向次数图
    subplot(2, 2, 4);
    bar(turnCount);
    title('Path Turn Count');
    xlabel('Turn Event');
    ylabel('Count');
    legend(sprintf('Turns: %d', turnCount));
    grid on;
end
