function evaluateAndVisualizeResults(path, curvature, distList, pathLength, ...
    pathSmoothness, minDistToObstacles, turnCount)

    % 1. 路径图
    figure;
    subplot(2, 2, 1);
    plot(path(:, 1), path(:, 2), '-o');
    title('Planned Path');
    xlabel('X');
    ylabel('Y');
    legend(sprintf('Length: %.2f', pathLength));
    grid on;

    % 2. 曲率变化图
    subplot(2, 2, 2);
    plot(2:(size(path, 1) - 1), curvature, '-o');
    title('Path Curvature');
    xlabel('Segment');
    ylabel('Curvature (rad)');
    legend(sprintf('Smoothness: %.2f', pathSmoothness));
    grid on;

    % 3. 路径安全性图
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
