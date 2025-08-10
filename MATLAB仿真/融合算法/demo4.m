clc;
clear;
close all;

% 仿真参数
mapSize = [100, 100];
start = [10, 10];
goal = [90, 90];
obstacleNum = 50;
robotRadius = 2;

% 生成地图
[map, obstacles] = generateMap(mapSize, obstacleNum);

% 可视化初始地图
figure;
hold on;
xlim([0 mapSize(1)]);
ylim([0 mapSize(2)]);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
drawObstacles(obstacles);

title('Enhanced Path Planning Near Goal Optimization');
xlabel('X');
ylabel('Y');

% 初始化变量
currentPos = start;
finalPath = [];
dynamicObstacles = generateDynamicObstacles(obstacles);

% 路径规划主循环
while norm(currentPos - goal) > robotRadius
    if shouldSwitchToLocalControl(currentPos, goal)
        % 使用改进的局部路径控制
        nextPos = LocalGoalApproach(currentPos, goal, obstacles, robotRadius);
    else
        % 使用改进的全局规划
        nextPos = GlobalPathUpdate(currentPos, goal, obstacles, robotRadius, dynamicObstacles);
    end

    % 更新路径和可视化
    finalPath = [finalPath; nextPos];
    plot(nextPos(1), nextPos(2), 'gx', 'MarkerSize', 8, 'LineWidth', 2);
    drawnow;

    % 更新当前位置
    currentPos = nextPos;
end

% 路径后处理：平滑
smoothedPath = smoothOptimizedPath(finalPath, obstacles, robotRadius);

% 可视化最终路径
plot(smoothedPath(:, 1), smoothedPath(:, 2), 'm-', 'LineWidth', 2);
legend('Start', 'Goal', 'Obstacles', 'Dynamic Path', 'Smoothed Path');

% 计算路径评价指标
pathLength = calculatePathLength(smoothedPath);
[pathSmoothness, ~] = calculatePathSmoothness(smoothedPath);
minDistToObstacles = min(calculateSafetyMetrics(smoothedPath, obstacles));
turnCount = calculateTurnCount(smoothedPath);

% 调用评价与可视化函数
evaluateAndVisualizeResults1(smoothedPath, obstacles, pathLength, pathSmoothness, minDistToObstacles, turnCount);


