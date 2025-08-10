clc;
clear;
close all;

% 仿真参数
mapSize = [100, 100]; % 地图尺寸
start = [10, 10]; % 起点
goal = [90, 90]; % 目标点
obstacleNum = 50; % 障碍物数量
robotRadius = 2; % 机器人半径

% 生成地图
[map, obstacles] = generateMap(mapSize, obstacleNum);

% 可视化初始地图
figure;
hold on;
xlim([0 mapSize(1)]);
ylim([0 mapSize(2)]);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % 起点
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % 目标点
drawObstacles(obstacles);

title('Enhanced Fusion Path Planning (DRL + RRT + DWA)');
xlabel('X');
ylabel('Y');

% 规划路径初始化
currentPos = start;
finalPath = [];
dynamicObstacles = generateDynamicObstacles(obstacles);

% 自适应切换策略融合路径规划
tic;
while norm(currentPos - goal) > robotRadius
    if shouldUseDWA(currentPos, goal, obstacles, dynamicObstacles)
        % 使用 DWA 局部路径规划
        [nextPos, dynamicObstacles] = ImprovedDWA(currentPos, goal, obstacles, dynamicObstacles, robotRadius, mapSize);
    else
        % 使用 RRT 进行全局路径规划
        rrtPath = OptimizedBiRRT(map, currentPos, goal, dynamicObstacles, robotRadius);
        % 如果 RRT 成功，取其下一步点；否则回退到 DWA
        if ~isempty(rrtPath)
            nextPos = rrtPath(2, :); % 获取全局路径的下一步
        else
            warning('RRT failed, reverting to DWA.');
            [nextPos, dynamicObstacles] = ImprovedDWA(currentPos, goal, obstacles, dynamicObstacles, robotRadius, mapSize);
        end
    end

    % 路径更新与可视化
    finalPath = [finalPath; nextPos]; %#ok<AGROW>
    plot(nextPos(1), nextPos(2), 'gx', 'MarkerSize', 8, 'LineWidth', 2);
    drawnow;

    % 更新当前位置
    currentPos = nextPos;
end
toc;

% 路径后处理：平滑与优化
smoothedPath = smoothPath(finalPath, obstacles, robotRadius);

% 可视化最终路径
plot(smoothedPath(:, 1), smoothedPath(:, 2), 'm-', 'LineWidth', 2);
legend('Start', 'Goal', 'Obstacles', 'Dynamic Path', 'Smoothed Path');

% 评价与结果展示
evaluateAndVisualizeResults(smoothedPath, obstacles);

