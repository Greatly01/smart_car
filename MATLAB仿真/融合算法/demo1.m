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

% 绘制障碍物
drawObstacles(obstacles); % 调用辅助函数绘制障碍物

title('Enhanced Path Planning with DRL, RRT, and DWA');
xlabel('X');
ylabel('Y');

% 全局路径规划（Bi-RRT with DRL）
rrtPath = BiRRT(map, start, goal, obstacles, robotRadius);

% 路径平滑
smoothedPath = smoothPath(rrtPath, obstacles, robotRadius);

% 可视化RRT路径
plot(smoothedPath(:, 1), smoothedPath(:, 2), 'b-', 'LineWidth', 2);

% 局部路径规划（DWA）和动态避障
finalPath = [];
currentPos = start;
dynamicObstacles = generateDynamicObstacles(obstacles);

while norm(currentPos - goal) > robotRadius
    % DRL策略选择（切换RRT或DWA）
    if shouldUseDWA(currentPos, goal, obstacles, dynamicObstacles)
        % DWA计算局部路径
        [nextPos, dynamicObstacles] = DWA(currentPos, goal, obstacles, dynamicObstacles, robotRadius, mapSize);
    else
        % 使用RRT全局规划剩余路径
        rrtPath = BiRRT(map, currentPos, goal, dynamicObstacles, robotRadius);
        nextPos = rrtPath(2, :); % 获取RRT的下一步
    end

    finalPath = [finalPath; nextPos]; %#ok<AGROW>
    
    % 可视化动态避障
    plot(nextPos(1), nextPos(2), 'gx', 'MarkerSize', 8, 'LineWidth', 2);
    drawnow;
    
    % 更新当前位置
    currentPos = nextPos;
end

% 最终路径可视化
plot(finalPath(:, 1), finalPath(:, 2), 'm-', 'LineWidth', 2);
legend('Start', 'Goal', 'Obstacles', 'RRT Smoothed Path', 'Final Path');















