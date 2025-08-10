% 主脚本部分
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

title('Optimized Path Planning with DRL, RRT, and DWA');
xlabel('X');
ylabel('Y');

% 全局路径规划（优化Bi-RRT）
tic;
rrtPath = OptimizedBiRRT(map, start, goal, obstacles, robotRadius);
toc;

% 路径平滑
smoothedPath = smoothPath(rrtPath, obstacles, robotRadius);

% 可视化路径
plot(smoothedPath(:, 1), smoothedPath(:, 2), 'b-', 'LineWidth', 2);

% 局部路径规划和动态避障可按需添加
% 1. 路径长度计算
pathLength = calculatePathLength(smoothedPath);

% 2. 路径平滑度计算（曲率变化）
[pathSmoothness, curvature] = calculatePathSmoothness(smoothedPath);

% 3. 路径安全性（与障碍物的最小距离）
[minDistToObstacles, distList] = calculatePathSafety(smoothedPath, obstacles);

% 4. 路径转向次数
turnCount = calculateTurnCount(smoothedPath);

% 5. 绘制评价结果图表
evaluateAndVisualizeResults(smoothedPath, curvature, distList, pathLength, ...
    pathSmoothness, minDistToObstacles, turnCount);

% 打印详细指标
fprintf('路径规划评价指标：\n');
fprintf('路径总长度：%.2f 单位\n', pathLength);
fprintf('路径平滑度（曲率变化总和）：%.2f\n', pathSmoothness);
fprintf('路径安全性（最小障碍物距离）：%.2f 单位\n', minDistToObstacles);
fprintf('路径转向次数：%d\n', turnCount);