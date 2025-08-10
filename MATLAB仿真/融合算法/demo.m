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
title('Initial Map');
xlabel('X');
ylabel('Y');

% 全局路径规划（RRT）
rrtPath = RRT(map, start, goal, obstacles, robotRadius);

% 可视化RRT路径
plot(rrtPath(:, 1), rrtPath(:, 2), 'b-', 'LineWidth', 2);

% 局部路径规划（DWA）和动态障碍物避障
finalPath = [];
currentPos = start;
while norm(currentPos - goal) > robotRadius
    % DWA计算局部路径
    [nextPos, dynamicObstacles] = DWA(currentPos, goal, obstacles, robotRadius, mapSize);
    finalPath = [finalPath; nextPos]; %#ok<AGROW>
    
    % 更新动态障碍物位置
    obstacles = updateObstacles(dynamicObstacles);
    
    % 可视化动态避障
    plot(nextPos(1), nextPos(2), 'gx', 'MarkerSize', 8, 'LineWidth', 2);
    drawnow;
    
    % 更新当前位置
    currentPos = nextPos;
end

% 最终路径可视化
plot(finalPath(:, 1), finalPath(:, 2), 'm-', 'LineWidth', 2);
legend('Start', 'Goal', 'Obstacles', 'RRT Path', 'Final Path');

%% 函数实现部分
% 地图生成函数
function [map, obstacles] = generateMap(mapSize, obstacleNum)
    map = zeros(mapSize);
    obstacles = rand(obstacleNum, 2) .* mapSize; % 随机生成障碍物
end

% 绘制障碍物
function drawObstacles(obstacles)
    for i = 1:size(obstacles, 1)
        rectangle('Position', [obstacles(i, :) - 2, 4, 4], 'Curvature', [1, 1], 'FaceColor', 'k');
    end
end

% RRT路径规划函数
function path = RRT(map, start, goal, obstacles, robotRadius)
    path = [start]; % 初始路径
    maxIter = 500;
    for i = 1:maxIter
        randPoint = rand(1, 2) .* size(map); % 随机采样点
        nearest = findNearest(randPoint, path); % 找到最近点
        newPoint = steer(nearest, randPoint, robotRadius); % 向随机点扩展
        if isCollisionFree(newPoint, obstacles, robotRadius)
            path = [path; newPoint]; %#ok<AGROW>
            if norm(newPoint - goal) < robotRadius
                path = [path; goal]; % 到达目标
                break;
            end
        end
    end
end

% 动态窗口法（DWA）
function [nextPos, dynamicObstacles] = DWA(currentPos, goal, obstacles, robotRadius, mapSize)
    % 简单模拟：计算向目标点运动的速度矢量
    direction = goal - currentPos;
    direction = direction / norm(direction) * robotRadius;
    nextPos = currentPos + direction; % 下一步位置
    
    % 动态障碍物模拟
    dynamicObstacles = obstacles + randn(size(obstacles)) * 0.5; % 随机移动
end

% 更新动态障碍物位置
function updatedObstacles = updateObstacles(dynamicObstacles)
    updatedObstacles = dynamicObstacles;
end

% 检查路径是否碰撞
function collisionFree = isCollisionFree(point, obstacles, robotRadius)
    collisionFree = true;
    for i = 1:size(obstacles, 1)
        if norm(point - obstacles(i, :)) < robotRadius
            collisionFree = false;
            return;
        end
    end
end

% 找到最近点
function nearest = findNearest(point, path)
    distances = vecnorm(path - point, 2, 2);
    [~, idx] = min(distances);
    nearest = path(idx, :);
end

% 向目标点扩展
function newPoint = steer(nearest, randPoint, stepSize)
    direction = randPoint - nearest;
    direction = direction / norm(direction) * stepSize;
    newPoint = nearest + direction;
end
