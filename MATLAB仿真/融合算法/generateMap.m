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