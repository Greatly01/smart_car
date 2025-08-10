function [map, startPos, goalPos] = createEnvironment()
    % 创建一个 100x100 的环境
    mapSize = 100;
    map = zeros(mapSize, mapSize);

    % 障碍物参数
    numObstacles = 5; % 障碍物数量
    minObstacleSize = 5; % 最小障碍物尺寸
    maxObstacleSize = 15; % 最大障碍物尺寸
    minObstacleSpacing = 10; % 最小障碍物间距

    % 生成障碍物
    for i = 1:numObstacles
        obstacleSize = randi([minObstacleSize, maxObstacleSize]);
        % 寻找一个合适的障碍物位置
        validPosition = false;
        while ~validPosition
            obstacleX = randi([obstacleSize + 1, mapSize - obstacleSize - 1]);
            obstacleY = randi([obstacleSize + 1, mapSize - obstacleSize - 1]);
            % 检查是否有重叠
            if isObstacleFree(map, obstacleX, obstacleSize, obstacleY, obstacleSize, minObstacleSpacing)
                validPosition = true;
                map(obstacleX-obstacleSize:obstacleX+obstacleSize, obstacleY-obstacleSize:obstacleY+obstacleSize) = 1;
            end
        end
    end

    % 确保有路径可达
    while ~isPathFeasible(map, mapSize)
        % 移除一个随机障碍物并尝试重新放置
        obstacleIndex = randi(numObstacles);
        obstacleX = randi([minObstacleSize + 1, mapSize - minObstacleSize - 1]);
        obstacleY = randi([minObstacleSize + 1, mapSize - minObstacleSize - 1]);
        map(obstacleIndex, :) = 0;
        map(:, obstacleIndex) = 0;
        map(obstacleX-minObstacleSize:obstacleX+minObstacleSize, obstacleY-minObstacleSize:obstacleY+minObstacleSize) = 1;
    end

    % 设置起始点和目标点
    startPos = [10, 10];
    goalPos = [90, 90];

    % 检查起始点和目标点是否可行
    while ~isPointFeasible(map, startPos) || ~isPointFeasible(map, goalPos)
        startPos = [randi([minObstacleSize+1, mapSize-minObstacleSize-1]), randi([minObstacleSize+1, mapSize-minObstacleSize-1])];
        goalPos = [randi([minObstacleSize+1, mapSize-minObstacleSize-1]), randi([minObstacleSize+1, mapSize-minObstacleSize-1])];
    end

    % 显示地图
    imagesc(map);
    colormap gray;
    hold on;
    scatter(startPos(2), startPos(1), 'r', 'filled');
    scatter(goalPos(2), goalPos(1), 'g', 'filled');
end

% 检查障碍物是否与其他障碍物重叠
function valid = isObstacleFree(map, centerX, sizeX, centerY, sizeY, minSpacing)
    valid = true;
    [rows, cols] = size(map);
    for i = max(1, centerX-sizeX-minSpacing):min(rows, centerX+sizeX+minSpacing)
        for j = max(1, centerY-sizeY-minSpacing):min(cols, centerY+sizeY+minSpacing)
            if map(i,j) == 1 && (abs(i-centerX) < sizeX || abs(j-centerY) < sizeY)
                valid = false;
                return;
            end
        end
    end
end

% 检查是否有可行路径
function feasible = isPathFeasible(map, mapSize)
    % 这里可以使用广度优先搜索或其他路径寻找算法检查
    % 返回true如果存在一条从左上角到右下角的路径
    feasible = false;
end

% 检查给定点是否可行
function feasible = isPointFeasible(map, point)
    feasible = map(point(1), point(2)) == 0;
end