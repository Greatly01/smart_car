function path = ImprovedRRTDynamic(start, goal, obstacles, robotRadius)
    maxIter = 500;
    stepSizeInitial = 10;
    stepSizeFinal = 2; % 靠近目标时减小步长
    goalBias = 0.4; % 目标采样概率
    tree = start;
    path = [];

    for iter = 1:maxIter
        % 动态调整步长
        stepSize = stepSizeInitial - (stepSizeInitial - stepSizeFinal) * iter / maxIter;

        % 目标偏向采样
        if rand < goalBias
            randPoint = goal;
        else
            randPoint = rand(1, 2) .* size(obstacles);
        end

        % 扩展树
        tree = extendTreeDynamic(tree, randPoint, obstacles, robotRadius, stepSize);

        % 检查是否到达目标
        if norm(tree(end, :) - goal) < stepSize
            path = [tree; goal];
            return;
        end
    end
end

function tree = extendTreeDynamic(tree, randPoint, obstacles, robotRadius, stepSize)
    nearestPoint = findNearest(randPoint, tree);
    direction = randPoint - nearestPoint;
    direction = direction / norm(direction) * stepSize;
    newPoint = nearestPoint + direction;

    % 检查路径是否安全
    if isPathSafe(nearestPoint, newPoint, obstacles, robotRadius)
        tree = [tree; newPoint];
    end
end
