function path = ImprovedRRT(map, start, goal, obstacles, robotRadius)
    maxIter = 300;
    goalBias = 0.5; % 提高目标采样概率
    stepSize = 5; % 每步扩展距离
    path = [];
    tree = start;

    for i = 1:maxIter
        % 目标偏向采样
        if rand < goalBias
            randPoint = goal;
        else
            randPoint = rand(1, 2) .* size(map);
        end

        % 扩展树
        tree = extendTreeOptimized(tree, randPoint, obstacles, robotRadius, stepSize);
        
        % 检查是否到达目标
        if norm(tree(end, :) - goal) < stepSize
            path = [tree; goal];
            return;
        end
    end
end
