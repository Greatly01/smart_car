% BiRRT 函数定义
function path = BiRRT(map, start, goal, obstacles, robotRadius)
    maxIter = 500; % 最大迭代次数
    startTree = start; % 起点树
    goalTree = goal; % 终点树
    path = []; % 最终路径

    for i = 1:maxIter
        % 随机采样点
        randPoint = rand(1, 2) .* size(map);
        
        % 扩展起点树
        startTree = extendTree(startTree, randPoint, obstacles, robotRadius);
        
        % 扩展终点树
        goalTree = extendTree(goalTree, randPoint, obstacles, robotRadius);
        
        % 检查两棵树是否连接
        if isConnected(startTree, goalTree, robotRadius)
            path = [startTree; flipud(goalTree)]; % 合并路径
            return;
        end
    end

    disp('BiRRT failed to find a path within the maximum iterations.');
end
