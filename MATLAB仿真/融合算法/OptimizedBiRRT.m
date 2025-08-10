function path = OptimizedBiRRT(map, start, goal, obstacles, robotRadius)
    maxIter = 300;
    goalBias = 0.3; % 提高目标采样概率
    startTree = start;
    goalTree = goal;
    path = [];

    for i = 1:maxIter
        % 动态调整采样范围
        if rand < goalBias
            randPoint = goal; % 直接采样目标点
        else
            randPoint = rand(1, 2) .* size(map);
        end
        
        % 扩展起点树和终点树
        startTree = extendTreeOptimized(startTree, randPoint, obstacles, robotRadius, 5);
        goalTree = extendTreeOptimized(goalTree, randPoint, obstacles, robotRadius, 5);
        
        % 检查是否连接
        if isConnected(startTree, goalTree, robotRadius)
            path = [startTree; flipud(goalTree)];
            path = smoothPath(path, obstacles, robotRadius); % 路径平滑
            return;
        end
    end
    
    warning('OptimizedBiRRT failed to find a path.');
end










% function path = OptimizedBiRRT(map, start, goal, obstacles, robotRadius)
%     maxIter = 300; % 最大迭代次数
%     goalBias = 0.2; % 直接采样目标点的概率
%     maxDistance = 5; % 每次扩展的最大距离
%     startTree = start;
%     goalTree = goal;
%     path = [];
% 
%     for i = 1:maxIter
%         % 随机采样点，加入目标偏向采样
%         if rand < goalBias
%             randPoint = goal; % 直接采样目标点
%         else
%             randPoint = rand(1, 2) .* size(map);
%         end
%         
%         % 扩展起点树
%         startTree = extendTreeOptimized(startTree, randPoint, obstacles, robotRadius, maxDistance);
%         
%         % 扩展终点树
%         goalTree = extendTreeOptimized(goalTree, randPoint, obstacles, robotRadius, maxDistance);
%         
%         % 检查两棵树是否连接
%         if isConnected(startTree, goalTree, robotRadius)
%             path = [startTree; flipud(goalTree)];
%             return;
%         end
%     end
%     
%     disp('BiRRT failed to find a path within the maximum iterations.');
% end
