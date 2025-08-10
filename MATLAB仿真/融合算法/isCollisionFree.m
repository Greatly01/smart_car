function collisionFree = isCollisionFree(point1, point2, obstacles, robotRadius)
    collisionFree = true;
    numSamples = 5; % 减少插值点数量
    
    % 遍历路径上的点
    for i = 0:1/numSamples:1
        interpPoint = point1 + i * (point2 - point1); % 插值点
        
        % 距离障碍物过近则判为碰撞
        for j = 1:size(obstacles, 1)
            if norm(interpPoint - obstacles(j, :)) < robotRadius
                collisionFree = false;
                return;
            end
        end
    end
end
