function newTree = extendTreeOptimized(tree, randPoint, obstacles, robotRadius, maxDistance)
    nearestPoint = findNearest(randPoint, tree); % 找到最近点
    
    % 限制扩展距离
    direction = randPoint - nearestPoint;
    distance = norm(direction);
    if distance > maxDistance
        direction = direction / distance * maxDistance;
    end
    newPoint = nearestPoint + direction;

    % 检查路径是否碰撞
    if isCollisionFree(nearestPoint, newPoint, obstacles, robotRadius)
        newTree = [tree; newPoint]; % 加入树
    else
        newTree = tree; % 路径不可行
    end
end
