% extendTree 函数定义
function newTree = extendTree(tree, randPoint, obstacles, robotRadius)
    % 找到树中最近的点
    nearestPoint = findNearest(randPoint, tree);
    
    % 从最近点向采样点扩展
    stepSize = 2; % 每步扩展的固定距离
    direction = randPoint - nearestPoint;
    direction = direction / norm(direction) * stepSize; % 归一化方向
    
    % 计算新点
    newPoint = nearestPoint + direction;
    
    % 检查新点是否碰撞
    if isCollisionFree(nearestPoint, newPoint, obstacles, robotRadius)
        newTree = [tree; newPoint]; % 将新点加入树
    else
        newTree = tree; % 无法扩展时，返回原树
    end
end
