% findNearest 函数定义
function nearestPoint = findNearest(randPoint, tree)
    distances = vecnorm(tree - randPoint, 2, 2); % 计算树中所有点到采样点的距离
    [~, idx] = min(distances); % 找到最小距离的索引
    nearestPoint = tree(idx, :);
end
