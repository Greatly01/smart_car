function pathLength = calculatePathLength(path)
    pathLength = sum(vecnorm(diff(path), 2, 2)); % 计算每段路径长度并求和
end


% function pathLength = calculatePathLength(path)
%     pathLength = sum(vecnorm(diff(path), 2, 2)); % 计算每段路径的距离并求和
% end
