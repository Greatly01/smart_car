% 判断是否使用DWA
function useDWA = shouldUseDWA(currentPos, goal, obstacles, dynamicObstacles)
    % 简单策略：离目标近时使用DWA
    useDWA = norm(currentPos - goal) < 20;
end
