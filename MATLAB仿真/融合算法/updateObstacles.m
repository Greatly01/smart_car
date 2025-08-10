% 更新动态障碍物位置
function updatedObstacles = updateObstacles(dynamicObstacles)
    updatedObstacles = dynamicObstacles + randn(size(dynamicObstacles)) * 0.5;
end