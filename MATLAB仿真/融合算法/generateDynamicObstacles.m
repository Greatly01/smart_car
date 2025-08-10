% 动态障碍物生成
function dynamicObstacles = generateDynamicObstacles(obstacles)
    dynamicObstacles = obstacles + randn(size(obstacles)) * 0.5; % 添加初始速度
end