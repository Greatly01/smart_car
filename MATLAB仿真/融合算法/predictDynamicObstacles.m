function predictedObstacles = predictDynamicObstacles(dynamicObstacles)
    velocity = [0.2, 0.2]; % 简单预测
    predictedObstacles = dynamicObstacles + velocity;
end
