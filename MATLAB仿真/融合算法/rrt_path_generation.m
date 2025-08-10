% MATLAB代码：初始路径生成（RRT）
function path = rrt_path_generation(start_point, goal_point, map)
    % 参数设置
    max_iterations = 1000;
    step_size = 5;
    nodes = start_point;
    for i = 1:max_iterations
        % 随机采样节点
        x_rand = [randi([1, size(map, 1)]), randi([1, size(map, 2)])];
        % 找到最近的节点
        [~, idx] = min(vecnorm(nodes - x_rand, 2, 2));
        nearest_node = nodes(idx, :);
        % 生成新节点
        direction = (x_rand - nearest_node) / norm(x_rand - nearest_node);
        new_node = nearest_node + step_size * direction;
        % 检查新节点是否有效
        if is_valid(new_node, map)
            nodes = [nodes; new_node];
            % 如果到达目标区域
            if norm(new_node - goal_point) < step_size
                path = [nodes; goal_point];
                return;
            end
        end
    end
    error('未找到可行路径');
end