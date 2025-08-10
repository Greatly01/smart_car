% MATLAB代码：路径优化（RRT*）
function optimized_path = rrt_star_optimization(path, map)
    % 重新连接以最小化路径长度
    for i = 1:length(path) - 1
        for j = i + 2:length(path)
            if is_line_collision_free(path(i, :), path(j, :), map)
                % 如果节点间无障碍物，则重新连接
                path(i+1:j-1, :) = [];
            end
        end
    end
    optimized_path = path;
end