function free = is_line_collision_free(node1, node2, map)
    % 检查两个节点之间的连线是否无障碍物
    line = linspace(node1, node2, 100);
    for i = 1:size(line, 1)
        if ~is_valid(line(i, :), map)
            free = false;
            return;
        end
    end
    free = true;
end
