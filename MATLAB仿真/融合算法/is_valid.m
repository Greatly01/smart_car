% 辅助函数
function valid = is_valid(node, map)
    % 检查节点是否在地图范围内且不在障碍物上
    if node(1) > 0 && node(1) <= size(map, 1) && node(2) > 0 && node(2) <= size(map, 2)
        valid = map(node(1), node(2)) == 0;
    else
        valid = false;
    end
end