%% 辅助函数：获取开放列表中f值最小的节点键值
function key = min_f_node_key(open_list)
    keys = open_list.keys;
    min_f = inf;
    key = '';
    for i = 1:length(keys)
        node = open_list(keys{i});
        if node.f < min_f
            min_f = node.f;
            key = keys{i};
        end
    end
end