clc;
clear;
close all;

%% 三维地图定义
nodes_3d = [0 0 0; 1 0 0; 1 1 0; 0 1 0; % 地面节点
            0.5 0.5 1;                   % 中间层节点
            0 0 2; 1 0 2; 1 1 2; 0 1 2]; % 顶层节点

edges = {1, [2, 4, 5], [1, 1.5, 2];   % 节点1连接到节点2、4、5
         2, [1, 3, 5], [1, 1.5, 1.2];
         3, [2, 4, 5, 8], [1.5, 1.5, 2.5, 3];
         4, [1, 3, 5], [1.5, 1.5, 2];
         5, [1, 2, 3, 4, 6, 7, 8, 9], [2, 1.2, 2.5, 2, 3, 3, 3, 3];
         6, [5, 7, 9], [3, 1, 1];
         7, [6, 5, 8], [1, 3, 1.5];
         8, [3, 5, 7, 9], [3, 3, 1.5, 2];
         9, [5, 6, 8], [3, 1, 2]};

start_node = 1; % 起点
end_node = 9;   % 终点

%% 初始化Dijkstra算法
S = [start_node, 0];
U(:,1) = setdiff(1:size(nodes_3d,1), start_node)'; 
U(:,2) = inf(size(U,1), 1);

for i = 1:length(edges{start_node,2})
    neighbor = edges{start_node,2}(i);
    dist = edges{start_node,3}(i);
    U(U(:,1) == neighbor, 2) = dist;
end

path_opt = cell(size(nodes_3d,1), 2);
path_opt(start_node,:) = {start_node, start_node};

%% 路径规划算法
tic;
while ~isempty(U)
    [dist_min, idx] = min(U(:,2));
    node_min = U(idx, 1);
    S(end+1,:) = [node_min, dist_min];
    U(idx,:) = [];
    
    path_opt(node_min,:) = {node_min, [path_opt{S(end-1, 1), 2}, node_min]};
    
    for i = 1:length(edges{node_min,2})
        neighbor = edges{node_min,2}(i);
        dist = edges{node_min,3}(i);
        idx_neighbor = find(U(:,1) == neighbor);
        if ~isempty(idx_neighbor)
            new_dist = dist_min + dist;
            if new_dist < U(idx_neighbor, 2)
                U(idx_neighbor, 2) = new_dist;
                path_opt{neighbor,2} = [path_opt{node_min,2}, neighbor];
            end
        end
    end
end
time_elapsed = toc;

%% 提取结果
shortest_path = path_opt{end_node, 2};
shortest_distance = S(S(:,1) == end_node, 2);
node_count = length(shortest_path);
average_node_distance = shortest_distance / (node_count - 1);

%% 可视化三维地图和最短路径
figure;
hold on;
grid on;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('三维地图与最短路径', 'FontSize', 14);

for i = 1:size(nodes_3d, 1)
    scatter3(nodes_3d(i,1), nodes_3d(i,2), nodes_3d(i,3), 100, 'filled');
    text(nodes_3d(i,1), nodes_3d(i,2), nodes_3d(i,3) + 0.1, sprintf('%d', i), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

for i = 1:size(edges, 1)
    for j = 1:length(edges{i,2})
        neighbor = edges{i,2}(j);
        plot3([nodes_3d(i,1), nodes_3d(neighbor,1)], ...
              [nodes_3d(i,2), nodes_3d(neighbor,2)], ...
              [nodes_3d(i,3), nodes_3d(neighbor,3)], 'k--');
    end
end

for i = 1:length(shortest_path)-1
    p1 = nodes_3d(shortest_path(i),:);
    p2 = nodes_3d(shortest_path(i+1),:);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'r-', 'LineWidth', 2);
end
legend({'节点', '边', '最短路径'}, 'Location', 'best');
hold off;

%% 路径评价参数可视化
figure;

% 绘制表格
data = {
    '起始节点', start_node;
    '目标节点', end_node;
    '路径长度', shortest_distance;
    '运行时间 (s)', time_elapsed;
    '节点数', node_count;
    '平均节点间距', average_node_distance;
};
uitable('Data', data, ...
        'ColumnName', {'指标', '值'}, ...
        'ColumnWidth', {120, 120}, ...
        'RowName', [], ...
        'Units', 'normalized', ...
        'Position', [0.2, 0.2, 0.6, 0.4]);
figure;

% 绘制柱状图
subplot(2, 1, 2);
bar([shortest_distance, time_elapsed, node_count, average_node_distance]);
xticks(1:4);
xticklabels({'路径长度', '运行时间', '节点数', '平均节点间距'});
ylabel('值');
title('路径规划评价参数', 'FontSize', 14);
grid on;

%% 输出结果
fprintf('起点: %d\n', start_node);
fprintf('终点: %d\n', end_node);
fprintf('最短路径: ');
disp(shortest_path);
fprintf('最短路径长度: %.2f\n', shortest_distance);
fprintf('路径规划运行时间: %.6f秒\n', time_elapsed);
fprintf('最短路径节点数: %d\n', node_count);
fprintf('平均节点间距: %.2f\n', average_node_distance);
