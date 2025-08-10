clc
clear
close all

%% 图定义
% 根据节点的邻近节点表及字母节点-数字节点对应表，构造节点元胞数组
nodes_dist = cell(0);
nodes_dist(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_dist(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_dist(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_dist(4,:) = {4, [3, 5], [3, 4]};
nodes_dist(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_dist(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_dist(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% 用户输入
start_node = 4; % 起始节点
end_node = 7;   % 目标节点

%% 算法初始化
% S集合和U集合
S = [start_node, 0];
U(:,1) = setdiff(1:length(nodes_dist), start_node)'; % 初始U集合
U(:,2) = inf(size(U,1), 1); % 初始距离为无穷大

% 初始化从起始节点的距离
for i = 1:length(nodes_dist{start_node, 2})
    neighbor = nodes_dist{start_node, 2}(i);
    dist = nodes_dist{start_node, 3}(i);
    U(U(:,1) == neighbor, 2) = dist;
end

% 最优路径初始化
path_opt = cell(length(nodes_dist),2);
path_opt(start_node,:) = {start_node, start_node};

%% 记录运行时间
tic

%% 循环遍历
while ~isempty(U)
    % 在U集合中找到当前最小距离值及对应节点
    [dist_min, idx] = min(U(:,2));
    node_min = U(idx, 1);
    S(end+1,:) = [node_min, dist_min]; % 将节点加入S集合
    U(idx,:) = []; % 从U集合移除
    
    % 更新最优路径集合
    path_opt(node_min, :) = {node_min, [path_opt{S(end-1, 1), 2}, node_min]};
    
    % 遍历该节点的邻接节点，更新U集合中的距离
    for i = 1:length(nodes_dist{node_min, 2})
        neighbor = nodes_dist{node_min, 2}(i);
        dist = nodes_dist{node_min, 3}(i);
        
        % 判断是否需要更新
        idx_neighbor = find(U(:,1) == neighbor);
        if ~isempty(idx_neighbor)
            new_dist = dist_min + dist;
            if new_dist < U(idx_neighbor, 2)
                U(idx_neighbor, 2) = new_dist; % 更新最短距离
                path_opt{neighbor, 2} = [path_opt{node_min, 2}, neighbor]; % 更新路径
            end
        end
    end
end

%% 结果
time_elapsed = toc; % 记录算法运行时间
shortest_path = path_opt{end_node, 2}; % 最短路径
shortest_distance = S(S(:,1) == end_node, 2); % 最短路径长度

fprintf('起点: %d\n', start_node);
fprintf('终点: %d\n', end_node);
fprintf('最短路径: ');
disp(shortest_path);
fprintf('最短路径长度: %.2f\n', shortest_distance);
fprintf('算法运行时间: %.6f秒\n', time_elapsed);

%% 可视化
figure;
subplot(2, 1, 1);
hold on;
% 绘制节点
for i = 1:length(nodes_dist)
    scatter(i, 0, 100, 'filled');
    text(i, 0.2, sprintf('%d', i), 'HorizontalAlignment', 'center');
end
% 绘制边
for i = 1:length(nodes_dist)
    for j = 1:length(nodes_dist{i, 2})
        neighbor = nodes_dist{i, 2}(j);
        plot([i, neighbor], [0, 0], 'k--');
    end
end
% 绘制最短路径
for i = 1:length(shortest_path)-1
    plot([shortest_path(i), shortest_path(i+1)], [0, 0], 'r-', 'LineWidth', 2);
end
title('图的节点与最短路径');
xlabel('节点');
ylabel('距离');
hold off;

% 性能指标表
subplot(2, 1, 2);
data = {'起始节点', '目标节点', '最短路径长度', '运行时间 (s)';
        start_node, end_node, shortest_distance, time_elapsed};
uitable('Data', data, 'ColumnName', {'指标', '值'}, ...
        'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.4]);
title('算法性能指标');
