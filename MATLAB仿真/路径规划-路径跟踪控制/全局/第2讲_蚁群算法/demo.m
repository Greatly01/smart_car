clc;
clear;
close all;

%% 三维社区场景定义
% 节点的三维坐标
nodes_3d = [0 0 0; 2 0 0; 2 2 0; 0 2 0;   % 地面
            1 1 1;                         % 中间节点
            0 0 2; 2 0 2; 2 2 2; 0 2 2];  % 顶层

% 邻接矩阵定义（路径及权重）
edges = {1, [2, 4, 5], [2, 2, 1.5];   % 节点1与2、4、5连接
         2, [1, 3, 5], [2, 2, 1.5];
         3, [2, 4, 5, 8], [2, 2, 2.5, 3];
         4, [1, 3, 5], [2, 2, 1.5];
         5, [1, 2, 3, 4, 6, 7, 8, 9], [1.5, 1.5, 2.5, 1.5, 2, 2, 2, 2];
         6, [5, 7, 9], [2, 1.5, 1.5];
         7, [6, 5, 8], [1.5, 2, 1.5];
         8, [3, 5, 7, 9], [3, 2, 1.5, 2];
         9, [5, 6, 8], [2, 1.5, 2]};

start_node = 1; % 起点
end_node = 9;   % 终点

%% 蚁群算法参数
m = 30;                      % 蚂蚁数量
alpha = 1;                   % 信息素重要性因子
beta = 5;                    % 启发式因子
rho = 0.1;                   % 信息素挥发因子
Q = 1;                       % 信息素常数
iter_max = 100;              % 最大迭代次数

% 初始化信息素和启发式因子
for i = 1:length(edges)
    edges{i, 4} = ones(1, length(edges{i, 3}));   % 信息素
    edges{i, 5} = 1 ./ edges{i, 3};              % 启发式因子
end

best_distances = zeros(iter_max, 1); % 每代最佳路径长度
avg_distances = zeros(iter_max, 1);  % 每代平均路径长度
best_paths = cell(iter_max, 1);      % 每代最佳路径

%% 蚁群算法迭代
for iter = 1:iter_max
    all_paths = cell(m, 1);          % 保存每只蚂蚁的路径
    all_distances = zeros(m, 1);     % 保存每只蚂蚁的路径长度

    for ant = 1:m
        current_node = start_node;
        path = current_node;
        total_distance = 0;

        while current_node ~= end_node
            neighbors = edges{current_node, 2};
            pheromone = edges{current_node, 4};
            heuristic = edges{current_node, 5};

            % 计算转移概率
            prob = (pheromone .^ alpha) .* (heuristic .^ beta);
            prob = prob / sum(prob);

            % 轮盘赌法选择下一节点
            cumulative_prob = cumsum(prob);
            rand_val = rand;
            next_idx = find(cumulative_prob >= rand_val, 1);
            next_node = neighbors(next_idx);

            % 更新路径和距离
            path(end + 1) = next_node;
            total_distance = total_distance + edges{current_node, 3}(next_idx);
            current_node = next_node;
        end

        all_paths{ant} = path;
        all_distances(ant) = total_distance;
    end

    % 更新每代最优路径
    [best_distances(iter), best_ant] = min(all_distances);
    avg_distances(iter) = mean(all_distances);
    best_paths{iter} = all_paths{best_ant};

    % 更新信息素
    delta_pheromone = cell(size(edges));
    for i = 1:length(edges)
        delta_pheromone{i, 3} = zeros(size(edges{i, 3}));
    end

    for ant = 1:m
        path = all_paths{ant};
        for j = 1:length(path) - 1
            from = path(j);
            to = path(j + 1);
            idx = find(edges{from, 2} == to);
            delta_pheromone{from, 3}(idx) = delta_pheromone{from, 3}(idx) + Q / all_distances(ant);
        end
    end

    for i = 1:length(edges)
        edges{i, 4} = (1 - rho) * edges{i, 4} + delta_pheromone{i, 3};
    end
end

%% 绘制三维地图与最佳路径
figure('Name', '三维社区场景', 'Color', 'w');
hold on;
grid on;
view(3);
xlabel('X', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z', 'FontSize', 12, 'FontWeight', 'bold');
title('三维社区场景与最佳路径', 'FontSize', 14, 'FontWeight', 'bold');

for i = 1:size(nodes_3d, 1)
    scatter3(nodes_3d(i, 1), nodes_3d(i, 2), nodes_3d(i, 3), 100, 'filled');
    text(nodes_3d(i, 1), nodes_3d(i, 2), nodes_3d(i, 3) + 0.1, sprintf('%d', i), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

for i = 1:size(edges, 1)
    for j = 1:length(edges{i, 2})
        neighbor = edges{i, 2}(j);
        plot3([nodes_3d(i, 1), nodes_3d(neighbor, 1)], ...
              [nodes_3d(i, 2), nodes_3d(neighbor, 2)], ...
              [nodes_3d(i, 3), nodes_3d(neighbor, 3)], 'k--');
    end
end

path = best_paths{end};
for i = 1:length(path) - 1
    p1 = nodes_3d(path(i), :);
    p2 = nodes_3d(path(i + 1), :);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'r-', 'LineWidth', 2);
end
legend({'节点', '边', '最短路径'}, 'Location', 'best', 'FontSize', 10);
hold off;

%% 绘制路径规划性能评价图表
figure('Name', '路径规划性能', 'Color', 'w');
subplot(2, 1, 1);
plot(1:iter_max, best_distances, 'b-', 'LineWidth', 2);
hold on;
plot(1:iter_max, avg_distances, 'r--', 'LineWidth', 2);
legend('最短路径', '平均路径');
xlabel('迭代次数');
ylabel('距离');
title('路径长度随迭代次数变化', 'FontSize', 14);

subplot(2, 1, 2);
bar([min(best_distances), mean(avg_distances)]);
xticks(1:2);
xticklabels({'最佳路径长度', '平均路径长度'});
ylabel('距离');
title('路径性能评价', 'FontSize', 14);
grid on;
