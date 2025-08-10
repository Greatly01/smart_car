clc;
clear;
close all;

%% 参数设置
rows = 2000; % 迷宫的行数
cols = 2000; % 迷宫的列数

% 初始化性能数据记录
performance_data = []; % 用于记录 [迭代次数, 搜索节点数, 最短路径长度, 累计运行时间]

%% 迷宫生成与验证
maze = [];
while true
    % 初始化迷宫
    maze = ones(rows * 2 + 1, cols * 2 + 1, 'uint8'); % 使用uint8节省内存
    maze(2:2:end-1, 2:2:end-1) = 0; % 在内部设置通道（初始封闭）

    visited_cells = false(rows, cols); % 用于标记单元格是否被访问过
    stack = zeros(rows * cols, 2, 'uint32'); % 预分配栈的大小，使用uint32节省内存
    stack_top = 1;

    % 邻居相对坐标（转换为uint32类型）
    neighbors = int32([0, 1; 1, 0; 0, -1; -1, 0]); % [右, 下, 左, 上]

    % 随机起点
    start_row = randi(rows, 'uint32');
    start_col = randi(cols, 'uint32');
    stack(stack_top, :) = [start_row, start_col];
    visited_cells(start_row, start_col) = true;

    % 深度优先搜索生成迷宫
    while stack_top > 0
        current = stack(stack_top, :);
        row = current(1);
        col = current(2);

        % 找到未访问的邻居
        unvisited_neighbors = [];
        rand_order = randperm(4); % 随机顺序访问邻居
        for i = rand_order
            r = int32(row) + neighbors(i, 1); % 确保数据类型一致
            c = int32(col) + neighbors(i, 2);
            if r >= 1 && r <= rows && c >= 1 && c <= cols && ~visited_cells(r, c)
                unvisited_neighbors(end+1, :) = [r, c]; %#ok<AGROW>
            end
        end

        if ~isempty(unvisited_neighbors)
            next = unvisited_neighbors(1, :);
            next_row = next(1);
            next_col = next(2);

            % 移除墙壁
            wall_row = row * 2 + (next_row - row);
            wall_col = col * 2 + (next_col - col);
            maze(wall_row, wall_col) = 0;

            % 将下一个单元格标记为已访问并压入栈
            stack_top = stack_top + 1;
            stack(stack_top, :) = [next_row, next_col];
            visited_cells(next_row, next_col) = true;
        else
            % 回溯
            stack_top = stack_top - 1;
        end
    end

    % 验证起点和终点是否连通
    start_point = [2, 2]; % 起点坐标
    end_point = [rows * 2, cols * 2]; % 终点坐标
    if is_path_exist(maze, start_point, end_point)
        break;
    end
end

%% BFS算法路径规划
% 初始化队列
queue = zeros(numel(maze), 2, 'uint32'); % 预分配队列大小
head = 1;
tail = 1;
queue(tail, :) = start_point;
tail = tail + 1;

visited = false(size(maze));
visited(start_point(1), start_point(2)) = true;
parent = zeros(size(maze, 1), size(maze, 2), 2, 'int32'); % 用于记录路径

found = false;

% 路径规划开始
tic; % 开始计时
iteration_count = 0;
while head < tail
    iteration_count = iteration_count + 1;
    current_pos = queue(head, :);
    head = head + 1;

    if isequal(current_pos, end_point)
        found = true;
        break;
    end

    % 获取邻居节点
    for i = 1:4
        next_pos = int32(current_pos) + neighbors(i, :);
        if next_pos(1) > 0 && next_pos(1) <= size(maze, 1) && ...
                next_pos(2) > 0 && next_pos(2) <= size(maze, 2) && ...
                maze(next_pos(1), next_pos(2)) == 0 && ...
                ~visited(next_pos(1), next_pos(2))
            visited(next_pos(1), next_pos(2)) = true;
            parent(next_pos(1), next_pos(2), :) = current_pos;
            queue(tail, :) = next_pos;
            tail = tail + 1;
        end
    end

    % 记录性能数据
    elapsed_time = toc; % 运行时间
    performance_data = [performance_data; iteration_count, tail - 1, NaN, elapsed_time];
end

% 重建路径
if found
    best_path = end_point;
    current_pos = end_point;
    while ~isequal(current_pos, start_point)
        current_pos = squeeze(parent(current_pos(1), current_pos(2), :))';
        best_path = [current_pos; best_path];
    end
    best_length = size(best_path, 1);

    % 更新性能数据
    performance_data(end, 3) = best_length;
else
    disp('未找到路径');
    best_path = [];
    best_length = inf;
end

%% 绘制结果
figure;
imagesc(~maze);
colormap(gray);
axis equal;
axis off;
hold on;

% 绘制起点和终点
plot(start_point(2), start_point(1), 'go', 'MarkerSize', 6, 'LineWidth', 1);
plot(end_point(2), end_point(1), 'ro', 'MarkerSize', 6, 'LineWidth', 1);

% 绘制最优路径
if ~isempty(best_path)
    plot(best_path(:, 2), best_path(:, 1), 'b', 'LineWidth', 1);
end
title('BFS算法路径规划');

%% 绘制动态性能图表
if ~isempty(performance_data)
    figure;
    subplot(3, 1, 1);
    plot(performance_data(:, 1), performance_data(:, 2), '-o');
    xlabel('迭代次数');
    ylabel('搜索节点数');
    title('搜索节点数随迭代次数的变化');

    subplot(3, 1, 2);
    plot(performance_data(:, 1), performance_data(:, 4), '-o');
    xlabel('迭代次数');
    ylabel('累计运行时间 (秒)');
    title('累计运行时间随迭代次数的变化');

    subplot(3, 1, 3);
    plot(performance_data(:, 1), performance_data(:, 3), '-o');
    xlabel('迭代次数');
    ylabel('最短路径长度');
    title('最短路径长度随迭代次数的变化');
else
    disp('没有可用的性能数据，无法绘制性能图表。');
end

%% 辅助函数：检查起点和终点是否连通
function result = is_path_exist(maze, start_point, end_point)
    neighbors = int32([0, 1; 1, 0; 0, -1; -1, 0]); % 4个方向
    visited = false(size(maze));
    queue = zeros(numel(maze), 2, 'uint32'); % 预分配队列大小
    head = 1;
    tail = 1;
    queue(tail, :) = start_point;
    tail = tail + 1;
    visited(start_point(1), start_point(2)) = true;

    while head < tail
        current_pos = queue(head, :);
        head = head + 1;

        if isequal(current_pos, end_point)
            result = true;
            return;
        end

        % 获取邻居节点
        for i = 1:4
            next_pos = int32(current_pos) + neighbors(i, :);
            if next_pos(1) > 0 && next_pos(1) <= size(maze, 1) && ...
                    next_pos(2) > 0 && next_pos(2) <= size(maze, 2) && ...
                    maze(next_pos(1), next_pos(2)) == 0 && ...
                    ~visited(next_pos(1), next_pos(2))
                visited(next_pos(1), next_pos(2)) = true;
                queue(tail, :) = next_pos;
                tail = tail + 1;
            end
        end
    end

    result = false;
end
