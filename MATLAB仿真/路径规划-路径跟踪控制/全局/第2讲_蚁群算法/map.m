clc;
clear;
close all;

%% 迷宫参数
rows = 100; % 迷宫的行数
cols = 100; % 迷宫的列数

%% 初始化迷宫
maze = ones(rows * 2 + 1, cols * 2 + 1); % 创建一个全墙壁的迷宫
maze(2:2:end-1, 2:2:end-1) = 0;         % 在内部设置通道（初始封闭）

visited = false(rows, cols); % 用于标记单元格是否被访问过
stack = [];                  % 用于保存访问路径的栈

% 邻居相对坐标
neighbors = [0, 1; 1, 0; 0, -1; -1, 0]; % [下, 右, 上, 左]

%% 随机起点
start_row = randi(rows);
start_col = randi(cols);
stack = [start_row, start_col];
visited(start_row, start_col) = true;

%% 深度优先搜索生成迷宫
while ~isempty(stack)
    % 获取当前单元格
    current = stack(end, :);
    row = current(1);
    col = current(2);
    
    % 找到未访问的邻居
    unvisited_neighbors = [];
    for i = 1:size(neighbors, 1)
        r = row + neighbors(i, 1);
        c = col + neighbors(i, 2);
        if r >= 1 && r <= rows && c >= 1 && c <= cols && ~visited(r, c)
            unvisited_neighbors = [unvisited_neighbors; r, c];
        end
    end
    
    if ~isempty(unvisited_neighbors)
        % 随机选择一个未访问的邻居
        next = unvisited_neighbors(randi(size(unvisited_neighbors, 1)), :);
        next_row = next(1);
        next_col = next(2);
        
        % 移除墙壁
        wall_row = row * 2 + (next_row - row);
        wall_col = col * 2 + (next_col - col);
        maze(wall_row, wall_col) = 0;
        
        % 将下一个单元格标记为已访问并压入栈
        visited(next_row, next_col) = true;
        stack = [stack; next_row, next_col];
    else
        % 如果没有未访问的邻居，则回溯
        stack(end, :) = [];
    end
end

%% 绘制迷宫
figure('Name', '迷宫', 'Color', 'w');
imagesc(~maze); % 将迷宫显示为图像
colormap(gray); % 设置颜色映射为灰度
axis equal;
axis off;
title('生成的迷宫');

%% 添加起点和终点
start_point = [2, 2]; % 起点坐标
end_point = [rows * 2, cols * 2]; % 终点坐标
hold on;
plot(start_point(2), start_point(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % 起点标记为绿色
plot(end_point(2), end_point(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % 终点标记为红色
legend({'起点', '终点'}, 'Location', 'northeast');

