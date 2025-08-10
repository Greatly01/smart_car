clc;
clear;
close all;

%% �Թ�����
% �Թ�����
rows = 20; % �Թ�������
cols = 30; % �Թ�������

% ��ʼ���Թ�
maze = ones(rows * 2 + 1, cols * 2 + 1, 'uint8'); % ʹ��uint8��ʡ�ڴ�
maze(2:2:end-1, 2:2:end-1) = 0; % ���ڲ�����ͨ������ʼ��գ�

visited_cells = false(rows, cols); % ���ڱ�ǵ�Ԫ���Ƿ񱻷��ʹ�
stack = zeros(rows * cols, 2, 'uint32'); % Ԥ����ջ�Ĵ�С��ʹ��uint32��ʡ�ڴ�
stack_top = 1;

% �ھ�������꣨ת��Ϊuint32���ͣ�
neighbors = int32([0, 1; 1, 0; 0, -1; -1, 0]); % [��, ��, ��, ��]

% ������
start_row = randi(rows, 'uint32');
start_col = randi(cols, 'uint32');
stack(stack_top, :) = [start_row, start_col];
visited_cells(start_row, start_col) = true;

% ����������������Թ�
while stack_top > 0
    % ��ȡ��ǰ��Ԫ��
    current = stack(stack_top, :);
    row = current(1);
    col = current(2);

    % �ҵ�δ���ʵ��ھ�
    unvisited_neighbors = [];
    rand_order = randperm(4); % ���˳������ھ�
    for i = rand_order
        r = int32(row) + neighbors(i, 1); % ȷ����������һ��
        c = int32(col) + neighbors(i, 2);
        if r >= 1 && r <= rows && c >= 1 && c <= cols && ~visited_cells(r, c)
            unvisited_neighbors(end+1, :) = [r, c]; %#ok<AGROW>
        end
    end

    if ~isempty(unvisited_neighbors)
        % ���ѡ��һ��δ���ʵ��ھ�
        next = unvisited_neighbors(1, :);
        next_row = next(1);
        next_col = next(2);

        % �Ƴ�ǽ��
        wall_row = row * 2 + (next_row - row);
        wall_col = col * 2 + (next_col - col);
        maze(wall_row, wall_col) = 0;

        % ����һ����Ԫ����Ϊ�ѷ��ʲ�ѹ��ջ
        stack_top = stack_top + 1;
        stack(stack_top, :) = [next_row, next_col];
        visited_cells(next_row, next_col) = true;
    else
        % ���û��δ���ʵ��ھӣ������
        stack_top = stack_top - 1;
    end
end

%% BFS�㷨·���滮
% ������յ�
start_point = [2, 2]; % ������꣨��maze�����е�λ�ã�
end_point = [rows * 2, cols * 2]; % �յ�����

% ��ʼ������
queue = zeros(numel(maze), 2, 'uint32'); % Ԥ������д�С
head = 1;
tail = 1;
queue(tail, :) = start_point;
tail = tail + 1;

% ���ʱ�Ǻ͸��ڵ��¼
visited = false(size(maze));
visited(start_point(1), start_point(2)) = true;
parent = zeros(size(maze, 1), size(maze, 2), 2, 'int32'); % ���ڼ�¼·��

found = false;

% BFS����
while head < tail
    current_pos = queue(head, :);
    head = head + 1;

    if isequal(current_pos, end_point)
        found = true;
        break;
    end

    % ��ȡ�ھӽڵ�
    for i = 1:4
        next_pos = int32(current_pos) + neighbors(i, :); % ������������һ��
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
end

% �ؽ�·��
if found
    best_path = end_point;
    current_pos = end_point;
    while ~isequal(current_pos, start_point)
        current_pos = squeeze(parent(current_pos(1), current_pos(2), :))';
        best_path = [current_pos; best_path];
    end
    best_length = size(best_path, 1);
else
    disp('δ�ҵ�·��');
    best_path = [];
    best_length = inf;
end

%% ���ƽ��
figure;
imagesc(~maze);
colormap(gray);
axis equal;
axis off;
hold on;

% ���������յ�
plot(start_point(2), start_point(1), 'go', 'MarkerSize', 6, 'LineWidth', 1);
plot(end_point(2), end_point(1), 'ro', 'MarkerSize', 6, 'LineWidth', 1);

% ��������·��
if ~isempty(best_path)
    plot(best_path(:, 2), best_path(:, 1), 'b', 'LineWidth', 1);
end
title('BFS�㷨·���滮');

%% ·�����۲���
disp('·�����۲���:');
disp(['���·������: ', num2str(best_length)]);
disp(['�����ڵ���: ', num2str(tail - 1)]);
