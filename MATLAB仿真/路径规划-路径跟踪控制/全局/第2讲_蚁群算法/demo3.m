clc;
clear;
close all;

%% ��������
rows = 2000; % �Թ�������
cols = 2000; % �Թ�������

% ��ʼ���������ݼ�¼
performance_data = []; % ���ڼ�¼ [��������, �����ڵ���, ���·������, �ۼ�����ʱ��]

%% �Թ���������֤
maze = [];
while true
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
            % ����
            stack_top = stack_top - 1;
        end
    end

    % ��֤�����յ��Ƿ���ͨ
    start_point = [2, 2]; % �������
    end_point = [rows * 2, cols * 2]; % �յ�����
    if is_path_exist(maze, start_point, end_point)
        break;
    end
end

%% BFS�㷨·���滮
% ��ʼ������
queue = zeros(numel(maze), 2, 'uint32'); % Ԥ������д�С
head = 1;
tail = 1;
queue(tail, :) = start_point;
tail = tail + 1;

visited = false(size(maze));
visited(start_point(1), start_point(2)) = true;
parent = zeros(size(maze, 1), size(maze, 2), 2, 'int32'); % ���ڼ�¼·��

found = false;

% ·���滮��ʼ
tic; % ��ʼ��ʱ
iteration_count = 0;
while head < tail
    iteration_count = iteration_count + 1;
    current_pos = queue(head, :);
    head = head + 1;

    if isequal(current_pos, end_point)
        found = true;
        break;
    end

    % ��ȡ�ھӽڵ�
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

    % ��¼��������
    elapsed_time = toc; % ����ʱ��
    performance_data = [performance_data; iteration_count, tail - 1, NaN, elapsed_time];
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

    % ������������
    performance_data(end, 3) = best_length;
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

%% ���ƶ�̬����ͼ��
if ~isempty(performance_data)
    figure;
    subplot(3, 1, 1);
    plot(performance_data(:, 1), performance_data(:, 2), '-o');
    xlabel('��������');
    ylabel('�����ڵ���');
    title('�����ڵ�������������ı仯');

    subplot(3, 1, 2);
    plot(performance_data(:, 1), performance_data(:, 4), '-o');
    xlabel('��������');
    ylabel('�ۼ�����ʱ�� (��)');
    title('�ۼ�����ʱ������������ı仯');

    subplot(3, 1, 3);
    plot(performance_data(:, 1), performance_data(:, 3), '-o');
    xlabel('��������');
    ylabel('���·������');
    title('���·����������������ı仯');
else
    disp('û�п��õ��������ݣ��޷���������ͼ��');
end

%% ������������������յ��Ƿ���ͨ
function result = is_path_exist(maze, start_point, end_point)
    neighbors = int32([0, 1; 1, 0; 0, -1; -1, 0]); % 4������
    visited = false(size(maze));
    queue = zeros(numel(maze), 2, 'uint32'); % Ԥ������д�С
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

        % ��ȡ�ھӽڵ�
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
