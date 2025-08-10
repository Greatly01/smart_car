clc;
clear;
close all;

%% ��ά������������
% �ڵ����ά����
nodes_3d = [0 0 0; 2 0 0; 2 2 0; 0 2 0;   % ����
            1 1 1;                         % �м�ڵ�
            0 0 2; 2 0 2; 2 2 2; 0 2 2];  % ����

% �ڽӾ����壨·����Ȩ�أ�
edges = {1, [2, 4, 5], [2, 2, 1.5];   % �ڵ�1��2��4��5����
         2, [1, 3, 5], [2, 2, 1.5];
         3, [2, 4, 5, 8], [2, 2, 2.5, 3];
         4, [1, 3, 5], [2, 2, 1.5];
         5, [1, 2, 3, 4, 6, 7, 8, 9], [1.5, 1.5, 2.5, 1.5, 2, 2, 2, 2];
         6, [5, 7, 9], [2, 1.5, 1.5];
         7, [6, 5, 8], [1.5, 2, 1.5];
         8, [3, 5, 7, 9], [3, 2, 1.5, 2];
         9, [5, 6, 8], [2, 1.5, 2]};

start_node = 1; % ���
end_node = 9;   % �յ�

%% ��Ⱥ�㷨����
m = 30;                      % ��������
alpha = 1;                   % ��Ϣ����Ҫ������
beta = 5;                    % ����ʽ����
rho = 0.1;                   % ��Ϣ�ػӷ�����
Q = 1;                       % ��Ϣ�س���
iter_max = 100;              % ����������

% ��ʼ����Ϣ�غ�����ʽ����
for i = 1:length(edges)
    edges{i, 4} = ones(1, length(edges{i, 3}));   % ��Ϣ��
    edges{i, 5} = 1 ./ edges{i, 3};              % ����ʽ����
end

best_distances = zeros(iter_max, 1); % ÿ�����·������
avg_distances = zeros(iter_max, 1);  % ÿ��ƽ��·������
best_paths = cell(iter_max, 1);      % ÿ�����·��

%% ��Ⱥ�㷨����
for iter = 1:iter_max
    all_paths = cell(m, 1);          % ����ÿֻ���ϵ�·��
    all_distances = zeros(m, 1);     % ����ÿֻ���ϵ�·������

    for ant = 1:m
        current_node = start_node;
        path = current_node;
        total_distance = 0;

        while current_node ~= end_node
            neighbors = edges{current_node, 2};
            pheromone = edges{current_node, 4};
            heuristic = edges{current_node, 5};

            % ����ת�Ƹ���
            prob = (pheromone .^ alpha) .* (heuristic .^ beta);
            prob = prob / sum(prob);

            % ���̶ķ�ѡ����һ�ڵ�
            cumulative_prob = cumsum(prob);
            rand_val = rand;
            next_idx = find(cumulative_prob >= rand_val, 1);
            next_node = neighbors(next_idx);

            % ����·���;���
            path(end + 1) = next_node;
            total_distance = total_distance + edges{current_node, 3}(next_idx);
            current_node = next_node;
        end

        all_paths{ant} = path;
        all_distances(ant) = total_distance;
    end

    % ����ÿ������·��
    [best_distances(iter), best_ant] = min(all_distances);
    avg_distances(iter) = mean(all_distances);
    best_paths{iter} = all_paths{best_ant};

    % ������Ϣ��
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

%% ������ά��ͼ�����·��
figure('Name', '��ά��������', 'Color', 'w');
hold on;
grid on;
view(3);
xlabel('X', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z', 'FontSize', 12, 'FontWeight', 'bold');
title('��ά�������������·��', 'FontSize', 14, 'FontWeight', 'bold');

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
legend({'�ڵ�', '��', '���·��'}, 'Location', 'best', 'FontSize', 10);
hold off;

%% ����·���滮��������ͼ��
figure('Name', '·���滮����', 'Color', 'w');
subplot(2, 1, 1);
plot(1:iter_max, best_distances, 'b-', 'LineWidth', 2);
hold on;
plot(1:iter_max, avg_distances, 'r--', 'LineWidth', 2);
legend('���·��', 'ƽ��·��');
xlabel('��������');
ylabel('����');
title('·����������������仯', 'FontSize', 14);

subplot(2, 1, 2);
bar([min(best_distances), mean(avg_distances)]);
xticks(1:2);
xticklabels({'���·������', 'ƽ��·������'});
ylabel('����');
title('·����������', 'FontSize', 14);
grid on;
