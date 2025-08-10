clc
clear
close all

%% ͼ����
% ���ݽڵ���ڽ��ڵ����ĸ�ڵ�-���ֽڵ��Ӧ������ڵ�Ԫ������
nodes_dist = cell(0);
nodes_dist(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_dist(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_dist(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_dist(4,:) = {4, [3, 5], [3, 4]};
nodes_dist(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_dist(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_dist(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% �û�����
start_node = 4; % ��ʼ�ڵ�
end_node = 7;   % Ŀ��ڵ�

%% �㷨��ʼ��
% S���Ϻ�U����
S = [start_node, 0];
U(:,1) = setdiff(1:length(nodes_dist), start_node)'; % ��ʼU����
U(:,2) = inf(size(U,1), 1); % ��ʼ����Ϊ�����

% ��ʼ������ʼ�ڵ�ľ���
for i = 1:length(nodes_dist{start_node, 2})
    neighbor = nodes_dist{start_node, 2}(i);
    dist = nodes_dist{start_node, 3}(i);
    U(U(:,1) == neighbor, 2) = dist;
end

% ����·����ʼ��
path_opt = cell(length(nodes_dist),2);
path_opt(start_node,:) = {start_node, start_node};

%% ��¼����ʱ��
tic

%% ѭ������
while ~isempty(U)
    % ��U�������ҵ���ǰ��С����ֵ����Ӧ�ڵ�
    [dist_min, idx] = min(U(:,2));
    node_min = U(idx, 1);
    S(end+1,:) = [node_min, dist_min]; % ���ڵ����S����
    U(idx,:) = []; % ��U�����Ƴ�
    
    % ��������·������
    path_opt(node_min, :) = {node_min, [path_opt{S(end-1, 1), 2}, node_min]};
    
    % �����ýڵ���ڽӽڵ㣬����U�����еľ���
    for i = 1:length(nodes_dist{node_min, 2})
        neighbor = nodes_dist{node_min, 2}(i);
        dist = nodes_dist{node_min, 3}(i);
        
        % �ж��Ƿ���Ҫ����
        idx_neighbor = find(U(:,1) == neighbor);
        if ~isempty(idx_neighbor)
            new_dist = dist_min + dist;
            if new_dist < U(idx_neighbor, 2)
                U(idx_neighbor, 2) = new_dist; % ������̾���
                path_opt{neighbor, 2} = [path_opt{node_min, 2}, neighbor]; % ����·��
            end
        end
    end
end

%% ���
time_elapsed = toc; % ��¼�㷨����ʱ��
shortest_path = path_opt{end_node, 2}; % ���·��
shortest_distance = S(S(:,1) == end_node, 2); % ���·������

fprintf('���: %d\n', start_node);
fprintf('�յ�: %d\n', end_node);
fprintf('���·��: ');
disp(shortest_path);
fprintf('���·������: %.2f\n', shortest_distance);
fprintf('�㷨����ʱ��: %.6f��\n', time_elapsed);

%% ���ӻ�
figure;
subplot(2, 1, 1);
hold on;
% ���ƽڵ�
for i = 1:length(nodes_dist)
    scatter(i, 0, 100, 'filled');
    text(i, 0.2, sprintf('%d', i), 'HorizontalAlignment', 'center');
end
% ���Ʊ�
for i = 1:length(nodes_dist)
    for j = 1:length(nodes_dist{i, 2})
        neighbor = nodes_dist{i, 2}(j);
        plot([i, neighbor], [0, 0], 'k--');
    end
end
% �������·��
for i = 1:length(shortest_path)-1
    plot([shortest_path(i), shortest_path(i+1)], [0, 0], 'r-', 'LineWidth', 2);
end
title('ͼ�Ľڵ������·��');
xlabel('�ڵ�');
ylabel('����');
hold off;

% ����ָ���
subplot(2, 1, 2);
data = {'��ʼ�ڵ�', 'Ŀ��ڵ�', '���·������', '����ʱ�� (s)';
        start_node, end_node, shortest_distance, time_elapsed};
uitable('Data', data, 'ColumnName', {'ָ��', 'ֵ'}, ...
        'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.4]);
title('�㷨����ָ��');
