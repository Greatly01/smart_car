%% Robot Kinematics and Trajectory Planning using Robotics Toolbox
% 本程序利用Robotics Toolbox构建6自由度机器人（采用Modified DH参数），
% 计算正逆运动学，规划关节空间轨迹，并在三维图中显示末端点位置及对应关节角度。

clc; clear; close all;

%% 1. 添加Robotics Toolbox路径
addpath(genpath('C:\Users\mingh\Desktop\MATLAB仿真\控制专题\RVC1\rvctools')); 

%% 1. 构建机器人模型（MDH参数）
L(1) = Link('d', 360,  'a', 1200, 'alpha', pi/2,  'modified');
L(2) = Link('d', 220,  'a', 1165, 'alpha', 0,     'modified');
L(3) = Link('d', 250,  'a', 360,  'alpha', pi/2,  'modified');
L(4) = Link('d', 800,  'a', 380,  'alpha', -pi/2, 'modified');
L(5) = Link('d', 600,  'a', 620,  'alpha', pi/2,  'modified');
L(6) = Link('d', 800,  'a', 715,  'alpha', 0,     'modified');
robot = SerialLink(L, 'name', 'My6DOFRobot', 'manufacturer', 'Custom');

%% 2. 设置末端起点和终点位姿（TCP）
T_start = transl(1000, 500, 800) * trotz(pi/4);
T_end   = transl(1600, -200, 1000) * trotz(-pi/3);
q_start = robot.ikcon(T_start);  % 求起点逆解
q_end   = robot.ikcon(T_end);      % 求终点逆解

%% 3. 规划关节空间轨迹（多项式轨迹）
t = linspace(0, 5, 100);           % 0~5秒，共100个采样点
[q_traj, ~, ~] = jtraj(q_start, q_end, t);

%% 4. 预计算所有采样点末端位置，便于后续绘图
num_samples = size(q_traj, 1);
positions = zeros(num_samples, 3);
for i = 1:num_samples
    T = robot.fkine(q_traj(i,:));
    positions(i,:) = transl(T);
end

%% 5. 设置三维图，显示工作空间与轨迹
figure;
hold on; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('机器人运动轨迹与末端位姿采样');
axis([-2000 2000 -2000 2000 0 2000]);
view(3);

% 绘制初始配置（用于确定工作空间）
robot.plot(q_start, 'workspace', [-2000 2000 -2000 2000 0 2000]);

% 绘制完整末端轨迹：蓝色连线及红色散点
plot3(positions(:,1), positions(:,2), positions(:,3), 'b-', 'LineWidth', 2);
scatter3(positions(:,1), positions(:,2), positions(:,3), 20, 'r', 'filled');

%% 6. 动画演示：利用animate更新机器人姿态（速度更快）
for i = 1:num_samples
    robot.animate(q_traj(i,:));  % 更新机器人配置
    drawnow;                     % 强制刷新图形窗口
end

%% 7. 输出部分采样点末端位置及关节角度
disp('采样点末端位置与关节角（部分输出）：');
for i = 1:10:num_samples
    T = robot.fkine(q_traj(i,:));
    pos = transl(T);
    fprintf('Step %d: 末端位置: [%.2f, %.2f, %.2f]  |  关节角 (rad): %s\n', ...
            i, pos(1), pos(2), pos(3), mat2str(q_traj(i,:), 4));
end
















% clc; clear; close all;
% addpath(genpath('C:\Users\mingh\Desktop\MATLAB仿真\控制专题\RVC1\rvctools')); 
% 
% % ========== 1. 定义机器人结构 ==========
% L(1) = Link('d', 360,  'a', 1200, 'alpha', pi/2, 'modified');
% L(2) = Link('d', 220,  'a', 1165, 'alpha', 0,    'modified');
% L(3) = Link('d', 250,  'a', 360,  'alpha', pi/2, 'modified');
% L(4) = Link('d', 800,  'a', 380,  'alpha', -pi/2,'modified');
% L(5) = Link('d', 600,  'a', 620,  'alpha', pi/2, 'modified');
% L(6) = Link('d', 800,  'a', 715,  'alpha', 0,    'modified');
% 
% robot = SerialLink(L, 'name', 'My6DOFRobot', 'manufacturer', 'Custom');
% 
% % ========== 2. 设置起点终点（TCP） ==========
% T1 = transl(1000, 500, 800) * trotz(pi/4);  % 起点位姿
% T2 = transl(1600, -200, 1000) * trotz(-pi/3); % 终点位姿
% 
% % ========== 3. 求逆解得到关节角 ==========
% q1 = robot.ikcon(T1);
% q2 = robot.ikcon(T2);
% 
% % ========== 4. 使用多项式进行轨迹规划 ==========
% t = linspace(0, 5, 100);
% [q_traj, qd, qdd] = jtraj(q1, q2, t);
% 
% % ========== 5. 显示轨迹及末端点 ==========
% figure; hold on; grid on;
% robot.plot(q1, 'workspace', [-2000 2000 -2000 2000 0 2000]);
% title('机器人运动轨迹演示');
% 
% % 可视化运动轨迹
% for i = 1:length(t)
%     robot.plot(q_traj(i,:), 'delay', 0.05);
%     T = robot.fkine(q_traj(i,:));
%     plot3(T(1,4), T(2,4), T(3,4), 'r.');
% end
% 
% % ========== 6. 输出采样点末端位置与关节角 ==========
% disp('采样点末端位置与关节角：');
% for i = 1:10:length(t)
%     T = robot.fkine(q_traj(i,:));
%     fprintf('Step %d:\n', i);
%     disp('End-effector Position:');
%     disp(T.t');
%     disp('Joint Angles (rad):');
%     disp(q_traj(i,:));
% end
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% % clc; clear; close all;
% % 
% % %% ===== 1. 初始化工具箱 =====
% % try
% %     startup_rvc;
% % catch
% %     warning('Robotics Toolbox 未初始化，请运行 startup_rvc.m');
% % end
% % 
% % % 正确结构顺序建模
% % L(1) = Link('d', 360,  'a', 0,    'alpha', 0,     'modified');
% % L(2) = Link('d', 0,    'a', 1200, 'alpha', pi/2,  'modified');
% % L(3) = Link('d', 220,  'a', 1165, 'alpha', 0,     'modified');
% % L(4) = Link('d', 250,  'a', 360,  'alpha', pi/2,  'modified');
% % L(5) = Link('d', 800,  'a', 380,  'alpha', -pi/2, 'modified');
% % L(6) = Link('d', 600,  'a', 620,  'alpha', pi/2,  'modified');
% % L(7) = Link('d', 800,  'a', 715,  'alpha', 0,     'modified');
% % 
% % robot = SerialLink(L, 'name', 'Custom6R'); robot.tool = eye(4);
% % 
% % % ✅ 确保贴地连接
% % robot.base = transl(0, 0, -360);
% % 
% % 
% % %% ===== 3. 设置起止目标末端点 =====
% % T_start = transl(1000, 500, 800) * trotz(pi/4);
% % T_end   = transl(1600, -200, 1000) * trotz(-pi/3);
% % 
% % q_start = robot.ikcon(T_start);
% % q_end   = robot.ikcon(T_end);
% % 
% % if isempty(q_start) || isempty(q_end)
% %     error('逆解失败，路径无法规划');
% % end
% % 
% % %% ===== 4. 轨迹规划（关节空间） =====
% % n_points = 100; % 演示分辨率
% % [q_traj, ~, ~] = jtraj(q_start, q_end, n_points);
% % 
% % 
% % %% ===== 5. 终极优化：考虑整机结构范围的动态轨迹演示 =====
% % figure('Name','6R 动态轨迹 - 全结构可视'); clf;
% % set(gcf, 'Color', 'w'); ax = axes(); hold on; grid on; axis equal;
% % xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
% % view(45, 25);
% % title('6R 机器人动态轨迹演示 - 整机结构全程可视');
% % 
% % % 🧠 获取所有姿态下的机器人结构（每段连杆末端 + TCP）包络坐标点
% % all_points = [];
% % 
% % for i = 1:n_points
% %     q = q_traj(i,:);
% %     
% %     % 计算每个连杆末端位置
% %     for j = 1:robot.n
% %         T_j = robot.A(1:j, q);  % 连乘前 j 个变换得到第 j 连杆末端
% %         all_points = [all_points; T_j.t'];
% %     end
% %     
% %     % 加入末端执行器 TCP 位置
% %     T_tcp = robot.fkine(q);
% %     all_points = [all_points; T_tcp.t'];
% % end
% % 
% % 
% % % 🔍 获取真实最大边界并扩展安全边距
% % margin = 500;
% % x_range = [min(all_points(:,1)) - margin, max(all_points(:,1)) + margin];
% % y_range = [min(all_points(:,2)) - margin, max(all_points(:,2)) + margin];
% % z_range = [min(all_points(:,3)) - margin, max(all_points(:,3)) + margin];
% % 
% % axis([x_range, y_range, z_range]);
% % 
% % % ✅ 初始绘制一次机器人
% % robot.plot(q_traj(1,:), ...
% %     'workspace', [x_range, y_range, z_range], ...
% %     'scale', 0.25, ...
% %     'floorlevel', -1, 'noshadow', 'noname', 'nowrist', ...
% %     'delay', 0);
% % 
% % % 初始化轨迹记录
% % tcp_path = zeros(3, n_points);
% % path_line = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2);
% % point = plot3(NaN, NaN, NaN, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
% % label = text(0,0,0,'', 'FontSize',10, 'Color', [0.1 0.1 0.8]);
% % 
% % % 主循环
% % for i = 1:n_points
% %     q_now = q_traj(i,:);
% %     T_now = robot.fkine(q_now);
% %     pos = T_now.t;
% %     tcp_path(:,i) = pos;
% % 
% %     robot.animate(q_now);
% %     set(path_line, 'XData', tcp_path(1,1:i), 'YData', tcp_path(2,1:i), 'ZData', tcp_path(3,1:i));
% %     set(point, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
% % 
% %     set(label, 'Position', pos + [150; 150; 150], ...
% %                'String', sprintf('TCP: [%.0f, %.0f, %.0f]', pos));
% % 
% %     pause(0.05);
% % end
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % % clc; clear; close all;
% % % 
% % % %% ===== 1. 初始化工具箱路径（需提前配置好） =====
% % % try
% % %     startup_rvc;
% % % catch
% % %     warning('Robotics Toolbox 未初始化，请运行 startup_rvc.m');
% % % end
% % % 
% % % %% ===== 2. 创建机器人模型（MDH参数） =====
% % % L(1) = Link('d', 360,  'a', 1200, 'alpha', pi/2, 'modified');
% % % L(2) = Link('d', 220,  'a', 1165, 'alpha', 0,    'modified');
% % % L(3) = Link('d', 250,  'a', 360,  'alpha', pi/2, 'modified');
% % % L(4) = Link('d', 800,  'a', 380,  'alpha', -pi/2,'modified');
% % % L(5) = Link('d', 600,  'a', 620,  'alpha', pi/2, 'modified');
% % % L(6) = Link('d', 800,  'a', 715,  'alpha', 0,    'modified');
% % % robot = SerialLink(L, 'name', 'Custom6R'); robot.tool = eye(4);
% % % 
% % % %% ===== 3. 目标点设置 =====
% % % targets = {
% % %     transl(1000, 500, 800) * trotz(pi/4),
% % %     transl(1400, 200, 1200) * trotz(pi/6),
% % %     transl(1600, -200, 1000) * trotz(-pi/3)
% % % };
% % % 
% % % mode = 'joint';     % 轨迹类型：'joint' 或 'cartesian'
% % % n_step = 30;         % 每段轨迹点数，建议 30 提速
% % % gif_enable = false;  % 是否保存为 GIF 动画
% % % 
% % % %% ===== 4. 逆解一次性获取所有目标点 =====
% % % q_targets = zeros(length(targets), 6);
% % % for i = 1:length(targets)
% % %     q = robot.ikcon(targets{i});
% % %     if isempty(q)
% % %         error('第 %d 个目标点逆解失败！', i);
% % %     end
% % %     q_targets(i,:) = q;
% % % end
% % % 
% % % %% ===== 5. 插值轨迹生成 =====
% % % q_traj_all = [];
% % % tcp_traj_all = [];
% % % 
% % % for i = 1:size(q_targets,1)-1
% % %     q1 = q_targets(i,:);
% % %     q2 = q_targets(i+1,:);
% % %     
% % %     if strcmp(mode, 'joint')
% % %         q_traj = jtraj(q1, q2, n_step);
% % %     else
% % %         T1 = robot.fkine(q1);
% % %         T2 = robot.fkine(q2);
% % %         T_cart = ctraj(T1, T2, n_step);
% % %         q_traj = zeros(n_step,6);
% % %         for j = 1:n_step
% % %             q_traj(j,:) = robot.ikcon(T_cart(:,:,j));
% % %         end
% % %     end
% % % 
% % %     q_traj_all = [q_traj_all; q_traj];
% % % 
% % %     for j = 1:size(q_traj,1)
% % %         T = robot.fkine(q_traj(j,:));
% % %         tcp_traj_all = [tcp_traj_all; T.t'];
% % %     end
% % % end
% % % 
% % % %% ===== 6. 快速可视化动画（限制刷新） =====
% % % figure('Name','6R Robot Trajectory'); hold on; grid on;
% % % robot.plot(q_targets(1,:), 'workspace', [-2500 2500 -2500 2500 0 2500], 'delay', 0);
% % % xlabel('X'); ylabel('Y'); zlabel('Z'); title('Trajectory'); view(3);
% % % 
% % % tcp_path = tcp_traj_all';
% % % plot3(tcp_path(1,:), tcp_path(2,:), tcp_path(3,:), 'b-', 'LineWidth', 1.2);
% % % scatter3(tcp_path(1,1), tcp_path(2,1), tcp_path(3,1), 80, 'g', 'filled');
% % % scatter3(tcp_path(1,end), tcp_path(2,end), tcp_path(3,end), 80, 'm', 'filled');
% % % 
% % % % 控制绘图刷新（每10步刷新一次）
% % % for i = 1:10:size(q_traj_all,1)
% % %     robot.plot(q_traj_all(i,:), 'delay', 0.001);
% % % end
% % % legend('轨迹线', '起点', '终点');
% % % 
% % % %% ===== 7. 打印关键位置 & 导出轨迹 CSV =====
% % % fprintf('\n===== 每15步轨迹输出 =====\n');
% % % for i = 1:15:size(q_traj_all,1)
% % %     T = robot.fkine(q_traj_all(i,:));
% % %     fprintf('Step %d | TCP: [%.1f %.1f %.1f] | Joints: ', i, T.t);
% % %     fprintf('%.2f ', q_traj_all(i,:)); fprintf('\n');
% % % end
% % % 
% % % % 导出 CSV
% % % joint_deg = rad2deg(q_traj_all);
% % % csv_data = [joint_deg, tcp_traj_all];
% % % headers = {'q1','q2','q3','q4','q5','q6','x','y','z'};
% % % csv_name = 'trajectory_output.csv';
% % % fid = fopen(csv_name, 'w');
% % % fprintf(fid, '%s,', headers{1:end-1});
% % % fprintf(fid, '%s\n', headers{end}); fclose(fid);
% % % dlmwrite(csv_name, csv_data, '-append');
% % % fprintf('✅ 已保存轨迹到 CSV：%s\n', csv_name);
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 
% % % % clc; clear; close all;
% % % % 
% % % % %% ===== 1. 初始化 & 工具箱路径 =====
% % % % try
% % % %     startup_rvc;
% % % % catch
% % % %     warning('请确认 Robotics Toolbox 安装并执行 startup_rvc.m');
% % % % end
% % % % 
% % % % %% ===== 2. 构建 6R 机器人（MDH） =====
% % % % L(1) = Link('d', 360,  'a', 1200, 'alpha', pi/2, 'modified');
% % % % L(2) = Link('d', 220,  'a', 1165, 'alpha', 0,    'modified');
% % % % L(3) = Link('d', 250,  'a', 360,  'alpha', pi/2, 'modified');
% % % % L(4) = Link('d', 800,  'a', 380,  'alpha', -pi/2,'modified');
% % % % L(5) = Link('d', 600,  'a', 620,  'alpha', pi/2, 'modified');
% % % % L(6) = Link('d', 800,  'a', 715,  'alpha', 0,    'modified');
% % % % robot = SerialLink(L, 'name', 'Fast6R');
% % % % robot.tool = eye(4);
% % % % 
% % % % %% ===== 3. 目标末端位姿（可扩展多个） =====
% % % % targets = {
% % % %     transl(1000, 500, 800) * trotz(pi/4),
% % % %     transl(1400, 200, 1200) * trotz(pi/6),
% % % %     transl(1600, -200, 1000) * trotz(-pi/3)
% % % % };
% % % % 
% % % % % 模式选择：'joint' or 'cartesian'
% % % % mode = 'joint';
% % % % 
% % % % %% ===== 4. 求逆解并收集所有目标点 =====
% % % % q_targets = {}; % 使用 cell 数组避免出错
% % % % for i = 1:length(targets)
% % % %     q = robot.ikcon(targets{i});
% % % %     if isempty(q)
% % % %         error('第 %d 个目标点逆解失败，轨迹终止。', i);
% % % %     end
% % % %     q_targets{end+1} = q; % cell array 添加
% % % % end
% % % % 
% % % % if numel(q_targets) < 2
% % % %     error('目标点数量不足以生成轨迹（至少两个）');
% % % % end
% % % % 
% % % % %% ===== 5. 插值生成轨迹 =====
% % % % n_step = 50;
% % % % q_traj_all = [];
% % % % 
% % % % for i = 1:(length(q_targets)-1)
% % % %     q1 = q_targets{i};
% % % %     q2 = q_targets{i+1};
% % % %     
% % % %     if strcmp(mode, 'joint')
% % % %         q_segment = jtraj(q1, q2, n_step);
% % % %     else
% % % %         T1 = robot.fkine(q1);
% % % %         T2 = robot.fkine(q2);
% % % %         T_cart = ctraj(T1, T2, n_step);
% % % %         q_segment = zeros(n_step,6);
% % % %         for j = 1:n_step
% % % %             q_segment(j,:) = robot.ikcon(T_cart(:,:,j));
% % % %         end
% % % %     end
% % % %     
% % % %     q_traj_all = [q_traj_all; q_segment];
% % % % end
% % % % 
% % % % %% ===== 6. 可视化动画 =====
% % % % figure('Name','Fast 6R Robot Trajectory'); hold on; grid on;
% % % % robot.plot(q_targets{1}, 'workspace', [-2500 2500 -2500 2500 0 2500], 'delay', 0);
% % % % title('6R Robot - 快速轨迹可视化');
% % % % xlabel('X'); ylabel('Y'); zlabel('Z');
% % % % 
% % % % tcp_path = zeros(3, size(q_traj_all,1));
% % % % for i = 1:size(q_traj_all,1)
% % % %     robot.plot(q_traj_all(i,:), 'delay', 0.005);
% % % %     T_now = robot.fkine(q_traj_all(i,:));
% % % %     tcp_path(:,i) = T_now.t;
% % % %     if mod(i,5) == 0
% % % %         plot3(T_now(1,4), T_now(2,4), T_now(3,4), 'r.');
% % % %     end
% % % % end
% % % % 
% % % % % 补充轨迹线与起止点标记
% % % % plot3(tcp_path(1,:), tcp_path(2,:), tcp_path(3,:), 'b-', 'LineWidth', 1.2);
% % % % scatter3(tcp_path(1,1), tcp_path(2,1), tcp_path(3,1), 80, 'g', 'filled');
% % % % scatter3(tcp_path(1,end), tcp_path(2,end), tcp_path(3,end), 80, 'm', 'filled');
% % % % legend('TCP点','轨迹线','起点','终点');
% % % % 
% % % % %% ===== 7. 输出关键点轨迹信息 =====
% % % % fprintf('\n===== 轨迹关键点输出（每10步） =====\n');
% % % % for i = 1:10:size(q_traj_all,1)
% % % %     T = robot.fkine(q_traj_all(i,:));
% % % %     fprintf('Step %d | TCP: [%.1f %.1f %.1f] mm | Joints: [', i, T.t);
% % % %     fprintf('%.2f ', q_traj_all(i,:)); fprintf(']\n');
% % % % end
