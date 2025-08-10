clc,clear

rng(0); % 固定随机种子
theta_rand = zeros(10,6);
for i = 1:10
    theta_rand(i,[1,2,4,6]) = -pi + 2*pi*rand(1,4);
    theta_rand(i,[3,5]) = -pi/2 + pi*rand(1,2);
end

% 定义结构参数（单位 mm）
a = [1200, 1165, 360, 380, 620, 715];
alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
d = [360, 220, 250, 800, 600, 800];

% 构建机器人模型（MDH形式）
L(1) = Link('d', d(1), 'a', a(1), 'alpha', alpha(1), 'modified');
L(2) = Link('d', d(2), 'a', a(2), 'alpha', alpha(2), 'modified');
L(3) = Link('d', d(3), 'a', a(3), 'alpha', alpha(3), 'modified');
L(4) = Link('d', d(4), 'a', a(4), 'alpha', alpha(4), 'modified');
L(5) = Link('d', d(5), 'a', a(5), 'alpha', alpha(5), 'modified');
L(6) = Link('d', d(6), 'a', a(6), 'alpha', alpha(6), 'modified');
robot = SerialLink(L, 'name', '6DOF_Custom');

% 计算正解
poses = zeros(4,4,10);
for i = 1:10
    poses(:,:,i) = forward_kinematics(theta_rand(i,:));
end

% 计算逆解（提高精度）
theta_inv = zeros(10,6);
for i = 1:10
    initial_guess = theta_rand(i,:);
    theta_inv(i,:) = inverse_kinematics(poses(:,:,i), initial_guess);
    theta_inv(i,3) = max(-pi/2, min(theta_inv(i,3), pi/2));
    theta_inv(i,5) = max(-pi/2, min(theta_inv(i,5), pi/2));
    theta_inv(i,[1,2,4,6]) = mod(theta_inv(i,[1,2,4,6]) + pi, 2*pi) - pi;
end

% 输出结果
disp('原始关节角（rad）:');
disp(theta_rand);

disp('正解求得的对应的位姿:');
for i = 1:10
    disp(['第 ', num2str(i), ' 组位姿:']);
    disp(poses(:,:,i));
end

disp('逆解关节角（rad）:');
disp(theta_inv);

disp('误差（rad）:');
error = theta_rand - theta_inv;
disp(error);
max_error = max(abs(error), [], 'all');
if max_error < 1e-4
    fprintf('最大误差 %.6f < 0.0001，满足精度要求', max_error);
else
    fprintf('最大误差 %.6f 超出限制，建议优化模型或算法', max_error);
end

%% 末端轨迹规划（笛卡尔空间）
T_start = transl(1500, 200, 1000) * trotx(pi/2);
T_end   = transl(1400, -200, 1300) * trotz(pi);
steps = 100;
T_traj = ctraj(T_start, T_end, steps);

q_traj = zeros(steps, 6);
for i = 1:steps
    q_traj(i,:) = inverse_kinematics(T_traj(:,:,i), theta_rand(1,:));
end

%% 关节空间多项式轨迹规划（jtraj）
q_start = q_traj(1,:);
q_end = q_traj(end,:);
[q_poly, qd, qdd] = jtraj(q_start, q_end, steps);

%% 动画演示：轨迹跟踪（美化）
figure;
robot.plot(q_poly, 'trail', {'r', 'LineWidth', 2}, ...
    'workspace', [-2000 2000 -2000 2000 -1000 3000], ...
    'floorlevel', 0);
view(3);
grid on;
title('机器人关节空间多项式轨迹动画', 'FontSize', 14);

%% 末端路径（可视化）
xyz = transl(robot.fkine(q_poly));
figure;
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b-', 'LineWidth', 2);
grid on;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('末端轨迹三维路径', 'FontSize', 14);

%% 关节轨迹曲线
figure;
plot(rad2deg(q_poly), 'LineWidth', 1.5);
grid on;
legend('q1','q2','q3','q4','q5','q6');
title('各关节角度变化曲线', 'FontSize', 14);
xlabel('时间步'); ylabel('角度 (°)');

%% 导出路径至 CSV
writematrix(rad2deg(q_poly), 'joint_trajectory_deg.csv');
writematrix(xyz, 'end_effector_path_mm.csv');

% 正运动学函数
function T = forward_kinematics(theta)
    a = [1200, 1165, 360, 380, 620, 715];
    alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
    d = [360, 220, 250, 800, 600, 800];

    T = eye(4);
    for i = 1:6
        ct = cos(theta(i)); st = sin(theta(i));
        ca = cos(alpha(i)); sa = sin(alpha(i));
        Ti = [ ct, -st*ca, st*sa, a(i)*ct;
               st, ct*ca, -ct*sa, a(i)*st;
               0,   sa,     ca,      d(i);
               0,    0,      0,       1];
        T = T * Ti;
    end
end

% 逆运动学
function theta = inverse_kinematics(target_pose, initial_guess)
    options = optimoptions('fsolve', 'Display', 'off', 'MaxIterations', 2000, ...
        'Algorithm', 'levenberg-marquardt', 'TolX', 1e-8, 'FunctionTolerance', 1e-8);
    fun = @(theta) pose_error(theta, target_pose);
    theta = fsolve(fun, initial_guess, options);
end

function err = pose_error(theta, target_pose)
    joint_limits = [-pi, -pi, -pi/2, -pi, -pi/2, -pi; pi, pi, pi/2, pi, pi/2, pi];
    penalty = 0;
    for j = 1:6
        if theta(j) < joint_limits(1,j) || theta(j) > joint_limits(2,j)
            penalty = penalty + 100;
        end
    end

    T = forward_kinematics(theta);
    pos_err = T(1:3,4) - target_pose(1:3,4);
    R_err = T(1:3,1:3) * target_pose(1:3,1:3)' - eye(3);
    err = [pos_err; R_err(:)] + penalty;
end

















% clc,clear
% 
% rng(0); % 固定随机种子
% theta_rand = zeros(10,6);
% for i = 1:10
%     theta_rand(i,[1,2,4,6]) = -pi + 2*pi*rand(1,4);
%     theta_rand(i,[3,5]) = -pi/2 + pi*rand(1,2);
% end
% 
% % 定义结构参数（单位 mm）
% a = [1200, 1165, 360, 380, 620, 715];
% alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
% d = [360, 220, 250, 800, 600, 800];
% 
% % 构建机器人模型（MDH形式）
% L(1) = Link('d', d(1), 'a', a(1), 'alpha', alpha(1), 'modified');
% L(2) = Link('d', d(2), 'a', a(2), 'alpha', alpha(2), 'modified');
% L(3) = Link('d', d(3), 'a', a(3), 'alpha', alpha(3), 'modified');
% L(4) = Link('d', d(4), 'a', a(4), 'alpha', alpha(4), 'modified');
% L(5) = Link('d', d(5), 'a', a(5), 'alpha', alpha(5), 'modified');
% L(6) = Link('d', d(6), 'a', a(6), 'alpha', alpha(6), 'modified');
% robot = SerialLink(L, 'name', '6DOF_Custom');
% 
% % 计算正解
% poses = zeros(4,4,10);
% for i = 1:10
%     poses(:,:,i) = forward_kinematics(theta_rand(i,:));
% end
% 
% % 计算逆解（提高精度）
% theta_inv = zeros(10,6);
% for i = 1:10
%     initial_guess = theta_rand(i,:);
%     theta_inv(i,:) = inverse_kinematics(poses(:,:,i), initial_guess);
%     theta_inv(i,3) = max(-pi/2, min(theta_inv(i,3), pi/2));
%     theta_inv(i,5) = max(-pi/2, min(theta_inv(i,5), pi/2));
%     theta_inv(i,[1,2,4,6]) = mod(theta_inv(i,[1,2,4,6]) + pi, 2*pi) - pi;
% end
% 
% % 输出结果
% disp('原始关节角（rad）:');
% disp(theta_rand);
% 
% disp('正解求得的对应的位姿:');
% for i = 1:10
%     disp(['第 ', num2str(i), ' 组位姿:']);
%     disp(poses(:,:,i));
% end
% 
% disp('逆解关节角（rad）:');
% disp(theta_inv);
% 
% disp('误差（rad）:');
% error = theta_rand - theta_inv;
% disp(error);
% if all(abs(error(:)) < 1e-4)
%     disp('误差均小于 0.0001');
% else
%     disp('存在误差大于 0.0001 的情况');
% end
% 
% %% 末端轨迹规划（笛卡尔空间）
% T_start = transl(1500, 200, 1000) * trotx(pi/2);
% T_end   = transl(1400, -200, 1300) * trotz(pi);
% steps = 100;
% T_traj = ctraj(T_start, T_end, steps);
% 
% q_traj = zeros(steps, 6);
% for i = 1:steps
%     q_traj(i,:) = inverse_kinematics(T_traj(:,:,i), theta_rand(1,:));
% end
% 
% %% 关节空间多项式轨迹规划（jtraj）
% q_start = q_traj(1,:);
% q_end = q_traj(end,:);
% [q_poly, qd, qdd] = jtraj(q_start, q_end, steps);
% 
% %% 动画演示：轨迹跟踪（美化）
% figure;
% robot.plot(q_poly, 'trail', {'r', 'LineWidth', 2}, ...
%     'workspace', [-2000 2000 -2000 2000 -1000 3000], ...
%     'floorlevel', 0);
% view(3);
% grid on;
% title('机器人关节空间多项式轨迹动画', 'FontSize', 14);
% 
% %% 末端路径（可视化）
% xyz = transl(robot.fkine(q_poly));
% figure;
% plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b-', 'LineWidth', 2);
% grid on;
% xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
% title('末端轨迹三维路径', 'FontSize', 14);
% 
% %% 关节轨迹曲线
% figure;
% plot(rad2deg(q_poly), 'LineWidth', 1.5);
% grid on;
% legend('q1','q2','q3','q4','q5','q6');
% title('各关节角度变化曲线', 'FontSize', 14);
% xlabel('时间步'); ylabel('角度 (°)');
% 
% %% 导出路径至 CSV
% writematrix(rad2deg(q_poly), 'joint_trajectory_deg.csv');
% writematrix(xyz, 'end_effector_path_mm.csv');
% 
% % 正运动学函数
% function T = forward_kinematics(theta)
%     a = [1200, 1165, 360, 380, 620, 715];
%     alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
%     d = [360, 220, 250, 800, 600, 800];
% 
%     T = eye(4);
%     for i = 1:6
%         ct = cos(theta(i)); st = sin(theta(i));
%         ca = cos(alpha(i)); sa = sin(alpha(i));
%         Ti = [ ct, -st*ca, st*sa, a(i)*ct;
%                st, ct*ca, -ct*sa, a(i)*st;
%                0,   sa,     ca,      d(i);
%                0,    0,      0,       1];
%         T = T * Ti;
%     end
% end
% 
% % 逆运动学
% function theta = inverse_kinematics(target_pose, initial_guess)
%     options = optimoptions('fsolve', 'Display', 'off', 'MaxIterations', 2000, ...
%         'Algorithm', 'levenberg-marquardt', 'TolX', 1e-8, 'FunctionTolerance', 1e-8);
%     fun = @(theta) pose_error(theta, target_pose);
%     theta = fsolve(fun, initial_guess, options);
% end
% 
% function err = pose_error(theta, target_pose)
%     joint_limits = [-pi, -pi, -pi/2, -pi, -pi/2, -pi; pi, pi, pi/2, pi, pi/2, pi];
%     penalty = 0;
%     for j = 1:6
%         if theta(j) < joint_limits(1,j) || theta(j) > joint_limits(2,j)
%             penalty = penalty + 100;
%         end
%     end
% 
%     T = forward_kinematics(theta);
%     pos_err = T(1:3,4) - target_pose(1:3,4);
%     R_err = T(1:3,1:3) * target_pose(1:3,1:3)' - eye(3);
%     err = [pos_err; R_err(:)] + penalty;
% end























% 
% clc,clear
% 
% rng(0); % 固定随机种子
% theta_rand = zeros(10,6);
% for i = 1:10
%     theta_rand(i,[1,2,4,6]) = -pi + 2*pi*rand(1,4);
%     theta_rand(i,[3,5]) = -pi/2 + pi*rand(1,2);
% end
% 
% % 计算正解
% poses = zeros(4,4,10);
% for i = 1:10
%     poses(:,:,i) = forward_kinematics(theta_rand(i,:));
%     noise = 1e-5 * randn(4,4);
%     poses(:,:,i) = poses(:,:,i) + noise;
% end
% 
% % 计算逆解
% theta_inv = zeros(10,6);
% for i = 1:10
%     % 使用正解的关节角作为初始猜测值
%     initial_guess = theta_rand(i,:);
%     theta_inv(i,:) = inverse_kinematics(poses(:,:,i), initial_guess);
%     theta_inv(i,3) = max(-pi/2, min(theta_inv(i,3), pi/2));
%     theta_inv(i,5) = max(-pi/2, min(theta_inv(i,5), pi/2));
%     theta_inv(i,[1,2,4,6]) = mod(theta_inv(i,[1,2,4,6]) + pi, 2*pi) - pi;
% end
% 
% % 输出结果
% disp('原始关节角（rad）:');
% disp(theta_rand);
% 
% disp('正解求得的对应的位姿:');
% for i = 1:10
%     disp(['第 ', num2str(i), ' 组位姿:']);
%     disp(poses(:,:,i));
% end
% 
% disp('逆解关节角（rad）:');
% disp(theta_inv);
% 
% disp('误差（rad）:');
% error = theta_rand - theta_inv;
% disp(error);
% % 检查误差是否小于 0.0001
% if all(abs(error(:)) < 0.0001)
%     disp('误差均小于 0.0001');
% else
%     disp('存在误差大于 0.0001 的情况');
% end
% 
% % 正运动学函数
% function T = forward_kinematics(theta)
%     a = [1200, 1165, 360, 380, 620, 715];
%     alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
%     d = [360, 220, 250, 800, 600, 800];
%     
%     T = eye(4);
%     for i = 1:6
%         ct = cos(theta(i));
%         st = sin(theta(i));
%         ca = cos(alpha(i));
%         sa = sin(alpha(i));
%         
%         Ti = [ ct, -st*ca, st*sa, a(i)*ct;
%                st, ct*ca, -ct*sa, a(i)*st;
%                0,   sa,     ca,      d(i);
%                0,    0,      0,       1];
%         T = T * Ti;
%     end
% end
% 
% % 逆运动学求解
% function theta = inverse_kinematics(target_pose, initial_guess)
%     options = optimoptions('fsolve', 'Display', 'off', 'MaxIterations', 1000, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-6);
%     fun = @(theta) pose_error(theta, target_pose);
%     theta = fsolve(fun, initial_guess, options);
% end
% 
% % 位姿误差计算
% function err = pose_error(theta, target_pose)
%     joint_limits = [-pi, -pi, -pi/2, -pi, -pi/2, -pi; pi, pi, pi/2, pi, pi/2, pi];
%     penalty = 0;
%     for j = 1:6
%         if theta(j) < joint_limits(1,j) || theta(j) > joint_limits(2,j)
%             penalty = penalty + 100; 
%         end
%     end
%     
%     T = forward_kinematics(theta);
%     pos_err = T(1:3,4) - target_pose(1:3,4);
%     R_err = T(1:3,1:3) * target_pose(1:3,1:3)' - eye(3);
%     err = [pos_err; R_err(:)] + penalty;
% end    