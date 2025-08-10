% 验证代码
% 生成10组随机关节角（单位：弧度）
rng(0); % 固定随机种子
theta_rand = zeros(10,6);
for i = 1:10
    % 生成其他四个关节的角度，范围是 -π 到 π
    theta_rand(i,[1,2,4,6]) = -pi + 2*pi*rand(1,4);
    % 生成关节 3 和 5 的角度，范围是 -π/2 到 π/2
    theta_rand(i,[3,5]) = -pi/2 + pi*rand(1,2);
end

% 计算正解
poses = zeros(4,4,10);
for i = 1:10
    poses(:,:,i) = forward_kinematics(theta_rand(i,:));
end

% 计算逆解
theta_inv = zeros(10,6);
for i = 1:10
    theta_inv(i,:) = inverse_kinematics(poses(:,:,i), [0,0,0,0,0,0]);
    % 修正关节 3 和 5 的角度，使其在 -π/2 到 π/2 范围内
    theta_inv(i,3) = max(-pi/2, min(theta_inv(i,3), pi/2));
    theta_inv(i,5) = max(-pi/2, min(theta_inv(i,5), pi/2));
    % 修正其他关节的角度，使其在 -π 到 π 范围内
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
disp(theta_rand - theta_inv);

% 正运动学函数
function T = forward_kinematics(theta)
    a = [1200, 1165, 360, 380, 620, 715];
    alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
    d = [360, 220, 250, 800, 600, 800];
    
    T = eye(4);
    for i = 1:6
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        
        Ti = [ ct, -st*ca, st*sa, a(i)*ct;
               st, ct*ca, -ct*sa, a(i)*st;
               0,   sa,     ca,      d(i);
               0,    0,      0,       1];
        T = T * Ti;
    end
end

% 逆运动学求解（需优化工具支持）
function theta = inverse_kinematics(target_pose, initial_guess)
    options = optimoptions('fsolve', 'Display', 'off');
    fun = @(theta) pose_error(theta, target_pose);
    theta = fsolve(fun, initial_guess, options);
end

% 位姿误差计算
function err = pose_error(theta, target_pose)
    T = forward_kinematics(theta);
    pos_err = T(1:3,4) - target_pose(1:3,4);
    R_err = T(1:3,1:3) * target_pose(1:3,1:3)' - eye(3);
    err = [pos_err; R_err(:)];
end    