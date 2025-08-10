% 生成测试数据（添加关节限制）
theta_orig = [-pi/2, pi/4, 0, pi/3, -pi/4, 0;  % 手动添加典型值
             rand(9,6)*2*pi - pi];            % 随机生成其他值

results = cell(10,5);
for i = 1:10
    % 正解计算
    [T, ~] = forward_kinematics(theta_orig(i,:));
    
    % 逆解计算（添加可控扰动）
    theta_init = theta_orig(i,:) + 0.1*randn(1,6);
    theta_inv = inverse_kinematics(T, theta_init);
    
    % 验证逆解正确性
    [T_verify, ~] = forward_kinematics(theta_inv');
    pos_error = norm(T(1:3,4) - T_verify(1:3,4));
    rot_error = acos((trace(T(1:3,1:3)'*T_verify(1:3,1:3)) - 1)/2); % 旋转角误差
    
    % 结果存储
    results{i,1} = theta_orig(i,:);
    results{i,2} = T;
    results{i,3} = theta_inv';
    results{i,4} = pos_error;
    results{i,5} = rot_error;
end

% 显示结果（改进格式）
disp('==================== 验证结果 ====================');
fprintf('组号 | 位置误差(m) | 旋转误差(rad)\n');
fprintf('-----|-------------|---------------\n');
for i = 1:10
    fprintf('%4d | %10.6f | %10.6f\n', i, results{i,4}, results{i,5});
end


function [T, Ts] = forward_kinematics(theta)
    alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
    a = [0, 0.2, 0.6, 0.1, 0, 0];
    d = [0.1, 0, 0, 0.5, 0, 0.1];
    theta_offset = zeros(1,6);
    
    Ts = zeros(4,4,6);
    T = eye(4);
    for i = 1:6
        ct = cos(theta(i)+theta_offset(i));
        st = sin(theta(i)+theta_offset(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        
        Ti = [ct -st*ca st*sa a(i)*ct;
              st ct*ca -ct*sa a(i)*st;
              0 sa ca d(i);
              0 0 0 1];
        T = T * Ti;
        Ts(:,:,i) = T;
    end
end


function theta = inverse_kinematics(T_target, theta_init, max_iter, tol)
    if nargin < 3, max_iter = 200; end
    if nargin < 4, tol = 1e-6; end
    
    theta = theta_init(:); % 确保初始值为列向量
    for iter = 1:max_iter
        [T_current, Ts] = forward_kinematics(theta');
        J = compute_jacobian(Ts);
        
        % 位置误差（3×1列向量）
        pos_err = T_target(1:3,4) - T_current(1:3,4);
        
        % 修正旋转误差计算（强制转为3×1列向量）
        R_current = T_current(1:3,1:3);
        R_target = T_target(1:3,1:3);
        R_err = R_target * R_current';
        
        axang = rotm2axang(R_err);
        orn_err = axang(1:3)' * axang(4); % 转置为列向量
        
        err = [pos_err; orn_err];
        if norm(err) < tol, break; end
        
        d_theta = pinv(J) * err * 0.5;
        theta = theta + d_theta;
        theta = wrapToPi(theta);
    end
end

function J = compute_jacobian(Ts)
    J = zeros(6,6);
    o_end = Ts(1:3,4,end);
    for i = 1:6
        Ti = Ts(:,:,i);
        z = Ti(1:3,3);
        o = Ti(1:3,4);
        J(1:3,i) = cross(z, o_end - o);
        J(4:6,i) = z;
    end
end

function theta = wrapToPi(theta)
    theta = mod(theta + pi, 2*pi) - pi;
end