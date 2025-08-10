% 优化后的无人机六自由度PID控制仿真
% 包含位置与姿态控制、自适应PID控制、复杂环境建模以及传感器噪声与延迟
% 作者：ChatGPT
% 日期：2024-12-20

clear; clc; close all;

%% 仿真参数
dt = 0.01;                % 时间步长（秒）
t_final = 60;             % 仿真总时间（秒）
time = 0:dt:t_final;      % 时间向量
n = length(time);

%% 无人机参数
mass = 1.5;               % 无人机质量（kg）
g = 9.81;                 % 重力加速度（m/s^2）
I = diag([0.02, 0.02, 0.04]); % 无人机转动惯量（kg·m²）[roll, pitch, yaw]

%% 目标轨迹
x_target = 20 * sin(0.1 * time);     % 目标位置X（米）
y_target = 20 * cos(0.1 * time);     % 目标位置Y（米）
z_target = 10 + 5 * sin(0.05 * time);% 目标位置Z（米）

% 姿态目标
roll_target = zeros(1, n);    % 目标滚转角（弧度）
pitch_target = zeros(1, n);   % 目标俯仰角（弧度）
yaw_target = pi/4 * ones(1, n);% 目标偏航角（弧度）

% 将位置和姿态目标组合为3xN的数组
target_pos = [x_target; y_target; z_target];
target_att = [roll_target; pitch_target; yaw_target];

%% PID控制器参数初始化
% 位置PID参数 [X, Y, Z]
PID_pos.Kp = [2.0, 2.0, 4.0];
PID_pos.Ki = [0.5, 0.5, 1.0];
PID_pos.Kd = [1.0, 1.0, 2.0];

% 姿态PID参数 [roll, pitch, yaw]
PID_att.Kp = [8.0, 8.0, 4.0];
PID_att.Ki = [0.3, 0.3, 0.2];
PID_att.Kd = [2.0, 2.0, 1.0];

% 自适应调整参数
adapt_gain_pos = [0.05, 0.05, 0.1];    % 位置PID自适应增益
adapt_gain_att = [0.02, 0.02, 0.01];   % 姿态PID自适应增益

%% 初始化变量
% 状态变量
pos = zeros(3, n);        % 位置 [x; y; z]
vel = zeros(3, n);        % 速度 [vx; vy; vz]
acc = zeros(3, n);        % 加速度 [ax; ay; az] (控制输入)

% 姿态变量
att = zeros(3, n);        % 姿态角 [roll; pitch; yaw]
ang_vel = zeros(3, n);    % 角速度 [p; q; r]
ang_acc = zeros(3, n);    % 角加速度 [p_dot; q_dot; r_dot] (控制输入)

% 扰动
disturbance = zeros(3, n);        % 外部扰动力 [dx; dy; dz]
disturbance_att = zeros(3, n);    % 外部扰动力矩 [tau_roll; tau_pitch; tau_yaw]

% 环境建模：时变风场
wind_force = zeros(3, n);
for i = 1:n
    wind_force(:,i) = 2 * sin(0.05 * time(i)) * [1; 0; 0] + 1.5 * cos(0.03 * time(i)) * [0; 1; 0];
end

% 障碍物设置（球体）
obstacles = [
    15, 15, 10, 3;
    25, -10, 12, 4;
    5, 25, 8, 2.5
]; % 每行：[x, y, z, 半径]

% 传感器模型
sensor_delay = 0.1;                   % 传感器延迟（秒）
delay_steps = round(sensor_delay / dt);
sensor_noise_pos = 0.05;               % 位置传感器噪声标准差（米）
sensor_noise_att = 0.01;               % 姿态传感器噪声标准差（弧度）

% 初始化传感器延迟缓冲区
pos_buffer = zeros(3, delay_steps);
att_buffer = zeros(3, delay_steps);

% PID误差与积分项
error_pos = zeros(3, n);
integral_pos = zeros(3,1);
derivative_pos = zeros(3,1);

error_att = zeros(3, n);
integral_att = zeros(3,1);
derivative_att = zeros(3,1);

error_vel = zeros(3, n);
integral_vel = zeros(3,1);
derivative_vel = zeros(3,1);

error_ang_vel = zeros(3, n);
integral_ang_vel = zeros(3,1);
derivative_ang_vel = zeros(3,1);

% PID参数历史记录
PID_history.Kp_pos = zeros(3, n);
PID_history.Ki_pos = zeros(3, n);
PID_history.Kd_pos = zeros(3, n);

PID_history.Kp_att = zeros(3, n);
PID_history.Ki_att = zeros(3, n);
PID_history.Kd_att = zeros(3, n);

PID_history.Kp_vel = zeros(3, n);
PID_history.Ki_vel = zeros(3, n);
PID_history.Kd_vel = zeros(3, n);

PID_history.Kp_ang_vel = zeros(3, n);
PID_history.Ki_ang_vel = zeros(3, n);
PID_history.Kd_ang_vel = zeros(3, n);

% 初始化PID参数记录
PID_history.Kp_pos(:,1) = PID_pos.Kp';
PID_history.Ki_pos(:,1) = PID_pos.Ki';
PID_history.Kd_pos(:,1) = PID_pos.Kd';

PID_history.Kp_att(:,1) = PID_att.Kp';
PID_history.Ki_att(:,1) = PID_att.Ki';
PID_history.Kd_att(:,1) = PID_att.Kd';

PID_history.Kp_vel(:,1) = [1.5, 1.5, 1.5]';
PID_history.Ki_vel(:,1) = [0.3, 0.3, 0.3]';
PID_history.Kd_vel(:,1) = [0.7, 0.7, 0.7]';

PID_history.Kp_ang_vel(:,1) = [2.0, 2.0, 2.0]';
PID_history.Ki_ang_vel(:,1) = [0.2, 0.2, 0.2]';
PID_history.Kd_ang_vel(:,1) = [0.5, 0.5, 0.5]';

%% 仿真扰动设置
% 随机在任何时刻施加扰动
for i = 1:n
    if rand < 0.005 % 每个时间步有0.5%的概率施加扰动
        disturbance(:,i) = 3 * randn(3,1);          % 随机扰动力
        disturbance_att(:,i) = 0.5 * randn(3,1);    % 随机扰动力矩
    end
end

%% 仿真循环
for i = 2:n
    %% 传感器读取（含延迟与噪声）
    % 更新传感器缓冲区
    pos_buffer(:,mod(i-1, delay_steps)+1) = pos(:,i-1);
    att_buffer(:,mod(i-1, delay_steps)+1) = att(:,i-1);
    
    if i > delay_steps
        measured_pos = pos_buffer(:,mod(i-delay_steps-1, delay_steps)+1) + sensor_noise_pos * randn(3,1);
        measured_att = att_buffer(:,mod(i-delay_steps-1, delay_steps)+1) + sensor_noise_att * randn(3,1);
    else
        measured_pos = pos(:,i-1) + sensor_noise_pos * randn(3,1);
        measured_att = att(:,i-1) + sensor_noise_att * randn(3,1);
    end
    
    %% 位置控制器
    for axis = 1:3
        % 当前位置与目标位置误差
        error_pos(axis, i) = target_pos(axis, i) - measured_pos(axis);
        integral_pos(axis) = integral_pos(axis) + error_pos(axis, i) * dt;
        derivative_pos(axis) = (error_pos(axis, i) - error_pos(axis, i-1)) / dt;
        
        % 位置PID输出期望速度
        v_set = PID_pos.Kp(axis) * error_pos(axis, i) + ...
                PID_pos.Ki(axis) * integral_pos(axis) + ...
                PID_pos.Kd(axis) * derivative_pos(axis);
        
        % 速度误差
        error_vel(axis, i) = v_set - vel(axis, i-1);
        integral_vel(axis) = integral_vel(axis) + error_vel(axis, i) * dt;
        derivative_vel(axis) = (error_vel(axis, i) - error_vel(axis, i-1)) / dt;
        
        % 速度PID输出加速度（控制输入）
        acc(axis, i) = PID_pos.Kp(axis) * error_pos(axis, i) + ...
                      PID_pos.Ki(axis) * integral_pos(axis) + ...
                      PID_pos.Kd(axis) * derivative_pos(axis);
    end
    
    %% 姿态控制器
    for axis = 1:3
        % 当前姿态与目标姿态误差
        error_att(axis, i) = target_att(axis, i) - measured_att(axis);
        integral_att(axis) = integral_att(axis) + error_att(axis, i) * dt;
        derivative_att(axis) = (error_att(axis, i) - error_att(axis, i-1)) / dt;
        
        % 姿态PID输出期望角速度
        ang_vel_set = PID_att.Kp(axis) * error_att(axis, i) + ...
                      PID_att.Ki(axis) * integral_att(axis) + ...
                      PID_att.Kd(axis) * derivative_att(axis);
                  
        % 角速度误差
        error_ang_vel(axis, i) = ang_vel_set - ang_vel(axis, i-1);
        integral_ang_vel(axis) = integral_ang_vel(axis) + error_ang_vel(axis, i) * dt;
        derivative_ang_vel(axis) = (error_ang_vel(axis, i) - error_ang_vel(axis, i-1)) / dt;
        
        % 角速度PID输出角加速度（控制输入）
        ang_acc(axis, i) = PID_att.Kp(axis) * error_att(axis, i) + ...
                           PID_att.Ki(axis) * integral_att(axis) + ...
                           PID_att.Kd(axis) * derivative_att(axis);
    end
    
    %% 总控制输入
    % 位置控制输入（推力）
    F = mass * (acc(:,i) + [0; 0; g]) + wind_force(:,i) + disturbance(:,i);
    
    % 姿态控制输入（力矩）
    tau = I * (ang_acc(:,i) + disturbance_att(:,i));
    
    %% 动力学更新
    % 线性运动
    total_acc = F / mass + wind_force(:,i) / mass + disturbance(:,i) / mass;
    total_acc(3) = total_acc(3) - g;  % 考虑重力
    
    vel(:,i) = vel(:,i-1) + total_acc * dt;
    pos(:,i) = pos(:,i-1) + vel(:,i) * dt;
    
    % 姿态运动
    ang_acc_current = I \ (tau - cross(ang_vel(:,i-1), I * ang_vel(:,i-1)));
    ang_vel(:,i) = ang_vel(:,i-1) + ang_acc_current * dt;
    att(:,i) = att(:,i-1) + ang_vel(:,i) * dt;
    
    %% 自适应PID参数调整
    for axis = 1:3
        % 位置PID自适应
        if abs(error_pos(axis, i)) > 1.0
            PID_pos.Kp(axis) = PID_pos.Kp(axis) + adapt_gain_pos(axis) * dt;
            PID_pos.Ki(axis) = PID_pos.Ki(axis) + adapt_gain_pos(axis) * dt;
            PID_pos.Kd(axis) = PID_pos.Kd(axis) + adapt_gain_pos(axis) * dt;
        elseif abs(error_pos(axis, i)) < 0.3
            PID_pos.Kp(axis) = max(PID_pos.Kp(axis) - adapt_gain_pos(axis) * dt, 0);
            PID_pos.Ki(axis) = max(PID_pos.Ki(axis) - adapt_gain_pos(axis) * dt, 0);
            PID_pos.Kd(axis) = max(PID_pos.Kd(axis) - adapt_gain_pos(axis) * dt, 0);
        end
        
        % 姿态PID自适应
        if abs(error_att(axis, i)) > 0.2
            PID_att.Kp(axis) = PID_att.Kp(axis) + adapt_gain_att(axis) * dt;
            PID_att.Ki(axis) = PID_att.Ki(axis) + adapt_gain_att(axis) * dt;
            PID_att.Kd(axis) = PID_att.Kd(axis) + adapt_gain_att(axis) * dt;
        elseif abs(error_att(axis, i)) < 0.05
            PID_att.Kp(axis) = max(PID_att.Kp(axis) - adapt_gain_att(axis) * dt, 0);
            PID_att.Ki(axis) = max(PID_att.Ki(axis) - adapt_gain_att(axis) * dt, 0);
            PID_att.Kd(axis) = max(PID_att.Kd(axis) - adapt_gain_att(axis) * dt, 0);
        end
    end
    
    %% 记录PID参数
    PID_history.Kp_pos(:,i) = PID_pos.Kp';
    PID_history.Ki_pos(:,i) = PID_pos.Ki';
    PID_history.Kd_pos(:,i) = PID_pos.Kd';
    
    PID_history.Kp_att(:,i) = PID_att.Kp';
    PID_history.Ki_att(:,i) = PID_att.Ki';
    PID_history.Kd_att(:,i) = PID_att.Kd';
    
    PID_history.Kp_vel(:,i) = [1.5, 1.5, 1.5]';
    PID_history.Ki_vel(:,i) = [0.3, 0.3, 0.3]';
    PID_history.Kd_vel(:,i) = [0.7, 0.7, 0.7]';
    
    PID_history.Kp_ang_vel(:,i) = [2.0, 2.0, 2.0]';
    PID_history.Ki_ang_vel(:,i) = [0.2, 0.2, 0.2]';
    PID_history.Kd_ang_vel(:,i) = [0.5, 0.5, 0.5]';
    
    %% 障碍物检测与避障
    for obs = 1:size(obstacles,1)
        obs_pos = obstacles(obs,1:3)';
        obs_radius = obstacles(obs,4);
        distance = norm(pos(:,i) - obs_pos);
        if distance < (obs_radius + 1.0) % 预警距离
            % 简单避障策略：反向调整目标位置
            pos_error_vector = pos(:,i) - obs_pos;
            avoidance_force = 10 * (pos_error_vector / distance);
            F = F + avoidance_force;
            % 更新加速度
            acc(:,i) = acc(:,i) + avoidance_force / mass;
        end
    end
end

%% 动态展示设置
% 创建图形窗口
figure('Name','优化后的无人机六自由度PID控制仿真','NumberTitle','off');
set(gcf, 'Position', [100, 100, 1600, 900]);

% 创建3D动画子图
ax1 = subplot(2,3,1);
hold on;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('无人机三维位置与姿态动画');
axis equal;
xlim([min(target_pos(1,:))-5, max(target_pos(1,:))+5]);
ylim([min(target_pos(2,:))-15, max(target_pos(2,:))+15]);
zlim([0, max(target_pos(3,:))+10]);

% 绘制障碍物
for obs = 1:size(obstacles,1)
    [X, Y, Z] = sphere(20);
    X = obstacles(obs,4) * X + obstacles(obs,1);
    Y = obstacles(obs,4) * Y + obstacles(obs,2);
    Z = obstacles(obs,4) * Z + obstacles(obs,3);
    surf(X, Y, Z, 'FaceColor', 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

% 目标点
plot3(target_pos(1,1), target_pos(2,1), target_pos(3,1), 'g*', 'MarkerSize',10, 'DisplayName','目标点');

% 无人机模型（简化为一个带有姿态表示的箭头）
drone_plot = plot3(pos(1,1), pos(2,1), pos(3,1), 'bo', 'MarkerSize',8, 'MarkerFaceColor','b', 'DisplayName','无人机');
quiver_handle = quiver3(pos(1,1), pos(2,1), pos(3,1), ...
                        cos(att(3,1)) * cos(att(2,1)), ...
                        sin(att(3,1)) * cos(att(2,1)), ...
                        sin(att(2,1)), 'k', 'LineWidth',1.5, 'MaxHeadSize', 0.5);

% 创建位置轨迹
traj_plot = plot3(pos(1,1), pos(2,1), pos(3,1), 'b-', 'LineWidth',1.5, 'DisplayName','轨迹');

% 创建实时数据子图
ax2 = subplot(2,3,2);
hold on; grid on;
xlabel('时间 (s)');
ylabel('位置误差 (m)');
title('位置误差随时间变化');
colors = ['r','g','b'];
legend_entries = {'X误差','Y误差','Z误差'};
plot_handle_pos = zeros(3,1);
for axis =1:3
    plot_handle_pos(axis) = plot(time(1), error_pos(axis,1), [colors(axis) '-'], 'DisplayName',legend_entries{axis});
end
legend('show');

ax3 = subplot(2,3,3);
hold on; grid on;
xlabel('时间 (s)');
ylabel('姿态误差 (rad)');
title('姿态误差随时间变化');
legend_entries_att = {'Roll误差','Pitch误差','Yaw误差'};
plot_handle_att = zeros(3,1);
for axis =1:3
    plot_handle_att(axis) = plot(time(1), error_att(axis,1), [colors(axis) '-'], 'DisplayName',legend_entries_att{axis});
end
legend('show');

ax4 = subplot(2,3,4);
hold on; grid on;
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('无人机速度随时间变化');
legend_entries_vel = {'Vx','Vy','Vz'};
plot_handle_vel = zeros(3,1);
for axis =1:3
    plot_handle_vel(axis) = plot(time(1), vel(axis,1), [colors(axis) '-'], 'DisplayName',legend_entries_vel{axis});
end
legend('show');

ax5 = subplot(2,3,5);
hold on; grid on;
xlabel('时间 (s)');
ylabel('姿态角速度 (rad/s)');
title('姿态角速度随时间变化');
legend_entries_ang_vel = {'p','q','r'};
plot_handle_ang_vel = zeros(3,1);
for axis =1:3
    plot_handle_ang_vel(axis) = plot(time(1), ang_vel(axis,1), [colors(axis) '-'], 'DisplayName',legend_entries_ang_vel{axis});
end
legend('show');

ax6 = subplot(2,3,6);
hold on; grid on;
xlabel('时间 (s)');
ylabel('PID参数 Kp');
title('PID参数 Kp 随时间变化');
legend_entries_pid = {'Kp\_pos X','Kp\_pos Y','Kp\_pos Z'};
plot_handle_pid = zeros(3,1);
for axis =1:3
    plot_handle_pid(axis) = plot(time(1), PID_history.Kp_pos(axis,1), [colors(axis) '-'], 'DisplayName', legend_entries_pid{axis});
end
legend('show');

%% 动画循环
for i = 2:n
    %% 更新无人机位置
    set(drone_plot, 'XData', pos(1,i), 'YData', pos(2,i), 'ZData', pos(3,i));
    
    % 更新无人机姿态表示（箭头方向）
    quiver_dir = [cos(att(3,i)) * cos(att(2,i)), ...
                 sin(att(3,i)) * cos(att(2,i)), ...
                 sin(att(2,i))];
    set(quiver_handle, 'XData', pos(1,i), 'YData', pos(2,i), 'ZData', pos(3,i), ...
                     'UData', quiver_dir(1), 'VData', quiver_dir(2), 'WData', quiver_dir(3));
    
    % 更新轨迹
    set(traj_plot, 'XData', pos(1,1:i), 'YData', pos(2,1:i), 'ZData', pos(3,1:i));
    
    %% 更新位置误差图
    for axis =1:3
        set(plot_handle_pos(axis), 'XData', time(1:i), 'YData', error_pos(axis,1:i));
    end
    
    %% 更新姿态误差图
    for axis =1:3
        set(plot_handle_att(axis), 'XData', time(1:i), 'YData', error_att(axis,1:i));
    end
    
    %% 更新速度图
    for axis =1:3
        set(plot_handle_vel(axis), 'XData', time(1:i), 'YData', vel(axis,1:i));
    end
    
    %% 更新姿态角速度图
    for axis =1:3
        set(plot_handle_ang_vel(axis), 'XData', time(1:i), 'YData', ang_vel(axis,1:i));
    end
    
    %% 更新PID参数图
    for axis =1:3
        set(plot_handle_pid(axis), 'XData', time(1:i), 'YData', PID_history.Kp_pos(axis,1:i));
    end
    
    drawnow;
end

%% 静态绘图
figure('Name','优化后的无人机六自由度PID控制仿真 - 结果','NumberTitle','off');
set(gcf, 'Position', [100, 100, 1600, 900]);

% 位置误差
subplot(2,3,1);
hold on; grid on;
plot(time, error_pos(1,:), 'r', 'LineWidth',1.5);
plot(time, error_pos(2,:), 'g', 'LineWidth',1.5);
plot(time, error_pos(3,:), 'b', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('位置误差 (m)');
title('位置误差随时间变化');
legend('X误差','Y误差','Z误差');

% 姿态误差
subplot(2,3,2);
hold on; grid on;
plot(time, error_att(1,:), 'r', 'LineWidth',1.5);
plot(time, error_att(2,:), 'g', 'LineWidth',1.5);
plot(time, error_att(3,:), 'b', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('姿态误差 (rad)');
title('姿态误差随时间变化');
legend('Roll误差','Pitch误差','Yaw误差');

% 速度
subplot(2,3,3);
hold on; grid on;
plot(time, vel(1,:), 'r', 'LineWidth',1.5);
plot(time, vel(2,:), 'g', 'LineWidth',1.5);
plot(time, vel(3,:), 'b', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('无人机速度随时间变化');
legend('Vx','Vy','Vz');

% 姿态角速度
subplot(2,3,4);
hold on; grid on;
plot(time, ang_vel(1,:), 'r', 'LineWidth',1.5);
plot(time, ang_vel(2,:), 'g', 'LineWidth',1.5);
plot(time, ang_vel(3,:), 'b', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('姿态角速度 (rad/s)');
title('姿态角速度随时间变化');
legend('p','q','r');

% 控制输入加速度
subplot(2,3,5);
hold on; grid on;
plot(time, acc(1,:), 'r', 'LineWidth',1.5);
plot(time, acc(2,:), 'g', 'LineWidth',1.5);
plot(time, acc(3,:), 'b', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('控制加速度 (m/s^2)');
title('控制输入加速度随时间变化');
legend('Ax','Ay','Az');

% PID参数 Kp
subplot(2,3,6);
hold on; grid on;
plot(time, PID_history.Kp_pos(1,:), 'r', 'LineWidth',1.5);
plot(time, PID_history.Kp_pos(2,:), 'g', 'LineWidth',1.5);
plot(time, PID_history.Kp_pos(3,:), 'b', 'LineWidth',1.5);
xlabel('时间 (s)');
ylabel('PID Kp');
title('PID参数 Kp 随时间变化');
legend('Kp\_pos X','Kp\_pos Y','Kp\_pos Z');

%% 仿真结束提示
disp('仿真完成！');
