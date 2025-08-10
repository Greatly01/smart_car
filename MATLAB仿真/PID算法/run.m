% 麦克纳姆轮机器人运动学仿真
% 作者：hm
% 日期: 11.15

clear; clc; close all;

% 机器人参数
R = 0.05;      % 轮子半径，单位：米
L = 0.2;       % 机器人长度的一半，单位：米
W = 0.15;      % 机器人宽度的一半，单位：米

% 时间参数
dt = 0.1;      % 仿真时间步长，单位：秒
t_total = 10;  % 仿真总时间，单位：秒
t = 0:dt:t_total;

% 初始化变量
num_steps = length(t);
x = zeros(1, num_steps);
y = zeros(1, num_steps);
theta = zeros(1, num_steps);

% 输入速度
v_x = 0.2;       % X方向线速度，单位：米/秒
v_y = 0.1;       % Y方向线速度，单位：米/秒
omega_z = pi/10; % 角速度，单位：弧度/秒

% 计算轮子转速
V = zeros(4, num_steps);
for i = 1:num_steps
    % 逆运动学计算轮子线速度
    V(:, i) = (1/R) * [1, -1, (L + W); 
                       1,  1, (L + W); 
                       1, -1, -(L + W); 
                       1,  1, -(L + W)] * [v_x; v_y; omega_z];
end

% 仿真机器人运动轨迹
for i = 2:num_steps
    % 正运动学计算机器人速度
    V_wheels = V(:, i-1);
    Vb = (R/4) * [1, 1, 1, 1; 
                  -1, 1, -1, 1; 
                  -1/(L + W), -1/(L + W), 1/(L + W), 1/(L + W)] * V_wheels;
    v_bx = Vb(1);
    v_by = Vb(2);
    omega_bz = Vb(3);

    % 将速度从机器人坐标系转换到全局坐标系
    v_wx = cos(theta(i-1)) * v_bx - sin(theta(i-1)) * v_by;
    v_wy = sin(theta(i-1)) * v_bx + cos(theta(i-1)) * v_by;

    % 更新位置和姿态
    x(i) = x(i-1) + v_wx * dt;
    y(i) = y(i-1) + v_wy * dt;
    theta(i) = theta(i-1) + omega_bz * dt;
end

% 绘制机器人运动轨迹
figure;
plot(x, y, 'b-', 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Mecanum Wheel Robot Trajectory');
grid on;
axis equal;

% 绘制机器人姿态变化
figure;
plot(t, theta, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Orientation \theta (rad)');
title('Robot Orientation Over Time');
grid on;

% 绘制各轮子速度
figure;
plot(t, V(1, :), 'r-', t, V(2, :), 'g-', t, V(3, :), 'b-', t, V(4, :), 'k-');
xlabel('Time (s)');
ylabel('Wheel Linear Velocity (m/s)');
title('Wheel Linear Velocities Over Time');
legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4');
grid on;
