% 电机PID控制仿真
% 作者：
% 日期：

% 清空环境变量和命令窗口
clear all;
close all;
clc;

%% 电机参数定义
J = 0.01;   % 转动惯量，单位：kg*m^2
b = 0.1;    % 粘性摩擦系数，单位：N*m*s
K = 0.01;   % 电机转矩常数，单位：N*m/A
R = 1;      % 电阻，单位：Ohm
L = 0.5;    % 电感，单位：H

%% 建立电机的传递函数模型
s = tf('s');  % 拉普拉斯算子
% 电机的传递函数：P(s) = K / [ (J*s + b)*(L*s + R) + K^2 ]
P_motor = K / ( (J*s + b)*(L*s + R) + K^2 );

%% PID控制器设计
% PID参数，可以根据需要调整
Kp = 100;   % 比例系数
Ki = 200;   % 积分系数
Kd = 10;    % 微分系数

% 建立PID控制器的传递函数
C_pid = pid(Kp, Ki, Kd);

%% 闭环控制系统
% 构建闭环传递函数
sys_cl = feedback(C_pid * P_motor, 1);

%% 仿真设置
t = 0:0.001:2;  % 仿真时间，单位：秒

% 设定输入为单位阶跃信号
step_input = 1;

%% 仿真系统响应
% 计算系统的阶跃响应
[y, t] = step(sys_cl * step_input, t);

%% 绘制系统响应曲线
figure;
plot(t, y, 'b-', 'LineWidth', 2);
grid on;
title('电机在PID控制下的阶跃响应');
xlabel('时间 (秒)');
ylabel('转速 (rad/s)');
legend('系统响应');

%% 分析系统性能指标
% 计算超调量、上升时间、调节时间等
S = stepinfo(sys_cl * step_input, 'RiseTimeLimits', [0.1, 0.9]);

% 显示系统性能指标
fprintf('系统性能指标：\n');
fprintf('上升时间（从10%%到90%%）：%.4f 秒\n', S.RiseTime);
fprintf('超调量：%.2f%%\n', S.Overshoot);
fprintf('峰值时间：%.4f 秒\n', S.PeakTime);
fprintf('稳态值：%.4f\n', S.SettlingMin);

%% Bode图和根轨迹分析
% 绘制Bode图
figure;
bode(sys_cl);
grid on;
title('闭环系统的Bode图');

% 绘制根轨迹
figure;
rlocus(P_motor * C_pid);
grid on;
title('开环系统的根轨迹');

%% 结论
% 通过仿真，可以观察到电机在PID控制下的动态响应特性。调整PID参数，可以优化系统性能。
