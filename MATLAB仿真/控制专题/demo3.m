clc; clear; close all;

%% Step 输入
t = 0:0.01:5;
r = zeros(size(t));
r(t >= 0.2) = 1;  % 在 t=0.2s 时阶跃输入

%% 控制对象（二阶系统）
plant = tf(25, [1 10 25]);

%% PID 控制器参数（优化匹配图像）
Kp = 60;
Ki = 40;
Kd = 2;
PID = pid(Kp, Ki, Kd);

%% 构建整个系统结构（包含 PID 控制器 和 被控对象）
sys_open = series(PID, plant);
sys_closed = feedback(sys_open, 1);  % 闭环系统（单位负反馈）

%% 闭环系统输出响应（系统输出）
[y, t_out] = lsim(sys_closed, r, t);  % 用 lsim 接收自定义输入

%% 使用差分方式估算 PID 控制器输出
% e = r - y，u = PID(e)
e = r(:) - y(:);
u = zeros(size(e));
dt = t(2) - t(1);
integral = 0;

for k = 2:length(t)
    integral = integral + e(k) * dt;
    derivative = (e(k) - e(k-1)) / dt;
    u(k) = Kp * e(k) + Ki * integral + Kd * derivative;
end

%% 可视化输出
figure('Color', 'k');
plot(t, r, 'Color', [1, 0.5, 0], 'LineWidth', 1.5); hold on; % 橙色：输入
plot(t, u, 'y', 'LineWidth', 1.5);                          % 黄色：PID输出
plot(t, y, 'c', 'LineWidth', 1.5);                          % 蓝色：系统输出
legend('输入信号', 'PID输出', '系统响应', 'TextColor', 'w');
grid on;
title('22383211 张三', 'Color', 'w', 'FontSize', 14);
xlabel('时间 (s)', 'Color', 'w'); ylabel('幅值', 'Color', 'w');
ylim([-0.16096, 1.44862]);

set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
