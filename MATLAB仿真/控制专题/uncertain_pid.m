%%MATLAB script to optimize fuzzy PID controller to outperform classical PID

clc; clear; close all;

%% 被控对象传递函数 G(s) = 5.16 / (s*(630s+1))
G = tf(5.16, [630 1 0]);

%% 参数设定
T = 0.1; sim_time = 50;
n = round(sim_time / T) + 1;
time_vec = T * (0:n-1)';
input_signal = 100 * ones(n, 1);

%% 初始 PID 参数
Kp0 = 80; Ki0 = 1; Kd0 = 40; % 更合理的初始值
PID_base = pid(Kp0, Ki0, Kd0);
y_pid = lsim(feedback(PID_base * G, 1), input_signal, time_vec);

%% 模糊 PID 控制器初始化
y_fuzzy = zeros(n, 1); y_fuzzy(1) = 0;
kp_vec = zeros(n, 1); ki_vec = zeros(n, 1); kd_vec = zeros(n, 1);
e = zeros(n, 1); ec = zeros(n, 1);

for k = 2:n
    e(k) = input_signal(k) - y_fuzzy(k-1);
    ec(k) = e(k) - e(k-1);
    [dKp, dKi, dKd] = fuzzy_rule(e(k), ec(k));
    kp = max(1, min(500, Kp0 + dKp));
    ki = max(0, min(500, Ki0 + dKi));
    kd = max(0, min(500, Kd0 + dKd));
    kp_vec(k) = kp; ki_vec(k) = ki; kd_vec(k) = kd;
    PID_dyn = pid(kp, ki, kd);
    sys_cl = feedback(PID_dyn * G, 1);
    [y_tmp, ~] = lsim(sys_cl, input_signal(1:k), time_vec(1:k));
    y_fuzzy(k) = y_tmp(end);
end

%% 图5可视化与指标
figure;
plot(time_vec, y_pid, 'r--', 'LineWidth', 1.5); hold on;
plot(time_vec, y_fuzzy, 'g', 'LineWidth', 2);
title('图5：阶跃响应'); legend('模糊PID', '传统PID'); grid on;
saveas(gcf, 'fig5_step_response.png');

step_error_pid = abs(input_signal - y_pid);
step_error_fuzzy = abs(input_signal - y_fuzzy);
IAE_pid = trapz(time_vec, step_error_pid);
IAE_fuzzy = trapz(time_vec, step_error_fuzzy);
overshoot_pid = (max(y_pid) - 100) / 100 * 100;
overshoot_fuzzy = (max(y_fuzzy) - 100) / 100 * 100;
settling_time_pid = time_vec(find(abs(y_pid - 100) < 2, 1));
settling_time_fuzzy = time_vec(find(abs(y_fuzzy - 100) < 2, 1));

%% 图6参数变化
G2 = tf(5.16, [315 1 0]);
y_pid2 = y_pid; y_fuzzy2 = y_fuzzy;
for k = round(20/T)+1:n
    sys_pid = feedback(PID_base * G2, 1);
    [y_tmp, ~] = lsim(sys_pid, input_signal(1:k), time_vec(1:k));
    y_pid2(k) = y_tmp(end);
    kp = kp_vec(k); ki = ki_vec(k); kd = kd_vec(k);
    PID_dyn = pid(kp, ki, kd);
    sys_fuzzy = feedback(PID_dyn * G2, 1);
    [y_tmp2, ~] = lsim(sys_fuzzy, input_signal(1:k), time_vec(1:k));
    y_fuzzy2(k) = y_tmp2(end);
end

figure;
plot(time_vec, y_pid2, 'r--'); hold on;
plot(time_vec, y_fuzzy2, 'g');
title('图6：参数变化响应'); grid on;
saveas(gcf, 'fig6_param_change.png');

IAE_pid2 = trapz(time_vec, abs(input_signal - y_pid2));
IAE_fuzzy2 = trapz(time_vec, abs(input_signal - y_fuzzy2));

%% 图7扰动响应
input_dist = input_signal;
input_dist(time_vec >= 20) = 200;
y_pid3 = lsim(feedback(PID_base * G, 1), input_dist, time_vec);
y_fuzzy3 = zeros(n, 1); y_fuzzy3(1) = 0;

for k = 2:n
    e_k = input_dist(k) - y_fuzzy3(k-1);
    if k == 2
        ec_k = 0;
    else
        ec_k = e_k - (input_dist(k-1) - y_fuzzy3(k-2));
    end
    [dKp, dKi, dKd] = fuzzy_rule(e_k, ec_k);
    kp = max(1, min(500, Kp0 + dKp));
    ki = max(0, min(500, Ki0 + dKi));
    kd = max(0, min(500, Kd0 + dKd));
    PID_dyn = pid(kp, ki, kd);
    sys_cl = feedback(PID_dyn * G, 1);
    [y_tmp, ~] = lsim(sys_cl, input_dist(1:k), time_vec(1:k));
    y_fuzzy3(k) = y_tmp(end);
end

figure;
plot(time_vec, y_pid3, 'r--'); hold on;
plot(time_vec, y_fuzzy3, 'g');
title('图7：扰动响应'); grid on;
saveas(gcf, 'fig7_disturbance.png');

IAE_pid3 = trapz(time_vec, abs(200 - y_pid3));
IAE_fuzzy3 = trapz(time_vec, abs(200 - y_fuzzy3));

%% 输出对比表格
T_compare = table([IAE_pid; IAE_pid2; IAE_pid3], [IAE_fuzzy; IAE_fuzzy2; IAE_fuzzy3], ...
    [overshoot_pid; NaN; NaN], [overshoot_fuzzy; NaN; NaN], ...
    [settling_time_pid; NaN; NaN], [settling_time_fuzzy; NaN; NaN], ...
    'VariableNames', {'IAE_PID', 'IAE_Fuzzy', 'Overshoot_PID', 'Overshoot_Fuzzy', 'Ts_PID', 'Ts_Fuzzy'}, ...
    'RowNames', {'Step (图5)', 'Param Change (图6)', 'Disturbance (图7)'});

disp(T_compare)
writetable(T_compare, 'fuzzy_pid_compare.csv', 'WriteRowNames', true);

%% 模糊规则函数
function [dKp, dKi, dKd] = fuzzy_rule(e, ec)
    dKp = 0.6 * interp1([-100 -50 0 50 100], [100 60 0 -60 -100], bound(e, -100, 100), 'linear');
    dKi = 0.5 * interp1([-100 -50 0 50 100], [80 50 0 -50 -80], bound(e, -100, 100), 'linear');
    dKd = 0.3 * interp1([-50 -25 0 25 50], [60 30 0 -30 -60], bound(ec, -50, 50), 'linear');
end

function x_clipped = bound(x, xmin, xmax)
    x_clipped = min(max(x, xmin), xmax);
end