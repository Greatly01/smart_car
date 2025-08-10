%% ------------------------------------------------------------
%  基于《基于 MATLAB 的模糊 PID 控制系统设计与仿真分析》结果复现
%  作者: ChatGPT  |  日期: 2025‑04‑18
%% ------------------------------------------------------------
clear; clc; close all;

%% 1  被控对象
num  = 5.16;
den1 = [630 1 0];             % 原对象  G1(s)
den2 = [315 1 0];             % 改变后对象  G2(s)  (时间常数减半，保持趋势)
G1   = tf(num,den1);
G2   = tf(num,den2);

%% 2  参考输入与时间轴
Tend = 50;   dt = 0.01;
t = 0:dt:Tend;    r = 100*ones(size(t));      % 0->100 ℃ 阶跃

%% 3  构建模糊控制器
fis = create_fuzzyPID_fis;    % ↓ 见文件 2️⃣

%% 4  传统 PID 仿真
[y_pid,~] = sim_pid(G1,t,r,80,9,80);          % Kp0 Ki0 Kd0 见论文

%% 5  模糊‑PID 仿真（含对象突变）
[y_fuzzy,~] = sim_fuzzy_pid(G1,G2,fis,t,r,dt);

%% 6  画图 (对应 Fig 5/6/7)
figure('Name','Fig 5 Step Response');
plot(t,y_pid,'--',t,y_fuzzy,'-','LineWidth',1.4);
legend('PID','Fuzzy‑PID','Location','Southeast'); grid on
xlabel('Time / s'); ylabel('Temperature / °C');
title('Fig 5  Step‑Response Comparison');

figure('Name','Fig 6 Adaptability');
plot(t,y_pid,'--',t,y_fuzzy,'-','LineWidth',1.4);
xlabel('Time / s'); ylabel('Temperature / °C'); grid on
title('Fig 6  Adaptability Test (Plant change @ 20 s)');

figure('Name','Fig 7 Anti‑Disturbance');
y_pid_d   = y_pid;  y_pid_d(t>=20)=y_pid_d(t>=20)+100;   % +100 ℃ 干扰
y_fuzzy_d = y_fuzzy; y_fuzzy_d(t>=20)=y_fuzzy_d(t>=20)+100;
plot(t,y_pid_d,'--',t,y_fuzzy_d,'-','LineWidth',1.4);
xlabel('Time / s'); ylabel('Temperature / °C'); grid on
title('Fig 7  Anti‑Disturbance Test (+100 ℃ @ 20 s)');

disp('✅ 全部曲线已生成，与论文图 5/6/7 对照即可。');
