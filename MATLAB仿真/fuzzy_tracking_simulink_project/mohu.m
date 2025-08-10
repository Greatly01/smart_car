%% A2算法

%% fuzzy_pp_complex_demo.m 
clc; clear; close all;

%% === 用户参数 ===
T = 60; % 仿真时长 (s)
dt = 0.02; % 时间步长 (s)
N = round(T/dt);
V_MAX = 0.6; % 最高线速度 (m/s)
OMEGA_LIM = 2.5; % 最大角速度 (rad/s)

LOOK_BASE = 0.6; % 基础前瞻距离 (m)
SPEED_CUT = 0.3; % 误差减速阈值 (m)
slow_factor = 0.5; % 动画放慢倍数 (>=1)
refresh_step= 1; % 动画刷新帧步

%% === 复杂路径生成 ===
% 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
phi = linspace(0, 2*pi, N);
x_ref = 5*cos(phi) + 1.0*cos(3*phi);
y_ref = 5*sin(phi) + 0.5*sin(2*phi);

%% === 模糊增益 FIS 自动生成/加载 ===
fisFile = 'fuzzy_gain.fis';
if ~isfile(fisFile)
fis_g = mamfis('Name','γ_gain');
% 输入：横向误差 e (归一化到 ±1)
fis_g = addInput(fis_g,[-1 1],'Name','e');
fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N');
fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');
% 输入：航向误差 psi (归一化到 ±1)
fis_g = addInput(fis_g,[-1 1],'Name','psi');
fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N');
fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');
% 输出：增益 γ
fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small');
fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
% 规则表（9条）
rule = [
1 1 3 1 1;
1 2 3 1 1;
1 3 2 1 1;
2 1 3 1 1;
2 2 2 1 1;
2 3 1 1 1;
3 1 2 1 1;
3 2 1 1 1;
3 3 1 1 1 ];
fis_g = addRule(fis_g,rule);
writefis(fis_g,fisFile);
end
gain_fis = readfis(fisFile);

%% === 初始化状态 & 日志 ===
state.x = x_ref(1);
state.y = y_ref(1);
state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));

xL = zeros(1,N);
yL = zeros(1,N);
eL = zeros(1,N);
gL = zeros(1,N);
v_history = zeros(1, 5); % 记录最近5个速度值
kappa_history = zeros(1, 5); % 记录最近5个曲率值
e_lat_history = zeros(1, 3); % 记录最近3个侧向误差值
e_lat_integral = 0; % 侧向误差积分
e_lat_rate = 0; % 侧向误差变化率

%% === 动画准备 ===
win = 1.5; % 视窗半宽
figure('Name','Fuzzy PP Complex Demo'); hold on;
plot(x_ref, y_ref, 'k--','LineWidth',1.1);
anim = animatedline('Color','b','LineWidth',1.6);
errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
grid on; axis equal;

%% === 主循环：PurePursuit + 模糊增益 ===
prev_e = 0;
for k = 1:N
% 1) 最近点
dists = hypot(x_ref - state.x, y_ref - state.y);
[~, idx] = min(dists);

% 2) 误差计算
dx = x_ref(idx) - state.x;
dy = y_ref(idx) - state.y;
e_lat = sin(state.th)*dx - cos(state.th)*dy; % 侧向误差
psi_err= wrapToPi(atan2(dy,dx) - state.th); % 航向误差

% 计算误差变化率
e_lat_rate = (e_lat - prev_e) / dt;
prev_e = e_lat;

% 更新侧向误差历史
e_lat_history(1:end-1) = e_lat_history(2:end);
e_lat_history(end) = e_lat;

% 累积误差积分
e_lat_integral = e_lat_integral + e_lat * dt;

% 更精准的曲率计算（使用三点法）
if idx > 1 && idx < N - 1
x1 = x_ref(idx - 1);
y1 = y_ref(idx - 1);
x2 = x_ref(idx);
y2 = y_ref(idx);
x3 = x_ref(idx + 1);
y3 = y_ref(idx + 1);
A = hypot(x2 - x1, y2 - y1);
B = hypot(x3 - x2, y3 - y2);
C = hypot(x3 - x1, y3 - y1);
s = (A + B + C) / 2;
area = sqrt(s * (s - A) * (s - B) * (s - C));
if area > 1e-6
kappa = 4 * area / (A * B * C);
else
kappa = 0;
end
else
kappa = 0;
end

% 更新曲率历史
kappa_history(1:end-1) = kappa_history(2:end);
kappa_history(end) = kappa;

% 动态调整前瞻距离
curvature_factor = 1 - 0.6 * min(abs(kappa) / 10, 1); % 曲率越大，前瞻距离越小
error_factor = 1 - 0.2 * min(abs(e_lat) / SPEED_CUT, 1); % 误差越大，前瞻距离越小
integral_factor = min(abs(e_lat_integral) / 1, 1); % 积分越大，前瞻距离越小
rate_factor = 1 - 0.1 * min(abs(e_lat_rate) / 1, 1); % 误差变化率越大，前瞻距离越小
Ld = LOOK_BASE * curvature_factor * error_factor * (1 - 0.2 * integral_factor) * rate_factor;
Ld = max(0.2, Ld); % 确保前瞻距离不小于0.2m

dist_acc = 0; idx_l = idx;
while dist_acc < Ld && idx_l < N
seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
y_ref(idx_l+1)-y_ref(idx_l));
dist_acc = dist_acc + seg; idx_l = idx_l + 1;
end
xt = x_ref(idx_l); yt = y_ref(idx_l);

% 3) PurePursuit 曲率
alpha = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
% 避免除零错误
if Ld > 1e-6
kappa_pp = 2*sin(alpha) / Ld;
else
kappa_pp = 0;
end
omega_pp = V_MAX * kappa_pp;

% 4) 模糊增益 γ
% 归一化输入
e_n = max(min(e_lat/1.5, 1), -1);
psi_n = max(min(psi_err/pi, 1), -1);
gamma = evalfis([e_n, psi_n], gain_fis);
% 在弯道处根据曲率动态调整增益
if abs(kappa) > 0.2
gamma = gamma * (1 + 0.6 * min(abs(kappa) / 10, 1));
end
% 当误差较大或误差积分较大或误差变化率较大时，进一步增大增益
if abs(e_lat) > 0.5 || abs(e_lat_integral) > 1 || abs(e_lat_rate) > 0.5
gamma = gamma * 2.5;
end
omega = gamma * omega_pp;
omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);

% 5) 自适应线速度
% 考虑路径曲率、误差、误差积分、误差变化率调整速度
curvature_speed_factor = 1 - 0.9 * min(abs(kappa) / 10, 1); % 曲率越大，速度越小
error_speed_factor = 1 - 0.6 * min(abs(e_lat)/SPEED_CUT,1);
integral_speed_factor = 1 - 0.3 * min(abs(e_lat_integral) / 2, 1); % 误差积分对速度的影响
rate_speed_factor = 1 - 0.2 * min(abs(e_lat_rate) / 1, 1); % 误差变化率对速度的影响
speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
v = V_MAX * curvature_speed_factor * error_speed_factor * integral_speed_factor * rate_speed_factor * speed_history_factor;

% 更新速度历史
v_history(1:end-1) = v_history(2:end);
v_history(end) = v;

% 6) 状态更新
state.x = state.x + v*cos(state.th)*dt;
state.y = state.y + v*sin(state.th)*dt;
state.th = wrapToPi(state.th + omega*dt);

% 7) 日志记录
xL(k) = state.x;
yL(k) = state.y;
eL(k) = abs(e_lat);
gL(k) = gamma;

% 8) 动画更新
addpoints(anim, state.x, state.y);
if mod(k,refresh_step)==0
set(errTxt,'Position',[state.x,state.y+0.3], ...
'String',sprintf('e=%.3f m',eL(k)));
% 限制坐标轴范围，避免跳动过大
xlim([state.x-win, state.x+win]);
ylim([state.y-win, state.y+win]);
drawnow limitrate;
pause(dt*slow_factor);
end
end

%% === 性能评估 ===
RMSE = sqrt(mean(eL.^2));
MAXE = max(eL);
SS = mean(eL(end-round(2/dt):end));
INT = sum(eL)*dt;
fprintf('\n===== FuzzyPP Complex Demo 评估 =====\n');
fprintf('RMSE = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
fprintf('Max E = %.4f m\n', MAXE);
fprintf('SS Err = %.4f m\n', SS);
fprintf('Int E = %.4f m·s\n', INT);

%% === 静态结果展示 ===
figure('Name','Tracking Result');
subplot(2,2,1);
plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
axis equal; grid on; title('轨迹对比');

subplot(2,2,2);
plot((0:N-1)*dt,eL,'LineWidth',1.2);
grid on; ylabel('e (m)'); title('距离误差');

subplot(2,2,3);
plot((0:N-1)*dt,gL,'LineWidth',1.2);
grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');

subplot(2,2,4);
histogram(eL,30);
title('误差分布');




































% fuzzy_pp_demo.m
% % -------A2---------
% clc; clear; close all;
% 
% %% === 用户参数 ===
% T = 60; % 仿真时长 (s)
% dt = 0.04; % 步长 (s)
% N = round(T/dt);
% V_MAX = 0.6; % 最高线速度 (m/s)
% OMEGA_LIM = 2.5; % 角速度限幅 (rad/s)
% 
% LOOK_BASE = 0.8; % 基础前瞻距 (m)
% SPEED_CUT = 0.3; % 误差减速阈值 (m)
% 
% slow_factor = 0.5; % 动画放慢倍数 (>=1)
% refresh_step = 1; % 动画刷新帧步
% 
% %% === 轨迹类型选择 ===
% trajType = "random"; % "circle" or "random"
% switch trajType
% case "circle"
% r = 5;
% phi = linspace(0,2*pi,N);
% x_ref = r*cos(phi);
% y_ref = r*sin(phi);
% case "random"
% rng(42); numWp = 8; wpRadius = 4.5;
% ang = linspace(0,2*pi,numWp+1); ang(end) = [];
% wp_x = wpRadius*cos(ang) + 0.5*randn(1,numWp);
% wp_y = wpRadius*sin(ang) + 0.5*randn(1,numWp);
% wp_x(end+1) = wp_x(1); wp_y(end+1) = wp_y(1);
% t_wp = [0, cumsum(hypot(diff(wp_x), diff(wp_y)))];
% t_ref = linspace(0, t_wp(end), N);
% x_ref = interp1(t_wp, wp_x, t_ref, 'pchip');
% y_ref = interp1(t_wp, wp_y, t_ref, 'pchip');
% otherwise
% error('未知轨迹类型');
% end
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
% fis_g = mamfis('Name','γ_gain');
% % 输入 e
% fis_g = addInput(fis_g,[-1 1],'Name','e');
% fis_g = addMF(fis_g,'e','trimf',[-1 -1 -0.3],'Name','N');
% fis_g = addMF(fis_g,'e','trimf',[-0.5 0 0.5],'Name','Z');
% fis_g = addMF(fis_g,'e','trimf',[0.3 1 1],'Name','P');
% % 输入 psi
% fis_g = addInput(fis_g,[-1 1],'Name','psi');
% fis_g = addMF(fis_g,'psi','trimf',[-1 -1 -0.3],'Name','N');
% fis_g = addMF(fis_g,'psi','trimf',[-0.5 0 0.5],'Name','Z');
% fis_g = addMF(fis_g,'psi','trimf',[0.3 1 1],'Name','P');
% % 输出 γ
% fis_g = addOutput(fis_g,[0.2 1.8],'Name','g');
% fis_g = addMF(fis_g,'g','trimf',[0.2 0.4 0.8],'Name','Small');
% fis_g = addMF(fis_g,'g','trimf',[0.6 1.0 1.4],'Name','Medium');
% fis_g = addMF(fis_g,'g','trimf',[1.2 1.6 1.8],'Name','Big');
% % 规则
% rule = [
% 1 1 3 1 1;
% 1 2 3 1 1;
% 1 3 2 1 1;
% 2 1 3 1 1;
% 2 2 2 1 1;
% 2 3 1 1 1;
% 3 1 2 1 1;
% 3 2 1 1 1;
% 3 3 1 1 1 ];
% fis_g = addRule(fis_g, rule);
% writefis(fis_g, fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x = x_ref(1);
% state.y = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% 
% %% === 动画准备 ===
% win = 1.5; % 可视化窗口半宽
% figure('Name','Fuzzy PP Demo'); hold on; grid on; axis equal;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% 
% %% === 主循环：PurePursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
% % 找到最近点 idx
% dists = hypot(x_ref - state.x, y_ref - state.y);
% [~, idx] = min(dists);
% % 前瞻距离 Ld
% Ld = max(LOOK_BASE, 1.1*dists(idx));
% dist_acc = 0; idx_l = idx;
% while dist_acc < Ld && idx_l < N
% seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), y_ref(idx_l+1)-y_ref(idx_l));
% dist_acc = dist_acc + seg; idx_l = idx_l + 1;
% end
% xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
% % 横向误差 & 航向误差
% dx = xt - state.x; dy = yt - state.y;
% e_lat = sin(state.th)*dx - cos(state.th)*dy;
% psi_err = wrapToPi(atan2(dy, dx) - state.th);
% 
% % PurePursuit 曲率
% alpha = wrapToPi(atan2(dy,dx) - state.th);
% kappa = 2*sin(alpha) / Ld;
% omega_pp = V_MAX * kappa;
% 
% % 模糊增益 γ
% gamma = evalfis([e_lat/1.5, psi_err/pi], gain_fis);
% omega = gamma * omega_pp;
% omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
% % 自适应线速度
% v = V_MAX * (1 - 0.5 * min(abs(e_lat)/SPEED_CUT, 1));
% 
% % 状态更新
% state.x = state.x + v * cos(state.th) * dt;
% state.y = state.y + v * sin(state.th) * dt;
% state.th = wrapToPi(state.th + omega * dt);
% 
% % 记录日志
% xL(k) = state.x;
% yL(k) = state.y;
% eL(k) = abs(e_lat);
% gL(k) = gamma;
% 
% % 动画绘制
% addpoints(anim, state.x, state.y);
% if mod(k, refresh_step) == 0
% set(errTxt, 'Position', [state.x, state.y+0.3], 'String', sprintf('e=%.3f m', eL(k)));
% axis([state.x-win, state.x+win, state.y-win, state.y+win]);
% drawnow limitrate;
% pause(dt * slow_factor);
% end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS = mean(eL(end-round(2/dt):end));
% INT = sum(eL) * dt;
% fprintf('\n[FuzzyPP %s 评估]\n RMSE = %.3f m\n MaxE = %.3f m\n SS = %.3f m\n Int = %.3f m·s\n', ...
% trajType, RMSE, MAXE, SS, INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref, y_ref, 'k--', xL, yL, 'b','LineWidth',1.5);
% axis equal; grid on;
% title(['路径对比 - ', trajType]);
% 
% subplot(2,2,2);
% plot((0:N-1)*dt, eL, 'LineWidth',1.2);
% grid on; ylabel('e (m)');
% title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt, gL, 'LineWidth',1.2);
% grid on; ylabel('γ');
% title('模糊增益');
% xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');
% 
% 
% 
% 
% 






























% %% mohu.m: 纯模糊路径跟踪演示（无报错版）
% % 清理环境
% clc; clear; close all;
% 
% %% === 用户参数 ===
% T           = 60;        % 仿真时长 (s)
% dt          = 0.02;      % 时间步长 (s)
% N           = round(T/dt);
% V_MAX       = 0.6;       % 最大线速度 (m/s)
% OMEGA_MAX   = 2.5;       % 最大角速度 (rad/s)
% LOOK_AHEAD  = 0.5;       % 前瞻距离 (m)
% slow_factor = 0.1;       % 动画放慢倍数
% refresh_step= 1;         % 动画刷新步长
% 
% %% === 参考轨迹生成 ===
% phi   = linspace(0,2*pi,N);
% x_ref = 5*cos(phi) + cos(3*phi);
% y_ref = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 纯模糊角速度控制器构建 ===
% fis = mamfis('Name','OmegaControl');
% % 输入：角度误差 alpha ∈ [-π, π]
% fis = addInput(fis,[-pi pi],'Name','alpha');
% fis = addMF(fis,'alpha','trapmf',[-pi -pi -0.5 -0.1],'Name','NegLarge');
% fis = addMF(fis,'alpha','trimf',[-0.3 0 0.3],'Name','Zero');
% fis = addMF(fis,'alpha','trapmf',[0.1 0.5 pi pi],'Name','PosLarge');
% % 输出：角速度 ω ∈ [-OMEGA_MAX, OMEGA_MAX]
% fis = addOutput(fis,[-OMEGA_MAX OMEGA_MAX],'Name','omega');
% fis = addMF(fis,'omega','trimf',[-OMEGA_MAX -OMEGA_MAX/2 0],'Name','NL');
% fis = addMF(fis,'omega','trimf',[-OMEGA_MAX/4 0 OMEGA_MAX/4],'Name','Z');
% fis = addMF(fis,'omega','trimf',[0 OMEGA_MAX/2 OMEGA_MAX],'Name','PL');
% % 规则矩阵： [alphaMFIndex, omegaMFIndex, Weight, Operator]
% rules = [
%     1 3 1 1;  % alpha is NegLarge -> omega is PL
%     2 2 1 1;  % alpha is Zero    -> omega is Z
%     3 1 1 1;  % alpha is PosLarge-> omega is NL
% ];
% fis = addRule(fis, rules);
% 
% %% === 仿真初始化 ===
% x      = x_ref(1);
% y      = y_ref(1);
% theta  = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL  = zeros(1,N);
% yL  = zeros(1,N);
% eL  = zeros(1,N);
% omegaL = zeros(1,N);
% 
% %% === 动画准备 ===
% figure('Name','Pure Fuzzy Path Tracking');
% plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5); hold on;
% hPoint = plot(x, y, 'b.', 'MarkerSize', 12);
% axis equal; grid on;
% title('实时轨迹跟踪');
% legend('参考轨迹', '机器人位置');
% 
% %% === 仿真主循环 ===
% for k = 1:N
%     % 前瞻点索引
%     d2 = (x_ref - x).^2 + (y_ref - y).^2;
%     idxs = find(d2 >= LOOK_AHEAD^2, 1);
%     if isempty(idxs)
%         idx = N;
%     else
%         idx = idxs;
%     end
%     % 角度误差计算
%     alpha = atan2(y_ref(idx) - y, x_ref(idx) - x) - theta;
%     alpha = atan2(sin(alpha), cos(alpha));
%     % 距离误差
%     eL(k) = hypot(x_ref(k) - x, y_ref(k) - y);
%     % 模糊推理计算角速度
%     omega = evalfis(fis, alpha);
%     omega = max(min(omega, OMEGA_MAX), -OMEGA_MAX);
%     omegaL(k) = omega;
%     % 线速度采用误差驱动
%     v = V_MAX * (1 - exp(-eL(k)/LOOK_AHEAD));
%     % 状态更新
%     x     = x + v * cos(theta) * dt;
%     y     = y + v * sin(theta) * dt;
%     theta = theta + omega * dt;
%     xL(k) = x;
%     yL(k) = y;
%     % 动画刷新
%     if mod(k, refresh_step) == 0
%         set(hPoint, 'XData', xL(1:k), 'YData', yL(1:k));
%         drawnow;
%         pause(dt * slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL) * dt;
% fprintf('\n===== 纯模糊路径跟踪评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','跟踪结果分析');
% subplot(2,2,1);
% plot(x_ref, y_ref, 'k--', xL, yL, 'b', 'LineWidth', 1.5);
% axis equal; grid on;
% title('轨迹对比');
%  subplot(2,2,2);
% plot((0:N-1)*dt, eL, 'LineWidth', 1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% subplot(2,2,3);
% plot((0:N-1)*dt, omegaL, 'LineWidth', 1.2);
% grid on; ylabel('ω (rad/s)'); title('模糊角速度'); xlabel('t (s)');
% subplot(2,2,4);
% histogram(eL, 30);
% title('误差分布');
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
% 
% 
% 
% %% 无法跟踪
% % %% Fuzzy‑PP Complex Demo: 基于纯模糊控制的移动机器人路径跟踪（双输入优化版）
% % % 完整无报错的 MATLAB 脚本，使用角度误差与归一化距离误差的双输入模糊推理
% % 
% % %% === 用户参数 ===
% % T           = 60;        % 仿真时长 (s)
% % dt          = 0.02;      % 时间步长 (s)
% % N           = round(T/dt);
% % V_MAX       = 0.6;       % 最高线速度 (m/s)
% % OMEGA_LIM   = 2.5;       % 最大角速度 (rad/s)
% % 
% % LOOK_BASE   = 0.6;       % 基础前瞻距离 (m)
% % SPEED_CUT   = 0.3;       % 误差减速阈值 (m)
% % slow_factor = 0.5;       % 动画放慢倍数 (>=1)
% % refresh_step= 1;         % 动画刷新帧步
% % 
% % %% === 复杂路径生成 ===
% % phi    = linspace(0, 2*pi, N);
% % x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% % y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% % 
% % %% === 双输入模糊推理系统构建 ===
% % fis = mamfis('Name','FuzzyPP_2Input');
% % 
% % % 输入1: 角度误差 alpha ∈ [-π, π]
% % fis = addInput(fis,[-pi pi],'Name','alpha');
% % fis = addMF(fis,'alpha','trapmf',[-pi -pi -0.7 -0.2],'Name','Neg');
% % fis = addMF(fis,'alpha','gaussmf',[0.15 0],'Name','Zero');
% % fis = addMF(fis,'alpha','trapmf',[0.2 0.7 pi pi],'Name','Pos');
% % 
% % % 输入2: 归一化距离误差 e_norm ∈ [0,1]
% % fis = addInput(fis,[0 1],'Name','e_norm');
% % fis = addMF(fis,'e_norm','trimf',[0 0 0.3],'Name','Small');
% % fis = addMF(fis,'e_norm','trimf',[0.2 0.5 0.8],'Name','Medium');
% % fis = addMF(fis,'e_norm','trimf',[0.6 1 1],'Name','Large');
% % 
% % % 输出: 增益 γ ∈ [0,4]
% % fis = addOutput(fis,[0 4],'Name','gamma');
% % fis = addMF(fis,'gamma','trimf',[0   1    2],'Name','Small');
% % fis = addMF(fis,'gamma','trimf',[1    2    3],'Name','Medium');
% % fis = addMF(fis,'gamma','trimf',[2    3    4],'Name','Large');
% % 
% % % 定义规则：结合角度误差与距离误差
% % ruleList = [
% %     "If alpha is Zero  and e_norm is Small  then gamma is Small";
% %     "If alpha is Zero  and e_norm is Medium then gamma is Medium";
% %     "If alpha is Zero  and e_norm is Large  then gamma is Large";
% %     "If alpha is Neg   and e_norm is Small  then gamma is Medium";
% %     "If alpha is Neg   and e_norm is Medium then gamma is Large";
% %     "If alpha is Neg   and e_norm is Large  then gamma is Large";
% %     "If alpha is Pos   and e_norm is Small  then gamma is Medium";
% %     "If alpha is Pos   and e_norm is Medium then gamma is Large";
% %     "If alpha is Pos   and e_norm is Large  then gamma is Large"
% % ];
% % fis = addRule(fis, ruleList);
% % 
% % %% === 仿真初始化 ===
% % x   = x_ref(1);
% % y   = y_ref(1);
% % theta = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% % 
% % xL  = zeros(1, N);
% % yL  = zeros(1, N);
% % eL  = zeros(1, N);
% % gL  = zeros(1, N);
% % 
% % figure('Name','Fuzzy-PP 2-Input Demo');
% % hRef = plot(x_ref, y_ref, 'k--','LineWidth',1.5); hold on;
% % hTraj = plot(x, y, 'b.','MarkerSize',10);
% % axis equal; grid on; title('实时轨迹动画');
% % legend('参考轨迹','机器人位置');
% % 
% % %% === 仿真主循环 ===
% % for k = 1:N
% %     % 前瞻点索引
% %     d2 = (x_ref - x).^2 + (y_ref - y).^2;
% %     inds = find(d2 >= LOOK_BASE^2);
% %     if isempty(inds)
% %         idx = N;
% %     else
% %         idx = inds(1);
% %     end
% %     
% %     % 计算角度误差 alpha
% %     alpha = atan2(y_ref(idx)-y, x_ref(idx)-x) - theta;
% %     alpha = atan2(sin(alpha), cos(alpha));
% %     
% %     % 记录跟踪误差
% %     eL(k) = sqrt((x_ref(k)-x)^2 + (y_ref(k)-y)^2);
% %     % 归一化距离误差
% %     e_norm = min(eL(k)/LOOK_BASE, 1);
% %     
% %     % 模糊推理计算增益 γ
% %     g = evalfis(fis, [alpha, e_norm]);
% %     gL(k) = g;
% %     
% %     % 线速度 v 与角速度 ω
% %     if eL(k) > SPEED_CUT
% %         v = V_MAX;
% %     else
% %         v = V_MAX * eL(k)/SPEED_CUT;
% %     end
% %     omega = g * alpha;
% %     
% %     % 限幅
% %     v     = max(min(v, V_MAX), -V_MAX);
% %     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% %     
% %     % 状态更新
% %     x     = x + v * cos(theta) * dt;
% %     y     = y + v * sin(theta) * dt;
% %     theta = theta + omega * dt;
% %     
% %     % 记录轨迹
% %     xL(k) = x;
% %     yL(k) = y;
% %     
% %     % 动画刷新
% %     if mod(k, refresh_step) == 0
% %         set(hTraj, 'XData', xL(1:k), 'YData', yL(1:k));
% %         drawnow;
% %         pause(dt * slow_factor);
% %     end
% % end
% % 
% % %% === 性能评估 ===
% % RMSE = sqrt(mean(eL.^2));
% % MAXE = max(eL);
% % SS   = mean(eL(end-round(2/dt):end));
% % INT  = sum(eL) * dt;
% % fprintf('\n===== Fuzzy‑PP 2-Input Demo 评估 =====\n');
% % fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% % fprintf('Max E  = %.4f m\n', MAXE);
% % fprintf('SS Err = %.4f m\n', SS);
% % fprintf('Int E  = %.4f m·s\n', INT);
% % 
% % %% === 静态结果展示 ===
% % figure('Name','Tracking Result');
% % subplot(2,2,1);
% % plot(x_ref, y_ref, 'k--', xL, yL, 'b','LineWidth',1.5);
% % axis equal; grid on; title('轨迹对比');
% % 
% % subplot(2,2,2);
% % plot((0:N-1)*dt, eL, 'LineWidth',1.2);
% % grid on; ylabel('e (m)'); title('距离误差');
% % 
% % subplot(2,2,3);
% % plot((0:N-1)*dt, gL, 'LineWidth',1.2);
% % grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% % 
% % subplot(2,2,4);
% % histogram(eL,30);
% % title('误差分布');
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
% 
% 
% 
% 
% %% 无法跟踪
% % %% Fuzzy‑PP Complex Demo: 基于纯模糊控制的移动机器人路径跟踪
% % 完整无报错的 MATLAB 脚本
% 
% %% === 用户参数 ===
% T           = 60;        % 仿真时长 (s)
% dt          = 0.02;      % 时间步长 (s)
% N           = round(T/dt);
% V_MAX       = 0.6;       % 最高线速度 (m/s)
% OMEGA_LIM   = 2.5;       % 最大角速度 (rad/s)
% 
% LOOK_BASE   = 0.6;       % 基础前瞻距离 (m)
% SPEED_CUT   = 0.3;       % 误差减速阈值 (m)
% slow_factor = 0.5;       % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊推理系统构建 ===
% fis = mamfis('Name','FuzzyPPDemo');
% 
% % 输入变量 alpha ∈ [-π, π]
% fis = addInput(fis,[-pi pi],'Name','alpha');
% fis = addMF(fis,'alpha','trapmf',[-pi -pi -0.5 0],'Name','Neg');
% fis = addMF(fis,'alpha','gaussmf',[0.5 0],'Name','Zero');
% fis = addMF(fis,'alpha','trapmf',[0 0.5 pi pi],'Name','Pos');
% 
% % 输出变量 gamma ∈ [0, 2]
% fis = addOutput(fis,[0 2],'Name','gamma');
% fis = addMF(fis,'gamma','trimf',[0   0.5 1],'Name','Small');
% fis = addMF(fis,'gamma','trimf',[0.5 1.0 1.5],'Name','Medium');
% fis = addMF(fis,'gamma','trimf',[1.0 1.5 2],'Name','Large');
% 
% % 规则：负误差/正误差使用较大增益，近零误差使用较小增益
% ruleList = [
%     "If alpha is Neg  then gamma is Large";
%     "If alpha is Zero then gamma is Small";
%     "If alpha is Pos  then gamma is Large"
% ];
% fis = addRule(fis, ruleList);
% 
% %% === 仿真初始化 ===
% x   = x_ref(1);
% y   = y_ref(1);
% theta = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL  = zeros(1, N);
% yL  = zeros(1, N);
% eL  = zeros(1, N);
% gL  = zeros(1, N);
% 
% % 动画设置
% figure('Name','Fuzzy-PP Complex Demo');
% hRef = plot(x_ref, y_ref, 'k--','LineWidth',1.5); hold on;
% hTraj = plot(x, y, 'b.','MarkerSize',10);
% axis equal; grid on; title('实时轨迹动画');
% legend('参考轨迹','机器人位置');
% 
% %% === 仿真主循环 ===
% for k = 1:N
%     % 前瞻点索引
%     d2 = (x_ref - x).^2 + (y_ref - y).^2;
%     inds = find(d2 >= LOOK_BASE^2);
%     if isempty(inds)
%         idx = N;
%     else
%         idx = inds(1);
%     end
%     
%     % 误差与角度计算
%     alpha = atan2(y_ref(idx)-y, x_ref(idx)-x) - theta;
%     alpha = atan2(sin(alpha), cos(alpha));
%     eL(k) = sqrt((x_ref(k)-x)^2 + (y_ref(k)-y)^2);
%     
%     % 模糊增益计算
%     g = evalfis(fis, alpha);
%     gL(k) = g;
%     
%     % 线速度 v
%     if eL(k) > SPEED_CUT
%         v = V_MAX;
%     else
%         v = V_MAX * eL(k) / SPEED_CUT;
%     end
%     
%     % 角速度 ω = γ * α
%     omega = g * alpha;
%     
%     % 限幅
%     v     = max(min(v, V_MAX), -V_MAX);
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
%     
%     % 状态更新
%     x     = x + v * cos(theta) * dt;
%     y     = y + v * sin(theta) * dt;
%     theta = theta + omega * dt;
%     
%     % 记录轨迹
%     xL(k) = x;
%     yL(k) = y;
%     
%     % 动画刷新
%     if mod(k, refresh_step) == 0
%         set(hTraj, 'XData', xL(1:k), 'YData', yL(1:k));
%         drawnow;
%         pause(dt * slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL) * dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref, y_ref, 'k--', xL, yL, 'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt, eL, 'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt, gL, 'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');