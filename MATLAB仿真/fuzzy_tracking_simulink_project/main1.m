% % fuzzy_pp_complex_demo.m
clc; clear; close all;

%% === 用户参数 ===
T           = 60;        % 仿真时长 (s)
dt          = 0.02;      % 时间步长 (s)
N           = round(T/dt);
V_MAX       = 0.6;       % 最高线速度 (m/s)
OMEGA_LIM   = 2.5;       % 最大角速度 (rad/s)

LOOK_BASE   = 0.6;       % 基础前瞻距离 (m)
SPEED_CUT   = 0.3;       % 误差减速阈值 (m)
slow_factor = 1;         % 动画放慢倍数 (>=1)
refresh_step= 1;         % 动画刷新帧步

%% === 复杂路径生成 ===
% 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
phi    = linspace(0, 2*pi, N);
x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
y_ref  = 5*sin(phi) + 0.5*sin(2*phi);

%% === 模糊增益 FIS 自动生成/加载 ===
fisFile = 'fuzzy_gain.fis';
if ~isfile(fisFile)
    fis_g = mamfis('Name','γ_gain');
    % 输入：横向误差 e (归一化到 ±1)
    fis_g = addInput(fis_g,[-1 1],'Name','e');
    fis_g = addMF(fis_g,'e','trimf',[-1 -1 -0.2],'Name','N');
    fis_g = addMF(fis_g,'e','trimf',[-0.4 0 0.4],'Name','Z');
    fis_g = addMF(fis_g,'e','trimf',[0.2 1 1],'Name','P');
    % 输入：航向误差 psi (归一化到 ±1)
    fis_g = addInput(fis_g,[-1 1],'Name','psi');
    fis_g = addMF(fis_g,'psi','trimf',[-1 -1 -0.2],'Name','N');
    fis_g = addMF(fis_g,'psi','trimf',[-0.4 0 0.4],'Name','Z');
    fis_g = addMF(fis_g,'psi','trimf',[0.2 1 1],'Name','P');
    % 输出：增益 γ
    fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
    fis_g = addMF(fis_g,'g','trimf',[0.1 0.5 1.0],'Name','Small');
    fis_g = addMF(fis_g,'g','trimf',[0.8 1.2 1.6],'Name','Medium');
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
state.x  = x_ref(1);
state.y  = y_ref(1);
state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));

xL = zeros(1,N);
yL = zeros(1,N);
eL = zeros(1,N);
gL = zeros(1,N);

%% === 动画准备 ===
win    = 1.5;  % 视窗半宽
figure('Name','Fuzzy PP Complex Demo'); hold on;
plot(x_ref, y_ref, 'k--','LineWidth',1.1);
anim   = animatedline('Color','b','LineWidth',1.6);
errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
grid on; axis equal;

%% === 主循环：Pure‑Pursuit + 模糊增益 ===
prev_e = 0;
for k = 1:N
    % 1) 最近点 + 动态前瞻
    dists = hypot(x_ref - state.x, y_ref - state.y);
    [~, idx] = min(dists);
    Ld = max(LOOK_BASE, 1.0 * dists(idx));
    dist_acc = 0; idx_l = idx;
    while dist_acc < Ld && idx_l < N
        seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
                    y_ref(idx_l+1)-y_ref(idx_l));
        dist_acc = dist_acc + seg; idx_l = idx_l + 1;
    end
    xt = x_ref(idx_l); yt = y_ref(idx_l);
    
    % 2) 误差计算
    dx     = xt - state.x;
    dy     = yt - state.y;
    e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
    psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
    
    % 3) Pure‑Pursuit 曲率
    alpha    = wrapToPi(atan2(dy,dx) - state.th);
    kappa    = 2*sin(alpha) / Ld;
    omega_pp = V_MAX * kappa;
    
    % 4) 模糊增益 γ
    % 归一化输入
    e_n   = max(min(e_lat/1.5, 1), -1);
    psi_n = max(min(psi_err/pi, 1), -1);
    gamma = evalfis([e_n, psi_n], gain_fis);
    omega = gamma * omega_pp;
    omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
    
    % 5) 自适应线速度
    v = V_MAX * (1 - 0.5 * min(abs(e_lat)/SPEED_CUT,1));
    
    % 6) 状态更新
    state.x  = state.x + v*cos(state.th)*dt;
    state.y  = state.y + v*sin(state.th)*dt;
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
            'String',sprintf('e=%.3f m',eL(k)));
        axis([state.x-win, state.x+win, state.y-win, state.y+win]);
        drawnow limitrate;
        pause(dt*slow_factor);
    end
end

%% === 性能评估 ===
RMSE = sqrt(mean(eL.^2));
MAXE = max(eL);
SS   = mean(eL(end-round(2/dt):end));
INT  = sum(eL)*dt;
fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
fprintf('Max E  = %.4f m\n', MAXE);
fprintf('SS Err = %.4f m\n', SS);
fprintf('Int E  = %.4f m·s\n', INT);

%% === 静态结果展示 ===
figure('Name','Tracking Result');
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
% 
% 
% 
% 
% 
% 
% 













% % fuzzy_pp_demo.m
% clc; clear; close all;
% 
% %% === 用户参数 ===
% T           = 60;       % 仿真时长 (s)
% dt          = 0.04;     % 步长 (s)
% N           = round(T/dt);
% V_MAX       = 0.6;      % 最高线速度 (m/s)
% OMEGA_LIM   = 2.5;      % 角速度限幅 (rad/s)
% 
% LOOK_BASE   = 0.8;      % 基础前瞻距 (m)
% SPEED_CUT   = 0.3;      % 误差减速阈值 (m)
% 
% slow_factor  = 1;       % 动画放慢倍数 (>=1)
% refresh_step = 1;       % 动画刷新帧步
% 
% %% === 轨迹类型选择 ===
% trajType = "random";    % "circle" or "random"
% switch trajType
%     case "circle"
%         r    = 5;
%         phi  = linspace(0,2*pi,N);
%         x_ref = r*cos(phi);
%         y_ref = r*sin(phi);
%     case "random"
%         rng(42); numWp = 8; wpRadius = 4.5;
%         ang = linspace(0,2*pi,numWp+1); ang(end) = [];
%         wp_x = wpRadius*cos(ang) + 0.5*randn(1,numWp);
%         wp_y = wpRadius*sin(ang) + 0.5*randn(1,numWp);
%         wp_x(end+1) = wp_x(1); wp_y(end+1) = wp_y(1);
%         t_wp = [0, cumsum(hypot(diff(wp_x), diff(wp_y)))];
%         t_ref = linspace(0, t_wp(end), N);
%         x_ref = interp1(t_wp, wp_x, t_ref, 'pchip');
%         y_ref = interp1(t_wp, wp_y, t_ref, 'pchip');
%     otherwise
%         error('未知轨迹类型');
% end
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入 e
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -1 -0.3],'Name','N');
%     fis_g = addMF(fis_g,'e','trimf',[-0.5 0 0.5],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.3 1 1],'Name','P');
%     % 输入 psi
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -1 -0.3],'Name','N');
%     fis_g = addMF(fis_g,'psi','trimf',[-0.5 0 0.5],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.3 1 1],'Name','P');
%     % 输出 γ
%     fis_g = addOutput(fis_g,[0.2 1.8],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.2 0.4 0.8],'Name','Small');
%     fis_g = addMF(fis_g,'g','trimf',[0.6 1.0 1.4],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.2 1.6 1.8],'Name','Big');
%     % 规则
%     rule = [
%         1 1 3 1 1;
%         1 2 3 1 1;
%         1 3 2 1 1;
%         2 1 3 1 1;
%         2 2 2 1 1;
%         2 3 1 1 1;
%         3 1 2 1 1;
%         3 2 1 1 1;
%         3 3 1 1 1 ];
%     fis_g = addRule(fis_g, rule);
%     writefis(fis_g, fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% 
% %% === 动画准备 ===
% win = 1.5;  % 可视化窗口半宽
% figure('Name','Fuzzy PP Demo'); hold on; grid on; axis equal;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 找到最近点 idx
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
%     % 前瞻距离 Ld
%     Ld = max(LOOK_BASE, 1.1*dists(idx));
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
%     
%     % 横向误差 & 航向误差
%     dx = xt - state.x; dy = yt - state.y;
%     e_lat   = sin(state.th)*dx - cos(state.th)*dy;
%     psi_err = wrapToPi(atan2(dy, dx) - state.th);
%     
%     % Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(dy,dx) - state.th);
%     kappa    = 2*sin(alpha) / Ld;
%     omega_pp = V_MAX * kappa;
%     
%     % 模糊增益 γ
%     gamma = evalfis([e_lat/1.5, psi_err/pi], gain_fis);
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
%     
%     % 自适应线速度
%     v = V_MAX * (1 - 0.5 * min(abs(e_lat)/SPEED_CUT, 1));
%     
%     % 状态更新
%     state.x  = state.x + v * cos(state.th) * dt;
%     state.y  = state.y + v * sin(state.th) * dt;
%     state.th = wrapToPi(state.th + omega * dt);
%     
%     % 记录日志
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
%     
%     % 动画绘制
%     addpoints(anim, state.x, state.y);
%     if mod(k, refresh_step) == 0
%         set(errTxt, 'Position', [state.x, state.y+0.3], 'String', sprintf('e=%.3f m', eL(k)));
%         axis([state.x-win, state.x+win, state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt * slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL) * dt;
% fprintf('\n[Fuzzy‑PP %s 评估]\n  RMSE = %.3f m\n  MaxE = %.3f m\n  SS   = %.3f m\n  Int  = %.3f m·s\n', ...
%     trajType, RMSE, MAXE, SS, INT);
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



















%% 误差0.03
%% =============================================================
%%  工程级模糊 + Pure‑Pursuit 路径跟踪（随机轨迹版）
%% -------------------------------------------------------------
%  • 可在圆形轨迹与随机样条轨迹之间切换
%  • Pure‑Pursuit 提供基础曲率 κ_pp，模糊控制器输出增益 γ 调速
%  • 支持动态演示、误差实时显示与统计评估
%% =============================================================

% clc; clear; close all;
% 
% %% === 用户参数 ===
% T = 60;                 % 仿真时长 (s)
% dt = 0.04;              % 步长 (s)
% N  = round(T/dt);
% V_MAX = 0.6;            % 最高线速度 (m/s)
% OMEGA_LIM = 2.5;        % 角速度限幅 (rad/s)
% 
% % Pure‑Pursuit / 模糊参数
% LOOK_BASE = 0.8;        % 基础前瞻距 (m)
% SPEED_CUT = 0.3;        % 误差减速阈值 (m)
% 
% slow_factor  = 1;       % 动画放慢倍数 (>=1)
% refresh_step = 1;       % 动画刷新帧步
% 
% %% === 轨迹类型选择 ===
% trajType = "random";    % "circle" or "random"
% 
% switch trajType
%     case "circle"
%         r = 5; phi = linspace(0,2*pi,N);
%         x_ref = r*cos(phi); y_ref = r*sin(phi);
%     case "random"
%         % 随机 waypoints 生成样条轨迹
%         rng(42);                     % 可重复随机
%         numWp = 8;                   % 路点数(>=4)
%         wpRadius = 4.5;              % 散布半径
%         ang = linspace(0,2*pi,numWp+1); ang(end)=[];
%         wp_x = wpRadius*cos(ang) + 0.5*randn(1,numWp);
%         wp_y = wpRadius*sin(ang) + 0.5*randn(1,numWp);
%         % 闭合路径
%         wp_x(end+1)=wp_x(1); wp_y(end+1)=wp_y(1);
%         % 样条路径 (Catmull-Rom)
%         t_wp = [0 cumsum(hypot(diff(wp_x), diff(wp_y)))];
%         t_ref = linspace(0,t_wp(end),N);
%         x_ref = interp1(t_wp, wp_x, t_ref, 'pchip');
%         y_ref = interp1(t_wp, wp_y, t_ref, 'pchip');
%     otherwise
%         error('未知轨迹类型');
% end
% 
% %% === 模糊增益 FIS (自动创建一次) ===
% if ~exist('fuzzy_gain.fis','file')
%     fis = mamfis('Name','γ_gain');
%     fis = addInput(fis,[-1 1],'Name','e');
%     fis = addMF(fis,'e','trimf',[-1 -1 -0.3],'Name','N');
%     fis = addMF(fis,'e','trimf',[-0.5 0 0.5],'Name','Z');
%     fis = addMF(fis,'e','trimf',[0.3 1 1],'Name','P');
%     fis = addInput(fis,[-1 1],'Name','psi');
%     fis = addMF(fis,'psi','trimf',[-1 -1 -0.3],'Name','N');
%     fis = addMF(fis,'psi','trimf',[-0.5 0 0.5],'Name','Z');
%     fis = addMF(fis,'psi','trimf',[0.3 1 1],'Name','P');
%     fis = addOutput(fis,[0.2 1.8],'Name','g');
%     fis = addMF(fis,'g','trimf',[0.2 0.4 0.8],'Name','Small');
%     fis = addMF(fis,'g','trimf',[0.6 1 1.4],'Name','Medium');
%     fis = addMF(fis,'g','trimf',[1.2 1.6 1.8],'Name','Big');
%     rule=[1 1 3 1 1;1 2 3 1 1;1 3 2 1 1;2 1 3 1 1;2 2 2 1 1;2 3 1 1 1;3 1 2 1 1;3 2 1 1 1;3 3 1 1 1];
%     fis = addRule(fis,rule); writefis(fis,'fuzzy_gain');
% end
% gain_fis = readfis('fuzzy_gain.fis');
% 
% %% === 机器人状态 & 日志 ===
% state.x = x_ref(1); state.y = y_ref(1);   % 从轨迹起点出发
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL=zeros(1,N); yL=zeros(1,N); eL=zeros(1,N); gL=zeros(1,N);
% 
% %% === 动画 ===
% figure('Name','Fuzzy PP Random Demo'); plot(x_ref,y_ref,'k--','LineWidth',1.1); hold on; grid on; axis equal;
% anim=animatedline('Color','b','LineWidth',1.6);
% errTxt=text(state.x,state.y+0.3,'e=0','Color','r','FontSize',11);
% win=1.5;
% 
% %% === 主循环 ===
% for k=1:N
%     %% 找最近点 idx
%     d = hypot(x_ref-state.x, y_ref-state.y); [~,idx]=min(d);
%     % 前瞻计算
%     Ld = max(LOOK_BASE, 1.1*d(idx));
%     dist=0; idx_l=idx;
%     while dist<Ld && idx_l<N
%         seg=hypot(x_ref(idx_l+1)-x_ref(idx_l), y_ref(idx_l+1)-y_ref(idx_l));
%         dist=dist+seg; idx_l=idx_l+1; end
%     xt=x_ref(idx_l); yt=y_ref(idx_l);
% 
%     %% 误差
%     dx=xt-state.x; dy=yt-state.y;
%     e_lat = sin(state.th)*dx - cos(state.th)*dy;          % +右侧
%     psi_err = wrapToPi(atan2(dy,dx)-state.th);
% 
%     %% Pure‑Pursuit 曲率
%     alpha = wrapToPi(atan2(dy,dx)-state.th);
%     kappa = 2*sin(alpha)/Ld; omega_pp = V_MAX*kappa;
% 
%     %% 模糊增益 γ
%     gamma = evalfis(gain_fis,[e_lat/1.5, psi_err/pi]);
%     omega = gamma * omega_pp; omega = max(min(omega,OMEGA_LIM),-OMEGA_LIM);
% 
%     %% 自适应速度 (误差大减速)
%     v = V_MAX * (1-0.5*min(abs(e_lat)/SPEED_CUT,1));
% 
%     %% 运动学
%     state.x = state.x + v*cos(state.th)*dt;
%     state.y = state.y + v*sin(state.th)*dt;
%     state.th= wrapToPi(state.th + omega*dt);
% 
%     %% 记录
%     xL(k)=state.x; yL(k)=state.y; eL(k)=abs(e_lat); gL(k)=gamma;
% 
%     %% 动画输出
%     addpoints(anim,state.x,state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3],'String',sprintf('e=%.3f',abs(e_lat)));
%         axis([state.x-win state.x+win state.y-win state.y+win]);
%         drawnow limitrate; pause(dt*slow_factor);
%     end
% end
% 
% %% === 评估 ===
% RMSE=sqrt(mean(eL.^2)); MAXE=max(eL); SS=mean(eL(end-round(2/dt):end)); INT=sum(eL)*dt;
% fprintf('\n[Fuzzy‑PP %s 评估] RMSE=%.3f m | Max=%.3f m | SS=%.3f m | Int=%.3f m·s\n',trajType,RMSE,MAXE,SS,INT);
% 
% %% === 静态结果 ===
% figure('Name','Tracking Result'); subplot(2,2,1); plot(x_ref,y_ref,'k--',xL,yL,'b'); axis equal; grid on; title(['路径对比 - ' trajType]);
% subplot(2,2,2); plot((0:N-1)*dt,eL,'LineWidth',1.2); grid on; ylabel('e(m)'); title('距离误差');
% subplot(2,2,3); plot((0:N-1)*dt,gL,'LineWidth',1.2); grid on; ylabel('γ'); title('模糊增益'); xlabel('t(s)');
% subplot(2,2,4); histogram(eL,30); title('误差分布');


















%% 误差0.05左右

%% =============================================================
%%  工程级模糊 + Pure‑Pursuit 路径跟踪重构版
%%  -------------------------------------------------------------
%  • 核心思想 :  Pure‑Pursuit 提供几何曲率 κ_pp
%                模糊控制器根据横向误差 e_lat 与航向误差 ψ_err
%                输出一个比例系数 γ ∈ [0.2, 1.8] 对 κ_pp 进行缩放 → ω
%  • 优势     :  PP 保证收敛方向，模糊调节收敛速度 & 抑制振荡
%  • 目标轨迹 : 圆 r = 5 m
%  • 评估     : RMSE、最大误差、稳态误差、积分误差、控制能量
%% =============================================================

% clc; clear; close all;
% 
% %% === 参数区 ===
% T = 60;              dt = 0.04;          N = round(T/dt);
% V_MAX = 0.6;         % 最大线速度
% OMEGA_LIM = 2.5;     % 角速度限幅 (rad/s)
% 
% LOOK_BASE = 0.8;     % 基础前瞻 (m)
% speed_cut = 0.3;     % 误差大于此值时减速比例阈值 (m)
% 
% slow_factor  = 1;    % 动画放慢倍数
% refresh_step = 1;    % 刷新步
% 
% %% === 轨迹 ===
% r = 5; phi = linspace(0,2*pi,N);
% x_ref = r*cos(phi); y_ref = r*sin(phi);
% 
% %% === 模糊控制器设计 ===
% % 模糊输出 γ ∈ [0.2, 1.8]  (γ=1 不变，>1 加快收敛，<1 减缓)
% if ~exist('fuzzy_gain.fis','file')
%     gain_fis = mamfis('Name','γ_gain');
%     % 输入 e_lat_norm (-1..1)
%     gain_fis = addInput(gain_fis,[-1 1],'Name','e');
%     gain_fis = addMF(gain_fis,'e','trimf',[-1 -1 -0.3],'Name','N');
%     gain_fis = addMF(gain_fis,'e','trimf',[-0.5 0 0.5],'Name','Z');
%     gain_fis = addMF(gain_fis,'e','trimf',[0.3 1 1],'Name','P');
%     % 输入 ψ_err_norm (-1..1)
%     gain_fis = addInput(gain_fis,[-1 1],'Name','psi');
%     gain_fis = addMF(gain_fis,'psi','trimf',[-1 -1 -0.3],'Name','N');
%     gain_fis = addMF(gain_fis,'psi','trimf',[-0.5 0 0.5],'Name','Z');
%     gain_fis = addMF(gain_fis,'psi','trimf',[0.3 1 1],'Name','P');
%     % 输出 γ
%     gain_fis = addOutput(gain_fis,[0.2 1.8],'Name','g');
%     gain_fis = addMF(gain_fis,'g','trimf',[0.2 0.4 0.8],'Name','Small');
%     gain_fis = addMF(gain_fis,'g','trimf',[0.6 1 1.4],'Name','Medium');
%     gain_fis = addMF(gain_fis,'g','trimf',[1.2 1.6 1.8],'Name','Big');
%     % 规则表 (9 条)
%     rule = [ ...
%         1 1 3 1 1;  % e N, psi N -> Big (远且角度大，加速纠正)
%         1 2 3 1 1;  % e N, psi Z -> Big
%         1 3 2 1 1;  % e N, psi P -> Medium
%         2 1 3 1 1;  % e Z, psi N -> Big
%         2 2 2 1 1;  % e Z, psi Z -> Medium
%         2 3 1 1 1;  % e Z, psi P -> Small
%         3 1 2 1 1;  % e P, psi N -> Medium
%         3 2 1 1 1;  % e P, psi Z -> Small
%         3 3 1 1 1]; % e P, psi P -> Small
%     gain_fis = addRule(gain_fis,rule);
%     writefis(gain_fis,'fuzzy_gain');
% else
%     gain_fis = readfis('fuzzy_gain.fis');
% end
% 
% %% === 机器人状态 ===
% state.x = r; state.y = 0; state.th = pi/2;
% 
% % 日志
% xL=zeros(1,N); yL=zeros(1,N); eL=zeros(1,N); gL=zeros(1,N);
% 
% %% === 动画 ===
% figure('Name','Fuzzy Gain‑PP Demo'); plot(x_ref,y_ref,'k--','LineWidth',1.1); hold on; grid on; axis equal;
% anim=animatedline('Color','b','LineWidth',1.6);
% errTxt=text(state.x,state.y+0.4,'e=0','Color','r','FontSize',12);
% win=1.5;
% 
% %% === 主循环 ===
% for k=1:N
%     %% 最近点
%     d = hypot(x_ref-state.x, y_ref-state.y); [~,idx]=min(d);
%     Ld = max(LOOK_BASE, 1.1*d(idx));
%     % 寻找前瞻点
%     s=0; idx_g=idx; while s<Ld && idx_g<N
%         ds=hypot(x_ref(idx_g+1)-x_ref(idx_g), y_ref(idx_g+1)-y_ref(idx_g));
%         s=s+ds; idx_g=idx_g+1; end
%     xt=x_ref(idx_g); yt=y_ref(idx_g);
% 
%     %% 误差
%     dx=xt-state.x; dy=yt-state.y;
%     e_lat = sin(state.th)*dx - cos(state.th)*dy;
%     psi_err = wrapToPi(atan2(dy,dx)-state.th);
% 
%     %% PP 曲率
%     alpha = wrapToPi(atan2(dy,dx)-state.th);
%     kappa = 2*sin(alpha)/Ld;
%     omega_pp = V_MAX*kappa;
% 
%     %% 模糊增益 γ
%     e_n = e_lat/1.5; psi_n = psi_err/pi;
%     gamma = evalfis(gain_fis,[e_n, psi_n]);
% 
%     omega = gamma * omega_pp;
%     omega = max(min(omega,OMEGA_LIM),-OMEGA_LIM);
% 
%     %% 速度调节 (误差大减速)
%     v = V_MAX * (1-0.5*min(abs(e_lat)/speed_cut,1));
% 
%     %% 运动学更新
%     state.x = state.x + v*cos(state.th)*dt;
%     state.y = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     %% 日志
%     xL(k)=state.x; yL(k)=state.y; eL(k)=abs(e_lat); gL(k)=gamma;
% 
%     %% 动画
%     addpoints(anim,state.x,state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.4],'String',sprintf('e=%.3f',abs(e_lat)));
%         axis([state.x-win state.x+win state.y-win state.y+win]);
%         drawnow limitrate; pause(dt*slow_factor);
%     end
% end
% 
% %% === 指标 ===
% RMSE=sqrt(mean(eL.^2)); MAXE=max(eL); SSE=mean(eL(end-round(2/dt):end)); INT=sum(eL)*dt;
% fprintf('\n[Fuzzy Gain‑PP 评估]  RMSE=%.3f m | Max=%.3f m | Steady=%.3f m | IntErr=%.3f m·s\n',RMSE,MAXE,SSE,INT);
% 
% %% === 静态结果 ===
% figure; subplot(2,2,1); plot(x_ref,y_ref,'k--',xL,yL,'b'); axis equal; grid on; title('路径对比');
% subplot(2,2,2); plot((0:N-1)*dt,eL,'LineWidth',1.2); grid on; title('距离误差'); ylabel('e(m)');
% subplot(2,2,3); plot((0:N-1)*dt,gL,'LineWidth',1.2); grid on; title('模糊增益 γ'); ylabel('γ'); xlabel('t(s)');
% subplot(2,2,4); histogram(eL,30); title('误差分布');
