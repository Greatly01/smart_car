% fuzzy_pp_puma560_full_integration.m
clc; clear; close all;

clc; clear; close all;

%% 询问用户是否跳过动画
skip_animation = input('是否跳过动画演示？(1=是, 0=否): ');

%% ==================== Part I: Fuzzy‑PP 控制生成笛卡尔轨迹 ====================
% 用户参数
T1 = 60;        dt1 = 0.02;      N1 = round(T1/dt1);
V_MAX = 0.6;    OMEGA_LIM = 2.5; LOOK_BASE = 0.6;  SPEED_CUT = 0.3;
slow_factor = 0.5;   refresh_step = 1;

% 复杂路径生成（完整保留）
phi = linspace(0,2*pi,N1);
x_ref1 = 5*cos(phi) + 1.0*cos(3*phi);
y_ref1 = 5*sin(phi) + 0.5*sin(2*phi);

% 模糊控制器生成（完整规则）
fisFile = 'fuzzy_gain.fis';
if ~isfile(fisFile)
    fis_g = mamfis('Name','γ_gain');
    
    % 输入：侧向误差 e（完整MF定义）
    fis_g = addInput(fis_g,[-1 1],'Name','e');
    fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N');
    fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
    fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');

    % 输入：航向误差 psi（完整MF定义）
    fis_g = addInput(fis_g,[-1 1],'Name','psi');
    fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N');
    fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
    fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');

    % 输出：增益 γ（完整MF定义）
    fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
    fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small');
    fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
    fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');

    % 完整规则表
    ruleList = [...
        1 1 3 1 1;   % Rule 1
        1 2 3 1 1;   % Rule 2  
        1 3 2 1 1;   % Rule 3
        2 1 3 1 1;   % Rule 4
        2 2 2 1 1;   % Rule 5
        2 3 1 1 1;   % Rule 6
        3 1 2 1 1;   % Rule 7
        3 2 1 1 1;   % Rule 8
        3 3 1 1 1];  % Rule 9
    
    fis_g = addRule(fis_g, ruleList);
    writefis(fis_g, fisFile);
end
gain_fis = readfis(fisFile);

% 初始化状态（完整保留）
state.x = x_ref1(1);
state.y = y_ref1(1);
state.th = atan2(y_ref1(2)-y_ref1(1), x_ref1(2)-x_ref1(1));
xL1 = zeros(1,N1); yL1 = zeros(1,N1);
eL1 = zeros(1,N1); gL1 = zeros(1,N1);
v_hist = zeros(1,5); 
e_lat_integral = 0; prev_e = 0; 
e_lat_hist = zeros(1,3);
kappa_hist = zeros(1,5);

% 动画初始化（完整窗口设置）
if ~skip_animation
    figure('Name','Fuzzy‑PP Complex Demo','NumberTitle','off');
    hold on; grid on; axis equal;
    plot(x_ref1,y_ref1,'k--','LineWidth',1.1);
    anim1 = animatedline('Color','b','LineWidth',1.6);
    errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
    win = 1.5;
end

%% Fuzzy-PP主循环（完整逻辑）
for k = 1:N1
    % 最近点搜索（完整实现）
    dists = hypot(x_ref1-state.x, y_ref1-state.y);
    [~, idx] = min(dists);
    dx = x_ref1(idx)-state.x; 
    dy = y_ref1(idx)-state.y;
    
    % 误差计算（完整公式）
    e_lat = sin(state.th)*dx - cos(state.th)*dy;
    psi_err = wrapToPi(atan2(dy,dx) - state.th);
    
    % 曲率估计（三点法完整实现）
    if idx>1 && idx<N1-1
        x1 = x_ref1(idx-1); y1 = y_ref1(idx-1);
        x2 = x_ref1(idx);   y2 = y_ref1(idx);
        x3 = x_ref1(idx+1); y3 = y_ref1(idx+1);
        A = hypot(x2-x1, y2-y1);
        B = hypot(x3-x2, y3-y2);
        C = hypot(x3-x1, y3-y1);
        s = (A+B+C)/2;
        area = sqrt(max(s*(s-A)*(s-B)*(s-C),0));
        if area > 1e-6
            kappa = 4*area/(A*B*C);
        else
            kappa = 0;
        end
    else
        kappa = 0;
    end
    kappa_hist = [kappa_hist(2:end), kappa];
    
    % 动态前瞻距离（完整调节逻辑）
    e_lat_rate = (e_lat - prev_e)/dt1;
    prev_e = e_lat;
    e_lat_hist = [e_lat_hist(2:end), e_lat];
    e_lat_integral = e_lat_integral + e_lat*dt1;
    
    curv_f = 1 - 0.6*min(abs(kappa)/10,1);
    err_f = 1 - 0.2*min(abs(e_lat)/SPEED_CUT,1);
    int_f = 1 - 0.2*min(abs(e_lat_integral)/1,1);
    rate_f = 1 - 0.1*min(abs(e_lat_rate)/1,1);
    Ld = max(0.2, LOOK_BASE*curv_f*err_f*(1-int_f)*rate_f);
    
    % 前瞻点搜索（完整实现）
    dist_acc = 0; idx_l = idx;
    while dist_acc < Ld && idx_l < N1
        dist_acc = dist_acc + hypot(x_ref1(idx_l+1)-x_ref1(idx_l),...
                                   y_ref1(idx_l+1)-y_ref1(idx_l));
        idx_l = idx_l + 1;
    end
    xt = x_ref1(idx_l); yt = y_ref1(idx_l);
    alpha = wrapToPi(atan2(yt-state.y, xt-state.x) - state.th);
    
    % Pure-Pursuit控制（完整公式）
    if Ld > 1e-6
        kappa_pp = 2*sin(alpha)/Ld;
    else
        kappa_pp = 0;
    end
    omega_pp = V_MAX * kappa_pp;
    
    % 模糊增益调节（完整逻辑）
    e_n = max(min(e_lat/1.5,1),-1);
    psi_n = max(min(psi_err/pi,1),-1);
    gamma = evalfis([e_n, psi_n], gain_fis);
    if abs(kappa) > 0.2
        gamma = gamma * (1 + 0.6*min(abs(kappa)/10,1));
    end
    if abs(e_lat) > 0.5 || abs(e_lat_integral) > 1 || abs(e_lat_rate) > 0.5
        gamma = gamma * 2.5;
    end
    omega = max(min(gamma*omega_pp, OMEGA_LIM), -OMEGA_LIM);
    
    % 自适应速度（完整调节策略）
    speed_f = 1 - 0.9*min(abs(kappa)/10,1);
    err_f2 = 1 - 0.6*min(abs(e_lat)/SPEED_CUT,1);
    int_f2 = 1 - 0.3*min(abs(e_lat_integral)/2,1);
    rate_f2 = 1 - 0.2*min(abs(e_lat_rate)/1,1);
    hist_f = 1 - 0.2*(mean(v_hist)/V_MAX);
    v = V_MAX * speed_f * err_f2 * int_f2 * rate_f2 * hist_f;
    v_hist = [v_hist(2:end), v];
    
    % 状态更新（完整运动学模型）
    state.x = state.x + v*cos(state.th)*dt1;
    state.y = state.y + v*sin(state.th)*dt1;
    state.th = wrapToPi(state.th + omega*dt1);
    xL1(k) = state.x; yL1(k) = state.y;
    eL1(k) = abs(e_lat);
    gL1(k) = gamma;
    
    % 实时动画（完整显示逻辑）
    if ~skip_animation
        addpoints(anim1, state.x, state.y);
        if mod(k,refresh_step)==0
            set(errTxt,'Position',[state.x,state.y+0.3],...
                      'String',sprintf('e=%.3f m',eL1(k)));
            xlim([state.x-win, state.x+win]);
            ylim([state.y-win, state.y+win]);
            drawnow limitrate; 
            pause(dt1*slow_factor);  % 原始速度控制
        end
    end
end

% 性能评估（完整输出）
RMSE1 = sqrt(mean(eL1.^2));
MAXE1 = max(eL1);
SS1 = mean(eL1(end-round(2/dt1):end));
INT1 = sum(eL1)*dt1;
fprintf('\n===== Fuzzy‑PP 评估结果 =====\n');
fprintf('RMSE: %.4f m | Max误差: %.4f m\n', RMSE1, MAXE1);
fprintf('稳态误差: %.4f m | 累计误差: %.4f m·s\n', SS1, INT1);




clc; clear; close all;

%% 询问用户是否跳过动画
skip_animation = input('是否跳过动画演示？(1=是, 0=否): ');

%% ==================== Part I: Fuzzy‑PP 控制生成笛卡尔轨迹 ====================
% 用户参数
T1 = 60;        dt1 = 0.02;      N1 = round(T1/dt1);
V_MAX = 0.6;    OMEGA_LIM = 2.5; LOOK_BASE = 0.6;  SPEED_CUT = 0.3;
slow_factor = 0.5;   refresh_step = 1;

% 复杂路径生成（完整保留）
phi = linspace(0,2*pi,N1);
x_ref1 = 5*cos(phi) + 1.0*cos(3*phi);
y_ref1 = 5*sin(phi) + 0.5*sin(2*phi);

% 模糊控制器生成（完整规则）
fisFile = 'fuzzy_gain.fis';
if ~isfile(fisFile)
    fis_g = mamfis('Name','γ_gain');
    
    % 输入：侧向误差 e（完整MF定义）
    fis_g = addInput(fis_g,[-1 1],'Name','e');
    fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N');
    fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
    fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');

    % 输入：航向误差 psi（完整MF定义）
    fis_g = addInput(fis_g,[-1 1],'Name','psi');
    fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N');
    fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
    fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');

    % 输出：增益 γ（完整MF定义）
    fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
    fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small');
    fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
    fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');

    % 完整规则表
    ruleList = [...
        1 1 3 1 1;   % Rule 1
        1 2 3 1 1;   % Rule 2  
        1 3 2 1 1;   % Rule 3
        2 1 3 1 1;   % Rule 4
        2 2 2 1 1;   % Rule 5
        2 3 1 1 1;   % Rule 6
        3 1 2 1 1;   % Rule 7
        3 2 1 1 1;   % Rule 8
        3 3 1 1 1];  % Rule 9
    
    fis_g = addRule(fis_g, ruleList);
    writefis(fis_g, fisFile);
end
gain_fis = readfis(fisFile);

% 初始化状态（完整保留）
state.x = x_ref1(1);
state.y = y_ref1(1);
state.th = atan2(y_ref1(2)-y_ref1(1), x_ref1(2)-x_ref1(1));
xL1 = zeros(1,N1); yL1 = zeros(1,N1);
eL1 = zeros(1,N1); gL1 = zeros(1,N1);
v_hist = zeros(1,5); 
e_lat_integral = 0; prev_e = 0; 
e_lat_hist = zeros(1,3);
kappa_hist = zeros(1,5);

% 动画初始化（完整窗口设置）
if ~skip_animation
    figure('Name','Fuzzy‑PP Complex Demo','NumberTitle','off');
    hold on; grid on; axis equal;
    plot(x_ref1,y_ref1,'k--','LineWidth',1.1);
    anim1 = animatedline('Color','b','LineWidth',1.6);
    errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
    win = 1.5;
end

%% Fuzzy-PP主循环（完整逻辑）
for k = 1:N1
    % 最近点搜索（完整实现）
    dists = hypot(x_ref1-state.x, y_ref1-state.y);
    [~, idx] = min(dists);
    dx = x_ref1(idx)-state.x; 
    dy = y_ref1(idx)-state.y;
    
    % 误差计算（完整公式）
    e_lat = sin(state.th)*dx - cos(state.th)*dy;
    psi_err = wrapToPi(atan2(dy,dx) - state.th);
    
    % 曲率估计（三点法完整实现）
    if idx>1 && idx<N1-1
        x1 = x_ref1(idx-1); y1 = y_ref1(idx-1);
        x2 = x_ref1(idx);   y2 = y_ref1(idx);
        x3 = x_ref1(idx+1); y3 = y_ref1(idx+1);
        A = hypot(x2-x1, y2-y1);
        B = hypot(x3-x2, y3-y2);
        C = hypot(x3-x1, y3-y1);
        s = (A+B+C)/2;
        area = sqrt(max(s*(s-A)*(s-B)*(s-C),0));
        if area > 1e-6
            kappa = 4*area/(A*B*C);
        else
            kappa = 0;
        end
    else
        kappa = 0;
    end
    kappa_hist = [kappa_hist(2:end), kappa];
    
    % 动态前瞻距离（完整调节逻辑）
    e_lat_rate = (e_lat - prev_e)/dt1;
    prev_e = e_lat;
    e_lat_hist = [e_lat_hist(2:end), e_lat];
    e_lat_integral = e_lat_integral + e_lat*dt1;
    
    curv_f = 1 - 0.6*min(abs(kappa)/10,1);
    err_f = 1 - 0.2*min(abs(e_lat)/SPEED_CUT,1);
    int_f = 1 - 0.2*min(abs(e_lat_integral)/1,1);
    rate_f = 1 - 0.1*min(abs(e_lat_rate)/1,1);
    Ld = max(0.2, LOOK_BASE*curv_f*err_f*(1-int_f)*rate_f);
    
    % 前瞻点搜索（完整实现）
    dist_acc = 0; idx_l = idx;
    while dist_acc < Ld && idx_l < N1
        dist_acc = dist_acc + hypot(x_ref1(idx_l+1)-x_ref1(idx_l),...
                                   y_ref1(idx_l+1)-y_ref1(idx_l));
        idx_l = idx_l + 1;
    end
    xt = x_ref1(idx_l); yt = y_ref1(idx_l);
    alpha = wrapToPi(atan2(yt-state.y, xt-state.x) - state.th);
    
    % Pure-Pursuit控制（完整公式）
    if Ld > 1e-6
        kappa_pp = 2*sin(alpha)/Ld;
    else
        kappa_pp = 0;
    end
    omega_pp = V_MAX * kappa_pp;
    
    % 模糊增益调节（完整逻辑）
    e_n = max(min(e_lat/1.5,1),-1);
    psi_n = max(min(psi_err/pi,1),-1);
    gamma = evalfis([e_n, psi_n], gain_fis);
    if abs(kappa) > 0.2
        gamma = gamma * (1 + 0.6*min(abs(kappa)/10,1));
    end
    if abs(e_lat) > 0.5 || abs(e_lat_integral) > 1 || abs(e_lat_rate) > 0.5
        gamma = gamma * 2.5;
    end
    omega = max(min(gamma*omega_pp, OMEGA_LIM), -OMEGA_LIM);
    
    % 自适应速度（完整调节策略）
    speed_f = 1 - 0.9*min(abs(kappa)/10,1);
    err_f2 = 1 - 0.6*min(abs(e_lat)/SPEED_CUT,1);
    int_f2 = 1 - 0.3*min(abs(e_lat_integral)/2,1);
    rate_f2 = 1 - 0.2*min(abs(e_lat_rate)/1,1);
    hist_f = 1 - 0.2*(mean(v_hist)/V_MAX);
    v = V_MAX * speed_f * err_f2 * int_f2 * rate_f2 * hist_f;
    v_hist = [v_hist(2:end), v];
    
    % 状态更新（完整运动学模型）
    state.x = state.x + v*cos(state.th)*dt1;
    state.y = state.y + v*sin(state.th)*dt1;
    state.th = wrapToPi(state.th + omega*dt1);
    xL1(k) = state.x; yL1(k) = state.y;
    eL1(k) = abs(e_lat);
    gL1(k) = gamma;
    
    % 实时动画（完整显示逻辑）
    if ~skip_animation
        addpoints(anim1, state.x, state.y);
        if mod(k,refresh_step)==0
            set(errTxt,'Position',[state.x,state.y+0.3],...
                      'String',sprintf('e=%.3f m',eL1(k)));
            xlim([state.x-win, state.x+win]);
            ylim([state.y-win, state.y+win]);
            drawnow limitrate; 
            pause(dt1*slow_factor);  % 原始速度控制
        end
    end
end

% 性能评估（完整输出）
RMSE1 = sqrt(mean(eL1.^2));
MAXE1 = max(eL1);
SS1 = mean(eL1(end-round(2/dt1):end));
INT1 = sum(eL1)*dt1;
fprintf('\n===== Fuzzy‑PP 评估结果 =====\n');
fprintf('RMSE: %.4f m | Max误差: %.4f m\n', RMSE1, MAXE1);
fprintf('稳态误差: %.4f m | 累计误差: %.4f m·s\n', SS1, INT1);


%% ==================== Part II: PUMA560 轨迹复现与评估 ======================
% ── 1. 自动缩放 ─────────────────────────────────────────────────────
reach_rad = 0.65;                          % PUMA560 可达半径 (m)
max_xy = max(vecnorm([x_ref1' y_ref1'], 2, 2));
SCALE  = 0.75 * reach_rad / max_xy;        % 留 25% 裕量
xs = x_ref1 * SCALE;
ys = y_ref1 * SCALE;

% ── 2. 轨迹中心平移到机器人基座 ────────────────────────────
xc = mean(xs); yc = mean(ys);              % 轨迹质心
xs_shift = xs - xc; ys_shift = ys - yc;    % 平移后坐标 (基座原点)

% ── 3. 载入模型并设置 base ────────────────────────────────────────
mdl_puma560; robot = p560;
robot.base = transl(xc, yc, 0);            % 将基座平移到原路径中心位置

% ── 4. 闭式逆解 & 关键帧 ────────────────────────────────────────────
z_tcp = 0.45;                              % TCP 高度 (m)
R_tcp = trotz(pi);                         % 朝 -Z
qs = zeros(N1,6); q_prev = zeros(1,6);

for i = 1:N1
    Tgoal = transl(xs_shift(i), ys_shift(i), z_tcp) * R_tcp;
    sol   = robot.ikine6s(Tgoal);
    if isempty(sol)
        error('ikine6s 无解, 请调节 SCALE 或 z_tcp');
    end
    [~, idx] = min(vecnorm(sol - q_prev, 2, 2));
    qs(i,:)  = sol(idx,:);
    q_prev   = qs(i,:);
end

% ── 5. 机械臂运动评估参数计算 ────────────────────────────────
% 关节空间参数
joint_vel = diff(qs) / dt1;                % 关节速度 (rad/s)
joint_accel = diff(joint_vel) / dt1;        % 关节加速度 (rad/s²)
joint_jerk = diff(joint_accel) / dt1;       % 关节急动度 (rad/s³)
stepnorm = vecnorm(diff(qs), 2, 2);         % 相邻关节位移范数

% 笛卡尔空间参数
tcp_pos = zeros(N1, 3);
for i = 1:N1
    tcp_pos(i, :) = robot.fkine(qs(i,:)).t';
end
tcp_vel = diff(tcp_pos) / dt1;              % 末端速度 (m/s)
tcp_accel = diff(tcp_vel) / dt1;            % 末端加速度 (m/s²)

% ── 6. 动画展示（支持跳过）──────────────────────────────────────
if ~skip_animation
    figure('Name','PUMA560 轨迹复现 (平移校准)','NumberTitle','off');
    robot.plot(qs(1,:), 'trail','b.','workspace',[-reach_rad reach_rad -reach_rad reach_rad 0 1]);
    view(135,25); hold on;
    plot3(xs_shift, ys_shift, z_tcp*ones(size(xs_shift)), 'k--', 'LineWidth',1.2);
    for i = 1:N1
        robot.animate(qs(i,:));
        drawnow limitrate;
    end
end

% ── 7. 详细评估输出 ───────────────────────────────────────────
fprintf('\n===== PUMA560 机械臂运动评估 =====\n');
fprintf('---------------- 关节空间参数 ----------------\n');
fprintf('最大相邻关节位移: %.4f rad\n', max(stepnorm));
fprintf('平均相邻关节位移: %.4f rad\n', mean(stepnorm));
fprintf('最大关节速度: %.4f rad/s\n', max(vecnorm(joint_vel, 2, 2)));
fprintf('平均关节速度: %.4f rad/s\n', mean(vecnorm(joint_vel, 2, 2)));
fprintf('最大关节加速度: %.4f rad/s²\n', max(vecnorm(joint_accel, 2, 2)));
fprintf('平均关节加速度: %.4f rad/s²\n', mean(vecnorm(joint_accel, 2, 2)));
fprintf('最大关节急动度: %.4f rad/s³\n', max(vecnorm(joint_jerk, 2, 2)));
fprintf('平均关节急动度: %.4f rad/s³\n', mean(vecnorm(joint_jerk, 2, 2)));

fprintf('\n---------------- 笛卡尔空间参数 ----------------\n');
fprintf('末端最大速度: %.4f m/s\n', max(vecnorm(tcp_vel, 2, 2)));
fprintf('末端平均速度: %.4f m/s\n', mean(vecnorm(tcp_vel, 2, 2)));
fprintf('末端最大加速度: %.4f m/s²\n', max(vecnorm(tcp_accel, 2, 2)));
fprintf('末端平均加速度: %.4f m/s²\n', mean(vecnorm(tcp_accel, 2, 2)));

%% End







% %% ==================== Part II: PUMA560 轨迹复现（姿态修正版） ====================
% % 轨迹适配参数
% reach_rad = 0.65; z_tcp = 0.75;
% max_xy = max(vecnorm([xL1; yL1],2));
% SCALE = 0.75 * reach_rad / max_xy;
% xs = xL1 * SCALE; ys = yL1 * SCALE;
% xc = mean(xs); yc = mean(ys);
% xs_shift = xs - xc; ys_shift = ys - yc;
% 
% % 机械臂初始化（姿态修正关键）
% mdl_puma560; robot = p560;
% new_base_height = 0.5; 
% robot.base = transl(xc, yc, new_base_height);  % 基座中心对准
% R_tcp = trotx(pi/2)*trotz(-pi/2);  % 修正末端姿态为垂直向下
% 
% % 逆运动学求解（完整实现）
% qs = zeros(N1,6); q_prev = qn; 
% for i = 1:N1
%     Tgoal = transl(xs_shift(i), ys_shift(i), z_tcp) * R_tcp;
%     sol = robot.ikine6s(Tgoal, 'ru');  % 闭式逆解
%     
%     if isempty(sol)
%         error('逆解失败在点%d: X=%.2f, Y=%.2f',i,xs_shift(i),ys_shift(i));
%     end
%     
%     % 选择最接近解确保连续性
%     [~, idx] = min(sum((sol - q_prev).^2, 2));
%     qs(i,:) = sol(idx,:);
%     q_prev = qs(i,:);
% end
% 
% % 轨迹平滑处理（完整滤波）
% qs = smoothdata(qs, 'gaussian', 15);
% 
% % 动画参数设置
% robot_slow_factor = 1;  % 机械臂速度因子（>1减速）
% refresh_step = 5;       % 跳帧数（提升流畅度）
% 
% % 创建独立动画窗口
% figure('Name','PUMA560轨迹复现','NumberTitle','off');
% robot.plot(qs(1,:), 'trail','r-',...
%     'workspace', [-1 1 -1 1 0 1]*reach_rad*1.5);
% hold on;
% plot3(xs_shift, ys_shift, z_tcp*ones(size(xs_shift)), 'k--', 'LineWidth',1.2);
% view(135,25);
% 
% % 速度控制动画循环（完整实现）
% tic;
% last_time = 0;
% for i = 1:refresh_step:N1
%     % 速度控制核心
%     while toc < last_time + dt1*refresh_step*robot_slow_factor
%         pause(0.001);
%     end
%     last_time = toc;
%     
%     % 更新机械臂状态
%     robot.animate(qs(i,:));
%     
%     % 动态显示末端轨迹
%     if mod(i,50) == 0
%         tee = robot.fkine(qs(i,:)).t;
%         plot3(tee(1), tee(2), tee(3), 'g.', 'MarkerSize',8);
%         drawnow limitrate;
%     end
% end
% 
% % 计算关节速度 (rad/step)
% joint_vel = diff(qs) / dt1;
% joint_vel_norm = vecnorm(joint_vel, 2, 2);
% 
% % 计算关节加速度 (rad/step²)
% joint_accel = diff(joint_vel) / dt1;
% joint_accel_norm = vecnorm(joint_accel, 2, 2);
% 
% % 计算关节抖动 (急动度 jerk) (rad/step³)
% joint_jerk = diff(joint_accel) / dt1;
% joint_jerk_norm = vecnorm(joint_jerk, 2, 2);
% 
% % 计算平滑度 (基于功率谱密度的逆)
% fft_data = abs(fft(qs, [], 1));
% smoothness = sum(fft_data(1:floor(end/2), :), 1) ./ sum(fft_data, 1);
% 
% % 计算末端执行器位置
% tcp_pos = zeros(size(qs, 1), 3);
% for i = 1:size(qs, 1)
%     tcp_pos(i, :) = robot.fkine(qs(i, :)).t';
% end
% 
% % 计算末端执行器速度 (m/step)
% tcp_vel = diff(tcp_pos) / dt1;
% tcp_vel_norm = vecnorm(tcp_vel, 2, 2);
% 
% % 计算末端执行器加速度 (m/step²)
% tcp_accel = diff(tcp_vel) / dt1;
% tcp_accel_norm = vecnorm(tcp_accel, 2, 2);
% 
% % 关节空间性能评估（增强输出）
% stepnorm = vecnorm(diff(qs),2,2);
% fprintf('\n===== 机械臂运动评估 =====\n');
% fprintf('最大关节变化率: %.4f rad/step\n', max(stepnorm));
% fprintf('平均关节变化率: %.4f rad/step\n', mean(stepnorm));
% fprintf('最大关节速度: %.4f rad/s\n', max(joint_vel_norm));
% fprintf('平均关节速度: %.4f rad/s\n', mean(joint_vel_norm));
% fprintf('最大关节加速度: %.4f rad/s²\n', max(joint_accel_norm));
% fprintf('平均关节加速度: %.4f rad/s²\n', mean(joint_accel_norm));
% fprintf('最大关节抖动: %.4f rad/s³\n', max(joint_jerk_norm));
% fprintf('平均关节抖动: %.4f rad/s³\n', mean(joint_jerk_norm));
% fprintf('\n');
% fprintf('关节平滑度指标 (越高越平滑):\n');
% fprintf('关节1: %.4f\n', smoothness(1));
% fprintf('关节2: %.4f\n', smoothness(2));
% fprintf('关节3: %.4f\n', smoothness(3));
% fprintf('关节4: %.4f\n', smoothness(4));
% fprintf('关节5: %.4f\n', smoothness(5));
% fprintf('关节6: %.4f\n', smoothness(6));
% fprintf('\n');
% fprintf('末端执行器最大速度: %.4f m/s\n', max(tcp_vel_norm));
% fprintf('末端执行器平均速度: %.4f m/s\n', mean(tcp_vel_norm));
% fprintf('末端执行器最大加速度: %.4f m/s²\n', max(tcp_accel_norm));
% fprintf('末端执行器平均加速度: %.4f m/s²\n', mean(tcp_accel_norm));
% 
% % 绘制关节速度和加速度曲线
% figure('Name','关节速度和加速度分析','NumberTitle','off');
% subplot(2,1,1);
% plot((1:length(joint_vel_norm))*dt1, joint_vel_norm);
% title('关节速度分析');
% xlabel('时间 (s)');
% ylabel('关节速度 (rad/s)');
% grid on;
% 
% subplot(2,1,2);
% plot((1:length(joint_accel_norm))*dt1, joint_accel_norm);
% title('关节加速度分析');
% xlabel('时间 (s)');
% ylabel('关节加速度 (rad/s²)');
% grid on;
% 
% % 绘制末端执行器速度和加速度曲线
% figure('Name','末端执行器速度和加速度分析','NumberTitle','off');
% subplot(2,1,1);
% plot((1:length(tcp_vel_norm))*dt1, tcp_vel_norm);
% title('末端执行器速度分析');
% xlabel('时间 (s)');
% ylabel('速度 (m/s)');
% grid on;
% 
% subplot(2,1,2);
% plot((1:length(tcp_accel_norm))*dt1, tcp_accel_norm);
% title('末端执行器加速度分析');
% xlabel('时间 (s)');
% ylabel('加速度 (m/s²)');
% grid on;    

























% % %=======可以演示但姿态不对==============================================================
% % % fuzzy_pp_and_robot_pid_final.m
% % % 集成 Fuzzy-PP 纯追踪演示 与 PUMA560 关节 PID 控制演示
% % % 采用闭式逆解 ikine6s，自动缩放+平移校准 (方式二)
% % % MATLAB2022b + Peter Corke Robotics Toolbox 10.4
% % %=====================================================================
% clc; clear; close all;
% 
% %% ==================== Part I: 生成参考轨迹 ==========================
% % 用户参数
% T1           = 60;        % 仿真时长 (s)
% dt1          = 0.02;      % 时间步长 (s)
% N1           = round(T1/dt1);
% V_MAX        = 0.6;       % 最高线速度 (m/s)
% OMEGA_LIM    = 2.5;       % 最大角速度 (rad/s)
% LOOK_BASE    = 0.6;       % 基础前瞻距离 (m)
% SPEED_CUT    = 0.3;       % 误差减速阈值 (m)
% slow_factor  = 0.5;       % 动画放慢倍数 (>=1)
% refresh_step = 1;         % 动画刷新步长
% 
% % 复杂路径生成（叠加谐波：圆 + 三次 + 二次）
% phi1   = linspace(0,2*pi,N1);
% x_ref1 = 5*cos(phi1) + 1.0*cos(3*phi1);
% y_ref1 = 5*sin(phi1) + 0.5*sin(2*phi1);
% 
% % 模糊增益 FIS 自动生成/加载
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：侧向误差 e
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N');
%     fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');
%     % 输入：航向误差 psi
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N');
%     fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small');
%     fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
%     % 规则表
%     ruleList = [
%         1 1 3 1 1;
%         1 2 3 1 1;
%         1 3 2 1 1;
%         2 1 3 1 1;
%         2 2 2 1 1;
%         2 3 1 1 1;
%         3 1 2 1 1;
%         3 2 1 1 1;
%         3 3 1 1 1];
%     fis_g = addRule(fis_g, ruleList);
%     writefis(fis_g, fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% 
% % 初始化状态 & 日志
% state.x = x_ref1(1);
% state.y = y_ref1(1);
% state.th = atan2(y_ref1(2)-y_ref1(1), x_ref1(2)-x_ref1(1));
% xL1 = zeros(1,N1); yL1 = zeros(1,N1);
% eL1 = zeros(1,N1); gL1 = zeros(1,N1);
% v_hist = zeros(1,5);
% e_lat_integral = 0; prev_e = 0; e_lat_hist = zeros(1,3);
% 
% % 动画准备
% figure('Name','Fuzzy‑PP Complex Demo'); hold on;
% plot(x_ref1,y_ref1,'k--','LineWidth',1.1);
% anim1 = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% win = 1.5;
% 
% % 主循环: Pure‑Pursuit + 模糊增益
% for k = 1:N1
%     % 查找最近点
%     dists = hypot(x_ref1-state.x, y_ref1-state.y);
%     [~, idx] = min(dists);
%     dx = x_ref1(idx)-state.x;
%     dy = y_ref1(idx)-state.y;
%     % 侧向误差 & 航向误差
%     e_lat = sin(state.th)*dx - cos(state.th)*dy;
%     psi_err = wrapToPi(atan2(dy,dx) - state.th);
%     % 误差率 & 积分
%     e_lat_rate = (e_lat - prev_e)/dt1;
%     prev_e = e_lat;
%     e_lat_hist = [e_lat_hist(2:end), e_lat];
%     e_lat_integral = e_lat_integral + e_lat*dt1;
%     % 曲率计算 (三点法)
%     if idx>1 && idx<N1-1
%         x1 = x_ref1(idx-1); y1 = y_ref1(idx-1);
%         x2 = x_ref1(idx);   y2 = y_ref1(idx);
%         x3 = x_ref1(idx+1); y3 = y_ref1(idx+1);
%         A = hypot(x2-x1, y2-y1);
%         B = hypot(x3-x2, y3-y2);
%         C = hypot(x3-x1, y3-y1);
%         s = (A+B+C)/2;
%         area = sqrt(max(s*(s-A)*(s-B)*(s-C),0));
%         if area > 1e-6
%             kappa = 4*area/(A*B*C);
%         else
%             kappa = 0;
%         end
%     else
%         kappa = 0;
%     end
%     % 动态前瞻距离
%     curv_f = 1 - 0.6*min(abs(kappa)/10,1);
%     err_f  = 1 - 0.2*min(abs(e_lat)/SPEED_CUT,1);
%     int_f  = 1 - 0.2*min(abs(e_lat_integral)/1,1);
%     rate_f = 1 - 0.1*min(abs(e_lat_rate)/1,1);
%     Ld = max(0.2, LOOK_BASE*curv_f*err_f*(1-int_f)*rate_f);
%     % 前瞻点搜索
%     dist_acc = 0;
%     idx_l = idx;
%     while dist_acc < Ld && idx_l < N1
%         dist_acc = dist_acc + hypot(x_ref1(idx_l+1)-x_ref1(idx_l), y_ref1(idx_l+1)-y_ref1(idx_l));
%         idx_l = idx_l + 1;
%     end
%     xt = x_ref1(idx_l); yt = y_ref1(idx_l);
%     alpha = wrapToPi(atan2(yt-state.y, xt-state.x) - state.th);
%     % Pure-Pursuit 曲率及角速度
%     if Ld > 1e-6
%         kappa_pp = 2*sin(alpha)/Ld;
%     else
%         kappa_pp = 0;
%     end
%     omega_pp = V_MAX * kappa_pp;
%     % 模糊增益 γ
%     e_n   = max(min(e_lat/1.5,1),-1);
%     psi_n = max(min(psi_err/pi,1),-1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     if abs(kappa)>0.2
%         gamma = gamma * (1 + 0.6*min(abs(kappa)/10,1));
%     end
%     if abs(e_lat)>0.5 || abs(e_lat_integral)>1 || abs(e_lat_rate)>0.5
%         gamma = gamma * 2.5;
%     end
%     omega = max(min(gamma*omega_pp, OMEGA_LIM), -OMEGA_LIM);
%     % 自适应线速度
%     speed_f = 1 - 0.9*min(abs(kappa)/10,1);
%     err_sf  = 1 - 0.6*min(abs(e_lat)/SPEED_CUT,1);
%     int_sf  = 1 - 0.3*min(abs(e_lat_integral)/2,1);
%     rate_sf = 1 - 0.2*min(abs(e_lat_rate)/1,1);
%     hist_f  = 1 - 0.2*(mean(v_hist)/V_MAX);
%     v = V_MAX * speed_f * err_sf * int_sf * rate_sf * hist_f;
%     v_hist = [v_hist(2:end), v];
%     % 状态更新
%     state.x  = state.x + v*cos(state.th)*dt1;
%     state.y  = state.y + v*sin(state.th)*dt1;
%     state.th = wrapToPi(state.th + omega*dt1);
%     % 日志记录
%     xL1(k) = state.x;
%     yL1(k) = state.y;
%     eL1(k) = abs(e_lat);
%     gL1(k) = gamma;
%     % 动画更新
%     addpoints(anim1, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], 'String',sprintf('e=%.3f m',eL1(k)));
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate; pause(dt1*slow_factor);
%     end
% end
% % 性能评估
% RMSE1 = sqrt(mean(eL1.^2));
% MAXE1 = max(eL1);
% SS1   = mean(eL1(end-round(2/dt1):end));
% INT1  = sum(eL1)*dt1;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE1);
% fprintf('Max E  = %.4f m\n', MAXE1);
% fprintf('SS Err = %.4f m\n', SS1);
% fprintf('Int E  = %.4f m·s\n', INT1);
% 
% 
% 
% % %% ==================== Part II: PUMA560 轨迹复现 ======================
% % % ── 1. 自动缩放 ─────────────────────────────────────────────────────
% reach_rad = 0.65;                          % PUMA560 可达半径 (m)
% max_xy = max( vecnorm([x_ref1' y_ref1'],2,2) );
% SCALE  = 0.75 * reach_rad / max_xy;        % 留 25% 裕量
% xs = x_ref1 * SCALE;
% ys = y_ref1 * SCALE;
% 
% % ── 2. 轨迹中心平移到机器人基座 (方式二) ────────────────────────────
% xc = mean(xs); yc = mean(ys);              % 轨迹质心
% xs_shift = xs - xc; ys_shift = ys - yc;    % 平移后坐标 (基座原点)
% 
% % ── 3. 载入模型并设置 base ────────────────────────────────────────
% mdl_puma560; robot = p560;
% robot.base = transl(xc, yc, 0);            % 将基座平移到原路径中心位置
% 
% % ── 4. 闭式逆解 & 关键帧 ────────────────────────────────────────────
% z_tcp = 0.45;                              % TCP 高度 (m)
% R_tcp = trotz(pi);                         % 朝 -Z
% qs = zeros(N1,6); q_prev = zeros(1,6);
% 
% for i = 1:N1
%     Tgoal = transl(xs_shift(i), ys_shift(i), z_tcp) * R_tcp;
%     sol   = robot.ikine6s(Tgoal);
%     if isempty(sol)
%         error('ikine6s 无解, 请调节 SCALE 或 z_tcp');
%     end
%     [~, idx] = min(vecnorm(sol - q_prev,2,2));
%     qs(i,:)  = sol(idx,:);
%     q_prev   = qs(i,:);
% end
% 
% % ── 5. 动画展示 ───────────────────────────────────────────────────
% figure('Name','PUMA560 轨迹复现 (平移校准)','NumberTitle','off');
% robot.plot(qs(1,:), 'trail','b.','workspace',[-reach_rad reach_rad -reach_rad reach_rad 0 1]);
% view(135,25); hold on;
% plot3(xs_shift, ys_shift, z_tcp*ones(size(xs_shift)), 'k--', 'LineWidth',1.2);
% for i = 1:N1
%     robot.animate(qs(i,:));
%     drawnow limitrate;
% end
% 
% % ── 6. 平滑性评估 ─────────────────────────────────────────────────
% stepnorm = vecnorm(diff(qs),2,2);
% fprintf('\n===== 轨迹评估 =====\n');
% fprintf('最大相邻步范数: %.4f rad\n', max(stepnorm));
% fprintf('平均相邻步范数: %.4f rad\n', mean(stepnorm));
% 
% %% End

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
% % %% ==================== Part II: PUMA560 轨迹复现（姿态修正版） ====================
% % % 轨迹适配参数
% % reach_rad = 0.65; z_tcp = 0.9;
% % max_xy = max(vecnorm([xL1; yL1],2));
% % SCALE = 0.75 * reach_rad / max_xy;
% % xs = xL1 * SCALE; ys = yL1 * SCALE;
% % xc = mean(xs); yc = mean(ys);
% % xs_shift = xs - xc; ys_shift = ys - yc;
% % 
% % % 机械臂初始化（姿态修正关键）
% % mdl_puma560; robot = p560;
% % new_base_height = 0.7; 
% % robot.base = transl(xc, yc, new_base_height);  % 基座中心对准
% % R_tcp = trotx(pi/2)*trotz(-pi/2);  % 修正末端姿态为垂直向下
% % 
% % % 逆运动学求解（完整实现）
% % qs = zeros(N1,6); q_prev = qn; 
% % for i = 1:N1
% %     Tgoal = transl(xs_shift(i), ys_shift(i), z_tcp) * R_tcp;
% %     sol = robot.ikine6s(Tgoal, 'ru');  % 闭式逆解
% %     
% %     if isempty(sol)
% %         error('逆解失败在点%d: X=%.2f, Y=%.2f',i,xs_shift(i),ys_shift(i));
% %     end
% %     
% %     % 选择最接近解确保连续性
% %     [~, idx] = min(sum((sol - q_prev).^2, 2));
% %     qs(i,:) = sol(idx,:);
% %     q_prev = qs(i,:);
% % end
% % 
% % % 轨迹平滑处理（完整滤波）
% % qs = smoothdata(qs, 'gaussian', 15);
% % 
% % % 动画参数设置
% % robot_slow_factor = 1;  % 机械臂速度因子（>1减速）
% % refresh_step = 5;       % 跳帧数（提升流畅度）
% % 
% % % 创建独立动画窗口
% % figure('Name','PUMA560轨迹复现','NumberTitle','off');
% % robot.plot(qs(1,:), 'trail','r-',...
% %     'workspace', [-1 1 -1 1 0 1]*reach_rad*1.5);
% % hold on;
% % plot3(xs_shift, ys_shift, z_tcp*ones(size(xs_shift)), 'k--', 'LineWidth',1.2);
% % view(135,25);
% % 
% % % 速度控制动画循环（完整实现）
% % tic;
% % last_time = 0;
% % for i = 1:refresh_step:N1
% %     % 速度控制核心
% %     while toc < last_time + dt1*refresh_step*robot_slow_factor
% %         pause(0.001);
% %     end
% %     last_time = toc;
% %     
% %     % 更新机械臂状态
% %     robot.animate(qs(i,:));
% %     
% %     % 动态显示末端轨迹
% %     if mod(i,50) == 0
% %         tee = robot.fkine(qs(i,:)).t;
% %         plot3(tee(1), tee(2), tee(3), 'g.', 'MarkerSize',8);
% %         drawnow limitrate;
% %     end
% % end
% % 
% % % 关节空间性能评估（完整输出）
% % stepnorm = vecnorm(diff(qs),2,2);
% % fprintf('\n===== 机械臂运动评估 =====\n');
% % fprintf('最大关节变化率: %.4f rad/step\n', max(stepnorm));
% % fprintf('平均关节变化率: %.4f rad/step\n', mean(stepnorm));