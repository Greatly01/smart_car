%% 跟踪快且及时，误差最小
clc; clear; close all;

%% =============================================================
% 🚗 动态单轨 + LQR 前馈/反馈（含侧偏角）
% - 新增动态演示 (animatedline) & 更多评估指标
% =============================================================

%% 0) 参数 ------------------------------------------------------
T = 60;       dt = 0.02;    N = round(T/dt);

% 车辆物理
m  = 25;  Iz = 1.2;  Lf = 0.16;  Lr = 0.14;  L = Lf+Lr;
Cf = 1200; Cr = 1200;

v_set   = 0.8;   acc_max = 1.0;
delta_max = deg2rad(30);  rate_max = deg2rad(300); exec_delay = 0.05;

% LQR 权重
Q = diag([3, 0.6, 4, 0.25]);   R = 0.5;

% 噪声
pos_sd = 0.002; psi_sd = deg2rad(0.05);

%% 1) 参考轨迹 (圆) ---------------------------------------------
r_traj = 5;   s = linspace(0, 2*pi*r_traj, N);
ref.x   = r_traj * cos(s/r_traj);
ref.y   = r_traj * sin(s/r_traj);
ref.psi = unwrap(atan2(diff([ref.y ref.y(1)]), diff([ref.x ref.x(1)])));
ref.curv= ones(1,N)/r_traj;

%% 2) LQR 增益 ---------------------------------------------------
Vref = v_set;
A = [  0,                    1,              Vref,         0; 
       0, -(2*Cf+2*Cr)/(m*Vref),        0, -(2*Cf*Lf-2*Cr*Lr)/(m*Vref)-Vref;
       0,                    0,              0,           1;
       0, -(2*Cf*Lf-2*Cr*Lr)/(Iz*Vref), 0, -(2*Cf*Lf^2+2*Cr*Lr^2)/(Iz*Vref) ];
B = [0; 2*Cf/m; 0; 2*Cf*Lf/Iz];
K = lqr(A,B,Q,R);

%% 3) 初始化 ------------------------------------------------------
state = struct('x',r_traj,'y',0,'psi',pi/2,'vx',0,'vy',0,'r',0,'delta',0);
queue = zeros(1, round(exec_delay/dt)+1);

% 日志
log = struct('x',zeros(1,N),'y',zeros(1,N),'ey',zeros(1,N), ...
             'psi_err',zeros(1,N),'delta',zeros(1,N),'ay',zeros(1,N));
%% 动态演示设置 --------------------------------------------------
slow_factor = 1;           % 动画放慢倍数 (>=1)
fig = figure('Name','Dynamic Path Tracking');
plot(ref.x, ref.y, 'k--','LineWidth',1.2); hold on; grid on; axis equal;
vehLine = animatedline('Color','b','LineWidth',1.8);
errText = text(ref.x(1), ref.y(1)+0.5, 'e_y = 0.00 m','FontSize',12,'Color','r');
window_radius = 1.2;        % 相机跟随窗口尺寸 (m)

draw_step = 1;              % 每步刷新一次

%% 4) 主循环 -----------------------------------------------------
for k = 1:N
    %% 4‑1 噪声观测
    x_m = state.x + pos_sd*randn;  y_m = state.y + pos_sd*randn;
    psi_m = state.psi + psi_sd*randn;

    %% 4‑2 匹配参考点 & 误差
    [~,idx] = min(hypot(ref.x - x_m, ref.y - y_m));
    dx = x_m - ref.x(idx);  dy = y_m - ref.y(idx);
    ey = -sin(ref.psi(idx))*dx + cos(ref.psi(idx))*dy;
    psi_err = wrapToPi(psi_m - ref.psi(idx));

    %% 4‑3 LQR 控制
    X = [ey; state.vy; psi_err; state.r];
    delta_ff = atan(L * ref.curv(idx));
    delta_des = delta_ff - K*X;
    delta_des = max(min(delta_des, delta_max), -delta_max);

    %% 4‑4 执行器限幅+延迟
    step = max(min(delta_des - queue(end), rate_max*dt), -rate_max*dt);
    delta_cmd = queue(end) + step; queue = [queue(2:end) delta_cmd];
    delta_act = queue(1); state.delta = delta_act;

    %% 4‑5 纵向速度控制
    acc = max(min(v_set - state.vx, acc_max*dt), -acc_max*dt); state.vx = state.vx + acc;

    %% 4‑6 动态单轨更新
    alpha_f = atan2(state.vy + Lf*state.r, state.vx) - delta_act;
    alpha_r = atan2(state.vy - Lr*state.r, state.vx);
    Fyf = -Cf*alpha_f; Fyr = -Cr*alpha_r;
    ay  = (Fyf + Fyr)/m;                  % 侧向加速度记录

    state.vy = state.vy + (Fyf + Fyr)/m * dt - state.r*state.vx*dt;
    state.r  = state.r  + (Lf*Fyf - Lr*Fyr)/Iz * dt;
    state.psi= wrapToPi(state.psi + state.r*dt);
    state.x  = state.x + (state.vx*cos(state.psi) - state.vy*sin(state.psi))*dt;
    state.y  = state.y + (state.vx*sin(state.psi) + state.vy*cos(state.psi))*dt;

    %% 4‑7 记录 & 动画
    log.x(k)=state.x; log.y(k)=state.y; log.ey(k)=ey; log.psi_err(k)=psi_err; log.delta(k)=delta_act; log.ay(k)=ay;
            addpoints(vehLine, state.x, state.y);
    if mod(k,draw_step)==0
        % 更新误差文本
        set(errText,'Position',[state.x,state.y+0.5],'String',sprintf('e_y = %.3f m',ey));
        % 相机跟随
        axis([state.x-window_radius, state.x+window_radius, state.y-window_radius, state.y+window_radius]);
        drawnow limitrate;
        pause(dt*slow_factor);
    end
end

%% 5) 性能评估 --------------------------------------------------
rmse = sqrt(mean(log.ey.^2));
max_e = max(abs(log.ey));
ss    = mean(abs(log.ey(end- round(2/dt):end)));
int_e = sum(abs(log.ey))*dt;
% 控制能量 & 侧向加速度
u_energy = sum(log.delta.^2)*dt;
max_delta_rate = max(abs(diff(log.delta))/dt);
max_ay = max(abs(log.ay));

fprintf('\n[动态 LQR 评估] RMSE=%.3f m | Max=%.3f m | SS=%.3f m | Int=%.3f m·s\n', rmse, max_e, ss, int_e);
fprintf('Control Energy=%.4f (rad²·s) | Max δ̇=%.2f °/s | Max a_y=%.2f m/s²\n', u_energy, rad2deg(max_delta_rate), max_ay);

%% 6) 静态结果图 -------------------------------------------------
figure; plot(ref.x,ref.y,'k--','LineWidth',1.2); hold on; plot(log.x,log.y,'b','LineWidth',1.5);
axis equal; xlim([-6 6]); ylim([-6 6]); grid on; legend('参考','实际'); title('轨迹对比');
figure; subplot(3,1,1); plot((0:N-1)*dt, log.ey,'LineWidth',1.2); ylabel('e_y(m)'); title('横向误差'); grid on;
subplot(3,1,2); plot((0:N-1)*dt, rad2deg(log.psi_err)); ylabel('ψ_err(°)'); grid on; title('航向误差');
subplot(3,1,3); plot((0:N-1)*dt, rad2deg(log.delta)); ylabel('δ(°)'); xlabel('t(s)'); grid on; title('前轮转角');



















%误差未可视化，依旧较小
% clc; clear; close all;

%% =============================================================
% 🚗 动态单轨 + LQR 前馈/反馈（含侧偏角）
% - 新增动态演示 (animatedline) & 更多评估指标
% =============================================================

%% 0) 参数 ------------------------------------------------------
% T = 60;       dt = 0.02;    N = round(T/dt);
% 
% % 车辆物理
% m  = 25;  Iz = 1.2;  Lf = 0.16;  Lr = 0.14;  L = Lf+Lr;
% Cf = 1200; Cr = 1200;
% 
% v_set   = 0.8;   acc_max = 1.0;
% delta_max = deg2rad(30);  rate_max = deg2rad(300); exec_delay = 0.05;
% 
% % LQR 权重
% Q = diag([3, 0.6, 4, 0.25]);   R = 0.5;
% 
% % 噪声
% pos_sd = 0.002; psi_sd = deg2rad(0.05);
% 
% %% 1) 参考轨迹 (圆) ---------------------------------------------
% r_traj = 5;   s = linspace(0, 2*pi*r_traj, N);
% ref.x   = r_traj * cos(s/r_traj);
% ref.y   = r_traj * sin(s/r_traj);
% ref.psi = unwrap(atan2(diff([ref.y ref.y(1)]), diff([ref.x ref.x(1)])));
% ref.curv= ones(1,N)/r_traj;
% 
% %% 2) LQR 增益 ---------------------------------------------------
% Vref = v_set;
% A = [  0,                    1,              Vref,         0; 
%        0, -(2*Cf+2*Cr)/(m*Vref),        0, -(2*Cf*Lf-2*Cr*Lr)/(m*Vref)-Vref;
%        0,                    0,              0,           1;
%        0, -(2*Cf*Lf-2*Cr*Lr)/(Iz*Vref), 0, -(2*Cf*Lf^2+2*Cr*Lr^2)/(Iz*Vref) ];
% B = [0; 2*Cf/m; 0; 2*Cf*Lf/Iz];
% K = lqr(A,B,Q,R);
% 
% %% 3) 初始化 ------------------------------------------------------
% state = struct('x',r_traj,'y',0,'psi',pi/2,'vx',0,'vy',0,'r',0,'delta',0);
% queue = zeros(1, round(exec_delay/dt)+1);
% 
% % 日志
% log = struct('x',zeros(1,N),'y',zeros(1,N),'ey',zeros(1,N), ...
%              'psi_err',zeros(1,N),'delta',zeros(1,N),'ay',zeros(1,N));
% %% 动态演示设置 --------------------------------------------------
% slow_factor = 1;           % 动画放慢倍数 (>=1)，越大越慢
% fig = figure('Name','Dynamic Path Tracking');
% plot(ref.x, ref.y, 'r--'); hold on; grid on; axis equal;
% vehLine = animatedline('Color','b','LineWidth',1.5);
% draw_step = 2;              % 每 draw_step 步刷新一次
% 
% %% 4) 主循环 -----------------------------------------------------
% for k = 1:N
%     %% 4‑1 噪声观测
%     x_m = state.x + pos_sd*randn;  y_m = state.y + pos_sd*randn;
%     psi_m = state.psi + psi_sd*randn;
% 
%     %% 4‑2 匹配参考点 & 误差
%     [~,idx] = min(hypot(ref.x - x_m, ref.y - y_m));
%     dx = x_m - ref.x(idx);  dy = y_m - ref.y(idx);
%     ey = -sin(ref.psi(idx))*dx + cos(ref.psi(idx))*dy;
%     psi_err = wrapToPi(psi_m - ref.psi(idx));
% 
%     %% 4‑3 LQR 控制
%     X = [ey; state.vy; psi_err; state.r];
%     delta_ff = atan(L * ref.curv(idx));
%     delta_des = delta_ff - K*X;
%     delta_des = max(min(delta_des, delta_max), -delta_max);
% 
%     %% 4‑4 执行器限幅+延迟
%     step = max(min(delta_des - queue(end), rate_max*dt), -rate_max*dt);
%     delta_cmd = queue(end) + step; queue = [queue(2:end) delta_cmd];
%     delta_act = queue(1); state.delta = delta_act;
% 
%     %% 4‑5 纵向速度控制
%     acc = max(min(v_set - state.vx, acc_max*dt), -acc_max*dt); state.vx = state.vx + acc;
% 
%     %% 4‑6 动态单轨更新
%     alpha_f = atan2(state.vy + Lf*state.r, state.vx) - delta_act;
%     alpha_r = atan2(state.vy - Lr*state.r, state.vx);
%     Fyf = -Cf*alpha_f; Fyr = -Cr*alpha_r;
%     ay  = (Fyf + Fyr)/m;                  % 侧向加速度记录
% 
%     state.vy = state.vy + (Fyf + Fyr)/m * dt - state.r*state.vx*dt;
%     state.r  = state.r  + (Lf*Fyf - Lr*Fyr)/Iz * dt;
%     state.psi= wrapToPi(state.psi + state.r*dt);
%     state.x  = state.x + (state.vx*cos(state.psi) - state.vy*sin(state.psi))*dt;
%     state.y  = state.y + (state.vx*sin(state.psi) + state.vy*cos(state.psi))*dt;
% 
%     %% 4‑7 记录 & 动画
%     log.x(k)=state.x; log.y(k)=state.y; log.ey(k)=ey; log.psi_err(k)=psi_err; log.delta(k)=delta_act; log.ay(k)=ay;
%         addpoints(vehLine, state.x, state.y);
%     if mod(k,draw_step)==0
%         drawnow limitrate;
%         pause(dt*slow_factor);   % 放慢动画
%     end
% end
% 
% 
% %% 5) 性能评估 --------------------------------------------------
% rmse = sqrt(mean(log.ey.^2));
% max_e = max(abs(log.ey));
% ss    = mean(abs(log.ey(end- round(2/dt):end)));
% int_e = sum(abs(log.ey))*dt;
% % 控制能量 & 侧向加速度
% u_energy = sum(log.delta.^2)*dt;
% max_delta_rate = max(abs(diff(log.delta))/dt);
% max_ay = max(abs(log.ay));
% 
% fprintf('\n[动态 LQR 评估] RMSE=%.3f m | Max=%.3f m | SS=%.3f m | Int=%.3f m·s\n', rmse, max_e, ss, int_e);
% fprintf('Control Energy=%.4f (rad²·s) | Max δ̇=%.2f °/s | Max a_y=%.2f m/s²\n', u_energy, rad2deg(max_delta_rate), max_ay);
% 
% %% 6) 静态结果图 -------------------------------------------------
% figure; plot(ref.x,ref.y,'r--',log.x,log.y,'b'); axis equal; grid on; legend('参考','实际'); title('轨迹对比');
% figure; subplot(3,1,1); plot((0:N-1)*dt, log.ey); ylabel('e_y(m)'); title('横向误差'); grid on;
% subplot(3,1,2); plot((0:N-1)*dt, rad2deg(log.psi_err)); ylabel('ψ_err(°)'); grid on; title('航向误差');
% subplot(3,1,3); plot((0:N-1)*dt, rad2deg(log.delta)); ylabel('δ(°)'); xlabel('t(s)'); grid on; title('前轮转角');


















% 误差小，演示快
% clc; clear; close all;
% 
% %% =============================================================
% % 🚗 动态单轨 + LQR 前馈/反馈（含侧偏角）
% % - 新增动态演示 (animatedline) & 更多评估指标
% % =============================================================
% 
% %% 0) 参数 ------------------------------------------------------
% T = 60;       dt = 0.02;    N = round(T/dt);
% 
% % 车辆物理
% m  = 25;  Iz = 1.2;  Lf = 0.16;  Lr = 0.14;  L = Lf+Lr;
% Cf = 1200; Cr = 1200;
% 
% v_set   = 0.8;   acc_max = 1.0;
% delta_max = deg2rad(30);  rate_max = deg2rad(300); exec_delay = 0.05;
% 
% % LQR 权重
% Q = diag([3, 0.6, 4, 0.25]);   R = 0.5;
% 
% % 噪声
% pos_sd = 0.002; psi_sd = deg2rad(0.05);
% 
% %% 1) 参考轨迹 (圆) ---------------------------------------------
% r_traj = 5;   s = linspace(0, 2*pi*r_traj, N);
% ref.x   = r_traj * cos(s/r_traj);
% ref.y   = r_traj * sin(s/r_traj);
% ref.psi = unwrap(atan2(diff([ref.y ref.y(1)]), diff([ref.x ref.x(1)])));
% ref.curv= ones(1,N)/r_traj;
% 
% %% 2) LQR 增益 ---------------------------------------------------
% Vref = v_set;
% A = [  0,                    1,              Vref,         0; 
%        0, -(2*Cf+2*Cr)/(m*Vref),        0, -(2*Cf*Lf-2*Cr*Lr)/(m*Vref)-Vref;
%        0,                    0,              0,           1;
%        0, -(2*Cf*Lf-2*Cr*Lr)/(Iz*Vref), 0, -(2*Cf*Lf^2+2*Cr*Lr^2)/(Iz*Vref) ];
% B = [0; 2*Cf/m; 0; 2*Cf*Lf/Iz];
% K = lqr(A,B,Q,R);
% 
% %% 3) 初始化 ------------------------------------------------------
% state = struct('x',r_traj,'y',0,'psi',pi/2,'vx',0,'vy',0,'r',0,'delta',0);
% queue = zeros(1, round(exec_delay/dt)+1);
% 
% % 日志
% log = struct('x',zeros(1,N),'y',zeros(1,N),'ey',zeros(1,N), ...
%              'psi_err',zeros(1,N),'delta',zeros(1,N),'ay',zeros(1,N));
% %% 动态演示设置 --------------------------------------------------
% fig = figure('Name','Dynamic Path Tracking');
% plot(ref.x, ref.y, 'r--'); hold on; grid on; axis equal;
% vehLine = animatedline('Color','b','LineWidth',1.5);
% 
% %% 4) 主循环 -----------------------------------------------------
% for k = 1:N
%     %% 4‑1 噪声观测
%     x_m = state.x + pos_sd*randn;  y_m = state.y + pos_sd*randn;
%     psi_m = state.psi + psi_sd*randn;
% 
%     %% 4‑2 匹配参考点 & 误差
%     [~,idx] = min(hypot(ref.x - x_m, ref.y - y_m));
%     dx = x_m - ref.x(idx);  dy = y_m - ref.y(idx);
%     ey = -sin(ref.psi(idx))*dx + cos(ref.psi(idx))*dy;
%     psi_err = wrapToPi(psi_m - ref.psi(idx));
% 
%     %% 4‑3 LQR 控制
%     X = [ey; state.vy; psi_err; state.r];
%     delta_ff = atan(L * ref.curv(idx));
%     delta_des = delta_ff - K*X;
%     delta_des = max(min(delta_des, delta_max), -delta_max);
% 
%     %% 4‑4 执行器限幅+延迟
%     step = max(min(delta_des - queue(end), rate_max*dt), -rate_max*dt);
%     delta_cmd = queue(end) + step; queue = [queue(2:end) delta_cmd];
%     delta_act = queue(1); state.delta = delta_act;
% 
%     %% 4‑5 纵向速度控制
%     acc = max(min(v_set - state.vx, acc_max*dt), -acc_max*dt); state.vx = state.vx + acc;
% 
%     %% 4‑6 动态单轨更新
%     alpha_f = atan2(state.vy + Lf*state.r, state.vx) - delta_act;
%     alpha_r = atan2(state.vy - Lr*state.r, state.vx);
%     Fyf = -Cf*alpha_f; Fyr = -Cr*alpha_r;
%     ay  = (Fyf + Fyr)/m;                  % 侧向加速度记录
% 
%     state.vy = state.vy + (Fyf + Fyr)/m * dt - state.r*state.vx*dt;
%     state.r  = state.r  + (Lf*Fyf - Lr*Fyr)/Iz * dt;
%     state.psi= wrapToPi(state.psi + state.r*dt);
%     state.x  = state.x + (state.vx*cos(state.psi) - state.vy*sin(state.psi))*dt;
%     state.y  = state.y + (state.vx*sin(state.psi) + state.vy*cos(state.psi))*dt;
% 
%     %% 4‑7 记录 & 动画
%     log.x(k)=state.x; log.y(k)=state.y; log.ey(k)=ey; log.psi_err(k)=psi_err; log.delta(k)=delta_act; log.ay(k)=ay;
%     addpoints(vehLine, state.x, state.y);
%     if mod(k,5)==0, drawnow limitrate; end
% end
% 
% %% 5) 性能评估 --------------------------------------------------
% rmse = sqrt(mean(log.ey.^2));
% max_e = max(abs(log.ey));
% ss    = mean(abs(log.ey(end- round(2/dt):end)));
% int_e = sum(abs(log.ey))*dt;
% % 控制能量 & 侧向加速度
% u_energy = sum(log.delta.^2)*dt;
% max_delta_rate = max(abs(diff(log.delta))/dt);
% max_ay = max(abs(log.ay));
% 
% fprintf('\n[动态 LQR 评估] RMSE=%.3f m | Max=%.3f m | SS=%.3f m | Int=%.3f m·s\n', rmse, max_e, ss, int_e);
% fprintf('Control Energy=%.4f (rad²·s) | Max δ̇=%.2f °/s | Max a_y=%.2f m/s²\n', u_energy, rad2deg(max_delta_rate), max_ay);
% 
% %% 6) 静态结果图 -------------------------------------------------
% figure; plot(ref.x,ref.y,'r--',log.x,log.y,'b'); axis equal; grid on; legend('参考','实际'); title('轨迹对比');
% figure; subplot(3,1,1); plot((0:N-1)*dt, log.ey); ylabel('e_y(m)'); title('横向误差'); grid on;
% subplot(3,1,2); plot((0:N-1)*dt, rad2deg(log.psi_err)); ylabel('ψ_err(°)'); grid on; title('航向误差');
% subplot(3,1,3); plot((0:N-1)*dt, rad2deg(log.delta)); ylabel('δ(°)'); xlabel('t(s)'); grid on; title('前轮转角');
% 
% 
% 
















%%误差也小
% clc; clear; close all;
% 
% %% =============================================================
% %  🚗  工程级路径跟踪控制 —— 动态单轨(含侧偏角) + LQR 前馈/反馈
% %  -------------------------------------------------------------
% %  • 车辆模型 : 线性化动态单轨 (侧向速度 v_y 与偏航速率 r)
% %      状态向量  X = [e_y  v_y  ψ_err  r]ʰ  (4x1)
% %  • 控制器   : δ = δ_ff  − K·X   (LQR)
% %  • 轨迹      : 圆 r=5 m (可替换 CSV/GPS)
% %  • 工程要素  : 速率/幅值/延迟限幅，传感器高斯噪声，纵向匀速 PI
% % =============================================================
% 
% %% 0) 常量 & 车辆物理参数 ---------------------------------------
% T = 60;      dt = 0.02;   N = round(T/dt);
% 
% m  = 25;      % 车辆质量 (kg)
% Iz = 1.2;     % 绕 z 转动惯量 (kg·m²)
% Lf = 0.16;    % 前轴距质心距离 (m)
% Lr = 0.14;    % 后轴距质心距离 (m)
% L  = Lf + Lr; % 轴距
% Cf = 1200;    % 前轮侧偏刚度 (N/rad)
% Cr = 1200;    % 后轮侧偏刚度 (N/rad)
% 
% v_set   = 0.8;    % 目标纵向速度 (m/s)
% acc_max = 1.0;    % 加速度限 (m/s²)
% 
% delta_max = deg2rad(30);   % 转角幅值限
% rate_max  = deg2rad(300);  % 转角速率限
% exec_delay= 0.05;          % 延迟 (s)
% 
% %% 1) 参考轨迹 (任意，可替换) -----------------------------------
% r_traj = 5; s = linspace(0, 2*pi*r_traj, N);
% ref.x = r_traj * cos(s/r_traj);
% ref.y = r_traj * sin(s/r_traj);
% ref.psi = unwrap(atan2(diff([ref.y ref.y(1)]), diff([ref.x ref.x(1)])));
% ref.curv= ones(1,N)/r_traj;   % κ_ref
% 
% %% 2) 设计 LQR (线性化动态模型) ---------------------------------
% Vref = v_set;     % 在线性化点采用恒速
% A = [  0,                    1,              Vref,         0;
%        0, -(2*Cf+2*Cr)/(m*Vref),        0, -(2*Cf*Lf-2*Cr*Lr)/(m*Vref)-Vref;
%        0,                    0,              0,           1;
%        0, -(2*Cf*Lf-2*Cr*Lr)/(Iz*Vref), 0, -(2*Cf*Lf^2+2*Cr*Lr^2)/(Iz*Vref) ];
% B = [0; 2*Cf/m; 0; 2*Cf*Lf/Iz];
% 
% Q = diag([2, 0.5, 4, 0.2]);   % 可调 (纵向误差/侧向速度/航向误差/偏航速率)
% R = 0.6;                       % 转角权重
% K = lqr(A, B, Q, R);
% 
% %% 3) 状态结构 --------------------------------------------------
% state = struct('x',r_traj,'y',0,'psi',pi/2,'vx',0,'vy',0,'r',0,'delta',0);
% q_len = round(exec_delay/dt)+1; queue = zeros(1,q_len);
% 
% %% 4) 日志 ------------------------------------------------------
% log = struct('x',zeros(1,N),'y',zeros(1,N),'ey',zeros(1,N), ...
%              'psi_err',zeros(1,N),'delta',zeros(1,N));
% 
% %% 5) 控制 & 仿真循环 -------------------------------------------
% for k = 1:N
%     %% 5‑1  带噪观测
%     x_m = state.x + 0.002*randn;  y_m = state.y + 0.002*randn;
%     psi_m = state.psi + deg2rad(0.05)*randn;
% 
%     %% 5‑2  最近参考点 & 误差坐标
%     [~,idx] = min(hypot(ref.x - x_m, ref.y - y_m));
%     dx = x_m - ref.x(idx); dy = y_m - ref.y(idx);
%     ey = -sin(ref.psi(idx))*dx + cos(ref.psi(idx))*dy;   % 横向偏差
%     psi_err = wrapToPi(psi_m - ref.psi(idx));
% 
%     %% 5‑3  线性状态向量 (使用测量 vy ≈ state.vy, r ≈ state.r)
%     X = [ey; state.vy; psi_err; state.r];
% 
%     %% 5‑4  曲率前馈 & LQR 反馈
%     delta_ff = atan(L * ref.curv(idx));
%     delta_fb = -K * X;
%     delta_des = delta_ff + delta_fb;
%     delta_des = max(min(delta_des, delta_max), -delta_max);
% 
%     %% 5‑5  执行器限幅 & 延迟
%     d_step = max(min(delta_des - queue(end), rate_max*dt), -rate_max*dt);
%     delta_cmd = queue(end) + d_step; queue = [queue(2:end) delta_cmd];
%     delta_act = queue(1); state.delta = delta_act;
% 
%     %% 5‑6  纵向速度闭环 (简单 P)
%     acc = max(min(v_set - state.vx, acc_max*dt), -acc_max*dt); state.vx = state.vx + acc;
% 
%     %% 5‑7  动态单轨更新 (Euler)
%     % 侧向力
%     alpha_f = atan2(state.vy + Lf*state.r, state.vx) - delta_act;
%     alpha_r = atan2(state.vy - Lr*state.r, state.vx);
%     Fyf = -Cf*alpha_f; Fyr = -Cr*alpha_r;
% 
%     % 更新方程
%     state.vy = state.vy + (Fyf + Fyr)/m * dt - state.r * state.vx * dt;
%     state.r  = state.r  + (Lf*Fyf - Lr*Fyr)/Iz * dt;
%     state.psi = wrapToPi(state.psi + state.r * dt);
%     state.x   = state.x + (state.vx*cos(state.psi) - state.vy*sin(state.psi)) * dt;
%     state.y   = state.y + (state.vx*sin(state.psi) + state.vy*cos(state.psi)) * dt;
% 
%     %% 5‑8  日志
%     log.x(k)=state.x; log.y(k)=state.y; log.ey(k)=ey; log.psi_err(k)=psi_err; log.delta(k)=delta_act;
% end
% 
% %% 6) 性能指标 --------------------------------------------------
% rmse = sqrt(mean(log.ey.^2)); max_e = max(abs(log.ey)); ss = mean(abs(log.ey(end- round(2/dt):end)));
% int_e = sum(abs(log.ey))*dt;
% 
% fprintf('\n[动态 LQR 评估] RMSE=%.3f m | Max=%.3f m | SS=%.3f m | Int=%.3f m·s\n',rmse,max_e,ss,int_e);
% 
% %% 7) 可视化 -----------------------------------------------------
% figure; plot(ref.x,ref.y,'r--',log.x,log.y,'b'); axis equal; grid on; legend('参考','实际'); title('动态单轨 LQR 跟踪');
% figure; subplot(2,1,1); plot((0:N-1)*dt,log.ey); ylabel('e_y (m)'); title('横向误差'); grid on;
% subplot(2,1,2); plot((0:N-1)*dt,rad2deg(log.psi_err)); ylabel('ψ_err (°)'); xlabel('t (s)'); grid on; title('航向误差');
% 

















%% 误差大，跟踪效果差
% clc; clear; close all;
% 
% %% =============================================================
% %  工程级路径跟踪控制（第 II 版）
% %  ──  曲率前馈 (增益γ_ff)  +  Stanley PI 反馈  +  单轨动力学  +  执行器约束
% %  新增：
% %   1) γ_ff > 1  抵消低速欠转现象
% %   2) Stanley 横向误差 + 航向误差 PI（积分抗稳态漂移）
% %   3) 积分抗饱和（简单背算）
% %   4) 参数分区自适应：低速/常速两套增益
% % =============================================================
% 
% %% 0) 时域/车辆参数 --------------------------------------------------
% T    = 60;         dt = 0.02;   N = round(T/dt);
% L    = 0.3;        % 轴距
% v_des= 0.7;        % 巡航速度
% acc_max = 1.0;     % 纵向加速度上限
% 
% steer_max       = deg2rad(30);   % 转角幅值
% steer_rate_max  = deg2rad(180);  % 转角速率上限
% exec_delay      = 0.1;           % 执行器延迟 FIFO
% 
% %% 控制增益 ------------------------------------------------------------
% gamma_ff = 1.15;         % 曲率前馈增益 (>1 提高转向)
% % 速度分区:  v<=0.4  使用高增益, v>0.4 使用正常增益
% params_low  = struct('k_e',8.0, 'k_psi',2.0, 'k_i',0.4);
% params_norm = struct('k_e',4.0, 'k_psi',1.2, 'k_i',0.15);
% 
% %% 噪声 ---------------------------------------------------------------
% pos_noise_std = 0.003;
% head_noise_std = deg2rad(0.1);
% 
% %% 1) 参考轨迹  (圆 r=5 m) ----------------------------------------------
% r = 5;
% t      = linspace(0,2*pi,N);
% ref.x  = r*cos(t);
% ref.y  = r*sin(t);
% ref.psi= unwrap(atan2(diff([ref.y ref.y(1)]), diff([ref.x ref.x(1)])));
% ref.kappa = ones(1,N)/r;   % 圆曲率常量
% 
% %% 2) 初始状态 & 延迟队列 ---------------------------------------------
% state = struct('x',r,'y',0,'theta',pi/2,'v',0,'delta',0);
% delta_fifo = zeros(1,round(exec_delay/dt)+1);
% int_e = 0;   % 积分项初始化
% 
% %% 3) 日志预分配 --------------------------------------------------------
% log = struct('x',zeros(1,N),'y',zeros(1,N),'e_ct',zeros(1,N), ...
%              'psi_err',zeros(1,N),'delta_cmd',zeros(1,N),'delta_act',zeros(1,N));
% 
% %% 4) 主循环 -----------------------------------------------------------
% for k = 1:N
%     %% 4‑1  传感器噪声
%     xm = state.x + pos_noise_std*randn;
%     ym = state.y + pos_noise_std*randn;
%     psim = state.theta + head_noise_std*randn;
% 
%     %% 4‑2  最近轨迹点
%     [~, idx] = min(hypot(ref.x - xm, ref.y - ym));
%     e_ct  =  sin(ref.psi(idx))*(xm-ref.x(idx)) - cos(ref.psi(idx))*(ym-ref.y(idx));
%     psi_err = wrapToPi(psim - ref.psi(idx));
% 
%     %% 4‑3  区分速度段选择增益
%     gains = params_low;
%     if state.v > 0.4
%         gains = params_norm;
%     end
% 
%     %% 4‑4  积分项更新 (带漂移抑制, 积分限幅)
%     int_e = int_e + e_ct*dt;
%     int_e = max(min(int_e, 0.3), -0.3);
% 
%     %% 4‑5  控制律
%     delta_ff = atan(gamma_ff * L * ref.kappa(idx));
%     delta_fb = atan(gains.k_e * e_ct / (state.v + 0.05)) + gains.k_psi*psi_err + gains.k_i*int_e;
%     delta_des = delta_ff + delta_fb;
%     delta_des = max(min(delta_des, steer_max), -steer_max);
% 
%     %% 4‑6  执行器受限 & 延迟
%     d_rate = max(min(delta_des - delta_fifo(end), steer_rate_max*dt), -steer_rate_max*dt);
%     delta_cmd = delta_fifo(end) + d_rate;
%     delta_fifo = [delta_fifo(2:end) delta_cmd];
%     delta_act  = delta_fifo(1);
% 
%     %% 4‑7  纵向速度一阶趋近
%     acc = max(min(v_des - state.v, acc_max*dt), -acc_max*dt);
%     state.v = state.v + acc;
% 
%     %% 4‑8  车辆模型
%     state.x = state.x + state.v*cos(state.theta)*dt;
%     state.y = state.y + state.v*sin(state.theta)*dt;
%     state.theta = wrapToPi(state.theta + state.v/L*tan(delta_act)*dt);
% 
%     %% 4‑9  日志
%     log.x(k)=state.x; log.y(k)=state.y; log.e_ct(k)=e_ct;
%     log.psi_err(k)=psi_err; log.delta_cmd(k)=delta_des; log.delta_act(k)=delta_act;
% end
% 
% %% 5) 评估与可视化 -----------------------------------------------------
% E_rmse = sqrt(mean(log.e_ct.^2)); E_max=max(abs(log.e_ct)); E_int=sum(abs(log.e_ct))*dt;
% E_ss = mean(abs(log.e_ct(end-round(2/dt):end)));
% 
% fprintf('\n[性能评估 ‑ γFF + Stanley PI]\n');
% fprintf('RMSE  : %.4f m\n最大误差: %.4f m\n稳态误差: %.4f m\n误差积分: %.4f m·s\n',E_rmse,E_max,E_ss,E_int);
% 
% figure('Name','路径跟踪'); plot(ref.x,ref.y,'r--',log.x,log.y,'b'); axis equal; grid on;
% legend('参考','实际'); xlabel('X(m)'); ylabel('Y(m)'); title('前馈+Stanley‑PI + 执行器限制');
% 
% figure('Name','误差与控制');
% subplot(3,1,1);plot((0:N-1)*dt,log.e_ct);ylabel('e_{ct}(m)');title('横向误差');grid on;
% subplot(3,1,2);plot((0:N-1)*dt,rad2deg(log.delta_cmd), (0:N-1)*dt,rad2deg(log.delta_act));ylabel('δ(°)');legend('cmd','act');grid on;title('前轮转角');
% subplot(3,1,3);plot((0:N-1)*dt,rad2deg(log.psi_err));ylabel('ψ_err (°)');xlabel('t(s)');grid on;title('航向误差');
% 
















%% 误差小，但未动态演示
% clc; clear; close all;
% 
% %% ======================  参数与环境  ======================
% T        = 60;        % 仿真总时长  (s)
% dt       = 0.02;      % 时间步长    (s)
% N        = round(T/dt);
% 
% v_const  = 0.6;       % 线速度常数  (m/s)
% Ld_base  = 0.8;       % 基础前瞻距离 (m)，会随速度动态调整
% 
% %% ======================  圆形参考轨迹  =====================
% r = 5;                                    % 圆半径 5 m
% phi_ref = linspace(0, 2*pi, N);           % 轨迹角度参数
% x_ref   = r * cos(phi_ref);
% y_ref   = r * sin(phi_ref);
% 
% %% ======================  机器人初始位姿  ===================
% % 起点在 (5, 0)，朝向 +y 方向（与圆轨迹切线一致）
% x =  r;       
% y =  0;
% theta =  pi/2;
% 
% %% ======================  预分配日志向量  ==================
% [x_log, y_log, theta_log] = deal(zeros(1,N));
% [e_ct_log, omega_log]     = deal(zeros(1,N));
% 
% %% ======================  主循环：Pure Pursuit  ============
% for k = 1:N
%     % ---------- 1) 选取最近轨迹点 ----------
%     dists = hypot(x_ref - x, y_ref - y);
%     [~, idx_min] = min(dists);
% 
%     % ---------- 2) 动态前瞻距离 Ld ----------
%     Ld = max(Ld_base, v_const * 1.2);          % 依据速度增大前瞻
%     idx_goal = idx_min;
%     path_len = 0;
%     while path_len < Ld && idx_goal < N
%         % 累加弦长直到超过前瞻距离
%         idx_goal = idx_goal + 1;
%         dx_p = x_ref(idx_goal) - x_ref(idx_goal - 1);
%         dy_p = y_ref(idx_goal) - y_ref(idx_goal - 1);
%         path_len = path_len + hypot(dx_p, dy_p);
%     end
%     goal_x = x_ref(idx_goal);
%     goal_y = y_ref(idx_goal);
% 
%     % ---------- 3) 计算横向误差（Stanley 跨轨） ----------
%     % 从车辆到最近点的矢量 (ex,ey)
%     ex = x_ref(idx_min) - x;
%     ey = y_ref(idx_min) - y;
%     % 圆轨迹切线方向角 (角度沿圆方向前进)
%     path_theta = atan2(-x_ref(idx_min), y_ref(idx_min));   % 切线方向
%     % 横向误差符号由叉积确定
%     e_ct =  sign( sin(path_theta)*(ex) - cos(path_theta)*(ey) ) * min(dists);  % 横向偏差
% 
%     % ---------- 4) Pure‑Pursuit 求曲率 ----------
%     alpha = wrapToPi( atan2(goal_y - y, goal_x - x) - theta );
%     kappa = 2 * sin(alpha) / Ld;                         % 纯追踪曲率
%     omega = v_const * kappa;                             % 转换为角速度
%     % 角速度限幅，确保动力学可行
%     omega_max = 3.0;  omega = max(min(omega, omega_max), -omega_max);
% 
%     % ---------- 5) 状态更新 ----------
%     x     = x + v_const * cos(theta) * dt;
%     y     = y + v_const * sin(theta) * dt;
%     theta = wrapToPi(theta + omega * dt);
% 
%     % ---------- 6) 日志 ----------
%     x_log(k)       = x;
%     y_log(k)       = y;
%     theta_log(k)   = theta;
%     e_ct_log(k)    = e_ct;
%     omega_log(k)   = omega;
% end
% 
% %% ======================  可视化  ===========================
% figure('Name','路径跟踪结果');
% plot(x_ref, y_ref, 'r--', 'LineWidth',1.3); hold on;
% plot(x_log, y_log, 'b',  'LineWidth',1.8);
% legend('参考轨迹','实际轨迹','Location','SouthEast');
% axis equal; grid on;
% xlabel('X (m)'); ylabel('Y (m)'); title('Pure‑Pursuit 路径跟踪');
% 
% figure('Name','误差与控制输出');
% subplot(3,1,1); plot((0:N-1)*dt, e_ct_log, 'LineWidth',1.2); ylabel('横向误差 e_{ct} (m)'); grid on; title('横向误差');
% subplot(3,1,2); plot((0:N-1)*dt, omega_log, 'LineWidth',1.2); ylabel('\omega (rad/s)'); grid on; title('角速度指令');
% subplot(3,1,3); plot((0:N-1)*dt, rad2deg(theta_log), 'LineWidth',1.2); ylabel('\theta (°)'); xlabel('时间 (s)'); grid on; title('航向角');
% 
% %% ======================  性能评估  =========================
% tracking_error_sum = sum(abs(e_ct_log)) * dt;
% max_ct             = max(abs(e_ct_log));
% steady_state_ct    = mean(abs(e_ct_log(end- round(2/dt) : end)));
% rmse_ct            = sqrt(mean(e_ct_log.^2));
% 
% fprintf('\n[性能评估] (Pure‑Pursuit)\n');
% fprintf('误差积分  : %.4f m·s\n', tracking_error_sum);
% fprintf('最大横向误差: %.4f m\n', max_ct);
% fprintf('稳态误差    : %.4f m\n', steady_state_ct);
% fprintf('RMSE        : %.4f m\n', rmse_ct);
% 
% 
% 
% 
% 
% 












%% 误差极大极大
% clc; clear; close all;
% 
% %% 模拟参数初始化
% dt = 0.1;               % 时间步长
% T = 60;                 % 模拟总时间
% N = round(T / dt);      % 仿真步数
% v = 0.5;                % 线速度（单位：m/s）
% 
% % 初始位姿（X，Y，朝向）
% x = 0; y = 0; theta = 0;
% 
% %% 生成参考路径（圆形）
% t_ref = 0:dt:T;
% r = 5;
% x_ref = r * cos(t_ref);
% y_ref = r * sin(t_ref);
% 
% %% 加载模糊控制器
% fis = readfis('fuzzy_controller.fis');
% 
% %% 日志变量预分配
% x_log = zeros(1, N);
% y_log = zeros(1, N);
% theta_log = zeros(1, N);
% e_log = zeros(1, N);
% e_theta_log = zeros(1, N);
% 
% %% 跟踪控制主循环
% for k = 1:N
%     % 当前目标点
%     xt = x_ref(k);
%     yt = y_ref(k);
% 
%     % 误差计算
%     dx = xt - x;
%     dy = yt - y;
%     e = sqrt(dx^2 + dy^2);  % 欧氏距离误差
%     desired_theta = atan2(dy, dx);
%     e_theta = wrapToPi(desired_theta - theta);  % 角度误差限制在 [-pi, pi]
% 
%     % 归一化输入（防止超出模糊控制器定义域）
%     e_norm = max(min(e, 1), -1);
%     e_theta_norm = max(min(e_theta / pi, 1), -1);
% 
%     % 模糊控制器输出角速度
%     omega = evalfis([e_norm, e_theta_norm], fis);
% 
%     % 移动机器人运动学更新
%     x = x + v * cos(theta) * dt;
%     y = y + v * sin(theta) * dt;
%     theta = theta + omega * dt;
% 
%     % 存储日志
%     x_log(k) = x;
%     y_log(k) = y;
%     theta_log(k) = theta;
%     e_log(k) = e;
%     e_theta_log(k) = e_theta;
% end
% 
% %% 路径跟踪可视化
% figure;
% plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5); hold on;
% plot(x_log, y_log, 'b', 'LineWidth', 2);
% legend('目标路径', '实际路径');
% xlabel('X 坐标 (m)'); ylabel('Y 坐标 (m)');
% title('基于模糊控制的移动机器人路径跟踪');
% grid on; axis equal;
% 
% %% 误差分析图像
% figure;
% subplot(2,1,1);
% plot(t_ref(1:N), e_log, 'LineWidth', 1.5);
% ylabel('距离误差 e (m)');
% title('距离误差随时间变化');
% grid on;
% 
% subplot(2,1,2);
% plot(t_ref(1:N), e_theta_log, 'LineWidth', 1.5);
% xlabel('时间 (s)'); ylabel('角度误差 e_{\theta} (rad)');
% title('角度误差随时间变化');
% grid on;
