
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

















