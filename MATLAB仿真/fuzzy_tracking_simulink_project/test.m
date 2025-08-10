% ===== Fuzzy‑PP Complex Demo 评估 =====
% RMSE   = 0.0210 m (目标 ≤ 0.05 m)
% Max E  = 0.0676 m
% SS Err = 0.0041 m
% Int E  = 0.7923 m·s

% clc; clear; close all;
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
% slow_factor = 0.5;        % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N');
%     fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N');
%     fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small');
%     fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% kappa_history = zeros(1, 5); % 记录最近5个曲率值
% e_lat_history = zeros(1, 3); % 记录最近3个侧向误差值
% e_lat_integral = 0; % 侧向误差积分
% e_lat_rate = 0; % 侧向误差变化率
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 计算误差变化率
%     e_lat_rate = (e_lat - prev_e) / dt;
%     prev_e = e_lat;
% 
%     % 更新侧向误差历史
%     e_lat_history(1:end-1) = e_lat_history(2:end);
%     e_lat_history(end) = e_lat;
% 
%     % 累积误差积分
%     e_lat_integral = e_lat_integral + e_lat * dt;
% 
%     % 更精准的曲率计算（使用三点法）
%     if idx > 1 && idx < N - 1
%         x1 = x_ref(idx - 1);
%         y1 = y_ref(idx - 1);
%         x2 = x_ref(idx);
%         y2 = y_ref(idx);
%         x3 = x_ref(idx + 1);
%         y3 = y_ref(idx + 1);
%         A = hypot(x2 - x1, y2 - y1);
%         B = hypot(x3 - x2, y3 - y2);
%         C = hypot(x3 - x1, y3 - y1);
%         s = (A + B + C) / 2;
%         area = sqrt(s * (s - A) * (s - B) * (s - C));
%         if area > 1e-6
%             kappa = 4 * area / (A * B * C);
%         else
%             kappa = 0;
%         end
%     else
%         kappa = 0;
%     end
% 
%     % 更新曲率历史
%     kappa_history(1:end-1) = kappa_history(2:end);
%     kappa_history(end) = kappa;
% 
%     % 动态调整前瞻距离
%     curvature_factor = 1 - 0.6 * min(abs(kappa) / 10, 1); % 曲率越大，前瞻距离越小
%     error_factor = 1 - 0.2 * min(abs(e_lat) / SPEED_CUT, 1); % 误差越大，前瞻距离越小
%     integral_factor = min(abs(e_lat_integral) / 1, 1); % 积分越大，前瞻距离越小
%     rate_factor = 1 - 0.1 * min(abs(e_lat_rate) / 1, 1); % 误差变化率越大，前瞻距离越小
%     Ld = LOOK_BASE * curvature_factor * error_factor * (1 - 0.2 * integral_factor) * rate_factor;
%     Ld = max(0.2, Ld); % 确保前瞻距离不小于0.2m
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa_pp = 2*sin(alpha) / Ld;
%     else
%         kappa_pp = 0;
%     end
%     omega_pp = V_MAX * kappa_pp;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     % 在弯道处根据曲率动态调整增益
%     if abs(kappa) > 0.2
%         gamma = gamma * (1 + 0.6 * min(abs(kappa) / 10, 1));
%     end
%     % 当误差较大或误差积分较大或误差变化率较大时，进一步增大增益
%     if abs(e_lat) > 0.5 || abs(e_lat_integral) > 1 || abs(e_lat_rate) > 0.5
%         gamma = gamma * 2.5;
%     end
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差、误差积分、误差变化率调整速度
%     curvature_speed_factor = 1 - 0.9 * min(abs(kappa) / 10, 1); % 曲率越大，速度越小
%     error_speed_factor = 1 - 0.6 * min(abs(e_lat)/SPEED_CUT,1);
%     integral_speed_factor = 1 - 0.3 * min(abs(e_lat_integral) / 2, 1); % 误差积分对速度的影响
%     rate_speed_factor = 1 - 0.2 * min(abs(e_lat_rate) / 1, 1); % 误差变化率对速度的影响
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * curvature_speed_factor * error_speed_factor * integral_speed_factor * rate_speed_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');



























 
%% ===== Fuzzy‑PP Complex Demo 评估 =====
% RMSE   = 0.0216 m (目标 ≤ 0.05 m)
% Max E  = 0.0677 m
% SS Err = 0.0043 m
% Int E  = 0.8240 m·s

% clc; clear; close all;
% % 
% % %% === 用户参数 ===
% T           = 60;        % 仿真时长 (s)
% dt          = 0.02;      % 时间步长 (s)
% N           = round(T/dt);
% V_MAX       = 0.6;       % 最高线速度 (m/s)
% OMEGA_LIM   = 2.5;       % 最大角速度 (rad/s)
% 
% LOOK_BASE   = 0.6;       % 基础前瞻距离 (m)
% SPEED_CUT   = 0.3;       % 误差减速阈值 (m)
% slow_factor = 0.5;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N');
%     fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N');
%     fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small');
%     fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% kappa_history = zeros(1, 5); % 记录最近5个曲率值
% e_lat_history = zeros(1, 3); % 记录最近3个侧向误差值
% e_lat_integral = 0; % 侧向误差积分
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 更新侧向误差历史
%     e_lat_history(1:end-1) = e_lat_history(2:end);
%     e_lat_history(end) = e_lat;
% 
%     % 累积误差积分
%     e_lat_integral = e_lat_integral + e_lat * dt;
% 
%     % 计算路径曲率
%     if idx > 1 && idx < N
%         dx1 = x_ref(idx) - x_ref(idx - 1);
%         dy1 = y_ref(idx) - y_ref(idx - 1);
%         dx2 = x_ref(idx + 1) - x_ref(idx);
%         dy2 = y_ref(idx + 1) - y_ref(idx);
%         theta1 = atan2(dy1, dx1);
%         theta2 = atan2(dy2, dx2);
%         dtheta = wrapToPi(theta2 - theta1);
%         ds = hypot(dx2, dy2);
%         kappa = dtheta / ds;
%     else
%         kappa = 0;
%     end
% 
%     % 更新曲率历史
%     kappa_history(1:end-1) = kappa_history(2:end);
%     kappa_history(end) = kappa;
% 
%     % 动态调整前瞻距离
%     curvature_factor = 1 - 0.5 * min(abs(kappa) / 10, 1); % 曲率越大，前瞻距离越小
%     error_factor = 1 - 0.2 * min(abs(e_lat) / SPEED_CUT, 1); % 误差越大，前瞻距离越小
%     integral_factor = min(abs(e_lat_integral) / 1, 1); % 积分越大，前瞻距离越小
%     Ld = LOOK_BASE * curvature_factor * error_factor * (1 - 0.2 * integral_factor);
%     Ld = max(0.2, Ld); % 确保前瞻距离不小于0.2m
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa_pp = 2*sin(alpha) / Ld;
%     else
%         kappa_pp = 0;
%     end
%     omega_pp = V_MAX * kappa_pp;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     % 在弯道处根据曲率动态调整增益
%     if abs(kappa) > 0.2
%         gamma = gamma * (1 + 0.5 * min(abs(kappa) / 10, 1));
%     end
%     % 当误差较大或误差积分较大时，进一步增大增益
%     if abs(e_lat) > 0.5 || abs(e_lat_integral) > 1
%         gamma = gamma * 2;
%     end
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差、误差积分调整速度
%     curvature_speed_factor = 1 - 0.8 * min(abs(kappa) / 10, 1); % 曲率越大，速度越小
%     error_speed_factor = 1 - 0.6 * min(abs(e_lat)/SPEED_CUT,1);
%     integral_speed_factor = 1 - 0.3 * min(abs(e_lat_integral) / 2, 1); % 误差积分对速度的影响
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * curvature_speed_factor * error_speed_factor * integral_speed_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');
% 
% 
% 
% 






















% ===== Fuzzy‑PP Complex Demo 评估 =====
% RMSE   = 0.0342 m (目标 ≤ 0.05 m)
% Max E  = 0.0945 m
% SS Err = 0.0250 m
% Int E  = 1.3424 m·s

% 
% 
% clc; clear; close all;
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
% slow_factor = 0.5;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N'); 
%     fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N'); 
%     fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small'); 
%     fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% kappa_history = zeros(1, 5); % 记录最近5个曲率值
% e_lat_history = zeros(1, 3); % 记录最近3个侧向误差值
% e_lat_integral = 0; % 侧向误差积分
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 更新侧向误差历史
%     e_lat_history(1:end-1) = e_lat_history(2:end);
%     e_lat_history(end) = e_lat;
% 
%     % 累积误差积分
%     e_lat_integral = e_lat_integral + e_lat * dt;
% 
%     % 动态调整前瞻距离，考虑曲率变化率、误差变化率和误差积分
%     curvature_rate = (kappa_history(end) - kappa_history(1)) / length(kappa_history);
%     error_rate = (e_lat_history(end) - e_lat_history(1)) / length(e_lat_history);
%     integral_factor = min(abs(e_lat_integral) / 1, 1); % 积分因子，最大为1
%     Ld = LOOK_BASE + 0.8 * abs(e_lat) + 0.3 * mean(v_history) - 0.3 * max(kappa_history) - 0.1 * abs(curvature_rate) - 0.1 * abs(error_rate) - 0.2 * integral_factor;
%     Ld = max(LOOK_BASE, Ld);
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa    = 2*sin(alpha) / Ld;
%     else
%         kappa = 0;
%     end
%     omega_pp = V_MAX * kappa;
% 
%     % 更新曲率历史
%     kappa_history(1:end-1) = kappa_history(2:end);
%     kappa_history(end) = kappa;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     % 在转弯时根据曲率变化率动态调整增益
%     if abs(kappa) > 0.2
%         if abs(curvature_rate) > 0.1
%             gamma = gamma * 1.5;
%         else
%             gamma = gamma * 1.2;
%         end
%     end
%     % 当误差较大或误差积分较大时，进一步增大增益
%     if abs(e_lat) > 0.5 || abs(e_lat_integral) > 1
%         gamma = gamma * 2;
%     end
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差、曲率变化率、误差变化率和误差积分调整速度
%     curvature_factor = 1 - 0.9 * min(abs(kappa) / 10, 1); 
%     error_factor = 1 - 0.6 * min(abs(e_lat)/SPEED_CUT,1);
%     curvature_rate_factor = 1 - 0.2 * min(abs(curvature_rate) / 1, 1); 
%     error_rate_factor = 1 - 0.2 * min(abs(error_rate) / 0.5, 1); 
%     integral_speed_factor = 1 - 0.3 * min(abs(e_lat_integral) / 2, 1); % 误差积分对速度的影响
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * error_factor * curvature_factor * curvature_rate_factor * error_rate_factor * integral_speed_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');































% ===== Fuzzy‑PP Complex Demo 评估 =====
% RMSE   = 0.0390 m (目标 ≤ 0.05 m)
% Max E  = 0.1068 m
% SS Err = 0.0086 m
% Int E  = 1.5046 m·s
% 
% 
% 
% clc; clear; close all;
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
% slow_factor = 0.5;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.9 -0.1],'Name','N'); % 更窄的负隶属度函数
%     fis_g = addMF(fis_g,'e','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.1 0.9 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.9 -0.1],'Name','N'); % 更窄的负隶属度函数
%     fis_g = addMF(fis_g,'psi','trimf',[-0.7 0 0.7],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.1 0.9 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.2 0.5],'Name','Small'); % 更窄的小隶属度函数
%     fis_g = addMF(fis_g,'g','trimf',[0.4 1.0 1.6],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% kappa_history = zeros(1, 5); % 记录最近5个曲率值
% e_lat_history = zeros(1, 3); % 记录最近3个侧向误差值
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 更新侧向误差历史
%     e_lat_history(1:end-1) = e_lat_history(2:end);
%     e_lat_history(end) = e_lat;
% 
%     % 动态调整前瞻距离，考虑曲率变化率和误差变化率
%     curvature_rate = (kappa_history(end) - kappa_history(1)) / length(kappa_history);
%     error_rate = (e_lat_history(end) - e_lat_history(1)) / length(e_lat_history);
%     Ld = LOOK_BASE + 0.8 * abs(e_lat) + 0.3 * mean(v_history) - 0.3 * max(kappa_history) - 0.1 * abs(curvature_rate) - 0.1 * abs(error_rate);
%     Ld = max(LOOK_BASE, Ld);
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa    = 2*sin(alpha) / Ld;
%     else
%         kappa = 0;
%     end
%     omega_pp = V_MAX * kappa;
% 
%     % 更新曲率历史
%     kappa_history(1:end-1) = kappa_history(2:end);
%     kappa_history(end) = kappa;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     % 在转弯时根据曲率变化率动态调整增益
%     if abs(kappa) > 0.2
%         if abs(curvature_rate) > 0.1
%             gamma = gamma * 1.5;
%         else
%             gamma = gamma * 1.2;
%         end
%     end
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差、曲率变化率和误差变化率调整速度
%     curvature_factor = 1 - 0.9 * min(abs(kappa) / 10, 1); % 更大的曲率影响
%     error_factor = 1 - 0.6 * min(abs(e_lat)/SPEED_CUT,1);
%     curvature_rate_factor = 1 - 0.2 * min(abs(curvature_rate) / 1, 1); % 考虑曲率变化率
%     error_rate_factor = 1 - 0.2 * min(abs(error_rate) / 0.5, 1); % 考虑误差变化率
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * error_factor * curvature_factor * curvature_rate_factor * error_rate_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');
% 
























% ===== Fuzzy‑PP Complex Demo 评估 =====
% RMSE   = 0.0407 m (目标 ≤ 0.05 m)
% Max E  = 0.1129 m
% SS Err = 0.0091 m
% Int E  = 1.5576 m·s
% 
% 
% clc; clear; close all;
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
% slow_factor = 0.5;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.8 -0.2],'Name','N'); 
%     fis_g = addMF(fis_g,'e','trimf',[-0.6 0 0.6],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.2 0.8 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.8 -0.2],'Name','N'); 
%     fis_g = addMF(fis_g,'psi','trimf',[-0.6 0 0.6],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.2 0.8 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.3 0.6],'Name','Small'); 
%     fis_g = addMF(fis_g,'g','trimf',[0.5 1.0 1.5],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.2 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% kappa_history = zeros(1, 3); % 记录最近3个曲率值
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 动态调整前瞻距离，增加曲率影响
%     Ld = LOOK_BASE + 0.8 * abs(e_lat) + 0.3 * mean(v_history) - 0.2 * max(kappa_history); 
%     Ld = max(LOOK_BASE, Ld);
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa    = 2*sin(alpha) / Ld;
%     else
%         kappa = 0;
%     end
%     omega_pp = V_MAX * kappa;
% 
%     % 更新曲率历史
%     kappa_history(1:end-1) = kappa_history(2:end);
%     kappa_history(end) = kappa;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     % 在转弯时增加增益调整
%     if abs(kappa) > 0.2
%         gamma = gamma * 1.2; 
%     end
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差和速度历史调整速度
%     curvature_factor = 1 - 0.8 * min(abs(kappa) / 10, 1); % 增大曲率影响
%     error_factor = 1 - 0.5 * min(abs(e_lat)/SPEED_CUT,1);
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * error_factor * curvature_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.4f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');




















%% 
% clc; clear; close all;
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
% slow_factor = 0.5;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.8 -0.2],'Name','N'); % 调整隶属度函数
%     fis_g = addMF(fis_g,'e','trimf',[-0.6 0 0.6],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.2 0.8 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.8 -0.2],'Name','N'); % 调整隶属度函数
%     fis_g = addMF(fis_g,'psi','trimf',[-0.6 0 0.6],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.2 0.8 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.3 0.6],'Name','Small'); % 调整隶属度函数
%     fis_g = addMF(fis_g,'g','trimf',[0.5 1.0 1.5],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.2 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 动态调整前瞻距离
%     Ld = LOOK_BASE + 0.8 * abs(e_lat) + 0.3 * mean(v_history); % 根据误差和平均速度调整
%     Ld = max(LOOK_BASE, Ld);
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa    = 2*sin(alpha) / Ld;
%     else
%         kappa = 0;
%     end
%     omega_pp = V_MAX * kappa;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差和速度历史调整速度
%     curvature_factor = 1 - 0.6 * min(abs(kappa) / 10, 1); % 假设最大曲率为10
%     error_factor = 1 - 0.5 * min(abs(e_lat)/SPEED_CUT,1);
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * error_factor * curvature_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');clc; clear; close all;
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
% %% === 用户参数 ===
% T           = 60;        % 仿真时长 (s)
% dt          = 0.02;      % 时间步长 (s)
% N           = round(T/dt);
% V_MAX       = 0.6;       % 最高线速度 (m/s)
% OMEGA_LIM   = 2.5;       % 最大角速度 (rad/s)
% 
% LOOK_BASE   = 0.6;       % 基础前瞻距离 (m)
% SPEED_CUT   = 0.3;       % 误差减速阈值 (m)
% slow_factor = 1;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -0.8 -0.2],'Name','N'); % 调整隶属度函数
%     fis_g = addMF(fis_g,'e','trimf',[-0.6 0 0.6],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.2 0.8 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -0.8 -0.2],'Name','N'); % 调整隶属度函数
%     fis_g = addMF(fis_g,'psi','trimf',[-0.6 0 0.6],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.2 0.8 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.3 0.6],'Name','Small'); % 调整隶属度函数
%     fis_g = addMF(fis_g,'g','trimf',[0.5 1.0 1.5],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.2 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% v_history = zeros(1, 5); % 记录最近5个速度值
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
% 
%     % 2) 误差计算
%     dx     = x_ref(idx) - state.x;
%     dy     = y_ref(idx) - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
% 
%     % 动态调整前瞻距离
%     Ld = LOOK_BASE + 0.8 * abs(e_lat) + 0.3 * mean(v_history); % 根据误差和平均速度调整
%     Ld = max(LOOK_BASE, Ld);
% 
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
% 
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(yt - state.y, xt - state.x) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa    = 2*sin(alpha) / Ld;
%     else
%         kappa = 0;
%     end
%     omega_pp = V_MAX * kappa;
% 
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
% 
%     % 5) 自适应线速度
%     % 考虑路径曲率、误差和速度历史调整速度
%     curvature_factor = 1 - 0.6 * min(abs(kappa) / 10, 1); % 假设最大曲率为10
%     error_factor = 1 - 0.5 * min(abs(e_lat)/SPEED_CUT,1);
%     speed_history_factor = 1 - 0.2 * (mean(v_history) / V_MAX);
%     v = V_MAX * error_factor * curvature_factor * speed_history_factor;
% 
%     % 更新速度历史
%     v_history(1:end-1) = v_history(2:end);
%     v_history(end) = v;
% 
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
% 
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
% 
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');

























%% 还可
% clc; clear; close all;
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
% slow_factor = 0.5;         % 动画放慢倍数 (>=1)
% refresh_step= 1;         % 动画刷新帧步
% 
% %% === 复杂路径生成 ===
% % 使用叠加谐波轨迹：圆 + 三次谐波 + 二次谐波
% phi    = linspace(0, 2*pi, N);
% x_ref  = 5*cos(phi) + 1.0*cos(3*phi);
% y_ref  = 5*sin(phi) + 0.5*sin(2*phi);
% 
% %% === 模糊增益 FIS 自动生成/加载 ===
% fisFile = 'fuzzy_gain.fis';
% if ~isfile(fisFile)
%     fis_g = mamfis('Name','γ_gain');
%     % 输入：横向误差 e (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','e');
%     fis_g = addMF(fis_g,'e','trimf',[-1 -1 -0.2],'Name','N');
%     fis_g = addMF(fis_g,'e','trimf',[-0.4 0 0.4],'Name','Z');
%     fis_g = addMF(fis_g,'e','trimf',[0.2 1 1],'Name','P');
%     % 输入：航向误差 psi (归一化到 ±1)
%     fis_g = addInput(fis_g,[-1 1],'Name','psi');
%     fis_g = addMF(fis_g,'psi','trimf',[-1 -1 -0.2],'Name','N');
%     fis_g = addMF(fis_g,'psi','trimf',[-0.4 0 0.4],'Name','Z');
%     fis_g = addMF(fis_g,'psi','trimf',[0.2 1 1],'Name','P');
%     % 输出：增益 γ
%     fis_g = addOutput(fis_g,[0.1 2.0],'Name','g');
%     fis_g = addMF(fis_g,'g','trimf',[0.1 0.5 1.0],'Name','Small');
%     fis_g = addMF(fis_g,'g','trimf',[0.8 1.2 1.6],'Name','Medium');
%     fis_g = addMF(fis_g,'g','trimf',[1.4 1.8 2.0],'Name','Large');
%     % 规则表（9条）
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
%     fis_g = addRule(fis_g,rule);
%     writefis(fis_g,fisFile);
% end
% gain_fis = readfis(fisFile);
% 
% %% === 初始化状态 & 日志 ===
% state.x  = x_ref(1);
% state.y  = y_ref(1);
% state.th = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
% 
% xL = zeros(1,N);
% yL = zeros(1,N);
% eL = zeros(1,N);
% gL = zeros(1,N);
% 
% %% === 动画准备 ===
% win    = 1.5;  % 视窗半宽
% figure('Name','Fuzzy PP Complex Demo'); hold on;
% plot(x_ref, y_ref, 'k--','LineWidth',1.1);
% anim   = animatedline('Color','b','LineWidth',1.6);
% errTxt = text(state.x, state.y+0.3, '', 'Color','r','FontSize',11);
% grid on; axis equal;
% 
% %% === 主循环：Pure‑Pursuit + 模糊增益 ===
% prev_e = 0;
% for k = 1:N
%     % 1) 最近点 + 动态前瞻
%     dists = hypot(x_ref - state.x, y_ref - state.y);
%     [~, idx] = min(dists);
%     Ld = max(LOOK_BASE, 1.0 * dists(idx));
%     dist_acc = 0; idx_l = idx;
%     while dist_acc < Ld && idx_l < N
%         seg = hypot(x_ref(idx_l+1)-x_ref(idx_l), ...
%                     y_ref(idx_l+1)-y_ref(idx_l));
%         dist_acc = dist_acc + seg; idx_l = idx_l + 1;
%     end
%     xt = x_ref(idx_l); yt = y_ref(idx_l);
%     
%     % 2) 误差计算
%     dx     = xt - state.x;
%     dy     = yt - state.y;
%     e_lat  = sin(state.th)*dx - cos(state.th)*dy;    % 侧向误差
%     psi_err= wrapToPi(atan2(dy,dx) - state.th);       % 航向误差
%     
%     % 3) Pure‑Pursuit 曲率
%     alpha    = wrapToPi(atan2(dy,dx) - state.th);
%     % 避免除零错误
%     if Ld > 1e-6
%         kappa    = 2*sin(alpha) / Ld;
%     else
%         kappa = 0;
%     end
%     omega_pp = V_MAX * kappa;
%     
%     % 4) 模糊增益 γ
%     % 归一化输入
%     e_n   = max(min(e_lat/1.5, 1), -1);
%     psi_n = max(min(psi_err/pi, 1), -1);
%     gamma = evalfis([e_n, psi_n], gain_fis);
%     omega = gamma * omega_pp;
%     omega = max(min(omega, OMEGA_LIM), -OMEGA_LIM);
%     
%     % 5) 自适应线速度
%     v = V_MAX * (1 - 0.5 * min(abs(e_lat)/SPEED_CUT,1));
%     
%     % 6) 状态更新
%     state.x  = state.x + v*cos(state.th)*dt;
%     state.y  = state.y + v*sin(state.th)*dt;
%     state.th = wrapToPi(state.th + omega*dt);
%     
%     % 7) 日志记录
%     xL(k) = state.x;
%     yL(k) = state.y;
%     eL(k) = abs(e_lat);
%     gL(k) = gamma;
%     
%     % 8) 动画更新
%     addpoints(anim, state.x, state.y);
%     if mod(k,refresh_step)==0
%         set(errTxt,'Position',[state.x,state.y+0.3], ...
%             'String',sprintf('e=%.3f m',eL(k)));
%         % 限制坐标轴范围，避免跳动过大
%         xlim([state.x-win, state.x+win]);
%         ylim([state.y-win, state.y+win]);
%         drawnow limitrate;
%         pause(dt*slow_factor);
%     end
% end
% 
% %% === 性能评估 ===
% RMSE = sqrt(mean(eL.^2));
% MAXE = max(eL);
% SS   = mean(eL(end-round(2/dt):end));
% INT  = sum(eL)*dt;
% fprintf('\n===== Fuzzy‑PP Complex Demo 评估 =====\n');
% fprintf('RMSE   = %.4f m (目标 ≤ 0.05 m)\n', RMSE);
% fprintf('Max E  = %.4f m\n', MAXE);
% fprintf('SS Err = %.4f m\n', SS);
% fprintf('Int E  = %.4f m·s\n', INT);
% 
% %% === 静态结果展示 ===
% figure('Name','Tracking Result');
% subplot(2,2,1);
% plot(x_ref,y_ref,'k--',xL,yL,'b','LineWidth',1.5);
% axis equal; grid on; title('轨迹对比');
% 
% subplot(2,2,2);
% plot((0:N-1)*dt,eL,'LineWidth',1.2);
% grid on; ylabel('e (m)'); title('距离误差');
% 
% subplot(2,2,3);
% plot((0:N-1)*dt,gL,'LineWidth',1.2);
% grid on; ylabel('γ'); title('模糊增益'); xlabel('t (s)');
% 
% subplot(2,2,4);
% histogram(eL,30);
% title('误差分布');