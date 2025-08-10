function demo_stepper_fuzzy_pid
    %% 1) 系统 & 仿真参数
    G     = tf(20, [0.04 6 1]);  % 电机传递函数
    T_end = 1; dt = 0.002;
    t     = 0:dt:T_end;
    r     = ones(size(t));       % 0→1 阶跃

    %% 2) 经验 PID 参数
    Kp0 = 7;  Ki = 1.6;  Kd0 = 0.1;

    %% 3) 预计算两路响应
    [y_pid,   ~    ] = sim_pid      (G, t, r,    Kp0, Ki, Kd0);
    [y_fuz, Kp_hist] = sim_fuzzy_pid(G, t, r, dt, Kp0, Ki, Kd0);

    %% 4) 动态演示
    figure('Color','k','Position',[200 100 700 550]);

    % —— 上图：阶跃响应对比 —— 
    ax1 = subplot(2,1,1);
    hold(ax1,'on'); ax1.Color='k'; ax1.XColor='w'; ax1.YColor='w';
    xlim(ax1,[0 T_end]); ylim(ax1,[0 1.2]);
    title(ax1,'Stepper Motor Step Response','Color','w');
    xlabel(ax1,'Time (s)','Color','w'); ylabel(ax1,'Output','Color','w');
    h_ref = animatedline(ax1,'Color','y','LineWidth',2);
    h_pid = animatedline(ax1,'Color','w','LineWidth',1.5);
    h_fuz = animatedline(ax1,'Color','r','LineWidth',1.5);
    legend(ax1,{'Step','Traditional PID','Fuzzy‑PID'},'TextColor','w');

    % —— 下图：Kp 调度 —— 
    ax2 = subplot(2,1,2);
    hold(ax2,'on'); ax2.Color='k'; ax2.XColor='w'; ax2.YColor='w';
    xlim(ax2,[0 T_end]); ylim(ax2,[0 Kp0*1.5]);
    title(ax2,'Gain Scheduling: K_p','Color','w');
    xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'K_p','Color','w');
    h_Kp = animatedline(ax2,'Color','g','LineWidth',2);

    for k = 1:length(t)
        addpoints(h_ref, t(k), r(k));
        addpoints(h_pid, t(k), y_pid(k));
        addpoints(h_fuz, t(k), y_fuz(k));
        addpoints(h_Kp,  t(k), Kp_hist(k));
        drawnow;
        pause(dt);
    end
end

%% 传统 PID 离散仿真
function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
    dt   = t(2)-t(1); N = numel(t);
    sysd = ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
    y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
    eInt=0; ePrev=0;
    for k=2:N
        e    = r(k)-y(k-1);
        de   = (e-ePrev)/dt;
        eInt = eInt + e*dt;
        u(k) = Kp*e + Ki*eInt + Kd*de;
        x    = A*x + B*u(k);
        y(k) = C*x + D*u(k);
        ePrev= e;
    end
end

%% 模糊 PID 离散仿真：基于 |e| 分段调度 Kp，无超调
function [y,Kp_hist] = sim_fuzzy_pid(G,t,r,dt,Kp0,Ki,Kd0)
    N = numel(t);
    sysd = ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
    y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
    eInt=0; ePrev=0;
    Kp_hist = zeros(1,N);

    for k=2:N
        e    = r(k)-y(k-1);
        de   = (e-ePrev)/dt;
        eInt = eInt + e*dt;

        % —— “模糊”分段调度规则 —— 
        ae = abs(e);
        if     ae > 0.4
            alpha = 0.7;        % 大误差抑制比例增益
        elseif ae > 0.05
            alpha = 1.0;        % 中误差正常增益
        else
            alpha = 1.3;        % 微小误差轻微放大 Kp，加快收敛
        end
        % Kd 也可调度（此例保持常数或轻微增大阻尼）
        if de > 0
            beta = 1.5;
        else
            beta = 1.0;
        end
        Kp = Kp0 * alpha;
        Kd = Kd0 * beta;
        % ————————————————————————

        u(k)   = Kp*e + Ki*eInt + Kd*de;
        x      = A*x + B*u(k);
        y(k)   = C*x + D*u(k);
        ePrev  = e;
        Kp_hist(k) = Kp;
    end
end


















% function demo_stepper_fuzzy_pid
%     %% 1) 模型 & 仿真设置
%     G     = tf(20, [0.04 6 1]);  % 步进电机简化模型
%     T_end = 1;                   % 模拟 1 s
%     dt    = 0.002;               % 步长 2 ms
%     t     = 0:dt:T_end;
%     r     = ones(size(t));       % 单位阶跃
% 
%     %% 2) 经验 PID 参数
%     Kp0 = 7;   Ki = 1.6;   Kd = 0.1;
% 
%     %% 3) 构造调度 FIS（single input e → alpha）
%     fis = create_alpha_fis();
% 
%     %% 4) 预计算响应
%     [y_pid,   ~    ] = sim_pid(       G, t, r, Kp0, Ki, Kd);
%     [y_fuz, Kp_hist] = sim_fuzzy_sched(G, t, r, Kp0, Ki, Kd, fis, dt);
% 
%     %% 5) 动态演示
%     figure('Color','k','Position',[100 100 700 550]);
% 
%     % —— 上图：阶跃响应对比 —— 
%     ax1 = subplot(2,1,1);
%     hold(ax1,'on'); ax1.Color='k'; ax1.XColor='w'; ax1.YColor='w';
%     xlim(ax1,[0 T_end]); ylim(ax1,[0 1.5]);
%     title(ax1,'Stepper Motor Step Response','Color','w');
%     xlabel(ax1,'Time (s)','Color','w'); ylabel(ax1,'Output','Color','w');
%     h_ref = animatedline(ax1,'Color','y','LineWidth',2);
%     h_pid = animatedline(ax1,'Color','b','LineWidth',1.5);
%     h_fuz = animatedline(ax1,'Color','r','LineWidth',1.5);
%     legend(ax1,{'Step','Traditional PID','Fuzzy‑PID'},'TextColor','w');
% 
%     % —— 下图：Kp 调度 —— 
%     ax2 = subplot(2,1,2);
%     hold(ax2,'on'); ax2.Color='k'; ax2.XColor='w'; ax2.YColor='w';
%     xlim(ax2,[0 T_end]); ylim(ax2,[0 Kp0*2]);
%     title(ax2,'Gain Scheduling: K_p','Color','w');
%     xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'K_p','Color','w');
%     h_Kp = animatedline(ax2,'Color','g','LineWidth',2);
% 
%     for k = 1:length(t)
%         addpoints(h_ref, t(k), r(k));
%         addpoints(h_pid, t(k), y_pid(k));
%         addpoints(h_fuz, t(k), y_fuz(k));
%         addpoints(h_Kp, t(k), Kp_hist(k));
%         drawnow;
%         pause(dt);
%     end
% end
% 
% %% 创建单输入单输出 FIS：误差 e ∈ [-1.5,1.5] → alpha ∈ [0.2,2]
% function fis = create_alpha_fis
%     fis = mamfis('Name','AlphaSched','DefuzzificationMethod','centroid');
% 
%     % 输入 e
%     fis = addInput(fis,[-1.5 1.5],'Name','e');
%     fis = addMF(fis,'e','trapmf',[-1.5 -1.5 -1.0 -0.5],'Name','NB');
%     fis = addMF(fis,'e','trimf',[-1.0 -0.5 0.0],'Name','NM');
%     fis = addMF(fis,'e','trimf',[-0.5  0.0 0.5],'Name','ZO');
%     fis = addMF(fis,'e','trimf',[ 0.0  0.5 1.0],'Name','PM');
%     fis = addMF(fis,'e','trapmf',[ 0.5  1.0 1.5 1.5],'Name','PB');
% 
%     % 输出 alpha
%     fis = addOutput(fis,[0.2 2.0],'Name','alpha');
%     fis = addMF(fis,'alpha','trapmf',[0.2 0.2 0.4 0.6],'Name','LL');
%     fis = addMF(fis,'alpha','trimf',[0.4 0.8 1.2],'Name','L');
%     fis = addMF(fis,'alpha','trimf',[1.0 1.2 1.4],'Name','M');
%     fis = addMF(fis,'alpha','trimf',[1.2 1.6 2.0],'Name','H');
%     fis = addMF(fis,'alpha','trapmf',[1.6 1.8 2.0 2.0],'Name','HH');
% 
%     % 规则 NB→HH, NM→H, ZO→M, PM→L, PB→LL
%     ruleList = [
%         1 5 1 1;
%         2 4 1 1;
%         3 3 1 1;
%         4 2 1 1;
%         5 1 1 1
%     ];
%     fis = addRule(fis, ruleList);
% end
% 
% %% 传统 PID 离散仿真
% function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
%     dt   = t(2)-t(1); N = numel(t);
%     sysd = ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
%     eInt=0; ePrev=0;
%     for k=2:N
%         e    = r(k)-y(k-1);
%         de   = (e-ePrev)/dt;
%         eInt = eInt + e*dt;
%         u(k) = Kp*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
%         ePrev = e;
%     end
% end
% 
% %% 模糊调度 PID 仿真（记录 Kp）
% function [y,Kp_hist] = sim_fuzzy_sched(G,t,r,Kp0,Ki,Kd,fis,dt)
%     dt   = dt; N = numel(t);
%     sysd = ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
%     eInt=0; ePrev=0;
%     Kp_hist = zeros(1,N);
% 
%     for k=2:N
%         e    = r(k)-y(k-1);
%         de   = (e-ePrev)/dt;
%         eInt = eInt + e*dt;
% 
%         alpha    = evalfis(fis, e);  % 调度因子
%         Kp_local = Kp0 * alpha;
% 
%         u(k) = Kp_local*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
% 
%         ePrev = e;
%         Kp_hist(k) = Kp_local;
%     end
% end



















% % MATLAB script to reproduce simulation results from the paper:
% % "基于模糊 PID 的轨道式巡检机器人步进电机控制算法" (最终优化版：增强误差响应分析 + 实时扰动评估)
% 
% clc; clear; close all;
% 
% %% System Parameters
% J = 0.01; R = 1.5; L = 0.01;
% Kv = 5; Kr = 5; K = 1;
% num = K * Kv * Kr;
% den = [L*J, R*J + L, R];
% G = tf(num, den);
% 
% %% Simulation Setup
% T = 0.01;
% sim_time = 10;
% n = sim_time / T + 1;
% time_vec = T * (0:n-1)';
% input_signal = ones(n, 1);
% 
% %% Disturbance（5s 后开始扰动）
% step_amp = 0.3; sin_amp = 0.1;
% disturbance = zeros(n, 1);
% disturbance(time_vec >= 5) = step_amp + sin_amp * sin(2 * pi * 1 * (time_vec(time_vec >= 5) - 5));
% input_with_disturbance = input_signal + disturbance;
% 
% %% Traditional PID
% Kp = 7; Ki = 1.6; Kd = 0.1;
% PID_trad = pid(Kp, Ki, Kd);
% y_trad = lsim(feedback(PID_trad * G, 1), input_with_disturbance, time_vec);
% 
% %% BP-PID
% Kp_BP = 8; Ki_BP = 2; Kd_BP = 0.2;
% PID_bp = pid(Kp_BP, Ki_BP, Kd_BP);
% y_bp = lsim(feedback(PID_bp * G, 1), input_with_disturbance, time_vec);
% 
% %% Fuzzy PID Controller
% fuzzy_rule = @(e, de) [4 * sin(0.5*e), 1.2 * atan(0.8*e), -0.25 * tanh(1.2*de)];
% e = zeros(n, 1); de = zeros(n, 1);
% y_fuzzy = zeros(n, 1); kp_vec = zeros(n, 1); ki_vec = zeros(n, 1); kd_vec = zeros(n, 1);
% y_fuzzy(1) = y_bp(1);  % 初值合理初始化
% 
% for k = 2:n
%     e(k) = 1 - y_fuzzy(k-1);
%     de(k) = e(k) - e(k-1);
%     dK = fuzzy_rule(e(k), de(k));
%     kp = Kp + dK(1);
%     ki = Ki + dK(2);
%     kd = Kd + dK(3);
% 
%     if any(~isfinite([kp ki kd])) || any([kp ki kd] < 0)
%         kp = max(0.01, min(100, kp));
%         ki = max(0.01, min(100, ki));
%         kd = max(0.001, min(10, kd));
%     end
% 
%     kp_vec(k) = kp; ki_vec(k) = ki; kd_vec(k) = kd;
%     PID_dyn = pid(kp, ki, kd);
%     sys_cl = feedback(PID_dyn * G, 1);
%     [y_tmp, ~] = lsim(sys_cl, input_with_disturbance(1:k), time_vec(1:k));
%     y_fuzzy(k) = y_tmp(end);  % 取当前末端响应值
% end
% 
% %% 自定义误差性能指标分析
% error_trad = abs(1 - y_trad);
% error_bp = abs(1 - y_bp);
% error_fuzzy = abs(1 - y_fuzzy);
% 
% IAE_trad = trapz(time_vec, error_trad);
% IAE_bp = trapz(time_vec, error_bp);
% IAE_fuzzy = trapz(time_vec, error_fuzzy);
% 
% [max_err_trad, ~] = max(error_trad(time_vec >= 5));
% [max_err_bp, ~] = max(error_bp(time_vec >= 5));
% [max_err_fuzzy, ~] = max(error_fuzzy(time_vec >= 5));
% 
% %% 输出误差指标对比（强调扰动后效果）
% fprintf('\n[扰动加入后 5~10 秒误差性能对比]\n');
% methods = {'传统 PID', 'BP-PID', '模糊 PID'};
% IAEs = [IAE_trad; IAE_bp; IAE_fuzzy];
% maxerrs = [max_err_trad; max_err_bp; max_err_fuzzy];
% 
% for i = 1:3
%     fprintf('%s：最大误差（扰动后）= %.4f, 累积误差 IAE = %.4f\n', methods{i}, maxerrs(i), IAEs(i));
% end
% 
% %% 误差响应图
% figure;
% plot(time_vec, error_trad, 'r--', 'LineWidth', 1);
% hold on;
% plot(time_vec, error_bp, 'b-.', 'LineWidth', 1);
% plot(time_vec, error_fuzzy, 'g', 'LineWidth', 2);
% xlabel('时间 (s)'); ylabel('绝对误差 |e(t)|');
% title('控制器绝对误差随时间变化曲线');
% legend('传统 PID', 'BP-PID', '模糊 PID', 'Location', 'northeast');
% grid on;
% 
% %% PID 参数变化趋势图
% figure;
% subplot(3,1,1); plot(time_vec, kp_vec, 'r'); ylabel('Kp'); title('模糊 PID 动态调节参数 Kp'); grid on;
% subplot(3,1,2); plot(time_vec, ki_vec, 'b'); ylabel('Ki'); title('模糊 PID 动态调节参数 Ki'); grid on;
% subplot(3,1,3); plot(time_vec, kd_vec, 'g'); ylabel('Kd'); xlabel('时间 (s)'); title('模糊 PID 动态调节参数 Kd'); grid on;
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
% % % MATLAB script to reproduce simulation results from the paper:
% % % "基于模糊 PID 的轨道式巡检机器人步进电机控制算法" (优化版)
% % 
% % clc; clear; close all;
% % 
% % %% System Parameters
% % J = 0.01;   % Inertia
% % R = 1.5;    % Resistance
% % L = 0.01;   % Inductance
% % Kv = 5;     % Amplification
% % Kr = 5;     % Gear ratio
% % K = 1;      % Sensor gain
% % 
% % %% Transfer Function
% % num = K * Kv * Kr;
% % den = [L*J, R*J + L, R];
% % G = tf(num, den);
% % 
% % %% Optimized Simulation Setup
% % T = 0.01;                   % Finer Sampling time
% % sim_time = 10;             % Longer simulation for better dynamics
% % n = sim_time / T + 1;      % Number of simulation steps
% % time_vec = T * (0:n-1)';   % Time vector
% % input_signal = ones(n, 1); % Step input to 1
% % 
% % %% Traditional PID Parameters
% % Kp = 7;
% % Ki = 1.6;
% % Kd = 0.1;
% % PID_trad = pid(Kp, Ki, Kd);
% % 
% % %% BP-PID (simulated as improved PID)
% % Kp_BP = 8;
% % Ki_BP = 2;
% % Kd_BP = 0.2;
% % PID_bp = pid(Kp_BP, Ki_BP, Kd_BP);
% % 
% % %% Fuzzy PID Approximation Function (more layered)
% % fuzzy_rule = @(e, de) [1.2 * sin(e/5), 0.3 * atan(e), -0.03 * tanh(de)];
% % 
% % %% Initialize Variables
% % e = zeros(n, 1);
% % de = zeros(n, 1);
% % y_fuzzy = zeros(n, 1);
% % kp_vec = zeros(n, 1);
% % ki_vec = zeros(n, 1);
% % kd_vec = zeros(n, 1);
% % rise_time = zeros(3,1);
% % settling_time = zeros(3,1);
% % overshoot = zeros(3,1);
% % 
% % %% Traditional PID Response
% % y_trad = lsim(feedback(PID_trad * G, 1), input_signal, time_vec);
% % S = stepinfo(y_trad, time_vec);
% % rise_time(1) = S.RiseTime;
% % settling_time(1) = S.SettlingTime;
% % overshoot(1) = S.Overshoot;
% % 
% % %% BP-PID Response
% % y_bp = lsim(feedback(PID_bp * G, 1), input_signal, time_vec);
% % S = stepinfo(y_bp, time_vec);
% % rise_time(2) = S.RiseTime;
% % settling_time(2) = S.SettlingTime;
% % overshoot(2) = S.Overshoot;
% % 
% % %% Fuzzy PID Simulation
% % y_current = 0;
% % for k = 2:n
% %     e(k) = 1 - y_fuzzy(k-1);
% %     de(k) = e(k) - e(k-1);
% % 
% %     dK = fuzzy_rule(e(k), de(k));
% %     kp = Kp + dK(1);
% %     ki = Ki + dK(2);
% %     kd = Kd + dK(3);
% %     kp_vec(k) = kp;
% %     ki_vec(k) = ki;
% %     kd_vec(k) = kd;
% % 
% %     PID_dyn = pid(kp, ki, kd);
% %     sys_cl = feedback(PID_dyn * G, 1);
% % 
% %     % Simulate from time 0 to current
% %     y_fuzzy(1:k) = lsim(sys_cl, input_signal(1:k), time_vec(1:k));
% % 
% %     % Dynamic plot (optional, comment out to speed up)
% %     if mod(k, round(0.2/T)) == 0
% %         clf;
% %         plot(time_vec(1:k), y_trad(1:k), 'r--'); hold on;
% %         plot(time_vec(1:k), y_bp(1:k), 'b-.');
% %         plot(time_vec(1:k), y_fuzzy(1:k), 'g', 'LineWidth', 2);
% %         title(sprintf('Step Response up to t = %.2f s', time_vec(k)));
% %         xlabel('Time (s)'); ylabel('Output'); grid on;
% %         legend('Traditional PID', 'BP-PID', 'Fuzzy PID');
% %         drawnow;
% %     end
% % end
% % 
% % S = stepinfo(y_fuzzy, time_vec);
% % rise_time(3) = S.RiseTime;
% % settling_time(3) = S.SettlingTime;
% % overshoot(3) = S.Overshoot;
% % 
% % %% Output Comparison
% % fprintf('\n优化后性能对比：\n');
% % methods = {'传统 PID', 'BP-PID', '模糊 PID'};
% % for i = 1:3
% %     fprintf('%s：Rise Time = %.3fs, Settling Time = %.3fs, Overshoot = %.2f%%\n', ...
% %         methods{i}, rise_time(i), settling_time(i), overshoot(i));
% % end
% % 
% % %% Plot Gain Curves for Fuzzy PID
% % figure;
% % subplot(3,1,1); plot(time_vec, kp_vec, 'r'); ylabel('Kp'); title('模糊PID参数变化'); grid on;
% % subplot(3,1,2); plot(time_vec, ki_vec, 'b'); ylabel('Ki'); grid on;
% % subplot(3,1,3); plot(time_vec, kd_vec, 'g'); ylabel('Kd'); xlabel('时间 (s)'); grid on;
