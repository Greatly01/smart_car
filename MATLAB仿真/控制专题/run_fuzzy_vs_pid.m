function demo_gain_scheduling_fast
    %% 1) 系统 & 时间
    G      = tf(5.16,[630 1 0]);   % 被控对象
    T_end  = 30;   dt = 0.01;
    t      = 0:dt:T_end;
    r      = 100 * ones(size(t));

    %% 2) 预计算传统 PID 和调度 PID
    Kp0 = 80; Ki = 9; Kd0 = 80;
    [y_pid,   ~] = sim_pid(G, t, r, Kp0, Ki, Kd0);
    [y_sched, Kp_hist, Kd_hist] = sim_scheduled_pid_fast(G, t, r, Kp0, Ki, Kd0, dt);

    %% 3) 动态演示
    figure('Color','k','Position',[100 100 900 600]);

    % 上图：输出响应
    ax1 = subplot(2,1,1);
    hold(ax1,'on'); ax1.Color='k'; ax1.XColor='w'; ax1.YColor='w';
    title(ax1,'Step Response: PID vs Fast‑Scheduled PID','Color','w');
    xlabel(ax1,'Time (s)','Color','w'); ylabel(ax1,'Temp (℃)','Color','w');
    xlim(ax1,[0 T_end]); ylim(ax1,[0 160]);
    h_step  = animatedline(ax1,'Color','b','LineWidth',2);
    h_pid   = animatedline(ax1,'Color','y','LineWidth',1.5);
    h_sched = animatedline(ax1,'Color','r','LineWidth',1.5);
    legend(ax1,{'Step','Traditional PID','Fast‑Scheduled PID'},...
           'TextColor','w','Location','northeast');

    % 下图：Kp & Kd 调度
    ax2 = subplot(2,1,2);
    hold(ax2,'on'); ax2.Color='k'; ax2.XColor='w'; ax2.YColor='w';
    title(ax2,'Gain Scheduling: Kp & Kd','Color','w');
    xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'Gain','Color','w');
    xlim(ax2,[0 T_end]); ylim(ax2,[0 200]);
    h_Kp = animatedline(ax2,'Color','g','LineWidth',2);
    h_Kd = animatedline(ax2,'Color','m','LineWidth',2);
    legend(ax2,{'Kp','Kd'},'TextColor','w','Location','northeast');

    % 实时动画
    for k = 1:length(t)
        % 响应
        addpoints(h_step,  t(k),   r(k));
        addpoints(h_pid,   t(k),   y_pid(k));
        addpoints(h_sched, t(k),   y_sched(k));
        % 增益
        addpoints(h_Kp, t(k), Kp_hist(k));
        addpoints(h_Kd, t(k), Kd_hist(k));
        drawnow;
        pause(dt);
    end
end

%% 传统 PID 离散仿真
function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
    dt = t(2)-t(1); N = numel(t);
    sysd = ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
    y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
    eInt=0; ePrev=0;
    for k=2:N
        e=r(k)-y(k-1);
        de=(e-ePrev)/dt;
        eInt=eInt+e*dt;
        u(k)=Kp*e + Ki*eInt + Kd*de;
        x=A*x + B*u(k); y(k)=C*x + D*u(k);
        ePrev=e;
    end
end

%% 快速调度 PID 离散仿真
function [y,u,Kp_hist,Kd_hist] = sim_scheduled_pid_fast(G,t,r,Kp0,Ki,Kd0,dt)
    dt=dt; N=numel(t);
    sysd=ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
    y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
    eInt=0; ePrev=0;
    Kp_hist=zeros(1,N); Kd_hist=zeros(1,N);

    for k=2:N
        e  = r(k)-y(k-1);
        de = (e-ePrev)/dt;

        % —————— 新调度策略 ——————
        ae = abs(e);
        if     ae > 40
            alpha = 0.3;      % 大误差抑制
        elseif ae > 5
            alpha = 2.0;      % 中误差强化 200%
        else
            alpha = 1.0;      % 小误差正常
        end

        be = abs(de);
        if     be > 10
            beta = 2.0;       % 快导数增阻尼
        elseif be > 5
            beta = 1.5;
        else
            beta = 1.0;
        end
        % ——————————————

        Kp = Kp0 * alpha;
        Kd = Kd0 * beta;

        eInt=eInt + e*dt;
        u(k)=Kp*e + Ki*eInt + Kd*de;
        x   =A*x + B*u(k); y(k)=C*x + D*u(k);

        ePrev=e;
        Kp_hist(k)=Kp;
        Kd_hist(k)=Kd;
    end
end
















% function demo_gain_scheduling_slow
%     %% 1) 系统 & 时间
%     G      = tf(5.16,[630 1 0]);
%     T_end  = 30;    dt = 0.01;
%     t      = 0:dt:T_end;
%     r      = 100 * ones(size(t));
% 
%     %% 2) 基准 PID 和调度 PID 预计算
%     Kp0 = 80; Ki = 9; Kd0 = 80;
%     [y_pid,   ~] = sim_pid(G, t, r, Kp0, Ki, Kd0);
%     [y_sched, Kp_hist, Kd_hist] = sim_scheduled_pid(G, t, r, Kp0, Ki, Kd0, dt);
% 
%     %% 3) 动态演示
%     figure('Color','k','Position',[200 100 800 600]);
% 
%     % 上图：响应
%     ax1 = subplot(2,1,1);
%     hold(ax1,'on'); ax1.Color='k'; ax1.XColor='w'; ax1.YColor='w';
%     xlim(ax1,[0 T_end]); ylim(ax1,[0 160]);
%     title(ax1,'Step Response: PID vs Scheduled','Color','w');
%     xlabel(ax1,'Time (s)','Color','w'); ylabel(ax1,'Temp (°C)','Color','w');
%     h_step  = animatedline(ax1,'Color','b','LineWidth',2);
%     h_pid   = animatedline(ax1,'Color','y','LineWidth',1.5);
%     h_sched = animatedline(ax1,'Color','r','LineWidth',1.5);
%     legend(ax1,{'Step','Traditional PID','Scheduled PID'},...
%            'TextColor','w','Location','northeast');
% 
%     % 下图：Kp & Kd 调度
%     ax2 = subplot(2,1,2);
%     hold(ax2,'on'); ax2.Color='k'; ax2.XColor='w'; ax2.YColor='w';
%     xlim(ax2,[0 T_end]); ylim(ax2,[0 200]);
%     title(ax2,'Gain Scheduling: Kp & Kd','Color','w');
%     xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'Gain','Color','w');
%     h_Kp = animatedline(ax2,'Color','g','LineWidth',2);
%     h_Kd = animatedline(ax2,'Color','m','LineWidth',2);
%     legend(ax2,{'Kp','Kd'},'TextColor','w','Location','northeast');
% 
%     % 实时动画
%     for k = 1:length(t)
%         % 上图数据
%         addpoints(h_step,  t(k),   r(k));
%         addpoints(h_pid,   t(k),   y_pid(k));
%         addpoints(h_sched, t(k),   y_sched(k));
%         % 下图数据
%         addpoints(h_Kp, t(k), Kp_hist(k));
%         addpoints(h_Kd, t(k), Kd_hist(k));
%         drawnow;
%         pause(dt);  % 按仿真步长放慢动画
%     end
% end
% 
% %% 以下仿真函数同上，略作重用
% function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
%     dt = t(2)-t(1); N = numel(t);
%     sysd=ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
%     eInt=0; ePrev=0;
%     for k=2:N
%         e=r(k)-y(k-1); de=(e-ePrev)/dt; eInt=eInt+e*dt;
%         u(k)=Kp*e + Ki*eInt + Kd*de;
%         x=A*x+B*u(k); y(k)=C*x+D*u(k);
%         ePrev=e;
%     end
% end
% 
% function [y,u,Kp_hist,Kd_hist] = sim_scheduled_pid(G,t,r,Kp0,Ki,Kd0,dt)
%     dt=dt; N=numel(t);
%     sysd=ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
%     eInt=0; ePrev=0;
%     Kp_hist=zeros(1,N); Kd_hist=zeros(1,N);
%     for k=2:N
%         e=r(k)-y(k-1); de=(e-ePrev)/dt;
%         % 调度逻辑
%         ae=abs(e);
%         if     ae>30, alpha=0.3;
%         elseif ae>10, alpha=1.2;
%         else        alpha=1.0; end
%         be=abs(de);
%         if     be>10, beta=3.0;
%         elseif be>5,  beta=1.5;
%         else          beta=1.0; end
%         Kp = Kp0*alpha; Kd = Kd0*beta;
%         eInt=eInt+e*dt;
%         u(k)=Kp*e + Ki*eInt + Kd*de;
%         x=A*x+B*u(k); y(k)=C*x+D*u(k);
%         ePrev=e;
%         Kp_hist(k)=Kp; Kd_hist(k)=Kd;
%     end
% end


















% function demo_gain_scheduling
%     %% 1) 系统 & 时间
%     G      = tf(5.16,[630 1 0]);   % 被控对象
%     T_end  = 30; dt = 0.01;
%     t      = 0:dt:T_end;
%     r      = 100*ones(size(t));    % 阶跃 0→100 °C
% 
%     %% 2) 基准 PID 参数
%     Kp0 = 80; Ki = 9; Kd0 = 80;
% 
%     %% 3) 传统 PID 仿真（提前计算）
%     [y_pid, ~] = sim_pid(G, t, r, Kp0, Ki, Kd0);
% 
%     %% 4) 增益调度 PID 仿真（返回增益序列）
%     [y_sched, Kp_hist, Kd_hist] = sim_scheduled_pid(G, t, r, Kp0, Ki, Kd0, dt);
% 
%     %% 5) 动态展示
%     figure('Color','k','Position',[200 100 800 600]);
% 
%     % 上方：输出响应
%     ax1 = subplot(2,1,1);
%     hold(ax1,'on');
%     ax1.Color = 'k'; ax1.XColor='w'; ax1.YColor='w';
%     title(ax1,'Step Response: PID vs Scheduled','Color','w');
%     xlabel(ax1,'Time (s)','Color','w'); ylabel(ax1,'Temp (°C)','Color','w');
%     xlim(ax1,[0 T_end]); ylim(ax1,[0 160]);
%     h_step  = animatedline(ax1,'Color','b','LineWidth',2);
%     h_pid   = animatedline(ax1,'Color','y','LineWidth',1.5);
%     h_sched = animatedline(ax1,'Color','r','LineWidth',1.5);
%     legend(ax1,{'Step','Traditional PID','Scheduled PID'},...
%            'TextColor','w','Location','northeast');
% 
%     % 下方：Kp & Kd 调度曲线
%     ax2 = subplot(2,1,2);
%     hold(ax2,'on');
%     ax2.Color = 'k'; ax2.XColor='w'; ax2.YColor='w';
%     title(ax2,'Gain Scheduling: Kp & Kd','Color','w');
%     xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'Gain','Color','w');
%     xlim(ax2,[0 T_end]); ylim(ax2,[0 200]);
%     h_Kp = animatedline(ax2,'Color','g','LineWidth',2);
%     h_Kd = animatedline(ax2,'Color','m','LineWidth',2);
%     legend(ax2,{'Kp','Kd'},'TextColor','w','Location','northeast');
% 
%     % 动态循环
%     for k = 1:length(t)
%         % 上图：响应
%         addpoints(h_step,  t(k),   r(k));
%         addpoints(h_pid,   t(k),   y_pid(k));
%         addpoints(h_sched, t(k),   y_sched(k));
%         % 下图：增益
%         addpoints(h_Kp, t(k), Kp_hist(k));
%         addpoints(h_Kd, t(k), Kd_hist(k));
%         drawnow limitrate
%     end
% end
% 
% %% 传统 PID 离散仿真
% function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
%     dt = t(2)-t(1); N = numel(t);
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
% %% 增益调度 PID 离散仿真（记录 Kp, Kd）
% function [y,u,Kp_hist,Kd_hist] = sim_scheduled_pid(G,t,r,Kp0,Ki,Kd0,dt)
%     dt = dt; N = numel(t);
%     sysd = ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     y=zeros(1,N); u=zeros(1,N); x=zeros(size(A,1),1);
%     eInt=0; ePrev=0;
%     Kp_hist = zeros(1,N); Kd_hist = zeros(1,N);
%     for k=2:N
%         e  = r(k)-y(k-1);
%         de = (e-ePrev)/dt;
%         % ———— 增益调度逻辑 ————
%         ae = abs(e);
%         if     ae>30
%             alpha = 0.3;   % 大误差抑制至 30%
%         elseif ae>10
%             alpha = 1.2;   % 中误差放大 20%
%         else
%             alpha = 1.0;   % 小误差正常
%         end
%         be = abs(de);
%         if     be>10
%             beta = 3.0;    % 大导数强化阻尼
%         elseif be>5
%             beta = 1.5;
%         else
%             beta = 1.0;
%         end
%         Kp = Kp0 * alpha;
%         Kd = Kd0 * beta;
%         % ——————————————
%         eInt = eInt + e*dt;
%         u(k) = Kp*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
%         ePrev = e;
%         Kp_hist(k) = Kp;
%         Kd_hist(k) = Kd;
%     end
% end















% function animated_gain_scheduled_pid
% % ANIMATED_GAIN_SCHEDULED_PID
% % 纯代码增益调度 PID 仿真对比：Traditional PID vs Scheduled‑Gain PID
% %
% % • 目标：振幅显著小于传统 PID，同时响应更快
% % • 动态动画：实时绘制阶跃、传统 PID、调度 PID 三条曲线
% %
% % Usage:
% %   >> animated_gain_scheduled_pid
% %
% 
%     %% 1) 系统 & 时间设置
%     G      = tf(5.16, [630 1 0]);    % 被控对象
%     T_end  = 30;                     % 模拟到 30 s
%     dt     = 0.01;                   % 步长 0.01 s
%     t      = 0:dt:T_end;
%     r      = 100 * ones(size(t));    % 0→100 ℃ 阶跃
% 
%     %% 2) 基准 PID 参数
%     Kp_base = 80;   Ki = 9;   Kd_base = 80;
% 
%     %% 3) 传统 PID 离散仿真
%     [y_pid, ~] = sim_pid(G, t, r, Kp_base, Ki, Kd_base);
% 
%     %% 4) 调度增益 PID 仿真
%     [y_sched, ~] = sim_scheduled_pid(G, t, r, Kp_base, Ki, Kd_base, dt);
% 
%     %% 5) 动态绘图
%     figure('Color','k');
%     ax = gca; ax.Color='k'; ax.XColor='w'; ax.YColor='w';
%     xlim([0 T_end]); ylim([0 160]);
%     xlabel('Time (s)','Color','w'); ylabel('Temperature (℃)','Color','w');
%     title('Gain‑Scheduled PID vs Traditional PID','Color','w');
%     hold on;
% 
%     h_step  = animatedline('Color','b','LineWidth',2);
%     h_pid   = animatedline('Color','y','LineWidth',1.5);
%     h_sched = animatedline('Color','r','LineWidth',1.5);
%     legend({'Step','Traditional PID','Scheduled‑Gain PID'}, ...
%            'TextColor','w','Location','northeast');
% 
%     for k = 1:length(t)
%         addpoints(h_step,  t(k),   r(k));
%         addpoints(h_pid,   t(k),   y_pid(k));
%         addpoints(h_sched, t(k),   y_sched(k));
%         drawnow limitrate
%     end
% end
% 
% %% 传统 PID 离散仿真
% function [y,u] = sim_pid(G, t, r, Kp, Ki, Kd)
%     dt = t(2) - t(1);
%     sysd = ss(c2d(G, dt));
%     A = sysd.A; B = sysd.B; C = sysd.C; D = sysd.D;
% 
%     N    = numel(t);
%     y    = zeros(1, N);
%     u    = zeros(1, N);
%     x    = zeros(size(A,1),1);
%     eInt = 0; ePrev = 0;
% 
%     for k = 2:N
%         e    = r(k) - y(k-1);
%         de   = (e - ePrev) / dt;
%         eInt = eInt + e*dt;
%         u(k) = Kp*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
%         ePrev = e;
%     end
% end
% 
% %% 增益调度 PID 离散仿真
% function [y,u] = sim_scheduled_pid(G, t, r, Kp0, Ki, Kd0, dt)
%     sysd = ss(c2d(G, dt));
%     A = sysd.A; B = sysd.B; C = sysd.C; D = sysd.D;
% 
%     N    = numel(t);
%     y    = zeros(1, N);
%     u    = zeros(1, N);
%     x    = zeros(size(A,1),1);
%     eInt = 0; ePrev = 0;
% 
%     for k = 2:N
%         % 误差及导数
%         e  = r(k) - y(k-1);
%         de = (e - ePrev) / dt;
% 
%         % —————— 增益调度规则 ——————
%         % Kp 调度 α 探索：加速+抑制
%         ae = abs(e);
%         if     ae > 30
%             alpha = 0.4;       % 大误差，减小比例增益
%         elseif ae > 10
%             alpha = 1.5;       % 中误差，加大比例增益
%         else
%             alpha = 1.0;       % 小误差，正常
%         end
%         Kp = Kp0 * alpha;
% 
%         % Kd 调度 β：大导数时加强阻尼
%         ade = abs(de);
%         if ade > 10
%             beta = 3.0;
%         elseif ade > 5
%             beta = 1.5;
%         else
%             beta = 1.0;
%         end
%         Kd = Kd0 * beta;
% 
%         % PID 计算
%         eInt = eInt + e*dt;
%         u(k) = Kp*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
% 
%         ePrev = e;
%     end
% end













% function gain_scheduled_fuzzy_pid
% % GAIN_SCHEDULED_FUZZY_PID
% % 纯代码仿真：Traditional PID vs 增益调度型 Fuzzy‑PID
% % 误差大时 Kp 降至 20%，显著抑制过冲
% 
%     %% 1) 系统 & 仿真参数
%     G      = tf(5.16,[630 1 0]);   % 被控对象
%     Tend   = 30; dt = 0.01;        
%     t      = 0:dt:Tend;           
%     r      = 100*ones(size(t));    % 阶跃 0→100℃
% 
%     %% 2) 构造单输入单输出 FIS（误差 → α）
%     fis = create_gain_schedule_fis();
% 
%     %% 3) 传统 PID 仿真
%     Kp0=80; Ki0=9; Kd0=80;
%     [y_pid, ~] = sim_pid(G, t, r, Kp0, Ki0, Kd0);
% 
%     %% 4) 增益调度型模糊 PID 仿真
%     [y_fuzzy, ~] = sim_fuzzy_pid4(G, t, r, fis, dt, Kp0, Ki0, Kd0);
% 
%     %% 5) 动态绘图
%     figure('Color','k');
%     ax = gca; ax.Color='k'; ax.XColor='w'; ax.YColor='w';
%     xlim([0 Tend]); ylim([0 160]);
%     xlabel('Time (s)','Color','w'); ylabel('Temperature (℃)','Color','w');
%     title('Gain‑Scheduled Fuzzy‑PID vs Traditional PID','Color','w');
%     hold on;
% 
%     hS = animatedline('Color','b','LineWidth',2);
%     hP = animatedline('Color','y','LineWidth',1.5);
%     hF = animatedline('Color','r','LineWidth',1.5);
%     legend({'Step','Traditional PID','Fuzzy‑PID'}, ...
%            'TextColor','w','Location','northeast');
% 
%     for k = 1:length(t)
%         addpoints(hS, t(k),   r(k));
%         addpoints(hP, t(k),   y_pid(k));
%         addpoints(hF, t(k),   y_fuzzy(k));
%         drawnow limitrate
%     end
% end
% 
% %% —— 构造增益调度型 FIS ——
% function fis = create_gain_schedule_fis
%     % 单输入 e ∈ [-100,100]，单输出 α ∈ [0.2,1]
%     labels = {'NB','NM','NS','ZO','PS','PM','PB'};
%     e_edges = [-100 -50 0 50 100];
%     
%     % DefuzzificationMethod 必填
%     fis = mamfis('Name','GainSched','DefuzzificationMethod','centroid');
%     
%     % 输入 e
%     fis = addInput(fis,[-100 100],'Name','e');
%     fis = addMF(fis,'e','trapmf',[-100 -100 -75 -50],'Name','NB');
%     fis = addMF(fis,'e','trimf',[-75 -50 -25],'Name','NM');
%     fis = addMF(fis,'e','trimf',[-25 0 25],'Name','NS');
%     fis = addMF(fis,'e','trimf',[0 25 50],'Name','PS');
%     fis = addMF(fis,'e','trimf',[25 50 75],'Name','PM');
%     fis = addMF(fis,'e','trapmf',[50 75 100 100],'Name','PB');
%     
%     % 输出 α
%     fis = addOutput(fis,[0.2 1],'Name','alpha');
%     % α 子集：LL(0.2–0.3), L(0.3–0.5), M(0.5–0.7), H(0.7–0.9), HH(0.9–1)
%     fis = addMF(fis,'alpha','trapmf',[0.2 0.2 0.25 0.3],'Name','LL');
%     fis = addMF(fis,'alpha','trimf',[0.25 0.35 0.5],'Name','L');
%     fis = addMF(fis,'alpha','trimf',[0.45 0.6 0.75],'Name','M');
%     fis = addMF(fis,'alpha','trimf',[0.7 0.85 0.95],'Name','H');
%     fis = addMF(fis,'alpha','trapmf',[0.9 0.95 1.0 1.0],'Name','HH');
%     
%     % 规则：大误差→小 α（降 Kp），中误差→中 α，小误差→高 α
%     ruleList = [ ...
%         "e==PB => alpha=LL";  % 误差极大 → α 最小
%         "e==PM => alpha=L";
%         "e==PS => alpha=M";
%         "e==NS => alpha=H";
%         "e==NM => alpha=H";
%         "e==NB => alpha=HH"    % 误差为负 → α 最大
%     ];
%     fis = addRule(fis,ruleList);
% end
% 
% %% —— 传统 PID 离散仿真 —— 
% function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
%     dt = t(2)-t(1); N=numel(t);
%     y=zeros(1,N); u=zeros(1,N);
%     eInt=0; ePrev=0;
%     sysd=ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     x=zeros(size(A,1),1);
%     for k=2:N
%         e  = r(k)-y(k-1);
%         de = (e-ePrev)/dt;
%         eInt = eInt + e*dt;
%         u(k) = Kp*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
%         ePrev= e;
%     end
% end
% 
% %% —— 增益调度型模糊 PID 仿真 —— 
% function [y,u] = sim_fuzzy_pid4(G,t,r,fis,dt,Kp0,Ki,Kd)
%     dt = dt; N=numel(t);
%     y=zeros(1,N); u=zeros(1,N);
%     eInt=0; ePrev=0;
%     sysd=ss(c2d(G,dt)); A=sysd.A; B=sysd.B; C=sysd.C; D=sysd.D;
%     x=zeros(size(A,1),1);
%     for k=2:N
%         e  = r(k)-y(k-1);
%         % 直接使用原始误差 e 评估 alpha
%         alpha = evalfis(fis, e);
%         Kp = alpha * Kp0;
%         de = (e-ePrev)/dt;
%         eInt = eInt + e*dt;
%         u(k) = Kp*e + Ki*eInt + Kd*de;
%         x    = A*x + B*u(k);
%         y(k) = C*x + D*u(k);
%         ePrev= e;
%     end
% end
