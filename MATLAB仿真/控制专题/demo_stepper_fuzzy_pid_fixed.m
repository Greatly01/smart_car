function demo_2dof_robot_fuzzy
    %% 1) 系统与仿真设置
    I   = [0.02, 0.015];   % 惯量
    b   = [0.1, 0.08];     % 阻尼
    Kp0 = [30, 25];        % PD 比例增益
    Kd0 = [3, 2.5];        % PD 微分增益
    Ki  = [0, 0];          % 忽略积分项
    T_end = 5; dt = 0.005; % 总时长 5s，步长 5ms
    t   = 0:dt:T_end;
    N   = numel(t);

    % 参考轨迹：关节1、关节2 正弦
    qd = [0.5*sin(2*pi*0.5*t);
          0.5*sin(2*pi*0.5*t + pi/4)];

    % 初始化变量
    q_pd    = zeros(2, N);
    dq_pd   = zeros(2, N);
    q_fz    = zeros(2, N);
    dq_fz   = zeros(2, N);
    prev_e  = zeros(2,1);
    eInt    = zeros(2,1);
    Kp_hist = zeros(2, N);

    %% 2) 仿真循环
    for k = 1:N
        for j = 1:2
            % 误差 & 误差微分
            e  = qd(j,k) - q_pd(j,k);
            if k==1
                de = 0;
            else
                de = (e - prev_e(j)) / dt;
            end
            prev_e(j) = e;

            % ——— 传统 PD 控制 ———
            tau_pd  = Kp0(j)*e + Kd0(j)*de;
            ddq_pd  = (tau_pd - b(j)*dq_pd(j,k)) / I(j);
            dq_pd(j,k+1) = dq_pd(j,k) + ddq_pd*dt;
            q_pd(j,k+1)  = q_pd(j,k)  + dq_pd(j,k+1)*dt;

            % ——— 模糊 PD 控制（分段调度 Kp） ———
            ae = abs(e);
            if     ae > 0.4
                alpha = 0.7;   % 大误差时抑制
            elseif ae > 0.1
                alpha = 1.0;   % 中等误差时正常
            else
                alpha = 1.3;   % 微小误差时放大
            end
            Kp_loc = Kp0(j) * alpha;
            tau_fz = Kp_loc*e + Kd0(j)*de;
            ddq_fz = (tau_fz - b(j)*dq_fz(j,k)) / I(j);
            dq_fz(j,k+1) = dq_fz(j,k) + ddq_fz*dt;
            q_fz(j,k+1)  = q_fz(j,k)  + dq_fz(j,k+1)*dt;

            Kp_hist(j,k) = Kp_loc;
        end
    end

    % 修剪末尾（因 k+1 可能溢出）
    q_pd  = q_pd(:,1:N);
    q_fz  = q_fz(:,1:N);

    %% 3) 动态演示
    figure('Color','k','Position',[200 100 1000 400]);
    % 关节1
    ax1 = subplot(1,2,1);
    hold(ax1,'on'); ax1.Color='k'; ax1.XColor='w'; ax1.YColor='w';
    title(ax1,'Joint 1','Color','w');
    xlabel(ax1,'Time (s)','Color','w'); ylabel(ax1,'q (rad)','Color','w');
    xlim(ax1,[0 T_end]); ylim(ax1,[-0.6 0.6]);
    h_ref1 = animatedline(ax1,'Color','y','LineWidth',2);
    h_pd1  = animatedline(ax1,'Color','w','LineWidth',1.5);
    h_fz1  = animatedline(ax1,'Color','r','LineWidth',1.5);
    legend(ax1,{'Ref','PD','Fuzzy-PD'},'TextColor','w');

    % 关节2
    ax2 = subplot(1,2,2);
    hold(ax2,'on'); ax2.Color='k'; ax2.XColor='w'; ax2.YColor='w';
    title(ax2,'Joint 2','Color','w');
    xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'q (rad)','Color','w');
    xlim(ax2,[0 T_end]); ylim(ax2,[-0.6 0.6]);
    h_ref2 = animatedline(ax2,'Color','y','LineWidth',2);
    h_pd2  = animatedline(ax2,'Color','w','LineWidth',1.5);
    h_fz2  = animatedline(ax2,'Color','r','LineWidth',1.5);
    legend(ax2,{'Ref','PD','Fuzzy-PD'},'TextColor','w');

    for k = 1:N
        % 关节1
        addpoints(h_ref1, t(k), qd(1,k));
        addpoints(h_pd1,  t(k), q_pd(1,k));
        addpoints(h_fz1,  t(k), q_fz(1,k));
        % 关节2
        addpoints(h_ref2, t(k), qd(2,k));
        addpoints(h_pd2,  t(k), q_pd(2,k));
        addpoints(h_fz2,  t(k), q_fz(2,k));
        drawnow; pause(dt);
    end

    %% 4) 输出关键参数
    fprintf('Joint | Ctrl      | Overshoot(%%) | RMSE\n');
    for j = 1:2
        Mp_pd = (max(q_pd(j,:)) - max(qd(j,:))) / max(qd(j,:)) * 100;
        Mp_fz = (max(q_fz(j,:)) - max(qd(j,:))) / max(qd(j,:)) * 100;
        rmse_pd = sqrt(mean((q_pd(j,:) - qd(j,:)).^2));
        rmse_fz = sqrt(mean((q_fz(j,:) - qd(j,:)).^2));
        fprintf('  %d   | PD        |     %5.2f    | %7.4f\n', j, Mp_pd, rmse_pd);
        fprintf('  %d   | Fuzzy-PD  |     %5.2f    | %7.4f\n', j, Mp_fz, rmse_fz);
    end
end
