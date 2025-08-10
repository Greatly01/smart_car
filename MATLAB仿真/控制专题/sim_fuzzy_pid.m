function [y,u] = sim_fuzzy_pid(G1,G2,fis,t,r,dt)
% G1/G2: 连续传递函数 (G2 用于 20s 后)、fis: Fuzzy‑PID 控制器

% 量化 & 比例因子
Ke=2; Kec=0.3;  Up=10; Ui=50; Ud=2;
% 初始 PID
Kp=80; Ki=9; Kd=80;

N = numel(t);
y = zeros(1,N);
u = zeros(1,N);
eInt  = 0;
ePrev = 0;

% 初始离散化 & 状态空间
sysd_ss = ss(c2d(G1,dt));
A = sysd_ss.A;  B = sysd_ss.B;
C = sysd_ss.C;  D = sysd_ss.D;
x = zeros(size(A,1),1);

for k = 2:N
    % 20s 后切换模型
    if abs(t(k)-20)<dt/2
        sysd_ss = ss(c2d(G2,dt));
        A = sysd_ss.A;  B = sysd_ss.B;
        C = sysd_ss.C;  D = sysd_ss.D;
    end
    
    % 计算误差
    e  = r(k) - y(k-1);
    de = (e - ePrev)/dt;
    
    % 模糊自整定
    dK  = evalfis(fis,[Ke*e, Kec*de]);
    Kp  = Kp + Up*dK(1);
    Ki  = Ki + Ui*dK(2);
    Kd  = Kd + Ud*dK(3);
    
    % PID 控制律
    eInt = eInt + e*dt;
    u(k) = Kp*e + Ki*eInt + Kd*de;
    
    % 状态更新 & 输出
    x    = A*x + B*u(k);
    y(k) = C*x + D*u(k);
    
    ePrev = e;
end
end
