function [y,u] = sim_pid(G,t,r,Kp,Ki,Kd)
% G:  连续传递函数； t,r: 时间向量和参考输入；Kp,Ki,Kd: PID 参数

dt = t(2)-t(1);
N  = numel(t);
y  = zeros(1,N);
u  = zeros(1,N);
eInt  = 0;
ePrev = 0;

% 先离散化再转换到状态空间
sysd_ss = ss(c2d(G,dt));
A = sysd_ss.A;  B = sysd_ss.B;
C = sysd_ss.C;  D = sysd_ss.D;
x = zeros(size(A,1),1);

for k = 2:N
    % 计算误差
    e  = r(k) - y(k-1);
    de = (e - ePrev)/dt;
    eInt = eInt + e*dt;
    
    % PID 控制律
    u(k) = Kp*e + Ki*eInt + Kd*de;
    
    % 状态更新 & 输出
    x    = A*x + B*u(k);
    y(k) = C*x + D*u(k);
    
    ePrev = e;
end
end
