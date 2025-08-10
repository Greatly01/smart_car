% %% 高精度导弹仿真系统（修复版）
% function missile_simulation_complete()
%     % 主函数初始化
%     close all; clearvars; clc;
%     
%     % 创建图形界面
%     fig = figure('Color','w','Position',[50 50 1400 800],...
%                  'KeyPressFcn',@keyboardControl);
%     initializeDisplay(fig);  % 初始化显示组件
%     
%     % 初始化仿真参数
%     [missile, propulsion, aero, control] = initSystemParameters();
%     state = initializeState();
%     
%     % 创建定时器（增加数据持久化存储）
%     timerObj = timer('ExecutionMode','fixedRate', 'Period',0.05,...
%                     'TimerFcn',@(~,~)simulationStep(state, missile, propulsion, aero, control, fig),...
%                     'StopFcn',@(~,~)disp('仿真结束'));
%     start(timerObj);
% 
%     %% 定时器回调函数
%     function simulationStep(state, missile, propulsion, aero, control, fig)
%         persistent trajectoryPlot speedPlot altPlot attPlot loadPlot missileObj
%         persistent t_history pos_history vel_history
%         try
%             % 首次运行初始化
%             if isempty(trajectoryPlot)
%                 [trajectoryPlot, speedPlot, altPlot, attPlot, loadPlot, missileObj] = initVisualization(fig);
%                 t_history = 0;
%                 pos_history = state.pos;
%                 vel_history = state.vel;
%             end
%             
%             % 动力学计算
%             [dstate, aero] = missileDynamics(state, missile, propulsion, aero, control);
%             
%             % 状态更新（改进的四元数积分）
%             dt = 0.05;
%             state.pos = state.pos + state.vel * dt;
%             state.vel = state.vel + dstate.vel * dt;
%             
%             % 四元数积分（正确方法）
%             omega = state.omega;
%             omega_norm = norm(omega);
%             if omega_norm > eps
%                 delta_quat = [cos(omega_norm*dt/2), sin(omega_norm*dt/2)*omega/omega_norm];
%                 state.quat = quatmultiply(state.quat, delta_quat);
%                 state.quat = state.quat/norm(state.quat);
%             end
%             
%             state.omega = state.omega + dstate.omega * dt;
%             state.mass = state.mass + dstate.mass * dt;
%             state.t = state.t + dt;
%             
%             % 坐标转换
%             [lla, enu] = ecef2lla(state.pos);
%             att = quat2eul(state.quat);
%             
%             % 更新可视化
%             updateVisualization(fig, missileObj, state, lla, enu, att, trajectoryPlot, speedPlot, altPlot, attPlot, loadPlot);
%             
%             % 终止条件
%             if lla(3) < 0 || state.t > 300
%                 stop(timerObj);
%                 delete(timerObj);
%             end
%         catch ME
%             disp(['仿真错误: ' ME.message]);
%             stop(timerObj);
%             delete(timerObj);
%         end
%     end
% end
% 
% %% 新增关键函数实现
% function thrust_dir = getThrustDirection(state)
%     % 从四元数获取推力方向（前向向量）
%     q = state.quat;
%     R = quat2rotm(q);
%     thrust_dir = R(:,1)'; % 取X轴方向
% end
% 
% %% 修改后的初始化函数
% function state = initializeState()
%     state = struct(...
%         'pos', [0, 0, 6378137+100],...  % 修正初始位置（Z轴向上）
%         'vel', [0, 0, 100],...           % 初始垂直速度
%         'quat', [1,0,0,0],...             % 初始姿态（朝上）
%         'omega', [0,0,0],...
%         'mass', 1500,...
%         't', 0);
% end
% 
% %% 显示初始化
% function initializeDisplay(fig)
%     % 创建控制面板
%     uicontrol(fig, 'Style','slider', 'Position',[20 20 120 20],...
%              'Min',0,'Max',30000,'Value',25000,'Tag','thrust');
%     uicontrol(fig, 'Style','text', 'Position',[20 45 120 15],...
%              'String','推力控制 (N)');
%     
%     % 创建绘图区域
%     subplot(2,3,[1 4], 'Parent',fig); 
%     ax1 = gca;
%     createEarthModel(ax1);
%     axis equal; grid on; hold on;
%     view(45,30);
%     
%     subplot(2,3,2, 'Parent',fig); title('速度曲线'); grid on; xlabel('时间(s)'); ylabel('速度(m/s)');
%     subplot(2,3,5, 'Parent',fig); title('高度曲线'); grid on; xlabel('时间(s)'); ylabel('高度(m)');
%     subplot(2,3,3, 'Parent',fig); title('姿态角'); grid on; xlabel('时间(s)'); ylabel('角度(deg)'); legend({'俯仰','偏航','滚转'});
%     subplot(2,3,6, 'Parent',fig); title('过载曲线'); grid on; xlabel('时间(s)'); ylabel('过载(g)');
% end
% 
% %% 地球模型创建
% function createEarthModel(ax)
%     % 生成参数化地球
%     [x,y,z] = sphere(50);
%     cdata = createEarthTexture(x,y,z);
%     
%     % 绘制带纹理的地球
%     surf(ax, x*6378137, y*6378137, z*6378137,...
%         'FaceColor','texturemap',...
%         'EdgeColor','none',...
%         'CData',cdata);
%     
%     % 添加经纬线
%     hold(ax, 'on');
%     [X,Y,Z] = sphere(20);
%     plot3(ax, X(:,11:end)*6378137, Y(:,11:end)*6378137, Z(:,11:end)*6378137, 'Color',[0.5 0.5 0.5], 'LineWidth',0.5);
% end
% 
% function cdata = createEarthTexture(x,y,z)
%     % 生成程序纹理
%     [~,el,~] = cart2sph(x,y,z);
%     land_mask = (el > 0) & (rand(size(x)) > 0.7); % 随机陆地分布
%     cdata = zeros([size(x),3]);
%     cdata(:,:,1) = 0.1 + 0.6*land_mask;   % 红色通道（陆地）
%     cdata(:,:,2) = 0.3 + 0.4*land_mask;    % 绿色通道
%     cdata(:,:,3) = 0.7 - 0.4*land_mask;    % 蓝色通道（海洋）
% end
% 
% %% 导弹参数初始化
% function [missile, propulsion, aero, control] = initSystemParameters()
%     missile = struct(...
%         'mass', 1500,...
%         'I', diag([2000, 1800, 2200]),...
%         'A_ref', 0.785,...
%         'length', 8.2);
%     
%     propulsion = struct(...
%         'thrust', 25000,...
%         'Isp', 280,...
%         'burn_time', 30);
%     
%     aero = struct(...
%         'Mach', [0.5, 1.0, 1.2, 2.0, 3.0],...
%         'alpha', deg2rad(-15:5:25),...
%         'CL', @(M,a) 0.1 + 0.25*a + 0.02*M.^2,...
%         'CD', @(M,a) 0.15 + 0.1*a.^2 + 0.05*abs(M-1));
%     
%     control = struct(...
%         'Kp', 0.8,...
%         'Ki', 0.1,...
%         'Kd', 0.3,...
%         'target', [100e3, 50e3, 0]);
% end
% 
% %% 六自由度动力学模型
% function [dstate, aero] = missileDynamics(state, missile, propulsion, aero, control)
%     % 环境参数
%     [~, ~, ~, atm] = atmosisa(state.pos(3));
%     g = [0, 0, -9.81]; % 简化重力模型
%     
%     % 气动力计算
%     V_rel = state.vel;
%     V_mag = norm(V_rel);
%     alpha = atan2(V_rel(3), sqrt(V_rel(1)^2 + V_rel(2)^2));
%     Mach = V_mag / atm.SpeedOfSound;
%     
%     CL = aero.CL(Mach, alpha);
%     CD = aero.CD(Mach, alpha);
%     q = 0.5*atm.Density*V_mag^2*missile.A_ref;
%     F_lift = q*CL * [sin(alpha), 0, cos(alpha)];
%     F_drag = q*CD * (-V_rel/V_mag);
%     
%     % 推进系统
%     if state.t < propulsion.burn_time
%         F_thrust = getThrustDirection(state) * propulsion.thrust;
%         m_dot = propulsion.thrust / (propulsion.Isp * 9.80665);
%     else
%         F_thrust = [0,0,0];
%         m_dot = 0;
%     end
%     
%     % 合力计算
%     F_total = F_thrust + F_lift + F_drag + missile.mass*g;
%     
%     % 力矩计算（简化模型）
%     r_cp = missile.length*0.4; % 压心位置
%     T_aero = cross(r_cp*[1,0,0], F_lift + F_drag);
%     
%     % 状态微分
%     dstate.vel = F_total / missile.mass;
%     dstate.omega = missile.I \ T_aero;
%     dstate.mass = -m_dot;
% end
% 
% %% 可视化初始化
% function [trajPlot, speedPlot, altPlot, attPlot, loadPlot, missileObj] = initVisualization(fig)
%     ax1 = subplot(2,3,[1 4]);
%     trajPlot = animatedline(ax1, 'Color','b', 'LineWidth',1);
%     missileObj = createMissile3D(ax1);
%     
%     ax2 = subplot(2,3,2); speedPlot = animatedline(ax2, 'Color','r');
%     ax3 = subplot(2,3,5); altPlot = animatedline(ax3, 'Color','m');
%     ax4 = subplot(2,3,3); attPlot = animatedline(ax4, 'Color','g');
%     ax5 = subplot(2,3,6); loadPlot = animatedline(ax5, 'Color','b');
% end
% 
% function missileObj = createMissile3D(ax)
%     % 创建参数化导弹模型
%     [X,Y,Z] = cylinder([0.2 0.5 0.3 0]);
%     missileObj = surf(ax, X, Y, Z*10,...
%         'FaceColor', [0.8 0.8 0.8],...
%         'EdgeColor', 'none',...
%         'FaceLighting', 'gouraud');
%     light(ax, 'Style','infinite');
% end
% 
% %% 辅助函数
% function [lla, enu] = ecef2lla(ecef)
%     % 简化坐标系转换
%     x = ecef(1); y = ecef(2); z = ecef(3);
%     lat = atan2(z, norm([x y]));
%     lon = atan2(y, x);
%     h = norm(ecef) - 6378137;
%     lla = [rad2deg(lat), rad2deg(lon), max(h,0)];
%     enu = [x, y, z]; % 简化的ENU转换
% end
% 
% %% 改进的可视化更新
% function updateVisualization(fig, missileObj, state, lla, enu, att, trajPlot, speedPlot, altPlot, attPlot, loadPlot)
%     % 更新导弹模型位置和姿态
%     scale = 1e4; % 模型缩放系数
%     R = quat2rotm(state.quat);
%     verts = get(missileObj, 'Vertices');
%     
%     % 应用缩放、旋转和平移
%     verts = verts * scale;                % 缩放模型
%     verts = (R * verts')';                % 应用旋转
%     verts = verts + enu;                  % 应用平移
%     
%     set(missileObj, 'Vertices', verts,...
%         'XData',verts(:,1), 'YData',verts(:,2), 'ZData',verts(:,3));
%     
%     % 更新轨迹（限制最大点数提升性能）
%     if numel(trajPlot.Points) < 5000
%         addpoints(trajPlot, enu(1), enu(2), lla(3));
%     else
%         clearpoints(trajPlot);
%         addpoints(trajPlot, enu(1), enu(2), lla(3));
%     end
%     
%     % 更新曲线数据
%     addpoints(speedPlot, state.t, norm(state.vel));
%     addpoints(altPlot, state.t, lla(3));
%     addpoints(attPlot, state.t, rad2deg(att));
%     addpoints(loadPlot, state.t, norm(state.vel)/9.81);
%     
%     % 强制刷新显示
%     drawnow limitrate nocallbacks;
% end
% 
% function keyboardControl(~, event)
%     % 键盘交互控制
%     global control
%     if ~isempty(control)
%         switch event.Key
%             case 'uparrow'
%                 control.thrust = min(control.thrust + 1000, 30000);
%             case 'downarrow'
%                 control.thrust = max(control.thrust - 1000, 0);
%         end
%     end
% end
% 
% function M = quat2mat(q)
%     % 四元数转旋转矩阵
%     w = q(1); x = q(2); y = q(3); z = q(4);
%     M = [1-2*y^2-2*z^2, 2*x*y-2*z*w, 2*x*z+2*y*w;
%          2*x*y+2*z*w, 1-2*x^2-2*z^2, 2*y*z-2*x*w;
%          2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x^2-2*y^2];
% end












% % 导弹轨迹仿真与分析
% 假设：三维空间运动、考虑重力、空气阻力、固定推力、程序俯仰角控制

%% 参数设置
clear; clc;

% 导弹参数
m0 = 3000;          % 初始质量 (kg)
thrust = 25000;     % 发动机推力 (N)
burn_time = 300;     % 发动机工作时间 (s)
dm = 18;            % 质量流量 (kg/s)
Cd = 0.15;          % 阻力系数
A_ref = 0.785;      % 参考面积 (m²)
g = 9.81;           % 重力加速度 (m/s²)
rho = 1.225;        % 空气密度 (kg/m³)

% 初始条件
x0 = 0;             % 初始x位置 (m)
y0 = 0;             % 初始y位置 (m)
z0 = 0;             % 初始z位置 (m)
v0 = 100;           % 初始速度 (m/s)
theta0 = 88;        % 初始俯仰角 (度)
psi0 = 45;          % 初始方位角 (度)

% 仿真参数
t_sim = 3;         % 总仿真时间 (s)
dt = 0.1;           % 时间步长 (s)

%% 初始化变量
t = 0:dt:t_sim;
N = length(t);
pos = zeros(N,3);   % 位置 [x,y,z]
vel = zeros(N,3);   % 速度 [vx,vy,vz]
acc = zeros(N,3);   % 加速度 [ax,ay,az]
mass = zeros(N,1);

% 初始状态
theta = deg2rad(theta0);
psi = deg2rad(psi0);
vel(1,:) = v0*[cos(theta)*cos(psi), cos(theta)*sin(psi), sin(theta)];
pos(1,:) = [x0, y0, z0];
mass(1) = m0;

%% 运动方程数值解算（欧拉法）
for i = 1:N-1
    % 当前状态
    v = norm(vel(i,:));
    m = mass(i);
    
    % 空气阻力计算
    F_drag = 0.5*rho*v^2*Cd*A_ref;
    drag_acc = -F_drag/m * (vel(i,:)/v);
    
    % 重力加速度
    g_acc = [0, 0, -g];
    
    % 推力计算（仅在工作时间内）
    if t(i) <= burn_time
        F_thrust = thrust;
        mass(i+1) = m - dm*dt;
        % 程序俯仰角控制（随时间变化示例）
        theta = theta0 - 0.5*t(i); 
        theta = deg2rad(max(theta, 5));  % 保持最小5度
    else
        F_thrust = 0;
        mass(i+1) = m;
        theta = deg2rad(5);  % 发动机关闭后保持固定角
    end
    
    % 推力加速度分量
    thrust_dir = [cos(theta)*cos(psi), cos(theta)*sin(psi), sin(theta)];
    thrust_acc = F_thrust/m * thrust_dir;
    
    % 总加速度
    acc(i,:) = thrust_acc + drag_acc + g_acc;
    
    % 数值积分
    vel(i+1,:) = vel(i,:) + acc(i,:)*dt;
    pos(i+1,:) = pos(i,:) + vel(i,:)*dt;
    
    % 碰撞检测（地面）
    if pos(i+1,3) < 0
        pos(i+1,3) = 0;
        vel(i+1,:) = [0,0,0];
        break
    end
end

%% 数据处理
% 截断数组到有效数据
valid_idx = find(pos(:,3)>=0, 1, 'last');
t = t(1:valid_idx);
pos = pos(1:valid_idx,:);
vel = vel(1:valid_idx,:);
acc = acc(1:valid_idx,:);

% 计算关键参数
apogee = max(pos(:,3));              % 最大高度
range = norm(pos(end,1:2));          % 水平射程
max_speed = max(sqrt(sum(vel.^2,2)));% 最大速度
impact_speed = norm(vel(end,:));     % 末端速度
% fprintf('%0.2f',apogee)

%% 可视化
figure('Color','w','Position',[100 100 1200 800])

% 3D轨迹
subplot(2,3,[1 4])
plot3(pos(:,1), pos(:,2), pos(:,3), 'b', 'LineWidth',2)
hold on
plot3(pos(1,1), pos(1,2), pos(1,3), 'go', 'MarkerSize',10, 'MarkerFaceColor','g')
plot3(pos(end,1), pos(end,2), pos(end,3), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r')
xlabel('X (m)'), ylabel('Y (m)'), zlabel('Altitude (m)')
title('3D 运动轨迹')
grid on, axis equal
view(30,45)

% 速度曲线
subplot(2,3,2)
speed = sqrt(sum(vel.^2,2));
plot(t, speed, 'r', 'LineWidth',2)
xlabel('Time (s)'), ylabel('Speed (m/s)')
title('速度时间曲线')
grid on

% 高度曲线
subplot(2,3,5)
plot(t, pos(:,3), 'm', 'LineWidth',2)
xlabel('Time (s)'), ylabel('Altitude (m)')
title('高度变化曲线')
grid on

% 弹道俯视图
subplot(2,3,3)
plot(pos(:,1), pos(:,2), 'b', 'LineWidth',2)
hold on
plot(pos(1,1), pos(1,2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g')
plot(pos(end,1), pos(end,2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r')
xlabel('X (m)'), ylabel('Y (m)')
title('弹道俯视图')
grid on, axis equal

% 加速度分量
subplot(2,3,6)
plot(t, acc(:,1), 'r', t, acc(:,2), 'g', t, acc(:,3), 'b', 'LineWidth',1.5)
xlabel('Time (s)'), ylabel('Acceleration (m/s²)')
title('加速度分量')
legend('X','Y','Z'), grid on

%% 结果显示
fprintf('关键性能参数：\n')
fprintf('最大高度：%.2f m\n', apogee)
fprintf('水平射程：%.2f m\n', range)
fprintf('最大速度：%.2f m/s\n', max_speed)
fprintf('末端速度：%.2f m/s\n', impact_speed)
fprintf('飞行时间：%.2f s\n', t(end))