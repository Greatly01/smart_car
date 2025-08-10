%%
% 
% * ITEM1
% * ITEM2
% 
clc
clear all

% 开始计时
tic

[f, n1] = ob(20); % 随机生成障碍物
Xinit = [1, 1];   % 定义初始点
Xgoal = [95, 90]; % 定义目标点
plot(Xinit(1), Xinit(2), 'ro');
plot(Xgoal(1), Xgoal(2), 'ko');
T = [Xinit(1), Xinit(2)]; % 已生成节点集合用顺序表的数据结构存储
Xnew = Xinit;
D(1) = 0; % 初始节点的父节点指向0

while distance(Xnew(1), Xnew(2), Xgoal(1), Xgoal(2)) > 3  % 进入目标区域
    Xrand = round(rand(1, 2) * 100) + 1; % 状态采样空间：横纵坐标均为整数，范围1~100
    k = 1; % 进入循环
    while k == 1
        k = 0; % 初始化采样成功
        for i = 1:n1
            if distance(Xrand(1), Xrand(2), f(i*3-2), f(i*3-1)) < (f(i*3) + 1) % 判断随机采样点是否在障碍物内
                k = 1; % 采样不成功
                break;
            end
        end
        Xrand = round(rand(1, 2) * 100); % 重新采样
    end
    min = 100; % 该参数对路径方向有影响
    for i = 1:size(T, 2)/2 % 遍历已生成节点集合
        if distance(T(2*i-1), T(2*i), Xrand(1), Xrand(2)) < min % 得到最近的节点
            min = distance(T(2*i-1), T(2*i), Xrand(1), Xrand(2));
            Xnear = [T(2*i-1), T(2*i)];
        end
    end
    stepsize = 3 / distance(Xnear(1), Xnear(2), Xrand(1), Xrand(2)); % 以一定步长前进，这里选择3
    Xnew = [Xrand(1) * stepsize + Xnear(1) * (1-stepsize), Xrand(2) * stepsize + Xnear(2) * (1-stepsize)];
    t = 0; % 初始状态未碰撞
    if Xnear(1) > Xnew(1)
        caiyang = -0.001;
    else
        caiyang = 0.001;
    end
    for i = Xnear(1):caiyang:Xnew(1) % 均匀采样进行碰撞检测
        for j = 1:n1
            if distance(f(3*j-2), f(3*j-1), i, Xnear(2) + (i-Xnear(1)) / (Xnew(1)-Xnear(1)) * (Xnew(2)-Xnear(2))) <= (f(3*j) + 1)
                t = 1; % 代表碰撞
                break;
            end
        end
        if t == 1
            break;
        end
    end
    if t == 0
        T = [T, Xnew(1), Xnew(2)];
        for i = 1:size(T, 2)/2 % 遍历已生成节点集合
            if (T(i*2-1) == Xnear(1)) && (T(i*2) == Xnear(2)) % 得到最近的节点的索引
                D(size(T, 2)/2) = i; % 记录父节点
                break;
            end
        end
        plot([Xnew(1), Xnear(1)], [Xnew(2), Xnear(2)], 'b-'); hold on; pause(0.005);
        plot(Xnew(1), Xnew(2), 'r.'); xlim([0, 100]); ylim([0, 100]);
    end
end

i = size(T, 2)/2;
jg = [i];
while D(i)
    i = D(i); % 通过链表回溯
    if D(i) ~= 0
        jg = [D(i), jg]; % 存储最短路径通过的节点
    end
end

% 初始化路径长度
pathLength = 0;

Fx = T(jg(1)*2-1); Fy = T(jg(1)*2);
i = 2;
while jg(i) ~= size(T, 2)/2
    x = T(jg(i)*2-1);
    y = T(jg(i)*2);
    pathLength = pathLength + distance(x, y, Fx, Fy); % 计算路径长度
    plot([x, Fx], [y, Fy], 'g-'); hold on; pause(0.05);
    Fx = x; Fy = y;
    i = i + 1;
end
pathLength = pathLength + distance(T(jg(i)*2-1), T(jg(i)*2), Fx, Fy); % 最后一段路径长度
plot([T(jg(i)*2-1), Fx], [T(jg(i)*2), Fy], 'g-'); hold on; pause(0.05);

% 结束计时
timeElapsed = toc;

% 输出结果
fprintf('Path planning took %.2f seconds.\n', timeElapsed);
fprintf('The length of the path is %.2f units.\n', pathLength);