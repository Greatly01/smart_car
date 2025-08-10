clear all;
close all;
clc;

%% Create and plot Obstacle

areax = 100;
areay = 100;
% Obstacle Locations
O_loc = [[[20 25 25 20]; [98 97 84 84]]; ...
         [[20 25 24 21]; [75 73 63 63]]; ...
         [[21 25 26 20]; [55 55 42 43]]; ...
         [[20 25 28 20]; [35 25 5 6]];   ...
         [[41 48 48 40]; [99 93 85 86]]; ...
         [[40 48 47 40]; [77 77 58 57]]; ...
         [[41 46 48 40]; [50 44 20 20]]; ...
         [[60 68 78 59]; [83 82 71 71]]; ...
         [[61 68 68 60]; [55 55 45 45]]; ...
         [[62 68 70 61]; [33 23 11 11]]; ...
         [[80 85 85 80]; [63 62 55 54]]; ...
         [[78 84 85 79]; [32 33 25 28]]; ...
         ];
swap = size(O_loc);
O_num = swap(1)/2;  % Number of obstacles

% Create obstacles
for i=1:O_num
    n = i*2;
    pgon(i) = polyshape(O_loc(n-1,:),O_loc(n,:));
end
% Plot obstacle
plot(pgon,'FaceColor',[1 0 0],'edgecolor','none');
xlim([0,100]);
ylim([0,100]);
hold on
axis square;
grid on

%% Insert beginning and end point

% 定义起点和终点
beginP = [10, 10];  % 起点坐标
endP = [90, 90];    % 终点坐标

% Plot begin and end point
scatter(beginP(1), beginP(2),40, 'red', 'filled','s');
scatter(endP(1), endP(2),40, 'blue', 'filled','s');
min_dis = Path_length([beginP; endP]);
disp("Minimum distance is: " + min_dis);
disp(".");
disp("..");
disp("...");

%% Initialize population
pop_num = 30;
init_num = 25;
famous = 100*ones(pop_num/5,12);

y_best = (endP(2)-beginP(2))/9*(1:9) + beginP(2);

pop = zeros(init_num,9);
for i=1:9
    pop(:,i) = random('Normal',y_best(i), 8, init_num,1);
end
begin_mat = repmat(beginP(2),init_num,1);
end_mat = repmat(endP(2),init_num,1);
pop = [begin_mat pop end_mat];
x_val = [beginP(1) 10:10:90 endP(1)];
check_flag = 0;
tic;
for j=1:init_num
    k=2;
    while k <= 9
        i=0;
        while i < O_num
            i = i+1;
            while Collision_detect(pgon(i),[x_val(1:k+1); pop(j,1:k+1)])
                pop(j,k:k+1) = random('Normal',(y_best(k)+y_best(k))/2 , 8, 1, 2);
                check_flag = 1;
            end      
            if check_flag == 1
                i = 0;
                check_flag = 0;
            end
        end
        k = k+1;
    end
end
disp(toc + " seconds primary population generating took!!")

% 初始化剩余种群
% pop = [pop; zeros(pop_num-init_num,9)];
swap = CrossOver(pop, pop_num-init_num);
swap = Rm_wr_child(pgon, x_val, swap);
disp("Primary population cost values:");
costs = Cost_func(O_loc, pop, x_val);
pop = [pop; swap];
Draw_path(pgon, x_val, pop);

%% Main Loop (Cost Assignment, Selection, Cross Over and Mutation)
cnt = 0;
it_num = 200;
results = zeros(it_num, 4);
for i=1:it_num
    disp("Iteration " + i + "  ====================>");
    [costs,results(i,2), results(i,3), results(i,4)] = Cost_func(O_loc, pop, x_val);
    rank = Sort_Costs(costs);
    results(i,1) = costs(rank(1));
    disp("Minimum error " + costs(rank(1)));
    famous = Hall_of_fame(pop, costs, rank, famous);
    pop = Path_Selection(rank, pop, init_num);
    swap = CrossOver(pop, pop_num-init_num);
    swap = Mutation(swap,cnt);
    swap = Rm_wr_child(pgon, x_val, swap);
    pop = [pop; swap];
    cnt = cnt+1;
end

% 绘制最终路径
Draw_path(pgon, x_val, famous(:,2:end));

% 计算并输出最优路径长度
optimal_path_length = results(end, 2);
disp("Optimal path length: " + optimal_path_length);

% 输出规划时间
total_time = toc;
disp("Total planning time: " + total_time + " seconds");

% 创建一个新的图形窗口，并将其命名为 "Results"
figure('Name', "Results");

% 使用 subplot 创建 2x2 的子图布局
subplot(2, 2, 1); % 第一个子图
plot(1:it_num, results(:,1));
title('Min Costs');
xlabel('Iteration');
ylabel('Cost');

subplot(2, 2, 2); % 第二个子图
plot(1:it_num, results(:,2), 1:it_num, min_dis*ones(1,it_num));
title('Min Length');
xlabel('Iteration');
ylabel('Length');
legend('Min Length', 'Minimum Distance');

subplot(2, 2, 3); % 第三个子图
plot(1:it_num, results(:,3));
title('Min Angle');
xlabel('Iteration');
ylabel('Angle');

subplot(2, 2, 4); % 第四个子图
plot(1:it_num, results(:,4));
title('Min Distance');
xlabel('Iteration');
ylabel('Distance');