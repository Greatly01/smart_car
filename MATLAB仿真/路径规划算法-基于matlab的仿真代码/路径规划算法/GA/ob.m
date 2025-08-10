function [f, n1] = ob(n)
    f = []; % 储存障碍物信息
    n1 = n; % 返回障碍物个数
    p = 0;  % 目前生成的障碍物个数

    % 循环生成指定数量的障碍物
    for i = 1:n
        k = 1;
        while(k)
            D = [rand(1,2)*60 + 15, rand(1,1)*1 + 3]; % 随机生成障碍物的坐标与半径，自行调整
            
            % 防止障碍物过于靠近目标点
            if distance(D(1), D(2), 90, 90) > (D(3) + 5)
                k = 0;
                
                % 检查障碍物之间是否过近
                for t = 1:p
                    if distance(D(1), D(2), f(3*t-2), f(3*t-1)) <= (D(3) + f(3*t) + 5)
                        k = 1;
                        break;
                    end
                end
            end
        end
        
        % 绘制障碍物
        alpha = 0:pi/40:2*pi;
        r = D(3);
        x = D(1) + r * cos(alpha);
        y = D(2) + r * sin(alpha);
        fill(x, y, 'k');
        axis equal;
        hold on;
        xlim([0, 100]);
        ylim([0, 100]);
        
        f = [f, D];
        p = p + 1;
    end
    
    hold all;
end

% 计算两点之间的欧几里得距离
function d = distance(x1, y1, x2, y2)
    d = sqrt((x1 - x2)^2 + (y1 - y2)^2);
end