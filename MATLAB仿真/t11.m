

% 主程序
num = 2;
den = conv([1 0], conv([0.3 1], [0.1 1]));
G = tf(num, den);
kc = 3;
yPm = 45 + 12;

Gc = cqjz_frequency(G, kc, yPm); 
GGC = G*kc * Gc; 

% 闭环系统
Gy_close = feedback(G*kc, 1);
Gx_close = feedback(GGC, 1);

% 输出校正后闭环传递函数
disp('校正后系统闭环传递函数:');
pretty_tf(Gx_close);


% 绘制阶跃响应
figure(1);
step(Gx_close, 'b');
hold on;
step(Gy_close, 'r');
grid on;
legend('校正后', '校正前', 'Location', 'southeast');

% 绘制Bode图
figure(2);
bode(G*kc, 'b');
hold on;
bode(GGC, 'r');
grid on;
legend('校正前', '校正后');

% 绘制Nyquist图
figure(3);
nyquist(Gx_close, 'b');
hold on;
nyquist(Gy_close, 'r');
grid on;
legend('校正后', '校正前');



% 子函数：频率响应法设计串联超前校正
function Gc = cqjz_frequency(G, kc, yPm)
    G = tf(G);
    [mag, pha, w] = bode(G*kc);
    Mag = 20*log10(mag(:))';
    [Gm, Pm, Wcg, Wcp] = margin(G*kc);
    phi = (yPm - Pm) * pi/180;
    alpha = (1 + sin(phi)) / (1 - sin(phi));
    Mm = -10*log10(alpha);
    Mcgm = interp1(Mag, w, Mm, 'spline');
    T = 1 / (Mcgm * sqrt(alpha));
    Tz = alpha * T;
    Gc = tf([Tz 1], [T 1]);
    
    % 调用自定义格式化函数输出传递函数
    disp('超前校正环节传递函数:');
    pretty_tf(Gc);
end



% 自定义传递函数格式化输出函数
function pretty_tf(tf_obj)
    [num, den] = tfdata(tf_obj, 'v');
    
    % 处理分子多项式
    num_str = poly2str(num, 's');
    num_str = strrep(num_str, 's^0', '');
    num_str = strrep(num_str, ' *', '');
    
    % 处理分母多项式
    den_str = poly2str(den, 's');
    den_str = strrep(den_str, 's^0', '');
    den_str = strrep(den_str, ' *', '');
    
    % 计算分隔线长度
    max_length = max(strlength(num_str), strlength(den_str));
    separator = repmat('-', 1, max_length);
    
    % 输出格式化结果
    disp(['   ' num_str]);
    disp(['   ' separator]);
    disp(['   ' den_str]);
    disp(' ');
end