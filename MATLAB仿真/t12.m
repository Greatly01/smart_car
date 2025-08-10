num = 2;
den = conv([1 0], conv([1 2.8], [1 0.8]));
G = tf(num, den);
zeta = 0.307;
Pm = 2 * sin(zeta) * 180 / pi;
dPm = Pm + 5;
kc = 2;

% 调用子函数获取校正环节
Gc = cqjz_frequency(G, kc, dPm);
G = G * kc;
GGC = G * Gc;

% 闭环系统
Gy_close = feedback(G, 1);
Gx_close = feedback(GGC, 1);

% 输出校正后闭环传递函数
disp('滞后校正环节传递函数:');
pretty_tf(Gc);
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
bode(G, 'b');
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








function Gc = cqjz_frequency(G, kc, dPm)
G = tf(G);
[mag, pha, w] = bode(G*kc);
Mag = 20*log10(mag(:))';
% 利用插值找到相位对应的频率
wcg = interp1(pha(:), w(:), dPm - 180, 'spline');
magdb = 20*log10(mag);
Gr = -interp1(w, magdb(1, :), wcg, 'spline');
alpha = 10^(Gr/20);
T = 10/(alpha*wcg);
Gc = tf([alpha*T, 1], [T, 1]);
end









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