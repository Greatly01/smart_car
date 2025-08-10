clc;
clear;
% 定义总范围的离散序列n
n = -5:12;

% 定义x1(n)序列
x1 = zeros(1, length(n));
for i = 1:length(n)
    if n(i) >= 0 && n(i) <= 3
        x1(i) = 1;
    end
end

% 定义x2(n)序列
x2 = zeros(1, length(n));
for i = 1:length(n)
    if n(i) >= 0 && n(i) <= 3
        x2(i) = n(i) + 1;
    elseif n(i) >= 4 && n(i) <= 7
        x2(i) = 8 - n(i);
    end
end

% 定义x3(n)序列
x3 = zeros(1, length(n));
for i = 1:length(n)
    if n(i) >= 0 && n(i) <= 3
        x3(i) = 4 - n(i);
    elseif n(i) >= 4 && n(i) <= 7
        x3(i) = n(i) - 3;
    end
end

% 绘制原始序列图像
figure(1);
subplot(3, 1, 1);
stem(n, x1);
title('x1(n)');
xlabel('n');
ylabel('x1');

subplot(3, 1, 2);
stem(n, x2);
title('x2(n)');
xlabel('n');
ylabel('x2');

subplot(3, 1, 3);
stem(n, x3);
title('x3(n)');
xlabel('n');
ylabel('x3');

% 进行FFT变换并绘制幅度特性曲线（N = 8）
N1 = 8;
X1_8 = fft(x1, N1);
X2_8 = fft(x2, N1);
X3_8 = fft(x3, N1);

figure(2);
k1 = 0:N1 - 1;
subplot(3, 1, 1);
stem(k1, abs(X1_8));
title('|X1(k)|, N = 8');
xlabel('k');
ylabel('|X1(k)|');

subplot(3, 1, 2);
stem(k1, abs(X2_8));
title('|X2(k)|, N = 8');
xlabel('k');
ylabel('|X2(k)|');

subplot(3, 1, 3);
stem(k1, abs(X3_8));
title('|X3(k)|, N = 8');
xlabel('k');
ylabel('|X3(k)|');

% 进行FFT变换并绘制幅度特性曲线（N = 16）
N2 = 16;
X1_16 = fft(x1, N2);
X2_16 = fft(x2, N2);
X3_16 = fft(x3, N2);

figure(3);
k2 = 0:N2 - 1;
subplot(3, 1, 1);
stem(k2, abs(X1_16));
title('|X1(k)|, N = 16');
xlabel('k');
ylabel('|X1(k)|');

subplot(3, 1, 2);
stem(k2, abs(X2_16));
title('|X2(k)|, N = 16');
xlabel('k');
ylabel('|X2(k)|');

subplot(3, 1, 3);
stem(k2, abs(X3_16));
title('|X3(k)|, N = 16');
xlabel('k');
ylabel('|X3(k)|');













% % (1) 对以下序列进行谱分析
% n = 0:7;
% % 序列x1
% x1 = zeros(1, length(n));
% for k = 0:3
%     x1(k + 1) = k + 1;
% end
% subplot(3, 1, 1);
% stem(n, x1);
% title('x1');
% xlabel('n');
% ylabel('Amplitude');
% 
% % 序列x2
% x2 = zeros(1, length(n));
% for k = 4:7
%     x2(k + 1) = 8 - (k - 3);
% end
% subplot(3, 1, 2);
% stem(n, x2);
% title('x2');
% xlabel('n');
% ylabel('Amplitude');
% 
% % 序列x3
% x3 = zeros(1, length(n));
% for k = 0:3
%     x3(k + 1) = 4 - k;
% end
% for k = 4:7
%     x3(k + 1) = k - 3;
% end
% subplot(3, 1, 3);
% stem(n, x3);
% title('x3');
% xlabel('n');
% ylabel('Amplitude');
% 
% 
% % (2) 对以下周期序列进行谱分析
% n2 = 0:7;
% % 序列x4
% x4 = cos(pi/4 * n2);
% subplot(3, 1, 1);
% stem(n2, x4);
% title('x4');
% xlabel('n');
% ylabel('Amplitude');
% 
% % 序列x5
% x5 = cos(pi/4 * n2)+ cos(pi/8 * n2);
% subplot(3, 1, 2);
% stem(n2, x5);
% title('x5');
% xlabel('n');
% ylabel('Amplitude');
% 
% 
% % (3) 对模拟周期信号进行谱分析
% Fs = 64;
% t = 0:1/Fs:1 - 1/Fs;
% % 序列x6
% x6 = cos(8 * pi * t)+ cos(16 * pi * t)+ cos(20 * pi * t);
% subplot(3, 1, 3);
% stem(t, x6);
% title('x6');
% xlabel('t');
% ylabel('Amplitude');
% 
% 
% % 以下是按照题目要求进行FFT和绘图
% % (1) 对以下序列进行谱分析（FFT部分）
% N1 = 8;
% N2 = 16;
% X1_8 = fft(x1, N1);
% X2_8 = fft(x2, N1);
% X3_8 = fft(x3, N1);
% X1_16 = fft(x1, N2);
% X2_16 = fft(x2, N2);
% X3_16 = fft(x3, N2);
% 
% % 计算幅频特性
% magX1_8 = abs(X1_8);
% magX2_8 = abs(X2_8);
% magX3_8 = abs(X3_8);
% magX1_16 = abs(X1_16);
% magX2_16 = abs(X2_16);
% magX3_16 = abs(X3_16);
% 
% % 绘制幅频特性曲线
% k1 = 0:N1 - 1;
% w1 = 2 * pi / N1 * k1;
% subplot(3, 2, 1);
% stem(w1, magX1_8);
% title('x1, N = 8');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 2);
% stem(w1, magX2_8);
% title('x2, N = 8');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 3);
% stem(w1, magX3_8);
% title('x3, N = 8');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% k2 = 0:N2 - 1;
% w2 = 2 * pi / N2 * k2;
% subplot(3, 2, 4);
% stem(w2, magX1_16);
% title('x1, N = 16');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 5);
% stem(w2, magX2_16);
% title('x2, N = 16');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 6);
% stem(w2, magX3_16);
% title('x3, N = 16');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% 
% % (2) 对以下周期序列进行谱分析（FFT部分）
% X4_8 = fft(x4, N1);
% X5_8 = fft(x5, N1);
% X4_16 = fft(x4, N2);
% X5_16 = fft(x5, N2);
% 
% magX4_8 = abs(X4_8);
% magX5_8 = abs(X5_8);
% magX4_16 = abs(X4_16);
% magX5_16 = abs(X5_16);
% 
% % 重新计算频率向量
% k3 = 0:N1 - 1;
% w3 = 2 * pi / N1 * k3;
% k4 = 0:N2 - 1;
% w4 = 2 * pi / N2 * k4;
% 
% subplot(3, 2, 1);
% stem(w3, magX4_8);
% title('x4, N = 8');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 2);
% stem(w3, magX5_8);
% title('x5, N = 8');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 3);
% stem(w4, magX4_16);
% title('x4, N = 16');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 2, 4);
% stem(w4, magX5_16);
% title('x5, N = 16');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% 
% % (3) 对模拟周期信号进行谱分析（FFT部分）
% N3 = 16;
% N4 = 32;
% N5 = 64;
% X6_16 = fft(x6, N3);
% X6_32 = fft(x6, N4);
% X6_64 = fft(x6, N5);
% 
% magX6_16 = abs(X6_16);
% magX6_32 = abs(X6_32);
% magX6_64 = abs(X6_64);
% 
% % 重新计算频率向量
% k5 = 0:N3 - 1;
% w5 = 2 * pi / N3 * k5;
% k6 = 0:N4 - 1;
% w6 = 2 * pi / N4 * k6;
% k7 = 0:N5 - 1;
% w7 = 2 * pi / N5 * k7;
% 
% subplot(3, 1, 1);
% stem(w5, magX6_16);
% title('x6, N = 16');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 1, 2);
% stem(w6, magX6_32);
% title('x6, N = 32');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');
% subplot(3, 1, 3);
% stem(w7, magX6_64);
% title('x6, N = 64');
% xlabel('Frequency (rad)');
% ylabel('Magnitude');