clc;
clear;
% 定义序列长度范围
n = 0:15;

% 定义序列x4(n)
x4 = cos(pi/4 * n);

% 定义序列x5(n)
x5 = cos(pi/4 * n)+cos(pi/8 * n);

% 绘制原始序列x4(n)和x5(n)
subplot(2,1,1);
stem(n, x4);
title('x4(n)');
xlabel('n');
ylabel('x4');

subplot(2,1,2);
stem(n, x5);
title('x5(n)');
xlabel('n');
ylabel('x5');

% 进行FFT变换并绘制幅度特性曲线（N = 8）
N1 = 8;
X4_8 = fft(x4, N1);
X5_8 = fft(x5, N1);

figure;
k1 = 0:N1 - 1;
subplot(2,1,1);
stem(k1, abs(X4_8));
title('|X4(k)|, N = 8');
xlabel('k');
ylabel('|X4(k)|');

subplot(2,1,2);
stem(k1, abs(X5_8));
title('|X5(k)|, N = 8');
xlabel('k');
ylabel('|X5(k)|');

% 进行FFT变换并绘制幅度特性曲线（N = 16）
N2 = 16;
X4_16 = fft(x4, N2);
X5_16 = fft(x5, N2);

figure;
k2 = 0:N2 - 1;
subplot(2,1,1);
stem(k2, abs(X4_16));
title('|X4(k)|, N = 16');
xlabel('k');
ylabel('|X4(k)|');

subplot(2,1,2);
stem(k2, abs(X5_16));
title('|X5(k)|, N = 16');
xlabel('k');
ylabel('|X5(k)|');