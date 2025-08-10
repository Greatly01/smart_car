clc;
clear;
% 定义时间范围和采样频率
t = 0:1/64:1 - 1/64;
Fs = 64;

% 定义模拟周期信号
xs = cos(8*pi*t)+cos(16*pi*t)+cos(20*pi*t);

% 绘制原始信号
figure(1);
plot(t,xs);
title('Original Signal x_s(t)');
xlabel('Time (t)');
ylabel('Amplitude');

% 进行FFT变换并绘制幅度特性曲线
N_values = [16, 32, 64];
for i = 1:length(N_values)
    N = N_values(i);
    Xs = fft(xs, N);
    k = 0:N - 1;
    figure(i + 1);
    subplot(2,1,1);
    stem(k,abs(Xs));
    title(['|Xs(k)|, N = ', num2str(N)]);
    xlabel('Frequency (k)');
    ylabel('Amplitude');
end