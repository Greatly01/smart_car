% 参数初始化
A = 444.128; % 信号的幅度,决定信号的最大值
a = 50 * sqrt(2) * pi; % 指数衰减系数,控制信号的衰减速度
w = 50 * sqrt(2) * pi; % 信号的角频率,控制信号的振荡速度
n = 0:63; % 时间点的离散序列,表示信号在0到63个采样点上被采样
fs = 1000; % 采样频率,表示每秒钟采样 1000 次,用于将连续信号离散化

% 生成信号
c = A * exp((-a) * n / fs) .* sin(w * n / fs); % 生成一个指数衰减的正弦信号

% 创建图形窗口
figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); % 全屏显示图形窗口

% 时域信号的绘制
subplot(2, 1, 1); % 调整为 2 行 1 列的子图布局，选择第一个子图
stem(n, c, 'r', 'MarkerSize', 6); % 使用红色，增大采样点大小
grid on; % 添加网格线
xlabel('n', 'FontSize', 12, 'FontWeight', 'bold'); % 设置横轴标签，字体加粗
ylabel('xa(n)', 'FontSize', 12, 'FontWeight', 'bold'); % 设置纵轴标签
title('时域信号 xa(n)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'b'); % 设置标题，字体加粗，蓝色
xlim([0 63]); % 限制横轴范围为 0 到 63
set(gca, 'FontSize', 10, 'Box', 'on', 'LineWidth', 1.5); % 设置坐标轴的样式

% 频域信号的计算与绘制
N = 64; % FFT 的点数
k = 0:N-1; % 创建索引序列
w = k * pi / (N / 2); % 将索引 k 转化为归一化的角频率
X = fft(c, N); % 对信号 c 进行快速傅里叶变换

subplot(2, 1, 2); % 选择第二个子图
plot(w / pi, abs(X), 'b-', 'LineWidth', 1.5); % 使用蓝色实线，增大线条宽度
grid on; % 添加网格线
xlabel('\omega / \pi', 'FontSize', 12, 'FontWeight', 'bold'); % 设置横轴标签
ylabel('|X(j\omega)|', 'FontSize', 12, 'FontWeight', 'bold'); % 设置纵轴标签
title('频域信号 |X(j\omega)|', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r'); % 设置标题，字体加粗，红色
xlim([0 2]); % 限制横轴范围为 0 到 2
set(gca, 'FontSize', 10, 'Box', 'on', 'LineWidth', 1.5); % 设置坐标轴的样式
