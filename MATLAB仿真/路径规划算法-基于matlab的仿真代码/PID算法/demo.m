% Water tank PID control script
clc;
clear;

% System parameters (assume a first-order system for the water tank)
K = 1;  % System gain
tau = 10;  % Time constant of the system
G = tf(K, [tau 1]);  % Transfer function of the system

% Desired setpoint (target water level)
target_level = 55;  % meters

% PID controller parameters (initial guesses)
Kp = 2;  % Proportional gain
Ki = 0.5;   % Integral gain
Kd = 0.1; % Derivative gain

% Create PID controller
C = pid(Kp, Ki, Kd);

% Open-loop system (plant)
figure;
step(G);
title('Open-loop System Response (Water Tank)');

% Closed-loop system with PID controller
T = feedback(C*G, 1);

% Simulate and plot closed-loop step response
figure;
step(T);
title('Closed-loop System Response with PID Control');
grid on;

% Tuning the PID controller using automatic tuning function
[C_tuned, info] = pidtune(G, 'PID');

% Display the tuned PID parameters
Kp_tuned = C_tuned.Kp;
Ki_tuned = C_tuned.Ki;
Kd_tuned = C_tuned.Kd;

fprintf('Tuned PID parameters:\n');
fprintf('Kp = %.4f\n', Kp_tuned);
fprintf('Ki = %.4f\n', Ki_tuned);
fprintf('Kd = %.4f\n', Kd_tuned);

% Apply the tuned PID controller
T_tuned = feedback(C_tuned * G, 1);

% Simulate and plot the tuned closed-loop step response
figure;
step(T_tuned);
title('Tuned Closed-loop System Response with PID Control');
grid on;

% Compare the responses of the original and tuned PID controllers
figure;
step(T, 'b', T_tuned, 'r');
legend('Original PID', 'Tuned PID');
title('Comparison of Original and Tuned PID Controllers');
grid on;

% Display system performance metrics
S = stepinfo(T_tuned);
fprintf('\nSystem Performance Metrics for Tuned PID Controller:\n');
fprintf('Rise Time: %.4f seconds\n', S.RiseTime);
fprintf('Settling Time: %.4f seconds\n', S.SettlingTime);
fprintf('Overshoot: %.2f%%\n', S.Overshoot);
fprintf('Steady-state Error: %.4f\n', abs(target_level - S.SettlingMin));

