% Define the transfer functions
s = tf('s');

% Plant transfer function with time delay (using Pade approximation for e^(-3s))
Gp = exp(-3*s) / (10*s + 1);
Gp = pade(Gp, 1);  % First-order Pade approximation for the delay

% Fuel quantity transfer function
Gw = 3 / (2*s + 1);

% Production load disturbance transfer function
Gf = 1.5 / (5*s + 1);

% Open-loop system (without disturbance)
system = Gp * Gw;

% PID controller design
Kp = 0.98463;   % Proportional gain
Ki = 0.081322; % Integral gain
Kd = 2.3056; % Derivative gain
PID_controller = pid(Kp, Ki, Kd);

% Closed-loop system with PID
closed_loop_system = feedback(PID_controller * system, 1);

% Simulation
t = 0:0.1:100;  % Simulation time
[y, t] = step(closed_loop_system, t);

% Plot the response
figure;
plot(t, y);
title('Closed-Loop Step Response with PID Control');
xlabel('Time (s)');
ylabel('Steam Temperature');
grid on;

% Tuning the PID parameters using the PID Tuner tool (optional)
pidTuner(system, 'PID');
