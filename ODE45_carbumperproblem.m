%% Differential Equation: Car Bumper Problem
% ODE 45

% Define parameters
M = 8;   % Mass - kg
D = 50;  % Damping coefficient - Ns/m
K = 100; % Spring constant - N/m

%Requires two first order equations

% x' = p/M (p is momentum, M is mass)

% p' = -D * p/M - Kx

% x is x(1) and p is x(2)

% Define the system of equations
odeSystem = @(t, x) [x(2) / M; (-D * x(2)) / M - K * x(1)];

% Set initial conditions
initialConditions = [0; 10*M]; 

% Time for simulation
tSpan = [0, 10]; 

% Solving ODE
[t, x] = ode45(odeSystem, tSpan, initialConditions);

% Plot the position, momentum over time
figure;
subplot(2, 1, 1);
plot(t, x(:, 1), 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs Time');
grid on;
subplot(2, 1, 2);
plot(t, x(:, 2), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Momentum (Ns)');
title('Momentum vs Time');
grid on;
% Plot legends
subplot(2, 1, 1);
legend({'Position'});
subplot(2, 1, 2);
legend({'Momentum'});