%% Differential Equation: Car Bumper Problem
%Dsolve

% Defining Parameters
M = 8;   % Mass - kg
D = 50;  % Damping coefficient - Ns/m
K = 100; % Spring constant - N/m

syms x(t)

eqn1 = M*diff(x,t,2) + D*diff(x,t) + K*x == 0;
%x'' is written as diff(x,t,2)

% x' is diff(x,t)
Dx = diff(x,t);

% Initial conditions
initialCon = [x(0)==0, Dx(0)==10];

%solving for X(t)
solutionX = dsolve(eqn1, initialCon)

%Solving for V(t) - Velocity as function of time
solutionV = diff(solutionX);

%Solving for A(t) - Acceleration as function of time
solutionA = diff(solutionV);

% Plot solutions with fplot
figure('Color','w','Position',[200 200 900 600]);

subplot(2,2,1)
fplot(solutionX, [0 10], 'LineWidth',1.5)
xlabel('Time (s)'); ylabel('Displacement (m)')
title('Displacement x(t)'); grid on

subplot(2,2,2)
fplot(solutionV, [0 10], 'LineWidth',1.5)
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('Velocity x''(t)'); grid on

subplot(2,2,3)
fplot(solutionA, [0 10], 'LineWidth',1.5)
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)')
title('Acceleration x''''(t)'); grid on