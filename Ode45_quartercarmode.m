%% Quarter car model

% Parameters
ms = 300;      % Sprung mass (kg)
mu = 40;       % Unsprung mass (kg)
ks = 22000;    % Suspension stiffness (N/m)
bs = 1500;     % Suspension damping coefficient (N*s/m)
kt = 190000;   % Tyre stiffness (N/m)
bt = 150;      % Tyre damping coefficient (N*s/m)

% time
tspan = [0, 2.5];

% Initial conditions
z0 = [0; 0; 0; 0];

[t, z] = ode45(@(t, z) quarter_car_ode(t, z, ms, mu, ks, bs, kt, bt), tspan, z0);

zs = z(:,1);  vs = z(:,2);
zu = z(:,3);  vu = z(:,4);

% Compute Road Elevation
[r_vec, dr_vec] = arrayfun(@road_profile, t);

% Compute Chassis Acceleration: (a_s = 1/ms * (-ks*(zs - zu) - bs*(vs -
% vu)))
accel_s = (-ks*(zs - zu) - bs*(vs - vu)) / ms;

% Plotting
figure('Position', [100, 100, 850, 650]);

subplot(2,1,1);
plot(t, r_vec, 'k--', 'LineWidth', 1.5); hold on;
plot(t, zu, 'b-', 'LineWidth', 1.2);
plot(t, zs, 'r-', 'LineWidth', 1.5);
title('Quarter Car Response with Tire Damping (b_t = 150 Ns/m)');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('Road Input r(t)', 'Unsprung Mass z_u', 'Sprung Mass z_s', 'Location', 'Northeast');
grid on;

subplot(2,1,2);
plot(t, accel_s/9.81, 'm-', 'LineWidth', 1.5);
title('Chassis Vertical Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (g)');
grid on;

function dydt = quarter_car_ode(t, z, ms, mu, ks, bs, kt, bt)
    zs = z(1); vs = z(2);
    zu = z(3); vu = z(4);

    [r, dr_dt] = road_profile(t);

    dxs_dt = vs;
    dvs_dt = (-ks*(zs - zu) - bs*(vs - vu)) / ms;

    dxu_dt = vu;
    dvu_dt = (ks*(zs - zu) + bs*(vs - vu) - kt*(zu - r) - bt*(vu - dr_dt)) / mu;

    dydt = [dxs_dt; dvs_dt; dxu_dt; dvu_dt];
end

% Smoothed Road Profile Function 
function [r, dr_dt] = road_profile(t)
    % Parameters for a smooth 10 cm step bump around t = 0.2 (seconds)
    bump_height = 0.10;  % 10 cm bump height (m)
    t_start = 0.2;       % Bump start time (seconds)
    steepness = 50;      % For smoothness

    % Smoothed step
    r = (bump_height / 2) * (1 + tanh(steepness * (t - t_start)));

    % Derivative: d/dt [tanh(u)] = sech^2(u) * du/dt
    dr_dt = (bump_height / 2) * steepness * (sech(steepness * (t - t_start)))^2;
end