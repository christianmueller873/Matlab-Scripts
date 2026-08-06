%% Six-Bar Pick-and-Place Linkage - Animation and CSV 
clear; clc; close all;

% IMPORTANT NOTE BEFORE RUNNING: Two previous scripts need to be ran in
% this specific order before running: "sixbar_kinematics_part1" then
% "sixbar_static_analysis_part2"

% NOTE: csv file with the trajectory data is saved under "trajectory_data"

% Load kinematic results (B, C, E, F, H trajectories)
load('part1_results.mat');

% Load static analysis results
load('part2_results.mat');

numPositions = length(theta);

% Plot the initial link lines
figure('Name', 'Six-Bar Linkage Animation');
hold on; grid on; axis equal;

% Link AB
hAB = plot([A(1) Bx(1)], [A(2) By(1)], 'o-', 'LineWidth', 2, 'Color', [0.85 0.65 0.13]);
% Link BC
hBC = plot([Bx(1) Cx(1)], [By(1) Cy(1)], 'o-', 'LineWidth', 2, 'Color', [0.47 0.67 0.19]);
% Link CD
hCD = plot([Cx(1) D(1)], [Cy(1) D(2)], 'o-', 'LineWidth', 2, 'Color', [0.30 0.75 0.93]);
% Link CE (BEC triangle)
hCE = plot([Cx(1) Ex(1)], [Cy(1) Ey(1)], 'o-', 'LineWidth', 2, 'Color', [0.47 0.67 0.19]);
% Link EF
hEF = plot([Ex(1) Fx(1)], [Ey(1) Fy(1)], 'o-', 'LineWidth', 2, 'Color', [0.49 0.18 0.56]);
% Link FG
hFG = plot([Fx(1) G(1)], [Fy(1) G(2)], 'o-', 'LineWidth', 2, 'Color', [0.49 0.18 0.56]);

% Joint markers
hJoints = plot([A(1) Bx(1) Cx(1) D(1) Ex(1) Fx(1) G(1)], ...
               [A(2) By(1) Cy(1) D(2) Ey(1) Fy(1) G(2)], ...
               'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

% Animated trajectory lines for B, C, E, F
hTrajB = plot(Bx(1), By(1), 'b.-', 'DisplayName', 'Trajectory of B');
hTrajC = plot(Cx(1), Cy(1), 'r.-', 'DisplayName', 'Trajectory of C');
hTrajE = plot(Ex(1), Ey(1), 'g.-', 'DisplayName', 'Trajectory of E');
hTrajF = plot(Fx(1), Fy(1), 'm.-', 'DisplayName', 'Trajectory of F');

legend([hTrajB hTrajC hTrajE hTrajF], 'Location', 'eastoutside');

% Setting axis limits
allX = [A(1) D(1) G(1) Bx Cx Ex Fx];
allY = [A(2) D(2) G(2) By Cy Ey Fy];

xPad = 15;
yPad = 15;

xlim([min(allX)-xPad, max(allX)+xPad]);
ylim([min(allY)-yPad, max(allY)+yPad]);

xlabel('X'); ylabel('Y');
title('Six-Bar Pick-and-Place Linkage Animation');

% Animate through all valid crank positions (slowed version - previous was
% too fast)
frameDelay = 0.08;   % seconds

for i = 1:numPositions

    % Update link lines
    set(hAB, 'XData', [A(1) Bx(i)], 'YData', [A(2) By(i)]);
    set(hBC, 'XData', [Bx(i) Cx(i)], 'YData', [By(i) Cy(i)]);
    set(hCD, 'XData', [Cx(i) D(1)], 'YData', [Cy(i) D(2)]);
    set(hCE, 'XData', [Cx(i) Ex(i)], 'YData', [Cy(i) Ey(i)]);
    set(hEF, 'XData', [Ex(i) Fx(i)], 'YData', [Ey(i) Fy(i)]);
    set(hFG, 'XData', [Fx(i) G(1)], 'YData', [Fy(i) G(2)]);

    % Update joint markers
    set(hJoints, 'XData', [A(1) Bx(i) Cx(i) D(1) Ex(i) Fx(i) G(1)], ...
                 'YData', [A(2) By(i) Cy(i) D(2) Ey(i) Fy(i) G(2)]);

    % Update trajectory lines
    set(hTrajB, 'XData', Bx(1:i), 'YData', By(1:i));
    set(hTrajC, 'XData', Cx(1:i), 'YData', Cy(1:i));
    set(hTrajE, 'XData', Ex(1:i), 'YData', Ey(1:i));
    set(hTrajF, 'XData', Fx(1:i), 'YData', Fy(1:i));

    drawnow;
    pause(frameDelay);
end

disp('Animation complete.');

% Save data into a spreadsheet (CSV file)

trajectoryData = table( ...
    theta', ...
    Bx', By', Cx', Cy', Ex', Ey', Fx', Fy', Hx', Hy', ...
    FA_all(:,1), FA_all(:,2), ...
    FB_all(:,1), FB_all(:,2), ...
    FC_all(:,1), FC_all(:,2), ...
    FD_all(:,1), FD_all(:,2), ...
    FE_all(:,1), FE_all(:,2), ...
    FF_all(:,1), FF_all(:,2), ...
    FG_all(:,1), FG_all(:,2), ...
    Tin_all, ...
    'VariableNames', { ...
        'Theta_deg', ...
        'Bx','By','Cx','Cy','Ex','Ey','Fx','Fy','Hx','Hy', ...
        'FAx','FAy','FBx','FBy','FCx','FCy','FDx','FDy', ...
        'FEx','FEy','FFx','FFy','FGx','FGy', ...
        'InputTorque'});

writetable(trajectoryData, 'trajectory_data.csv');

disp('Part complete: animation played and trajectory_data.csv saved.');