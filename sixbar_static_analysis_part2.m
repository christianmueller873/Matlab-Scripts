%% Six-Bar Pick-and-Place Linkage - Part 5, Second Script of Two
clear; clc; close all;

% Loading kinematic results from Part 1
load('part1_results.mat');

numPositions = length(theta);

% Masses of each link
Mass_AB  = 10;
Mass_BEC = 10;
Mass_CD  = 10;
Mass_EF  = 10;
Mass_FG  = 10;
g = 9.81;

% Applied load acting at H
Force_artifact = [0, -100];   % 100 N downward

% Storage for forces at every position
FA_all = zeros(numPositions, 2);
FB_all = zeros(numPositions, 2);
FC_all = zeros(numPositions, 2);
FD_all = zeros(numPositions, 2);
FE_all = zeros(numPositions, 2);
FF_all = zeros(numPositions, 2);
FG_all = zeros(numPositions, 2);
Tin_all = zeros(numPositions, 1);

% Loop over every crank position and run analysis
for i = 1:numPositions

    newB = [Bx(i), By(i)];
    newC = [Cx(i), Cy(i)];
    newE = [Ex(i), Ey(i)];
    newF = [Fx(i), Fy(i)];
    newH = [Hx(i), Hy(i)];

    [forces, Tin] = staticAnalysisSixBar(A, newB, newC, D, newE, newF, G, newH, ...
        Force_artifact, Mass_AB, Mass_BEC, Mass_CD, Mass_EF, Mass_FG, g);

    FA_all(i,:) = forces.FA;
    FB_all(i,:) = forces.FB;
    FC_all(i,:) = forces.FC;
    FD_all(i,:) = forces.FD;
    FE_all(i,:) = forces.FE;
    FF_all(i,:) = forces.FF;
    FG_all(i,:) = forces.FG;
    Tin_all(i)  = Tin;

    if mod(i, 30) == 0
        fprintf('Static analysis completed for position %d of %d (theta = %.0f deg)\n', ...
            i, numPositions, theta(i));
    end
end

disp('Static analysis complete for all valid crank positions.');

% Force magnitudes at each joint
FA_mag = vecnorm(FA_all, 2, 2);
FB_mag = vecnorm(FB_all, 2, 2);
FC_mag = vecnorm(FC_all, 2, 2);
FD_mag = vecnorm(FD_all, 2, 2);
FE_mag = vecnorm(FE_all, 2, 2);
FF_mag = vecnorm(FF_all, 2, 2);
FG_mag = vecnorm(FG_all, 2, 2);

% Plot force magnitudes
figure('Name', 'Force Magnitudes at All Joints');
hold on; grid on;
plot(theta, FA_mag, 'DisplayName', '|F_A|');
plot(theta, FB_mag, 'DisplayName', '|F_B|');
plot(theta, FC_mag, 'DisplayName', '|F_C|');
plot(theta, FD_mag, 'DisplayName', '|F_D|');
plot(theta, FE_mag, 'DisplayName', '|F_E|');
plot(theta, FF_mag, 'DisplayName', '|F_F|');
plot(theta, FG_mag, 'DisplayName', '|F_G|');
legend('Location', 'best');
xlabel('Crank angle \theta (deg)');
ylabel('Force magnitude (N)');
title('Joint Force Magnitudes vs Crank Angle');

% Plot input torque
figure('Name', 'Input Torque');
plot(theta, Tin_all, 'LineWidth', 1.5);
grid on;
xlabel('Crank angle \theta (deg)');
ylabel('Input Torque (N\cdotm)');
title('Static Input Torque vs Crank Angle');

% Save results
save('part2_results.mat', 'theta', 'FA_all','FB_all','FC_all','FD_all','FE_all','FF_all','FG_all', ...
    'Tin_all', 'FA_mag','FB_mag','FC_mag','FD_mag','FE_mag','FF_mag','FG_mag');

disp('Part 2 complete: forces and torque computed, plotted, and saved to part2_results.mat');

% Local Functions

function [forces, Tin_val] = staticAnalysisSixBar(A, B, C, D, E, F, G, H, ...
    Force_artifact, Mass_AB, Mass_BEC, Mass_CD, Mass_EF, Mass_FG, g)

% Centers of mass
    S1 = (A+B)/2;       % Link AB
    S2 = (B+C+E)/3;     % Link BCE (centroid of triangle)
    S3 = (C+D)/2;       % Link CD
    S4 = (E+F)/2;       % Link EF
    S5 = (F+G)/2;       % Link FG

    syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

    FA = [FAx FAy];
    FB = [FBx FBy];
    FC = [FCx FCy];
    FD = [FDx FDy];
    FE = [FEx FEy];
    FF = [FFx FFy];
    FGv = [FGx FGy];

    cross2 = @(r, Fv) r(1)*Fv(2) - r(2)*Fv(1);  

    Weight_AB  = [0, -Mass_AB*g];
    Weight_BEC = [0, -Mass_BEC*g];
    Weight_CD  = [0, -Mass_CD*g];
    Weight_EF  = [0, -Mass_EF*g];
    Weight_FG  = [0, -Mass_FG*g];

    % Link AB
    eqn1 = FA + FB + Weight_AB == 0;
    eqn2 = cross2(A-S1, FA) + cross2(B-S1, FB) + Tin == 0;

    % Link BCE
    eqn3 = -FB + FC + FE + Weight_BEC == 0;
    eqn4 = cross2(B-S2, -FB) + cross2(C-S2, FC) + cross2(E-S2, FE) == 0;

    % Link CD
    eqn5 = -FC + FD + Weight_CD == 0;
    eqn6 = cross2(C-S3, -FC) + cross2(D-S3, FD) == 0;

    % Link EF
    eqn7 = -FE + FF + Weight_EF == 0;
    eqn8 = cross2(E-S4, -FE) + cross2(F-S4, FF) == 0;

    % Link FG (load applied at H)
    eqn9  = -FF + FGv + Weight_FG + Force_artifact == 0;
    eqn10 = cross2(F-S5, -FF) + cross2(G-S5, FGv) + cross2(H-S5, Force_artifact) == 0;

    equations = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8 eqn9 eqn10];

    solution = solve(equations, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy ...
        FFx FFy FGx FGy Tin]);

    forces.FA = double([solution.FAx solution.FAy]);
    forces.FB = double([solution.FBx solution.FBy]);
    forces.FC = double([solution.FCx solution.FCy]);
    forces.FD = double([solution.FDx solution.FDy]);
    forces.FE = double([solution.FEx solution.FEy]);
    forces.FF = double([solution.FFx solution.FFy]);
    forces.FG = double([solution.FGx solution.FGy]);
    Tin_val   = double(solution.Tin);
end