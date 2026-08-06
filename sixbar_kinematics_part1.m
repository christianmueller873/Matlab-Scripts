%% Six-Bar Pick-and-Place Linkage - with H, First Script of Two
clear; clc; close all;

% Original joint coordinates
A = [7, 4];     % ground pivot
B0 = [5, 16];   % initial position of B
C0 = [25, 25];  % initial position of C
D = [23, 10];   % ground pivot
E0 = [18, 35];  % initial position of E
F0 = [43, 32];  % initial position of F
G = [45, 17];   % ground pivot

% Fixed link lengths
AB = calcDistance(A, B0);
BC = calcDistance(B0, C0);
DC = calcDistance(D, C0);
BE = calcDistance(B0, E0);
CE = calcDistance(C0, E0);
EF = calcDistance(E0, F0);
FG = calcDistance(F0, G);
AD = calcDistance(A, D);

fprintf('Link AB length = %.4f\n', AB);
fprintf('Link BC length = %.4f\n', BC);
fprintf('Link DC length = %.4f\n', DC);
fprintf('Link BE length = %.4f\n', BE);
fprintf('Link CE length = %.4f\n', CE);
fprintf('Link EF length = %.4f\n', EF);
fprintf('Link FG length = %.4f\n', FG);
fprintf('Ground link AD length = %.4f\n', AD);

% Location of H (center of link FG)
H0 = (F0 + G) / 2;
fprintf('H (reference pose) = [%.4f, %.4f]\n\n', H0(1), H0(2));

% Grashof check for the closed loop A-B-C-D-A
links = [AB, BC, DC, AD];
s = min(links);
l = max(links);
remaining = sort(links);
pq = remaining(2) + remaining(3);
sl = s + l;
fprintf('Grashof check (loop A-B-C-D-A): shortest+longest = %.4f, other two = %.4f\n', sl, pq);
if sl <= pq
    disp('--> Grashof-satisfied: a full 360 deg crank rotation IS geometrically possible.');
else
    disp('--> NON-GRASHOF (triple rocker): AB CANNOT complete a full 360 deg rotation.');
    disp('    The loop will stop as soon as it hits the first invalid position.');
end

% Input crank AB to solve B -> C -> E -> F -> H
numPositions = 360;
theta = linspace(0, 359, numPositions);

Bx = zeros(1, numPositions);  By = zeros(1, numPositions);
Cx = zeros(1, numPositions);  Cy = zeros(1, numPositions);
Ex = zeros(1, numPositions);  Ey = zeros(1, numPositions);
Fx = zeros(1, numPositions);  Fy = zeros(1, numPositions);
Hx = zeros(1, numPositions);  Hy = zeros(1, numPositions);

angle0_AB = atan2(B0(2) - A(2), B0(1) - A(1));

lastC = C0;
lastE = E0;
lastF = F0;

lastValidIdx = 0;

for i = 1:numPositions

    % new B
    newAngle_AB = angle0_AB + deg2rad(theta(i));
    newB = A + AB * [cos(newAngle_AB), sin(newAngle_AB)];

    % new C
    solC = circleIntersection(newB, BC, D, DC);
    if isempty(solC)
        fprintf('C cannot be determined at theta = %.0f deg. Stopping.\n', theta(i));
        break;
    end
    newC = chooseClosestPoint(solC, lastC);

    % new E
    solE = circleIntersection(newB, BE, newC, CE);
    if isempty(solE)
        fprintf('E cannot be determined at theta = %.0f deg. Stopping.\n', theta(i));
        break;
    end
    newE = chooseClosestPoint(solE, lastE);

    % new F
    solF = circleIntersection(newE, EF, G, FG);
    if isempty(solF)
        fprintf('F cannot be determined at theta = %.0f deg. Stopping.\n', theta(i));
        break;
    end
    newF = chooseClosestPoint(solF, lastF);

    % new H = midpoint of F and G
    newH = (newF + G) / 2;

    % Store results
    Bx(i) = newB(1);  By(i) = newB(2);
    Cx(i) = newC(1);  Cy(i) = newC(2);
    Ex(i) = newE(1);  Ey(i) = newE(2);
    Fx(i) = newF(1);  Fy(i) = newF(2);
    Hx(i) = newH(1);  Hy(i) = newH(2);

    lastC = newC;
    lastE = newE;
    lastF = newF;
    lastValidIdx = i;
end

% Arrays to valid positions
theta = theta(1:lastValidIdx);
Bx = Bx(1:lastValidIdx);  By = By(1:lastValidIdx);
Cx = Cx(1:lastValidIdx);  Cy = Cy(1:lastValidIdx);
Ex = Ex(1:lastValidIdx);  Ey = Ey(1:lastValidIdx);
Fx = Fx(1:lastValidIdx);  Fy = Fy(1:lastValidIdx);
Hx = Hx(1:lastValidIdx);  Hy = Hy(1:lastValidIdx);

fprintf('\nMechanism successfully solved for %d of %d candidate crank positions ', ...
    lastValidIdx, numPositions);
fprintf('(theta = 0 to %.0f deg).\n', theta(end));

% Plot results
figure('Name', 'Six-Bar Linkage - Part 1: Trajectories including H');

subplot(1, 2, 1);
hold on; grid on; axis equal;
plot([A(1) B0(1)], [A(2) B0(2)], 'o-', 'LineWidth', 2, 'Color', [0.85 0.65 0.13]); % AB
plot([B0(1) C0(1) E0(1) B0(1)], [B0(2) C0(2) E0(2) B0(2)], 'o-', 'LineWidth', 2, 'Color', [0.47 0.67 0.19]); % BCE
plot([D(1) C0(1)], [D(2) C0(2)], 'o-', 'LineWidth', 2, 'Color', [0.30 0.75 0.93]); % DC
plot([E0(1) F0(1)], [E0(2) F0(2)], 'o-', 'LineWidth', 2, 'Color', [0.49 0.18 0.56]); % EF
plot([F0(1) G(1)], [F0(2) G(2)], 'o-', 'LineWidth', 2, 'Color', [0.49 0.18 0.56]); % FG
plot(H0(1), H0(2), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % H
text(A(1), A(2)-1.5, 'A'); text(B0(1)-1.5, B0(2), 'B'); text(C0(1), C0(2)+1, 'C');
text(D(1), D(2)-1.5, 'D'); text(E0(1), E0(2)+1, 'E'); text(F0(1)+1, F0(2), 'F');
text(G(1), G(2)-1.5, 'G'); text(H0(1)+1, H0(2), 'H');
title('Original Mechanism (Reference Pose)');
xlabel('X'); ylabel('Y');

subplot(1, 2, 2);
hold on; grid on; axis equal;
plot(Bx, By, '.-', 'DisplayName', 'Trajectory of B');
plot(Cx, Cy, '.-', 'DisplayName', 'Trajectory of C');
plot(Ex, Ey, '.-', 'DisplayName', 'Trajectory of E');
plot(Fx, Fy, '.-', 'DisplayName', 'Trajectory of F');
plot(Hx, Hy, '.-', 'DisplayName', 'Trajectory of H');
plot(A(1), A(2), 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'A (ground)');
plot(D(1), D(2), 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'D (ground)');
plot(G(1), G(2), 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'G (ground)');
legend('Location', 'best');
title(sprintf('Joint Trajectories (theta = 0 to %.0f deg)', theta(end)));
xlabel('X'); ylabel('Y');

% Save results (for Part 2)
save('part1_results.mat', 'theta', 'Bx','By','Cx','Cy','Ex','Ey','Fx','Fy','Hx','Hy', ...
    'A','B0','C0','D','E0','F0','G','AB','BC','DC','BE','CE','EF','FG');

disp('Part 1 complete: B, C, E, F, H trajectories computed and saved to part1_results.mat');

% Local Functions

function d = calcDistance(p1, p2)
    d = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);
end

function chosen = chooseClosestPoint(candidates, reference)
    if size(candidates, 1) == 1
        chosen = candidates(1, :);
        return;
    end
    d1 = calcDistance(candidates(1, :), reference);
    d2 = calcDistance(candidates(2, :), reference);
    if d1 <= d2
        chosen = candidates(1, :);
    else
        chosen = candidates(2, :);
    end
end

function pts = circleIntersection(c1, r1, c2, r2)
    d = calcDistance(c1, c2);
    if d > (r1 + r2) || d < abs(r1 - r2) || d == 0
        pts = [];
        return;
    end
    a = (r1^2 - r2^2 + d^2) / (2 * d);
    hSq = r1^2 - a^2;
    if hSq < 0
        hSq = 0;
    end
    h = sqrt(hSq);
    xm = c1(1) + a * (c2(1) - c1(1)) / d;
    ym = c1(2) + a * (c2(2) - c1(2)) / d;
    xs1 = xm + h * (c2(2) - c1(2)) / d;
    ys1 = ym - h * (c2(1) - c1(1)) / d;
    xs2 = xm - h * (c2(2) - c1(2)) / d;
    ys2 = ym + h * (c2(1) - c1(1)) / d;
    if h == 0
        pts = [xs1, ys1];
    else
        pts = [xs1, ys1; xs2, ys2];
    end
end