%% Six-Bar Pick-and-Place Linkage - Circle Intersection Method
% PART 1 
clear; clc; close all;

% 1) Original joint coordinates 

A = [7, 4];     % ground pivot
B0 = [5, 16];   % initial position of B
C0 = [25, 25];  % initial position of C
D = [23, 10];   % ground pivot
E0 = [18, 35];  % initial position of E  (for Part 2)
F0 = [43, 32];  % initial position of F  (for Part 2)
G = [45, 17];   % ground pivot           (for Part 2)

% 2) Compute fixed link lengths using the distance function
AB = calcDistance(A, B0);
BC = calcDistance(B0, C0);
DC = calcDistance(D, C0);
AD = calcDistance(A, D);

fprintf('Link AB length = %.4f\n', AB);
fprintf('Link BC length = %.4f\n', BC);
fprintf('Link DC length = %.4f\n', DC);
fprintf('Ground link AD length = %.4f\n', AD);

% Grashof check for the closed loop A-B-C-D-A
links = [AB, BC, DC, AD];
s = min(links);
l = max(links);
remaining = sort(links);
pq = remaining(2) + remaining(3);
sl = s + l;
fprintf('\nGrashof check (loop A-B-C-D-A): shortest+longest = %.4f, other two = %.4f\n', sl, pq);
if sl <= pq
    disp('--> Grashof-satisfied: a full 360 deg crank rotation IS geometrically possible.');
else
    disp('--> NON-GRASHOF (triple rocker): AB CANNOT complete a full 360 deg rotation');
    disp('    with these coordinates. Some crank angles will have no valid C position.');
    disp('    If your assignment requires a true 360 deg input, double-check that these');
    disp('    coordinates actually belong to the pick-and-place mechanism.');
end

% 3) Sweep the input crank AB through 360 degrees (CCW)
numPositions = 360;
theta = linspace(0, 359, numPositions);  % 360 crank positions, 1 deg apart

% Trajectory storage
Bx = zeros(1, numPositions);
By = zeros(1, numPositions);
Cx = zeros(1, numPositions);
Cy = zeros(1, numPositions);

% Initial angle of link AB measured from A
angle0_AB = atan2(B0(2) - A(2), B0(1) - A(1));

% Use the original C position as the "last known" C
lastC = C0;
invalidCount = 0;

for i = 1:numPositions

    %Step 1: Rotate AB by theta(i) degrees (CCW) to get new B
    newAngle_AB = angle0_AB + deg2rad(theta(i));
    newB = A + AB * [cos(newAngle_AB), sin(newAngle_AB)];

    %%Step 2: Find C using circle intersection
    solutions = circleIntersection(newB, BC, D, DC);

    if isempty(solutions)

        newC = [NaN, NaN];
        invalidCount = invalidCount + 1;
    else
 
        newC = chooseClosestPoint(solutions, lastC);
    end

    %%Store results for this position
    Bx(i) = newB(1);
    By(i) = newB(2);
    Cx(i) = newC(1);
    Cy(i) = newC(2);

    % Update "last known" C for the next iteration (only if valid)
    if ~any(isnan(newC))
        lastC = newC;
    end
end

fprintf('\n%d of %d crank positions had no valid C (loop could not close).\n', ...
    invalidCount, numPositions);
if invalidCount > 0
    badTheta = theta(isnan(Cx));
    fprintf('Invalid theta range: %.0f deg to %.0f deg\n', min(badTheta), max(badTheta));
end

% 4) Plot results as a checkpoint for Part 1
figure('Name', 'Six-Bar Linkage - Part 1: B and C Trajectories');

% Original mechanism
subplot(1, 2, 1);
hold on; grid on; axis equal;
plot([A(1) B0(1)], [A(2) B0(2)], 'o-', 'LineWidth', 2, 'Color', [0.85 0.65 0.13]); % AB
plot([B0(1) C0(1) E0(1) B0(1)], [B0(2) C0(2) E0(2) B0(2)], 'o-', 'LineWidth', 2, 'Color', [0.47 0.67 0.19]); % BCE triangle
plot([D(1) C0(1)], [D(2) C0(2)], 'o-', 'LineWidth', 2, 'Color', [0.30 0.75 0.93]); % DC
plot([E0(1) F0(1)], [E0(2) F0(2)], 'o-', 'LineWidth', 2, 'Color', [0.49 0.18 0.56]); % EF
plot([F0(1) G(1)], [F0(2) G(2)], 'o-', 'LineWidth', 2, 'Color', [0.49 0.18 0.56]); % FG
text(A(1), A(2)-1.5, 'A'); text(B0(1)-1.5, B0(2), 'B'); text(C0(1), C0(2)+1, 'C');
text(D(1), D(2)-1.5, 'D'); text(E0(1), E0(2)+1, 'E'); text(F0(1)+1, F0(2), 'F');
text(G(1), G(2)-1.5, 'G');
title('Original Mechanism (Reference Pose)');
xlabel('X'); ylabel('Y');

% Trajectories of B and C over full crank rotation
subplot(1, 2, 2);
hold on; grid on; axis equal;
plot(Bx, By, 'b.-', 'DisplayName', 'Trajectory of B');
plot(Cx, Cy, 'r.-', 'DisplayName', 'Trajectory of C');
plot(A(1), A(2), 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'A (ground)');
plot(D(1), D(2), 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'D (ground)');
legend('Location', 'best');
title('Joint Trajectories: B and C (360 deg crank rotation)');
xlabel('X'); ylabel('Y');

% Save intermediate results so Part 2 can load and continue
% (positions of B and C for every crank angle)
save('part1_results.mat', 'theta', 'Bx', 'By', 'Cx', 'Cy', ...
    'A', 'B0', 'C0', 'D', 'E0', 'F0', 'G', 'AB', 'BC', 'DC');

disp('Part 1 complete: B and C trajectories computed and saved to part1_results.mat');

%Local Functions

function d = calcDistance(p1, p2)
% calcDistance
    d = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);
end


function chosen = chooseClosestPoint(candidates, reference) %%%%%1

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

    % Distance from c1 to the line connecting the two intersection points
    a = (r1^2 - r2^2 + d^2) / (2 * d);

    % Half length connecting the two intersection points
    hSq = r1^2 - a^2;
    if hSq < 0
        % Guard against tiny negative values from floating point error
        hSq = 0;
    end
    h = sqrt(hSq);

    % Point on the line between c1 and c2
    xm = c1(1) + a * (c2(1) - c1(1)) / d;
    ym = c1(2) + a * (c2(2) - c1(2)) / d;

    % Offset perpendicular to the c1-c2 line
    xs1 = xm + h * (c2(2) - c1(2)) / d;
    ys1 = ym - h * (c2(1) - c1(1)) / d;

    xs2 = xm - h * (c2(2) - c1(2)) / d;
    ys2 = ym + h * (c2(1) - c1(1)) / d;

    if h == 0
        pts = [xs1, ys1];  % tangent circles -> one solution
    else
        pts = [xs1, ys1; xs2, ys2];
    end
end