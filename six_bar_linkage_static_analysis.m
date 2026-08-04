%% Static Analysis of Six-Bar Linkage
clc
clear

%% Define joint coordinates
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

%% Define centers of mass
S1 = (A+B)/2;     % Center of mass of link AB
S2 = (B+C+E)/2;   % Approx. center of mass of link BEC
S3 = (C+D)/2;     % Center of mass of link CD
S4 = (E+F)/2;     % Center of mass of link EF
S5 = (F+G)/2;     % Center of mass of link FG

%% Static Equilibrium Equations
syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

FA = [FAx FAy 0];
FB = [FBx FBy 0];
FC = [FCx FCy 0];
FD = [FDx FDy 0];
FE = [FEx FEy 0];
FF = [FFx FFy 0];
FG = [FGx FGy 0];
Torque = [0 0 Tin];

%% Mass of each link
Mass_AB  = 10;
Mass_BEC = 10;
Mass_CD  = 10;
Mass_EF  = 10;
Mass_FG  = 10;

Force_Input = [50 0 0];

%% Equilibrium Equations

%Link AB
Weight_AB = [0 -Mass_AB*9.81 0];
eqn1 = FA + FB + Weight_AB == 0;
eqn2 = cross(A-S1,FA) + cross(B-S1,FB) + Torque == 0;

%Link BEC
Weight_BEC = [0 -Mass_BEC*9.81 0];
eqn3 = -FB + FC + FE + Weight_BEC == 0;
eqn4 = cross(B-S2,-FB) + cross(C-S2,FC) + cross(E-S2,FE) == 0;

%Link CD
Weight_CD = [0 -Mass_CD*9.81 0];
eqn5 = -FC + FD + Weight_CD == 0;
eqn6 = cross(C-S3,-FC) + cross(D-S3,FD) == 0;

%Link EF
Weight_EF = [0 -Mass_EF*9.81 0];
eqn7 = -FE + FF + Weight_EF == 0;
eqn8 = cross(E-S4,-FE) + cross(F-S4,FF) == 0;

%Link FG
Weight_FG = [0 -Mass_FG*9.81 0];
eqn9 = -FF + FG + Force_Input + Weight_FG == 0;
eqn10 = cross(F-S5,-FF) + cross(G-S5,FG) == 0;

equations = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8 eqn9 eqn10];

solution = solve(equations, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy ...
    FFx FFy FGx FGy Tin]);

FA_val = double([solution.FAx solution.FAy]);
FB_val = double([solution.FBx solution.FBy]);
FC_val = double([solution.FCx solution.FCy]);
FD_val = double([solution.FDx solution.FDy]);
FE_val = double([solution.FEx solution.FEy]);
FF_val = double([solution.FFx solution.FFy]);
FG_val = double([solution.FGx solution.FGy]);
Static_Torque = double(solution.Tin);

disp('Force at A (Ax, Ay):'); disp(FA_val)
disp('Force at B (Bx, By):'); disp(FB_val)
disp('Force at C (Cx, Cy):'); disp(FC_val)
disp('Force at D (Dx, Dy):'); disp(FD_val)
disp('Force at E (Ex, Ey):'); disp(FE_val)
disp('Force at F (Fx, Fy):'); disp(FF_val)
disp('Force at G (Gx, Gy):'); disp(FG_val)
disp('Static Torque required at A:'); disp(Static_Torque)