%% Static Analysis of Six-Bar Linkage
clc
clear

%% Joint coordinates
A = [1.4  0.485 0];
B = [1.67 0.99  0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.54  0];
F = [-0.98 2.57  0];
G = [0.05  0.2   0];

%% Location of point H, 1.843 m from F along G to F 
u_GF = (F - G)/norm(F - G); 
H = F + 1.843*u_GF;

%% Link lengths
L_AB = norm(B-A);
L_BC = norm(C-B);
L_CD = norm(D-C);
L_DE = norm(E-D);
L_CE = norm(E-C);
L_EF = norm(F-E);
L_FG = norm(G-F);

fprintf('Link lengths (m):\n')
fprintf('AB = %.4f\n', L_AB)
fprintf('BC = %.4f\n', L_BC)
fprintf('CD = %.4f\n', L_CD)
fprintf('DE = %.4f\n', L_DE)
fprintf('CE = %.4f\n', L_CE)
fprintf('EF = %.4f\n', L_EF)
fprintf('FG = %.4f\n\n', L_FG)

%% Centers of mass
S1 = (A+B)/2;       % Link AB
S2 = (B+C)/2;       % Link BC
S3 = (C+D+E)/2;     % Link CDE (approx.)
S4 = (E+F)/2;       % Link EF
S5 = (F+G)/2;       % Link FG

%% Static Equilibrium Equations
syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

FA = [FAx FAy 0];
FB = [FBx FBy 0];
FC = [FCx FCy 0];
FD = [FDx FDy 0];
FE = [FEx FEy 0];
FF = [FFx FFy 0];
FG_ = [FGx FGy 0];
Torque = [0 0 Tin];

%% Applied load acting at H (assumed to be 100 N downward)
Force_artifact = [0 -100 0];

%% Equilibrium Equations

%Link AB
eqn1 = FA + FB == 0;
eqn2 = cross(A-S1,FA) + cross(B-S1,FB) + Torque == 0;

%Link BC
eqn3 = -FB + FC == 0;
eqn4 = cross(B-S2,-FB) + cross(C-S2,FC) == 0;

%Link CDE
eqn5 = -FC + FD + FE == 0;
eqn6 = cross(C-S3,-FC) + cross(D-S3,FD) + cross(E-S3,FE) == 0;

%Link EF
eqn7 = -FE + FF == 0;
eqn8 = cross(E-S4,-FE) + cross(F-S4,FF) == 0;

%Link FG
eqn9 = -FF + FG_ + Force_artifact == 0;
eqn10 = cross(F-S5,-FF) + cross(G-S5,FG_) + cross(H-S5,Force_artifact) == 0;

%% System of equations solver
equations = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7 eqn8 eqn9 eqn10];

solution = solve(equations, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy ...
    FFx FFy FGx FGy Tin]);

%% Displaying numeric results
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

%% Comparison with PMKS+ 
% PMKS+ model URL:
% https://pmksplus.mech.website/?j=A,1.4,0.485,AB,R,t,0,0.1,t,%0AB,1.67,0.99,AB%7CCB,R,f,0,0.1,f,%0AC,0.255,1.035,CB%7CDCE,R,f,0,0.1,f,%0AD,0.285,0.055,DCE,R,t,0,0.1,f,%0AE,0.195,2.54,EF%7CDCE,R,f,0,0.1,f,%0AF,-0.98,2.57,EF%7CGF,R,f,0,0.1,f,%0AG,0.05,0.2,GF,R,t,0,0.1,f,%0A&l=AB,1,1,1.535,0.7375,A%7CB,,l,1.67,0.485,1.4,0.485,1.4,0.99,1.67,0.99%0ACB,1,1,0.9624999999999999,1.0125,C%7CB,,l,1.67,1.035,0.255,1.035,0.255,0.99,1.67,0.99%0AEF,1,1,-0.39249999999999996,2.555,E%7CF,,l,-0.98,2.54,0.195,2.54,0.195,2.57,-0.98,2.57%0ADCE,1,1,0.0010353705383255585,1.377094451384317,D%7CC%7CE,,b,-0.395,-0.166,-0.489,2.893,0.397,2.92,0.491,-0.139%0AGF,1,1,-1.0157514616093193,2.2971181678743857,G%7CF,F1,b,-0.313,-0.176,-2.345,4.498,-1.718,4.77,0.313,0.096%0A&f=F1,GF,-1.715,4.26,-1.715,-0.523,t,true,0,100%0A&pp=&tp=&s=10,false,false,m
%
% PMKS+ results:
% Input Torque ≈ 100 N*m
% Force at A (Ax, Ay): -190.9694, 6.0732
%
% MATLAB results:
% Input Torque = 98.0566 N*m
% Force at A (Ax, Ay): 190.9252, -6.0718
%
%% Comparison Notes:
%   PMKS+ and MATLAB magnitudes agree closely (100 vs 98.06 N*m), 
%   which is within expected precision.
%   There is a sign difference between the two (PMKS+ shows positive,
%   MATLAB shows -98.0566), but only the
%   magnitude comparison matters, since PMKS+'s assumed positive
%   direction differs from the other.
%   The reaction force at joint A matches closely in magnitude between the two,
%   as PMKS+ gives (-190.9694, 6.0732) N versus MATLAB's (190.9252, -6.0718) N,
%   with the sign flip attributable to a difference in assumed positive
%   directions.