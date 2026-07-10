%% Problem 1: Sum of Two Input Numbers

num1 = input('Enter the first number: ');
num2 = input('Enter the second number: ');

result = num1 + num2;

fprintf('%g+%g=%g\n', num1, num2, result);

%% Part B: Solving Simultaneous Equations

A = [2  3;
     5 -4];
b = [8; -13];
 
sol = A\b;
 
fprintf('\nSolving the system of equations:\n');
fprintf('   2x + 3y = 8\n');
fprintf('   5x - 4y = -13\n\n');
fprintf('Solution:  x = %g,  y = %g\n', sol(1), sol(2));
 