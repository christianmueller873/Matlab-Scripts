%% Problem 2: Odd/Even Based Operation on Two Numbers
%% Problem 2: Odd And Even Based Operation on Two Numbers
num1 = input('Enter the first number: ');
num2 = input('Enter the second number: ');

result = oddEvenOperation(num1, num2);

fprintf('Result: %g\n', result);

function result = oddEvenOperation(a, b)

    aIsOdd = mod(a, 2) ~= 0;
    bIsOdd = mod(b, 2) ~= 0;

    if aIsOdd && bIsOdd
        result = a + b;
        fprintf('Both numbers are odd -> adding them together.\n');

    elseif ~aIsOdd && ~bIsOdd
        result = max(a, b) - min(a, b);
        fprintf('Both numbers are even -> subtracting smaller from larger.\n');

    else
        result = a * b;
        fprintf('One number is odd and the other is even -> multiplying them.\n');
    end
end