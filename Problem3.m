%% Problem 3: Generate 10 Random Numbers with a For Loop and Plot

x = 1:10;
y = zeros(1,10);
 
for i = 1:10
    y(i) = rand;
end
 
disp(y)
 
plot(x,y,'-o')
xlabel('Number (1-10)')
ylabel('Random Number')
title('Random Numbers Generated')