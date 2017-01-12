function y = Terrain(x)
% terrain funcrion y(x)
% y(x0) is the height of the ground at x = x0.

% The size of y is the same as that of x.
% Both of them would be row vectors or column vectors.

n = size(x);
y = zeros(n);

n1 = size(y,1);
n2 = size(y,2);
if n1<n2
    n1 = n2;
end

for i=1:n1    
    if x(i)<2 || x(i)>6
        y(i) = 0;
    elseif x(i)<4
        y(i) = 0.25*x(i)-0.5;
    else
        y(i) = -0.25*x(i)+1.5;
    end
end
