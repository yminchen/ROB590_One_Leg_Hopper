function y = Terrain(x,terrain_label)
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

edge = Terrain_edge(terrain_label);
for i=1:n1    
    for j = 1:size(edge,1)+1
        if j==size(edge,1)+1
            y(i) = edge(j-1,2);
        elseif x(i)<edge(j,1) 
            if j==1 
                y(i) = edge(j,2);
            else
                y(i) = (edge(j,2)-edge(j-1,2))/(edge(j,1)-edge(j-1,1))*...
                    (x(i)-edge(j-1,1)) + edge(j-1,2);
            end
            break
        end
    end
end
    
    
    
    
%     for i=1:n1    
%         if x(i)<=1 
%             y(i) = 2;
%         elseif x(i)<2
%             y(i) = 0;
%         elseif x(i)<4
%             y(i) = 0.5*x(i)-1;
%         elseif x(i)<5
%             y(i) = x(i)-3;
%         elseif x(i)<6
%             y(i) = 2;
%         elseif x(i)<7
%             y(i) = -0.5*x(i)+5;
%         elseif x(i)<8
%             y(i) = x(i)-5.5;
%         elseif x(i)<9
%             y(i) = -0.5*x(i)+6.5;
%         elseif x(i)<11
%             y(i) = -x(i)+11;
%         else
%             y(i) = 0;
%         end
%     end
    
