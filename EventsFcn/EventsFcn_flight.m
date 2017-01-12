function [position,isterminal,direction] = EventsFcn_flight(t,x)

position = [(x(2)-x(4)*cos(x(3)+x(5))) - Terrain(x(1)-x(4)*sin(x(3)+x(5))),...
             x(2) - Terrain(x(1))]; % The value that we want to be zero
isterminal = [1, 1];    % Halt integration 
direction = [-1, -1];   % The direction that the zero is approached 