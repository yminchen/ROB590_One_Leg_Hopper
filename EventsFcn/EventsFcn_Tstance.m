function [position,isterminal,direction] = EventsFcn_Tstance(t,x,L)

position = x(3) - L*cos(x(5)); % The value that we want to be zero
isterminal = 1;  % Halt integration 
direction = 1;   % The direction that the zero is approached 