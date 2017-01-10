function [position,isterminal,direction] = EventsFcn_Cstance_rad_tan(t,x)

position = [x(4), pi/2-abs(x(2))]; % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 