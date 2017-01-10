function [position,isterminal,direction] = EventsFcn_Tstance_rad_tan(t,x,k,L_sp0,m2)

g = 9.81;

position = [k*(x(1)-L_sp0)*cos(x(2))-m2*g, pi/2-abs(x(2))]; % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 