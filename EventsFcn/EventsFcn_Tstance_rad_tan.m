function [position,isterminal,direction] = EventsFcn_Tstance_rad_tan(t,x,contact_pos,k,L_sp0,m2)

g = 9.81;

position = [k*(x(1)-L_sp0)*cos(x(2))-m2*g,...
    contact_pos(2)+x(1)*cos(x(2)) - Terrain(contact_pos(1)-x(1)*sin(x(2)))]; 
    % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 