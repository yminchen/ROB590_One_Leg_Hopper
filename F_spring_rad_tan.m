function dx = F_spring_rad_tan(t,x,d,k,L_sp0,L_m1,m1,I1,m2,thrust_flag,E_des,k_des)
% Derivative function for the model.
dx = zeros(6,1);

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

% current energy
% Energy_stance(s,phi,theta,ds,dphi,dtheta,g,k,L_sp0,L_m1,m1,I1)
%E = Energy_stance(x(1),x(2),x(3),x(4),x(5),x(6),g,k,L_sp0,L_m1,m1,I1);

%Controller for thrust  
if thrust_flag %&& (E_des > E) 
    k = k_des;
end

% Controller for spring angle 
if thrust_flag 
    F_ctrl = Controller_thrust(x);
%     F_ctrl = 0;
%     F_ctrl = Controller_compress(x);
else
    F_ctrl = Controller_compress(x);
end


% equation of motion
% F_CoriGrav_st(s,phi,theta,ds,dphi,dtheta,g,k,L_sp0,L_m1,m1,I1)
% InvMassMatrix_st(s,phi,theta,g,k,L_sp0,L_m1,m1,I1)
f_cg = F_CoriGrav_st(x(1),x(2),x(3),x(4),x(5),x(6),g,k,L_sp0,L_m1,m1,I1);
invM = InvMassMatrix_st(x(1),x(2),x(3),g,k,L_sp0,L_m1,m1,I1);
u = [-d*x(4); 0; F_ctrl];
%%% TODO: checkout if the damping force is correctly implemented.

dx(1:3) = x(4:6);
dx(4:6) = invM*(f_cg + u);


