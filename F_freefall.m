function dx = F_freefall(t,x,d,k,L_sp0,L_m1,m1,I1,m2,t_prev_stance,target_pos,k_f,max_dx_des)
% Derivative function for the model.
dx = zeros(10,1);

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

% Controller
F_ctrl = Controller_flight(x, L_sp0, t_prev_stance, target_pos, k_f, max_dx_des);

% equation of motion
% InvMassMatrix_fl(x1,y1,theta,s,phi,g,k,L_sp0,L_m1,m1,I1,m2)
% F_CoriGrav_fl(x1,y1,theta,s,phi,dx1,dy1,dtheta,ds,dphi,g,k,L_sp0,L_m1,m1,I1,m2)
invM = InvMassMatrix_fl(x(1),x(2),x(3),x(4),x(5),g,k,L_sp0,L_m1,m1,I1,m2);
f_cg = F_CoriGrav_fl(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),g,k,L_sp0,L_m1,m1,I1,m2);
u = [0; 0; 0; -d*x(9); F_ctrl];
%%% TODO: checkout if the damping force is correctly implemented.

dx(1:5) = x(6:10);
dx(6:10) = invM*(f_cg + u);