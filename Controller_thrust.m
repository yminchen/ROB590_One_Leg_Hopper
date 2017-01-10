function F = Controller_thrust(x)
% Function for state feedback controller.
%
% F:   Joint Torque (N*m)
F = zeros(1,1);

tar_vel = 0; 

% P controller parameters
kp = 2;    % 1.5
% kd = 0.2;   % 1.2
max_f = 200;    % maximum torque that can be applied
% P controller for desired angular velocity.
err = x(5)-x(6) - tar_vel;
% derr =  x(5)-x(6);     
f_final = kp*err;% + kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F = f_final;



