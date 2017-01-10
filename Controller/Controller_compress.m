function F = Controller_compress(x)
% Function for state feedback controller.
%
% F:   Joint Torque (N*m)
F = zeros(1,1);

tar_angle = 0; 

% PD controller parameters
kp = 20;    % 1.5
kd = 2.2;   % 1.2
max_f = 200;    % maximum torque that can be applied
% PD controller for desired phi.
err = x(2)-x(3) - tar_angle;
derr =  x(5)-x(6);     
f_final = kp*err + kd*derr;
if f_final > max_f
    f_final = max_f;
elseif f_final < -max_f
    f_final = -max_f;
end

% Assignment
F = f_final;



