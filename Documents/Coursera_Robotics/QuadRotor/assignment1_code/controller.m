function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
k_v = 50;
k_p = 400;
error = s_des - s;
z_2 = 0;
u = params.mass*(z_2 + params.gravity + k_p * error(1) + k_v * error(2));
if (u > params.u_max)
    u = params.u_max;
end


% FILL IN YOUR CODE HERE


end

