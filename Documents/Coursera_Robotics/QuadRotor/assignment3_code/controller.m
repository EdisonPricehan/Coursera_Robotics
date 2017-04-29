function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
% Gain of position
Kp = [100; 100; 800];
Kv = [1; 1; 1];
% Commanded position
r_des_ddot = des_state.acc + Kp .* (des_state.pos - state.pos) + Kv .* (des_state.vel - state.vel);
% Commanded thrust
F = params.mass * (params.gravity + r_des_ddot(3));

% Restrict F
if F < params.minF
    F = params.minF;
end
if F > params.maxF
    F = params.maxF;
end

% Define commanded phi and theta angles
phi_des = (1 / params.gravity) * (r_des_ddot(1) * sin(des_state.yaw) - r_des_ddot(2) * cos(des_state.yaw));
theta_des = (1 / params.gravity) * (r_des_ddot(1) * cos(des_state.yaw) + r_des_ddot(2) * sin(des_state.yaw));

% Calculate error terms
err_rot = [phi_des - state.rot(1); theta_des - state.rot(2); des_state.yaw - state.rot(3)];
err_omega = [- state.omega(1); -state.omega(2); des_state.yawdot - state.omega(3)];

% Gain of Angular 
Kp_ang = [160; 160; 160];
Kv_ang = [1; 1; 1];
% Commanded Moment
M = Kp_ang .* err_rot + Kv_ang .* err_omega;

% =================== Your code ends here ===================

end
