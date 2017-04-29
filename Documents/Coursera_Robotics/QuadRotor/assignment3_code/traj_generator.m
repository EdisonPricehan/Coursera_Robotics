function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

%persistent waypoints0 traj_time d0
%if nargin > 2
 %   d = waypoints(:,2:end) - waypoints(:,1:end-1);
  %  d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
  %  traj_time = [0, cumsum(d0)];
  %  waypoints0 = waypoints;
%else
 %   if(t > traj_time(end))
 %       t = traj_time(end);
 %   end
 %   t_index = find(traj_time >= t,1);
%
 %   if(t_index > 1)
 %       t = t - traj_time(t_index-1);
 %   end
 %   if(t == 0)
 %       desired_state.pos = waypoints0(:,1);
 %   else
 %       scale = t/d0(t_index-1);
 %       desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
 %   end
 %   desired_state.vel = zeros(3,1);
 %   desired_state.acc = zeros(3,1);
 %   desired_state.yaw = 0;
  %  desired_state.yawdot = 0;
%end
%


%% Fill in your code here

persistent coef_x coef_y coef_z waypoints0 traj_time d0
if nargin > 2
    % setup trajectory segment times
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    % solve for coefficients in x,y,z
    coef_x = getCoef(waypoints0(1,1:end)');
    coef_y = getCoef(waypoints0(2,1:end)');
    coef_z = getCoef(waypoints0(3,1:end)');
    
else
    % provide the trajectory point based on the coefficients
    if(t > traj_time(end))
        t = traj_time(end) - 0.0001;
    end
    
    t_index = find(traj_time >= t,1) - 1; %between 1:n
    
    if (t_index == 0)
        t_index = 1;
    end
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = 0 * waypoints0(:,1);
        desired_state.acc = 0 * waypoints0(:,1);
    else
        %create scaled time, value between 0 to 1
        scale = (t-traj_time(t_index)) / d0(t_index);
        
        index = (t_index-1)*8+1:t_index*8;
        
        %calculate position:
        t0 = polyT(8,0,scale)';
        desired_state.pos = [coef_x(index)' * t0; coef_y(index)' * t0; coef_z(index)' * t0];
        
        %calculate velocity:
        t1 = polyT(8,1,scale)';
        desired_state.vel = [coef_x(index)' * t1; coef_y(index)' * t1; coef_z(index)' * t1] .* (1/d0(t_index));
        
        %calculate acceleration:
        t2 = polyT(8,2,scale)';
        desired_state.acc = [coef_x(index)'*t2; coef_y(index)'*t2; coef_z(index)'*t2] .* (1/d0(t_index)^2);
    end
    
    % leave desired yaw and yawdot at zero
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

function [B] = polyT(n, k, t)
% A function that gives back the kth derivative of nth order polynomial
% w.r.t. t. (t can be any non-negative value)
B = zeros(n,1);
D = zeros(n,1);

% Init:
for i=1:n
    D(i) = i-1;
    B(i) = 1;
end

% Derivative:
for j=1:k
    for i=1:n
        B(i) = B(i) * D(i);
        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end

% put in t value
for i=1:n
    B(i) = B(i) * t^D(i);
end

% Transpose the T matrix so that the function returns a vertical vector
B = B';

end


function [coef, A, b] = getCoef(waypoints)
% Creates matrix A, b and solves for the coefficient vector coef.

% The number of segments 
n = size(waypoints,1)-1;

% b matrix is a 32x1 matrix in this case. Only set the first 8 values is
% enough since other elements are all 0s.
b = zeros(8*n,1);
for i=1:n
    b(i) = waypoints(i);
    b(i+n) = waypoints(i+1);
end

% A matrix is built up from all the constraints
A = zeros(8*n,8*n);

% Constraint 1 ==> Pi(t=0) = wi for all i=1:n
% Ex: P1(0) = w1, a11 = w1
% Ex: A(1,:)=[1 0 0 0 0 0 0 0 zero(1,8*(n-1))]
% Ex: b(1)=w1
for i=1:n
    A(i,((i-1)*8)+1:i*8) = polyT(8,0,0);
% Constraint 2 ==> Pi(t=1) = wi+1 for all i=1:n
% Ex: P1(1) = w2, a11+a12+?+a18 = w2
% Ex: A(n+1,:)=[1 1 1 1 1 1 1 1 zero(1,8*(n-1))]
% Ex: b(n+1)=w2
    A(i+n,((i-1)*8)+1:i*8) = polyT(8,0,1);
end

% Constraint 3 ==> P1_k(t=0) = 0 for all k=1..3 (derivative)
% Ex: P1_1(t=0)=0, a12 = 0
% Ex: A(2*n+1,:)=[0 1 0 0 0 0 0 0 zero(1,8*(n-1))]
% Ex: b(2*n+1)=0
for k=1:3
    A(2*n+k,1:8) = polyT(8,k,0);
% Constraint 4 ==> Pn_k(t=1) = 0 for all k=1..3 (derivative)
% Ex: Pn_1(t=1)=0, a12 + 2a13 + 3a14 +?+ 7a18 = 0
% Ex: A(2*n+3+1,:)=[zero(1,8*(n-1)) 0 1 2 3 4 5 6 7]
% Ex: b(2*n+3+1)=0
    A(2*n+3+k,(end-7):end) = polyT(8,k,1);
end

% Constraint 5 ==> Pi-1_k(t=1) = Pi_k(t=0) for all i=2..n and k=1..6
% Ex: P1_1(t=1)-P2_1(t=0) = 0, a12 + 2a13 +?+7a18 - a22 = 0
% Ex: A(2*n+6+1,)=[0 1 2 3 4 5 6 7 0 -1 0 0 0 0 0 0 zeros]
% Ex: b(2*n+6+1)=0
for i=2:n
    for k=1:6
        A(2*n+6+(i-2)*6+k, (i-2)*8+1:((i-2)*8+n*n)) = [polyT(8,k,1) -polyT(8,k,0)];
    end
end

% Now solve for the coefficients
coef = A\b;

end

