function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    %mu_m = zeros(4,1);
    p = 1e-2;
    v = 1;
    sigma_m = [p 0 0 0;
               0 p 0 0;
               0 0 v 0;
               0 0 0 v];
    
    %X_m = mvnrnd(mu_m,sigma_m,4);
    %Sigma_m = 1e6 * mvnpdf(X_m,mu_m,sigma_m);
    
    %mu_o = zeros(2,1);
    sigma_o = 0.01 * eye(2);
    %X_o = mvnrnd(mu_o,sigma_o,2);
    %Sigma_o = 0.01 * mvnpdf(X_o,mu_o,sigma_o);
    
    % time step (30Hz)
    dt = t - previous_t;

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.001 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    A = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
    C = [1 0 0 0;
         0 1 0 0];
    % construct noisy Gaussion covariance matrix P at the current time step
    P = A * param.P * A' + sigma_m;
    R = C * P * C' + sigma_o;
    % Kalman gain K
    K = P * C' * inv((R + C * P * C'));
    % measurement matrix z
    z = [x y]';
    % make predictions of the current state (4X1 matrix)
    state = (A * state' + K * (z - C * A * state'))';
    % extract positions of x and y from current state vector
    predictx = state(1) + dt * 10 * state(3);
    predicty = state(2) + dt * 10 * state(4);
    % upadte Gaussian covariance matrix of 
    param.P = P - K * C * P;
end
