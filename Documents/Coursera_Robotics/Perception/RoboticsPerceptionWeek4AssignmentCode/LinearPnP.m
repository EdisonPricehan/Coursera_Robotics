function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 3) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly
N = size(X,1);
x = [x ones(N,1)];
X = [X ones(N,1)];
x_c = zeros(size(x));
A = zeros(3*N,12);
for i = 1:N
    x_c(i,:) = (K \ x(i,:)')';
    u_c = x_c(i,1);
    v_c = x_c(i,2);
    w_c = x_c(i,3);
    A1 = [zeros(1,4) -w_c * X(i,:) v_c * X(i,:);
          w_c * X(i,:) zeros(1,4) -u_c * X(i,:);
          -v_c * X(i,:) u_c * X(i,:) zeros(1,4)];
    A(3*i-2:3*i,:) = A1;
end
[U S V] = svd(A);
P = V(:,end);
P = reshape(P,4,3);
P = P';
R = P(:,1:3);
t = P(:,end);
[U S V] = svd(R);
R = U * V';
if det(R) > 0
    t = t / S(1,1);
    C = -R' * t;
end
if det(R) < 0
    R = -R;
    t = -t / S(1,1);
    C = -R' * t;
end



