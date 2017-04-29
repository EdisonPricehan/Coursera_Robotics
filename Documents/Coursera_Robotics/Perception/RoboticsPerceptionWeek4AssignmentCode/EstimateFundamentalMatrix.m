function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2
N = size(x1,1);
A = zeros(N,9);
for i = 1:N
    u1 = x1(i,1);
    v1 = x1(i,2);
    u2 = x2(i,1);
    v2 = x2(i,2);
    A(i,1) = u1 * u2;
    A(i,2) = v1 * u2;
    A(i,3) = u2;
    A(i,4) = u1 * v2;
    A(i,5) = v1 * v2;
    A(i,6) = v2;
    A(i,7) = u1;
    A(i,8) = v1;
    A(i,9) = 1;
end
[U, S, V] = svd(A);
F = V(:,end);
F = reshape(F,3,3);
F = F';
[U, S, V] = svd(F);
S(3,3) = 0;
F = U * S * V';
F = F / norm(F);

