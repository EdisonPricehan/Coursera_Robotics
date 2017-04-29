function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;
L1 = 1.0;
L2 = 1.0;
eqn1 = L1 * cos(theta1) + L2 * cos(theta1 + theta2) == X;
eqn2 = L1 * sin(theta1) + L2 * sin(theta1 + theta2) == Y;
eqn = [eqn1, eqn2];
S = solve(eqn);
rads1 = double(S.theta1(2));
rads2 = double(S.theta2(2));
