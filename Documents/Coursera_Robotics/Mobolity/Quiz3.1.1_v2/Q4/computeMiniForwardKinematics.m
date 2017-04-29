function [endeff] = computeMiniForwardKinematics(rads1,rads2)
alpha = pi + (rads1 + rads2) / 2;
beta = rads1 - rads2;
L_longer = 2.0;
L_shorter = 1.0;

syms a b x;
eqn1 = cos(a) == (L_longer^2 + L_shorter^2 - x^2) / (2 * L_longer * L_shorter);
eqn2 = cos(b) == (L_longer^2 + x^2 - L_shorter^2) / (2 * L_longer * x);
eqn3 = cos(pi-beta/2) == (x^2 + L_shorter^2 - L_longer^2) / (2 * L_shorter * x);
eqn = [eqn1, eqn2, eqn3];
S = solve(eqn);
L = double(S.x(1));

X = L * cos(alpha);
Y = L * sin(alpha);
endeff = [X, Y];