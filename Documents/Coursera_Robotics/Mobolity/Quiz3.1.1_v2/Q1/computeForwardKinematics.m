function endeff = computeForwardKinematics(rads)


L = 1.0;
x = L * cos(rads);
y = L * sin(rads);


endeff = [x,y];

