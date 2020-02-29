L = zeros(1,7);
d = [0.36, 0, 0.42, 0, 0.4, 0, 0];
a = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];

[M, M_joints] = fwdKinSym(L(1:6), d(1:6), a(1:6));