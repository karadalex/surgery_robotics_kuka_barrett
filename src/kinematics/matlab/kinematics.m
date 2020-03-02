L = zeros(1,7);
d = [0.36, 0, 0.42, 0, 0.4, 0, 0];
a = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];

[M, M_joints] = fwdKinSym(L(2:7), d(2:7), a(2:7));

M_0_3 = eye(4);
for i=1:3
    M_0_3 = M_0_3 * M_joints(:,:,i);
end
p = M_0_3 * M_joints(:,4,4);

R_3_6 = eye(4);
for i=4:6
    R_3_6 = R_3_6 * M_joints(:,:,i);
end
R_3_6 = R_3_6(1:3,1:3);