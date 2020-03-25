%d = [0.36, 0, 0.42, 0, 0.4, 0, 0];
%a = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];

%d = [0.157, 0.203, 0.204, 0.216, 0.184, 0.216, 0.8];
%a = [0, -pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2];

%d = [0.157, 0.203, 0.420, 0.184, 0.216, 0.8];
%a = [0, -pi/2, 0, -pi/2, -pi/2, pi/2];

d = [0.36, 0, 0.42, 0, 0.4, 0, 0];
L = [0, 0, 0, 0, 0, 0, 0];
%a = [0, -pi/2, pi, -pi/2, -pi/2, pi/2];
a = [0, -pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2];

[M, M_joints] = fwdKinSym(L, d, a);

M_0_4 = eye(4);
for i=1:4
    M_0_4 = M_0_4 * M_joints(:,:,i);
end
p_0_5 = M_0_4 * M_joints(:,4,5);
p_1_5 = inv(M_joints(:,:,1)) * p_0_5;

M_4_7 = eye(4);
for i=5:7
    M_4_7 = M_4_7 * M_joints(:,:,i);
end
R_4_7 = M_4_7(1:3,1:3);


%% Inverse Kinematics
M_U_0 = pose(0,0,1,0,0,0);
M_U_TCP = pose(0, -0.68, 1.5, pi, 0, pi/2);
%M_U_TCP = pose(0.2, -0.68, 1.5, 0.2, 0.3, 0.4);
M_7_TCP = pose(0,0,0,0,0,0);
M_0_7 = inv(M_U_0) * M_U_TCP * inv(M_7_TCP);
px = M_0_7(1,4); py = M_0_7(2,4); pz = M_0_7(3,4);

pxy = sqrt(px^2 + py^2);
th1_1 = atan2(py/pxy, px/pxy);
th1_2 = atan2(-py/pxy, -px/pxy);
M_0_1 = subs(M_joints(:,:,1), "th1", th1_1);
p_0_5 = M_0_7(:,4);
p_1_5 = eval(inv(M_0_1) * p_0_5);
p_1_5_len = sqrt(p_1_5(1)^2 + p_1_5(2)^2 + p_1_5(3)^2);

c4 = (p_1_5_len^2 - d(3)^2 - d(5)^2) / (2*d(3)*d(5));
s4 = sqrt(1-c4^2);
th4_1 = atan2(s4, c4);
th4_2 = atan2(-s4, c4);

%phi_cos = (d(3)^2 + p_1_5_len^2 - d(5)^2) / (2*d(3)*p_1_5_len);
%phi = atan2(sqrt(1-phi_cos^2), phi_cos);
phi = atan2(d(5)*s4, d(3)+d(5)*c4);
th2_1 = atan2(sqrt(px^2 + py^2), p_1_5(3)) + phi;
th2_2 = atan2(sqrt(px^2 + py^2), p_1_5(3)) - phi;

th3 = 0;

R_0_4 = subs(M_0_4(1:3,1:3), sym('th', [1, 4]), [th1_1, th2_1, th3, th4_1]);
R_4_7 = eval(inv(R_0_4) * M_0_7(1:3,1:3));
ix = R_4_7(1,1); jx = R_4_7(1,2); kx = R_4_7(1,3);
iy = R_4_7(2,1); jy = R_4_7(2,2); ky = R_4_7(2,3);
iz = R_4_7(3,1); jz = R_4_7(3,2); kz = R_4_7(3,3);

%c6 = ky; s6 = sqrt(1-c6^2);
s6 = sqrt(kx^2 + kz^2);
th6_1 = atan2(s6, ky);
th6_2 = atan2(-s6, ky);
%th6_1 = atan2(s6, c6);
%th6_2 = atan2(-s6, c6);

th7_1 = atan2(jy, -iy); % corresponds to th6_1 where s6>0
th7_2 = atan2(-jy, iy); % corresponds to th6_1 where s6<0

th5_1 = atan2(-kz, kx); % corresponds to th6_1 where s6>0
th5_2 = atan2(kz, -kx); % corresponds to th6_1 where s6<0

% Validate the first solution
sol1 = [th1_1, th2_1, th3, th4_1, th5_2, th6_2, th7_2];
Mv_joints = subs(M_joints, sym('th', [1, 7]), sol1);
Mv_0_7 = eye(4);
for i = 1:1:7
    Mv_0_7 = Mv_0_7 * eval(Mv_joints(:,:,i));
end
