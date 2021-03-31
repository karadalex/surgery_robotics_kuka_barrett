%% Parameters
L = 0.68; % tool length
x = [0, -0.30];
y = [0, 0];
z = [-0.2, -0.2];
r = [0.1, 0.2];
angle = [pi, pi/2];
phi0 = [0, 3*pi/2];

%% Construct Arc trajectory
s0 = 0:0.01:angle(1)/(2*pi);
s1 = angle(2)/(2*pi):-0.01:0;
P = zeros(size(s0,2)+size(s1,2),3);
for i=1:size(s0,2)
    P(i,1) = r(1)*cos(2*pi*s0(i)+phi0(1)) + x(1);
    P(i,2) = r(1)*sin(2*pi*s0(i)+phi0(1)) + y(1);
    P(i,3) = z(1);
end
for i=1:size(s1,2)
    j = size(s0,2) + i;
    P(j,1) = r(2)*cos(2*pi*s1(i)+phi0(2)) + x(2);
    P(j,2) = r(2)*sin(2*pi*s1(i)+phi0(2)) + y(2);
    P(j,3) = z(2);
end

%% Plane
A = [-0.2, -0.2, 0.2, 0.2]*1.5;
B = [-0.2, 0.2, 0.2, -0.2]*1.5;
C = zeros(size(A));

%% TCP Path
tcp = fulcrumEffectPath(P, L);

%% Plot data
close all
figure;
hold on, grid on, view([45 45]);

plot3(P(:,1),P(:,2),P(:,3));
scatter3(0, 0, 0, 'g', 'filled');
f = fill3(A,B,C,'m');
f.FaceAlpha = 0.1;
plot3(tcp(:,1),tcp(:,2),tcp(:,3),'c');
scatter3(x,y,z,'r','filled');
for i=1:size(P)
    dx = [tcp(i,1), P(i,1)];
    dy = [tcp(i,2), P(i,2)];
    dz = [tcp(i,3), P(i,3)];
    line = plot3(dx,dy,dz,'black');
    line.Color(4) = 0.2;
end

xlabel('x');
ylabel('y');
zlabel('z');
legend({'Circular Arc trajectory', 'RCM (Fulcrum)', 'Separation plane', 'TCP Trajectory (transformed arc trajectory)', 'Center of arc'});
set(gca,'DataAspectRatio',[1 1 1]);