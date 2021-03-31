%% Parameters
L = 0.68; % tool length
x0 = 0.1;
y0 = -0.15;
z0 = -0.2;
r0 = 0.1;

%% Construct Circular trajectory
s = 0:0.01:1;
P = zeros(size(s,2),3);
for i=1:size(s,2)
    P(i,1) = r0*cos(2*pi*s(i)) + x0;
    P(i,2) = r0*sin(2*pi*s(i)) + y0;
    P(i,3) = z0;
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
legend({'Circular trajectory', 'RCM (Fulcrum)', 'Separation plane', 'TCP Trajectory (transformed circular trajectory)'});
set(gca,'DataAspectRatio',[1 1 1]);