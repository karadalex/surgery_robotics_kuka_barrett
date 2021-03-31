L = 0.68; % tool length

%% Generate path points
x = [-0.2, 0.2];
y = [-0.2, 0.2];
z = -0.2*ones(size(x));
P1 = [x(1); y(1); z(1)];
P2 = [x(2); y(2); z(2)];

%% Construct Line segments
s = 0:0.01:1;
P = zeros(size(s,2),3);
for i=1:size(s,2)
    Pi = s(i)*P1 + (1-s(i))*P2;
    P(i,:) = Pi.';
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
scatter3(x,y,z,'r','filled');
scatter3(0, 0, 0, 'g', 'filled');
f = fill3(A,B,C,'m');
f.FaceAlpha = 0.1;
plot3(tcp(:,1),tcp(:,2),tcp(:,3),'c');
for i=1:size(P)
    dx = [tcp(i,1), P(i,1)];
    dy = [tcp(i,2), P(i,2)];
    dz = [tcp(i,3), P(i,3)];
    line = plot3(dx,dy,dz,'black');
    line.Color(4) = 0.4;
end

xlabel('x');
ylabel('y');
zlabel('z');
legend({'Line segment trajectory', 'Control Points', 'RCM (Fulcrum)', 'Separation plane', 'TCP Trajectory (transformed line segment)'});
set(gca,'DataAspectRatio',[1 1 1]);