L = 0.68; % tool length

%% Generate path points
x = [-0.2, -0.25, -0.1, 0.2];
y = [-0.2, 0.2, -0.4, 0.2];
z = -0.2*ones(size(x));

P1 = [x(1); y(1); z(1)];
P2 = [x(2); y(2); z(2)];
P3 = [x(3); y(3); z(3)];
P4 = [x(4); y(4); z(4)];
G1 = [P1, P2, P3, P4];
G2 = [-1, 3, -3, 1; 3, -6, 3, 0; -3, 3, 0, 0; 1, 0, 0, 0];


%% Construct Bezier Curve
s = 0:0.01:1;
P = zeros(size(s,2),3);
for i=1:size(s,2)
    Gs = [s(i)^3; s(i)^2; s(i); 1];
    Pi = G1*G2*Gs;
    P(i,:) = Pi.';
end

%% Plane
A = [-0.2, -0.2, 0.2, 0.2]*1.5;
B = [-0.2, 0.2, 0.2, -0.2]*1.5;
C = zeros(size(A));

%% TCP Path
tcp = zeros(size(P));
dists = zeros(size(P,1),1); % all distances must be equal to the legth of the tool
for i=1:size(P)
    px = P(i,1); py = P(i,2); pz = P(i,3);
    r = sqrt(px^2+py^2+pz^2);
    th = atan2(sqrt(px^2+py^2), pz);
    phi = atan2(py, px);
    vx = [cos(th)*cos(phi); cos(th)*sin(phi); -sin(th)];
    vy = [-sin(phi); cos(phi); 0];
    vz = [-sin(th)*cos(phi); -sin(th)*sin(phi); cos(th)];
    vp = r*[sin(th)*cos(phi); sin(th)*sin(phi); cos(th)];
    T = zeros(4,4);
    R = [vx, vy, vz];
    T(1:3,1:3) = R;
    T(1:3,4) = vp;
    T(4,4) = 1;
    Td = eye(4);
    Td(1:3,4) = (r-L)/r*vp;
    tcp_point = Td*inv(T)*[P(i,:).'; 1];
    tcp(i,:) = tcp_point(1:3).';
    dist2 = (px-tcp(i,1))^2 + (py-tcp(i,2))^2 + (pz-tcp(i,3))^2;
    dists(i) = sqrt(dist2);
end

%% Plot data
close all
figure;
hold on, grid on, view([45 45]);

plot3(P(:,1),P(:,2),P(:,3));
scatter3(x,y,z,'r','filled');
scatter3(0, 0, 0, 'g', 'filled');
f = fill3(A,B,C,'m');
f.FaceAlpha = 0.1;
plot3(tcp(:,1),tcp(:,2),tcp(:,3));
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
legend({'Bezier Curve', 'Control Points', 'RCM (Fulcrum)'});
set(gca,'DataAspectRatio',[1 1 1]);