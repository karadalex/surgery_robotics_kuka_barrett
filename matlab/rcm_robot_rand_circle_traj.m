%% Parameters
L = 0.68; % tool length
x0 = 0.1;
y0 = -0.15;
z0 = -0.2;
r0 = 0.1;

U_T_F = eye(4);
U_T_F(:,4) = [0.5;0;0.5;1];
U_T_F(1:3,1:3) = eul2rotm([0, pi/6, pi/6]);
O_F = [0;0;0;1];

%% Construct Circular trajectory
s = 0:0.1:1;
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

%% Robot
iiwa = loadrobot("kukaIiwa14");
homeConfig = homeConfiguration(iiwa);
endEffector = "iiwa_link_ee_kuka";

%% TCP Path
tcp = fulcrumEffectPath(P, L);

%% Transform all taskspace
tcp = U_T_F*[tcp.';ones(1, size(tcp, 1))];
tcp = tcp.';
tcp = tcp(:,1:3);
P = U_T_F*[P.';ones(1, size(P, 1))];
P = P.';
P = P(:,1:3);
PlanePoints = U_T_F*[A;B;C;ones(1,4)];
O_F = U_T_F*O_F;

%% Inverse Kinematics
ikSolutionSpace = [];
ik = inverseKinematics('RigidBodyTree',iiwa);
targetPose = eye(4);
targetPose(1:3,4) = tcp(1,:).';
targetPose(1:3,1:3) = eul2rotm([pi/2, pi, 0]);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = iiwa.homeConfiguration;
ikSolutionSpace = ik('iiwa_link_ee_kuka',targetPose,weights,initialguess);
for i=2:size(P)  
    %tic;
    targetPose(1:3,4) = tcp(i,:).';
    
    px = tcp(i,1); py = tcp(i,2); pz = tcp(i,3);
    r = sqrt(px^2+py^2+pz^2);
    th = atan2(sqrt(px^2+py^2), pz);
    phi = atan2(py, px);
    vx = [cos(th)*cos(phi); cos(th)*sin(phi); -sin(th)];
    vy = [-sin(phi); cos(phi); 0];
    vz = [sin(th)*cos(phi); sin(th)*sin(phi); cos(th)];
    vp = r*[sin(th)*cos(phi); sin(th)*sin(phi); cos(th)];
    %targetPose(1:3,1:3) = eul2rotm([pi/2, pi, 0]);
    targetPose(1:3,1:3) = [vx, vy, -vz];
    
    % Use solution of previous waypoint as initial guess, to avoid huge
    % jumps of robot between waypoints and speed up calculation of inverse
    % kinematics, because neighboring waypoints will probably have
    % neighboring solutions
    [configSoln,solnInfo] = ik('iiwa_link_ee_kuka',targetPose,weights,ikSolutionSpace(i-1,:));
    %disp(solnInfo);
    ikSolutionSpace = [ikSolutionSpace; configSoln];
    show(iiwa, configSoln, 'PreservePlot', false, 'Frames', 'off');
    %toc;
end

%% Plot data
close all
figure;
show(iiwa, homeConfig, 'PreservePlot', false, 'Frames', 'off');
hold on, grid on, view([45 45]);

plot3(P(:,1),P(:,2),P(:,3));
scatter3(O_F(1), O_F(2), O_F(3), 'g', 'filled');
f = fill3(PlanePoints(1,:),PlanePoints(2,:),PlanePoints(3,:),'m');
f.FaceAlpha = 0.1;
plot3(tcp(:,1),tcp(:,2),tcp(:,3),'c');
for i=1:size(P)
    dx = [tcp(i,1), P(i,1)];
    dy = [tcp(i,2), P(i,2)];
    dz = [tcp(i,3), P(i,3)];
    line = plot3(dx,dy,dz,'black');
    line.Color(4) = 0.2;
    
    show(iiwa, ikSolutionSpace(i,:), 'PreservePlot', false, 'Frames', 'off');
    drawnow;
    pause(2);
end

xlabel('x');
ylabel('y');
zlabel('z');
legend({'Circular trajectory', 'RCM (Fulcrum)', 'Separation plane', 'TCP Trajectory (transformed circular trajectory)'});
set(gca,'DataAspectRatio',[1 1 1]);
axis([-0.4 1 -0.5 0.5 0 1.5]);