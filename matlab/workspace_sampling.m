iiwa = loadrobot("kukaIiwa14");
homeConfig = homeConfiguration(iiwa);
endEffector = "iiwa_link_ee_kuka";

show(iiwa, homeConfig, 'PreservePlot', false, 'Frames', 'off');
hold on
grid on
view([45 45]);
axis([-1 1 -1 1 0 1.5]);

a = -pi/2;
b = pi/2;
samples = 1*1e2;

% Speedup calculation of points with parallelization
parfor i=1:samples
    config = homeConfiguration(iiwa);
    q = (b-a).*rand(1,7) + a;
    for j=1:7
        config(j).JointPosition = q(1,j);
    end
    % Calculate Forward Kinematics
    pose = getTransform(iiwa, config, endEffector);
    J = geometricJacobian(iiwa,randomConfiguration(iiwa), endEffector);
    M = sqrt(det(J*J.'));
    scatter3(pose(1,4),pose(2,4),pose(3,4),30,M);
end
colorbar;