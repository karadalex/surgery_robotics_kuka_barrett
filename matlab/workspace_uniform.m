iiwa = loadrobot("kukaIiwa14");
homeConfig = homeConfiguration(iiwa);
endEffector = "iiwa_link_ee_kuka";

show(iiwa, homeConfig, 'PreservePlot', false, 'Frames', 'off');
hold on
grid on
view([45 45]);
axis([-1 1 -1 1 0 1.5]);

qr = -pi/2:1:pi/2;
[q1,q2,q3,q4,q5,q6,q7] = ndgrid(qr, qr, qr, qr, qr, qr, qr);

parfor i=1:size(qr,2)^7
    q = [q1(i),q2(i),q3(i),q4(i),q5(i),q6(i),q7(i)];
    config = homeConfiguration(iiwa);
    for j=1:7
        config(j).JointPosition = q(j);
    end
    pose = getTransform(iiwa, config, endEffector);
    scatter3(pose(1,4),pose(2,4),pose(3,4),'b');
    %show(iiwa, config, 'PreservePlot', false, 'Frames', 'off');
    %drawnow;
end

