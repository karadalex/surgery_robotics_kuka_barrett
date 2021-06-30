%% Trapezoidal Velocity Profile
accel2Val = [0, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, -0.5, 0, 0];

accel2 = [];
for i=1:size(accel2Val,2)
    accel2 = [accel2, accel2Val(i)*ones(1,100)];
end
ti = 0:1/size(accel2,2):1;
ti = ti(2:size(ti,2));

vel2 = 0.005*cumtrapz(accel2);
vel2 = vel2/max(vel2);
pos2 = 0.005*cumtrapz(vel2);
pos2 = 4*pos2/max(pos2) - 2;

close all;
subplot(1,2,1);
hold on, grid on
plot(ti,accel2);
plot(ti,vel2);
plot(ti,pos2);
legend(["acceleration","velocity","position"],'Location','best');
title('Trapezoidal velocity profile');
xlabel('time');


%% S-Curve Velocity Profile
jerkVal = [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0];

jerk = [];
for i=1:size(jerkVal,2)
    jerk = [jerk, jerkVal(i)*ones(1,100)];
end

accel1 = 0.005*cumtrapz(jerk);
vel1 = 0.005*cumtrapz(accel1);
vel1 = vel1/max(vel1);
pos1 = 0.005*cumtrapz(vel1);
pos1 = 4*pos1/max(pos1) - 2;

subplot(1,2,2);
hold on, grid on
plot(ti,accel1);
plot(ti,vel1);
plot(ti,pos1);
stairs(ti,jerk);
legend(["acceleration","velocity","position","jerk"],'Location','best');
title('S-Curve velocity profile');
xlabel('time');